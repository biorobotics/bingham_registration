/*
 *  File header for registration_main.cpp:
 *      The main function takes in input data files (.txt form) and performs dual
        quaternion registration with Bingham distribution-based filtering. It outputs
        the resutl pose (Xreg) and a record of all the intermediate poses as well (Xregsave).

        Input data:
            Use points only for pose estimation: 
                -pm [full path of the ptcld moving dataset]
                -pf [full path of the ptcld fixed dataset]
            Use both points and normals for pose estimation:
                Also provide:
                -nm [full path of the normal moving dataset]
                -nf [full path of the normal fixed dataset]
 * 
 */

#include <iostream>
#include <iomanip>
#include <limits>
#include <fstream>
#include <cstring>
#include <ctime>
#include "registration_est_bingham_kf_rgbd.h"

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

const int NUM_OF_RUNS = 10; // # of runs to run the registration for average performance

// Should at least provide the two ptcld datasets
int main(int argc, char *argv[]) {

    string movingPointsString;
    string fixedPointsString;
    string movingNormalsString;
    string fixedNormalsString;
    int useNormal = 0;
    int normalFileProvided = 0;
    int pointFileProvided = 0;

    // Replace filenames by the arguments
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];

        // Parse ptcld_moving from .txt
        if (arg == "-pm") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                movingPointsString = argv[i];
                pointFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-pm option requires filepath for moving pointcloud." 
                << std::endl;
                return 1;
            }
        }
        // Parse ptcld_fixed from .txt
        else if (arg == "-pf") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedPointsString = argv[i];
                pointFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-pf option requires filepath for fixed pointcloud." 
                << std::endl;
                return 1;
            }
        }
        // Parse normal moving from .txt
        else if (arg == "-nm") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                movingNormalsString = argv[i];
                useNormal = 1; 
                normalFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-nm option requires filepath for moving normals." 
                << std::endl;
                return 1;
            }
        }
        // Parse normal fixed from .txt
        else if (arg == "-nf") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedNormalsString = argv[i];
                useNormal = 1;
                normalFileProvided++;
            } else { // If there was no argument to the destination option.
                std::cerr << "-nm option requires filepath for moving normals." 
                << std::endl;
                return 1;
            }
        }
        // If an invalid argument is provided
        else {
            std::cerr << "Invalid argument. Usage: -pm .. -pv .. (optional) (-nm .. -nv ..)" 
             << std::endl;
            return 1;
        }
    }

    // Check if enough arguments were entered
    if (pointFileProvided != 2 || (useNormal && normalFileProvided != 2)) {
        std::cerr << "Not enough argument. Usage: -pm .. -pv .. (optional) (-nm .. -nv ..)" 
        << std::endl;
        return 1;
    }

    // Read .text files for data points
    ifstream sensedFile;    // stream for ptcld moving
    ifstream CADFile;       // stream for ptcld fixed
    ifstream sensedNormalFile;  // stream for normal moving
    ifstream CADNormalFile;  // stream for normal fixed

    cout << "\nStatic point cloud: " << fixedPointsString << "\nMoving point cloud: " 
    << movingPointsString << endl;
    
    sensedFile.open(movingPointsString, ifstream::in);
    CADFile.open(fixedPointsString, ifstream::in);
    // Check if files are valid
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedPointsString << ", " << movingPointsString << " not found" 
        << "\n";
        return 1; // exit if file not found
    } 

    // Open the normal-related files if normal option is used
    if (useNormal)
    {
        sensedNormalFile.open(movingNormalsString, ifstream::in);
        CADNormalFile.open(fixedNormalsString, ifstream::in);
        if (!sensedNormalFile.good() || !CADNormalFile.good()) {
            cout << "Files " << fixedNormalsString << ", " << movingNormalsString 
            << " not found" << "\n";
            return 1; // exit if file not found
        } 
    }
    
    // Vector for appending points
    std::vector<Eigen::Matrix<long double, 3, 1>,
                Eigen::aligned_allocator<Eigen::Matrix<long double, 3, 1>>
                > pointVector;
    
    // read sensedFile into ptcldMoving
    while (!sensedFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        sensedFile.getline(buf, MAX_CHARS_PER_LINE);
        // This extra detection is necessary for preventing bug
        if (sensedFile.eof())
            break;
        // store line in a vector temp
        istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);

        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(movingPointsString + 
                       ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingPointsString + 
                       ": Input data doesn't match dimension (too many per line)");
        
        // Add temp to list of vectors  
        pointVector.push_back(temp);
    }

    PointCloud ptcldMoving(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldMoving.col(i) = pointVector[i];
    
    pointVector.clear();    // Clear pointVector for the following read

    // read CADFile into ptcldFixed
    while (!CADFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        CADFile.getline(buf, MAX_CHARS_PER_LINE);
        // This extra detection is necessary for preventing bug
        if (CADFile.eof())
            break;
        // store line in a vector
        std::istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        if(iss.fail()) 
            call_error(fixedPointsString + 
                       ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingPointsString + 
                       ": Input data doesn't match dimension (too many per line)");
        // Add temp to list of vectors
        pointVector.push_back(temp);
    }

    PointCloud ptcldFixed(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldFixed.col(i) = pointVector[i];
    
    pointVector.clear();

    /*** The following registration is performed when normals are used ***/
    if (useNormal) {
        // Read sensedNormalFIle into normalMoving
        while (!sensedNormalFile.eof()) {
            // read an entire line into memory
            char buf[MAX_CHARS_PER_LINE];
            sensedNormalFile.getline(buf, MAX_CHARS_PER_LINE);
            // This extra detection is necessary for preventing bug
            if (sensedNormalFile.eof())
                break;
            // store line in a vector
            istringstream iss(buf);
            Vector3ld temp;
            iss >> temp(0) >> temp(1) >> temp(2);

            // Make sure all three were read in correctly
            if(iss.fail())
                call_error(movingNormalsString + 
                           ": Input data doesn't match dimension (too few per line)");
            
            // Make sure there are no more to read
            float eofCheck;
            iss >> eofCheck;
            if(iss.good()) 
                call_error(movingNormalsString + 
                           ": Input data doesn't match dimension (too many per line)");
            
            // Add temp to list of vectors  
            pointVector.push_back(temp);
        }

        PointCloud normalMoving(3, pointVector.size());

        for(int i=0; i<pointVector.size(); i++)
            normalMoving.col(i) = pointVector[i];
        
        pointVector.clear();

        // Read CADNormalFIle into normalFixed
        while (!CADNormalFile.eof()) {
            // read an entire line into memory
            char buf[MAX_CHARS_PER_LINE];
            CADNormalFile.getline(buf, MAX_CHARS_PER_LINE);
            // Extra detection
            if (CADNormalFile.eof())
                break;
            // store line in a vector
            istringstream iss(buf);
            Vector3ld temp;
            iss >> temp(0) >> temp(1) >> temp(2);

            // Make sure all three were read in correctly
            if(iss.fail())
                call_error(fixedNormalsString + 
                            ": Input data doesn't match dimension (too few per line)");
            
            // Make sure there are no more to read
            float eofCheck;
            iss >> eofCheck;
            if(iss.good()) 
                call_error(fixedNormalsString + 
                           ": Input data doesn't match dimension (too many per line)");
            
            // Add temp to list of vectors  
            pointVector.push_back(temp);
        }

        PointCloud normalFixed(3, pointVector.size());

        for(int i=0; i<pointVector.size(); i++)
            normalFixed.col(i) = pointVector[i];
        
        pointVector.clear();
        sensedNormalFile.close();
        CADNormalFile.close();

        // Force the memory to be freed
        pointVector.shrink_to_fit();
        
        sensedFile.close();
        CADFile.close();
        
        // For timing and outputing data purpose
        double timeSum = 0;
        ofstream myFile;
        myFile.open("result_normal.txt");

        for (int i = 0; i < NUM_OF_RUNS; i++) {
            clock_t begin = clock();    // For timing the performance

            // Run the registration function with normals
            struct RegistrationResult *result = registration_est_bingham_normal(
                                                &ptcldMoving, &ptcldFixed, 
                                                &normalMoving, &normalFixed);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            timeSum += elapsed_secs;
        
            cout << "Xreg:" << endl << result->Xreg.transpose() << endl << endl;
            cout << "Xregsave:" << endl << result->Xregsave.transpose() << endl;
            
            // For pose recording, just record the answer from one run
            if (i == 9)
            {
                myFile << "Xreg:" << endl << result->Xreg.transpose() << endl << endl;
                myFile << "Xregsave:" << endl << result->Xregsave.transpose() 
                       << endl << endl;
            }
        }
        cout << "Average registration runtime is: " << timeSum / NUM_OF_RUNS 
            << " seconds." << endl << endl;
        myFile << "Average registration runtime is: " << timeSum / NUM_OF_RUNS 
            << " seconds." << endl;
        myFile.close();

        return 0;
    }

    /*** The following registration is performed when normals are not used ***/
    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    
    double timeSum = 0;
    ofstream myFile;
    myFile.open("result_no_normal.txt");

    for (int i = 0; i < NUM_OF_RUNS; i++) {
        clock_t begin = clock();    // For timing the performance

        // Run the registration function without normals
        struct RegistrationResult *result = registration_est_bingham_kf_rgbd(&ptcldMoving, 
                                                                             &ptcldFixed);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        timeSum += elapsed_secs;
    
        cout << "Xreg:" << endl << result->Xreg.transpose() << endl << endl;
        cout << "Xregsave:" << endl << result->Xregsave.transpose() << endl;
        if (i == 9)
        {
            myFile << "Xreg:" << endl << result->Xreg.transpose() << endl << endl;
            myFile << "Xregsave:" << endl << result->Xregsave.transpose() << endl << endl;
        }
    }
    cout << "Average registration runtime is: " << timeSum / NUM_OF_RUNS 
                                                << " seconds." << endl << endl;
    myFile << "Average registration runtime is: " << timeSum / NUM_OF_RUNS 
                                                  << " seconds." << endl;
    myFile.close();

    return 0;
}