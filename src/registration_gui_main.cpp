/*
 *  File header for registration_gui_main.cpp:
        Provides the registration function that will be called by the python GUI
 *      The function takes in input data files (.txt form) and performs dual
        quaternion registration with Bingham distribution-based filtering. It outputs
        the resutl pose (Xreg) and a record of all the intermediate poses as well (Xregsave).
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
extern "C" long double* qf_register(char* movingData, char* fixedData) {

    long double* returnArray = new long double[6];

    cout << &movingData << endl;
    char* movingPointsString = movingData;

    char* fixedPointsString = fixedData;
    char* movingNormalsString;
    char* fixedNormalsString;

    // Read .text files for data points
    ifstream sensedFile;    // stream for ptcld moving
    ifstream CADFile;       // stream for ptcld fixed
    ifstream sensedNormalFile;  // stream for normal moving
    ifstream CADNormalFile;  // stream for normal fixed

    int useNormal = 0;

    cout << "\nStatic point cloud: " << fixedPointsString << "\nMoving point cloud: " 
    << movingPointsString << endl;
    
    sensedFile.open(movingPointsString, ifstream::in);
    CADFile.open(fixedPointsString, ifstream::in);
    // Check if files are valid
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedPointsString << ", " << movingPointsString << " not found" 
        << "\n";
        return returnArray; // exit if file not found
    } 

    // Open the normal-related files if normal option is used
    if (useNormal)
    {
        sensedNormalFile.open(movingNormalsString, ifstream::in);
        CADNormalFile.open(fixedNormalsString, ifstream::in);
        if (!sensedNormalFile.good() || !CADNormalFile.good()) {
            cout << "Files " << fixedNormalsString << ", " << movingNormalsString 
            << " not found" << "\n";
            return returnArray; // exit if file not found
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
            call_error(//movingPointsString + 
                       ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(//movingPointsString + 
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
            call_error(//fixedPointsString + 
                       ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(//movingPointsString + 
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
                call_error(//movingNormalsString + 
                           ": Input data doesn't match dimension (too few per line)");
            
            // Make sure there are no more to read
            float eofCheck;
            iss >> eofCheck;
            if(iss.good()) 
                call_error(//movingNormalsString + 
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
                call_error(//fixedNormalsString + 
                            ": Input data doesn't match dimension (too few per line)");
            
            // Make sure there are no more to read
            float eofCheck;
            iss >> eofCheck;
            if(iss.good()) 
                call_error(//fixedNormalsString + 
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
        struct RegistrationResult *result;
        for (int i = 0; i < NUM_OF_RUNS; i++) {
            clock_t begin = clock();    // For timing the performance

            // Run the registration function with normals
            result = registration_est_bingham_normal(&ptcldMoving, &ptcldFixed, 
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
        Map<Eigen::Matrix<long double, 6, 1>>(returnArray,6,1) = result->Xreg;
        return returnArray;
    }

    /*** The following registration is performed when normals are not used ***/
    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    
    double timeSum = 0;
    ofstream myFile;
    struct RegistrationResult *result;
    myFile.open("result_no_normal.txt");

        clock_t begin = clock();    // For timing the performance

        // Run the registration function without normals
        result = registration_est_bingham_kf_rgbd(&ptcldMoving, 
                                                  &ptcldFixed);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
        cout << "Xreg:" << endl << result->Xreg.transpose() << endl << endl;
        cout << "Xregsave:" << endl << result->Xregsave.transpose() << endl;

    cout << "Registration runtime is: " << elapsed_secs
                                                << " seconds." << endl << endl;
    myFile << "Registration runtime is: " << elapsed_secs
                                                  << " seconds." << endl;
    myFile.close();
    Map<Eigen::Matrix<long double, 6, 1>>(returnArray,6,1) = result->Xreg;
    return returnArray;
}

int main (void) {
    return 0;
}