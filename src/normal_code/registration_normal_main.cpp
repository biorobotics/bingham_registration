#include <iostream>
#include <iomanip>
#include <limits>
#include <fstream>
#include <cstring>
#include <ctime>
#include "registration_est_bingham_normal.h"

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

// main:
// Read sensed data and CAD data from .txt files and use them as input for
// running the registration function

int main(int argc, char *argv[]) {
    // Defaults
    string movingPointFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/ptcld_moving_6.txt";
    string fixedPointFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/ptcld_fixed_6.txt";
    string movingNormalFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/normal_moving_6.txt";
    string fixedNormalFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/normal_fixed_6.txt";
    // Replace filenames if arguments exist
    //cout << "Precision of long double is: " << numeric_limits<long double>::digits10 << endl;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-m"){
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                // Increment 'i' so we don't get the argument as the next argv[i].
                i++;
                movingPointFileString = argv[i];
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "-m option requires filepath for moving pointcloud." << std::endl;
                return 1;
            }
        }
        if (arg == "-f"){
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedPointFileString = argv[i]; 
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "-f option requires filepath for fixed pointcloud." << std::endl;
                return 1;
            }
        }
    }

    // Read .text files for data points
    ifstream sensedFile;
    ifstream CADFile;
    ifstream sensedNormalFile;
    ifstream CADNormalFile;

    cout << "\nStatic point cloud: " << fixedPointFileString << "\nMoving point cloud: " << movingPointFileString << endl;

    sensedFile.open(movingPointFileString, ifstream::in);
    CADFile.open(fixedPointFileString, ifstream::in);
    
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedPointFileString << ", " << movingPointFileString << " not found" << "\n";
        return 1; // exit if file not found
    } 

    cout << "\nStatic normal set: " << fixedNormalFileString << "\nMoving normal set " << movingNormalFileString << endl;

    sensedNormalFile.open(movingNormalFileString, ifstream::in);
    CADNormalFile.open(fixedNormalFileString, ifstream::in);
    
    if (!sensedNormalFile.good() || !CADNormalFile.good()) {
        cout << "Files " << fixedNormalFileString << ", " << movingNormalFileString << " not found" << "\n";
        return 1; // exit if file not found
    } 
    
    // Vector for appending points
    std::vector<Eigen::Matrix<long double, 3, 1>,Eigen::aligned_allocator<Eigen::Matrix<long double, 3, 1>>> pointVector;
    
    // read sensedFile into ptcldMoving
    while (!sensedFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        sensedFile.getline(buf, MAX_CHARS_PER_LINE);
        // store line in a vector
        if (sensedFile.eof())
            break;
        istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        //cout << "Temp is: " << setprecision(15) << temp << endl;
        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(movingPointFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingPointFileString + ": Input data doesn't match dimension (too many per line)");
        
        // Add temp to list of vectors  
        pointVector.push_back(temp);
    }
    PointCloud ptcldMoving(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldMoving.col(i) = pointVector[i];
    
    pointVector.clear();

    // read CADFile into ptcldFixed
    while (!CADFile.eof()) {
                // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        CADFile.getline(buf, MAX_CHARS_PER_LINE);
        if (CADFile.eof())
            break;
        // store line in a vector

        std::istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        if(iss.fail()) 
            call_error(fixedPointFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingPointFileString + ": Input data doesn't match dimension (too many per line)");
        // Add temp to list of vectors
        pointVector.push_back(temp);
    }

    PointCloud ptcldFixed(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldFixed.col(i) = pointVector[i];
    
    pointVector.clear();

    // Read sensedNormalFIle into normalMoving
        while (!sensedNormalFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        sensedNormalFile.getline(buf, MAX_CHARS_PER_LINE);
        if (sensedNormalFile.eof())
            break;
        // store line in a vector
        istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        //cout << "Temp is: " << setprecision(15) << temp << endl;
        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(movingNormalFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingNormalFileString + ": Input data doesn't match dimension (too many per line)");
        
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
        if (CADNormalFile.eof())
            break;
        // store line in a vector
        istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        //cout << "Temp is: " << setprecision(15) << temp << endl;
        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(fixedNormalFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(fixedNormalFileString + ": Input data doesn't match dimension (too many per line)");
        
        // Add temp to list of vectors  
        pointVector.push_back(temp);
    }

    PointCloud normalFixed(3, pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        normalFixed.col(i) = pointVector[i];
    
    pointVector.clear();

    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    sensedNormalFile.close();
    CADNormalFile.close();
        
    double timeSum = 0;
    ofstream myFile;
    myFile.open("Result_temp.txt");
    for (int i = 0; i < 1; i++) {
        clock_t begin = clock();    // For timing the performance

        // Run the registration function
        //cout << "col 86631 of normalFixed is: " << setprecision(18) << normalFixed.col(86630) << endl;
        struct RegistrationResult result = registration_est_bingham_normal(&ptcldMoving, 
                                                                            &ptcldFixed,
                                                                            &normalMoving,
                                                                            &normalFixed);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        timeSum += elapsed_secs;
    
        cout << "Xreg:" << endl << result.Xreg.transpose() << endl << endl;
        cout << "Xregsave:" << endl << result.Xregsave.transpose() << endl;
        if (i == 9)
        {
            myFile << "Xreg:" << endl << result.Xreg.transpose() << endl << endl;
            myFile << "Xregsave:" << endl << result.Xregsave.transpose() << endl << endl;
        }
    }
    cout << "Average registration runtime is: " << timeSum << " seconds." << endl << endl;
    myFile << "Average registration runtime is: " << timeSum << " seconds." << endl;
    myFile.close();

    return 0;
}

