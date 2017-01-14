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

// main:
// Read sensed data and CAD data from .txt files and use them as input for
// running the registration function

int main(int argc, char *argv[]) {
    // Defaults
    string movingFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/ptcld_moving_5.txt";
    string fixedFileString = "/home/olivia/qf_registration_ws/src/dual_quaternion_registration/data/ptcld_fixed_5.txt";
    // Replace filenames if arguments exist
    //cout << "Precision of long double is: " << numeric_limits<long double>::digits10 << endl;
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-m"){
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                // Increment 'i' so we don't get the argument as the next argv[i].
                i++;
                movingFileString = argv[i];
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "-m option requires filepath for moving pointcloud." << std::endl;
                return 1;
            }
        }
        if (arg == "-f"){
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                fixedFileString = argv[i]; 
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "-f option requires filepath for fixed pointcloud." << std::endl;
                return 1;
            }
        }
    }

    // Read .text files for data points
    ifstream sensedFile;
    ifstream CADFile;

    cout << "\nStatic point cloud: " << fixedFileString << "\nMoving point cloud: " << movingFileString << endl;

    sensedFile.open(movingFileString, ifstream::in);
    CADFile.open(fixedFileString, ifstream::in);
    
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedFileString << ", " << movingFileString << " not found" << "\n";
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
        istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        //cout << "Temp is: " << setprecision(15) << temp << endl;
        // Make sure all three were read in correctly
        if(iss.fail())
            call_error(movingFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingFileString + ": Input data doesn't match dimension (too many per line)");
        
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
        // store line in a vector
        std::istringstream iss(buf);
        Vector3ld temp;
        iss >> temp(0) >> temp(1) >> temp(2);
        if(iss.fail()) 
            call_error(fixedFileString + ": Input data doesn't match dimension (too few per line)");
        
        // Make sure there are no more to read
        float eofCheck;
        iss >> eofCheck;
        if(iss.good()) 
            call_error(movingFileString + ": Input data doesn't match dimension (too many per line)");
        // Add temp to list of vectors
        pointVector.push_back(temp);
    }

    PointCloud ptcldFixed(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        ptcldFixed.col(i) = pointVector[i];
    
    pointVector.clear();
    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    
    double timeSum = 0;
    ofstream myFile;
    myFile.open("Result_temp.txt");
    for (int i = 0; i < 1; i++) {
        clock_t begin = clock();    // For timing the performance

        // Run the registration function

        struct RegistrationResult result = registration_est_bingham_kf_rgbd(ptcldMoving, 
                                                                            ptcldFixed);

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
    cout << "Average registration runtime is: " << timeSum / 10 << " seconds." << endl << endl;
    myFile << "Average registration runtime is: " << timeSum / 10 << " seconds." << endl;
    myFile.close();

    return 0;
}

