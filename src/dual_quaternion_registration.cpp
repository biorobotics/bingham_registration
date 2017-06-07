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
#include <vector>
#include <stdexcept>
#include <registration_tools.h>
#include <registration_est_kf_rgbd.h>
#include <dual_quaternion_registration.h>

using namespace std;

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

const int NUM_OF_RUNS = 1; // # of runs to run the registration for average performance

void free_result(long double *ptr)
{
    free(ptr);
}

typedef std::vector<Eigen::Matrix<long double, 3, 1>,
                    Eigen::aligned_allocator<Eigen::Matrix<long double, 3, 1>>
                    > PointVector;

PointCloud fillPointCloud(char const * filePath){
    // Vector for appending points
    PointVector pointVector;
    ifstream openedFile;
    openedFile.open(filePath, ifstream::in);
    if (!openedFile.good()) {
        std::string err = "File ";
        err.append(filePath);
        err.append(" not found \n");
        call_error(err);
    } 

    // read openedFile into ptcldMoving
    while (!openedFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        openedFile.getline(buf, MAX_CHARS_PER_LINE);
        // This extra detection is necessary for preventing bug
        if (openedFile.eof())
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


    PointCloud ptcld(3,pointVector.size());
    for(int i=0; i<pointVector.size(); i++)
        ptcld.col(i) = pointVector[i];
    
    openedFile.close();
    // Force vector memory to free
    pointVector.clear();
    pointVector.shrink_to_fit();

    return ptcld;
}

// Should at least provide the two ptcld datasets
long double* qf_register(char const * movingData, char const * fixedData, 
                         double inlierRatio, int maxIterations, int windowSize,
                         double toleranceT, double toleranceR, double uncertaintyR) {

    long double* returnArray = new long double[7];

    char const * movingPointsString = movingData;
    char const * fixedPointsString = fixedData;
    char const * movingNormalsString = "";
    char const * fixedNormalsString = "";

    int useNormal = 0;
    
    PointCloud ptcldMoving = fillPointCloud(movingPointsString);
    PointCloud ptcldFixed = fillPointCloud(fixedPointsString);

    PointCloud normalMoving;
    PointCloud normalFixed;

    // Open the normal-related files if normal option is used
    if (useNormal)
    {
        normalMoving = fillPointCloud(movingNormalsString);
        normalFixed = fillPointCloud(fixedNormalsString);
    }
        
    double timeSum = 0;
    RegistrationResult result;
    for (int i = 0; i < NUM_OF_RUNS; i++) {
        clock_t begin = clock();    // For timing the performance

        if (useNormal) {
            // Run the registration function with normals
            result = registration_est_bingham_normal(&ptcldMoving, &ptcldFixed, 
                                                     &normalMoving, &normalFixed,
                                                     inlierRatio, maxIterations,
                                                     windowSize, toleranceT,
                                                     toleranceR, uncertaintyR);
        }
        else {
            // Run the registration function without normals
            result = registration_est_bingham_kf_rgbd(&ptcldMoving, &ptcldFixed,
                                                      inlierRatio, maxIterations, windowSize,
                                                      toleranceT, toleranceR, uncertaintyR);
        }

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        timeSum += elapsed_secs;
    }
    // Save results to txt
    ofstream myFile;
    myFile.open("result.txt");
    myFile << "Xreg:" << endl << result.Xreg.transpose() << endl << endl;
    myFile << "Xregsave:" << endl << result.Xregsave.transpose() 
                          << endl << endl;
    myFile << "Average registration runtime is: " << timeSum / NUM_OF_RUNS 
           << " seconds." << endl;
    myFile << "Average Registration error:" << result.error << endl;
    myFile.close();

	Eigen::Map<Eigen::Matrix<long double, 6, 1 > > (returnArray,6,1) = result.Xreg;
    returnArray[6] = result.error;

    return returnArray;
}

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
	int inlierRatio = 1; int maxIterations = 100; int windowSize = 20;
	double toleranceT = 0.001; double toleranceR = 0.009; double uncertainty = 300;
	long double *result = qf_register(movingPointsString.c_str(), fixedPointsString.c_str(), inlierRatio, maxIterations, windowSize, toleranceT, toleranceR, uncertainty);
    free(result);
    cout << "\nTEST\n\n";
}
