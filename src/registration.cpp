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
#include <registration_est_combined_rgbd.h>

#include <registration.h>

using namespace std;

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

const int NUM_OF_RUNS = 1; // # of runs to run the registration for average performance

// Should at least provide the two ptcld datasets
long double* combined_register(int registerOption, char const * providedTable, char const * movingData, char const * fixedData, char const * saveDest, 
                         double inlierRatio, int maxIterations, int windowSize,
                         double toleranceT, double toleranceR, double uncertaintyR) {

    long double* returnArray = new long double[7];

    char const * movingPointsString = movingData;
    char const * fixedPointsString = fixedData;
	char const * saveDestString = saveDest;
    char const * tableString = providedTable;
    char const * movingNormalsString = "";
    char const * fixedNormalsString = "";

    // Read .text files for data points
    ifstream sensedFile;    // stream for ptcld moving
    ifstream CADFile;       // stream for ptcld fixed
    ifstream tableFile;     // stream for table
    ifstream sensedNormalFile;  // stream for normal moving
    ifstream CADNormalFile;  // stream for normal fixed

    int useNormal = 0;
    int useTable = 0;
    sensedFile.open(movingPointsString, ifstream::in);
    CADFile.open(fixedPointsString, ifstream::in);
    tableFile.open(tableString, ifstream::in);
    // Check if files are valid
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "Files " << fixedPointsString << ", " << movingPointsString << " not found" 
        << "\n";
        return returnArray; // exit if file not found
    } 

    if (!tableFile.good()) {
        cout << "Table " << tableString << " not found. Will generate new table for this registration" 
        << "\n";
    }
    else {
        cout << "Table " << tableString << " provided for this registration" 
        << "\n";
        useTable = 1;
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

    PointCloud *tableProvided = (PointCloud*)malloc(sizeof(PointCloud));
    if (useTable) {
        // read tableFile into tableProvided
        while (!tableFile.eof()) {
            // read an entire line into memory
            char buf[MAX_CHARS_PER_LINE];
            tableFile.getline(buf, MAX_CHARS_PER_LINE);
            // This extra detection is necessary for preventing bug
            if (tableFile.eof())
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
     

    PointCloud tableProvided_temp(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++)
        tableProvided_temp.col(i) = pointVector[i];

    *tableProvided = tableProvided_temp;
    pointVector.clear();    // Clear pointVector for the following read
    }

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
    PointCloud normalMoving;
    PointCloud normalFixed;
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

        normalMoving = PointCloud(3, pointVector.size());

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

        normalFixed = PointCloud(3, pointVector.size());

        for(int i=0; i<pointVector.size(); i++)
            normalFixed.col(i) = pointVector[i];
        
        pointVector.clear();
        sensedNormalFile.close();
        CADNormalFile.close();
    }

    /*** The following registration is performed when normals are not used ***/
    // Force the memory to be freed
    pointVector.shrink_to_fit();
    
    sensedFile.close();
    CADFile.close();
    
    double timeSum = 0;
    struct RegistrationResult *result;
	for (int i = 0; i < NUM_OF_RUNS; i++) {

		if (registerOption == 0) {
			// Run the registration function without normals
			printf("Bingham Registration starts with kdtree structure\n");
			result = registration_est_bingham_kf_rgbd(&ptcldMoving, &ptcldFixed,
				inlierRatio, maxIterations, windowSize,
				toleranceT, toleranceR, uncertaintyR, registerOption, tableProvided);
            printf("Bingham Registration finished\n");

		}
		else if (registerOption == 1) {
			printf("Bingham Registration starts with new table structure\n");
            result = registration_est_bingham_kf_rgbd(&ptcldMoving, &ptcldFixed,
                inlierRatio, maxIterations, windowSize,
                toleranceT, toleranceR, uncertaintyR, registerOption, tableProvided);
            printf("Bingham Registration finished\n");
		}
        else if (registerOption == 2) {
            printf("Bingham Registration starts with provided table structure\n");
            result = registration_est_bingham_kf_rgbd(&ptRcldMoving, &ptcldFixed,
                inlierRatio, maxIterations, windowSize,
                toleranceT, toleranceR, uncertaintyR, registerOption, tableProvided);
            printf("Bingham Registration finished\n");
        }
		else {
			call_error(
				"Invalid register option");
		}
        timeSum += result->registrationTime;
    }
    // Save results to txt
    ofstream myFile;
    myFile.open(saveDest, ios_base::app);
    Eigen::IOFormat XregFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "", "", "", "");
    myFile << result->Xreg.transpose().format(XregFormat) << ", " << timeSum / NUM_OF_RUNS  << ", " << result->error << endl;
    myFile.close();

	Eigen::Map<Eigen::Matrix<long double, 6, 1 > > (returnArray,6,1) = result->Xreg;
    returnArray[6] = result->error;
    return returnArray;
}

int main(int argc, char *argv[]) {

    string movingPointsString;
    string fixedPointsString;
	string saveDestString;
    string movingNormalsString;
    string fixedNormalsString;
    string providedTableString = "";
    int useNormal = 0;
    int methodNum = 0;
    int normalFileProvided = 0;
    int pointFileProvided = 0;

    // Replace filenames by the arguments
    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        // Parse registration method
        if (arg == "-mt") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                methodNum = stoi(argv[i]);
            } else { // If there was no argument to the destination option.
                std::cerr << "-mt option requires method of registration: 0 for kdtree search, 1 for new table search, 2 for provided table search." 
                << std::endl;
                return 1;
            }
        }
        // Parse table from .txt
        else if (arg == "-tb") {
            if (i + 1 < argc) { // Make sure we aren't at the end of argv!
                i++; // Increment 'i' so we don't get the argument as the next argv[i].
                providedTableString = argv[i];
            } else { // If there was no argument to the destination option.
                std::cerr << "-tb option requires filepath for table." 
                << std::endl;
                return 1;
            }
        }
        // Parse ptcld_moving from .txt
        else if (arg == "-pm") {
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
		// Parse save destination
		else if (arg == "-sd") {
			if (i + 1 < argc) { // Make sure we aren't at the end of argv!
				i++; // Increment 'i' so we don't get the argument as the next argv[i].
				saveDestString = argv[i];
			}
			else { // If there was no argument to the destination option.
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
            std::cerr << "Invalid argument. Usage: -mt .. -pm .. -pf .. -sd .. (optional) (-tb .. -nm .. -nf ..)" 
             << std::endl;
            return 1;
        }
    }

    // Check if enough arguments were entered
    if (pointFileProvided != 2 || (useNormal && normalFileProvided != 2)) {
        std::cerr << "Not enough argument. Usage: -pm .. -pf .. (optional) (-nm .. -nf ..)" 
        << std::endl;
        return 1;
    }

    if (methodNum == 2 && providedTableString == "") {
        std::cerr << "Invalid table provided for the registration." 
        << std::endl;
        return 1;
    }
	int inlierRatio = 1; int maxIterations = 100; int windowSize = 20;
	double toleranceT = 0.0001; double toleranceR = 0.009; double uncertainty = 300.0;
	// By default 1 is bingham_register
	combined_register(methodNum, providedTableString.c_str(), movingPointsString.c_str(), fixedPointsString.c_str(), saveDestString.c_str(), inlierRatio, maxIterations, windowSize, toleranceT, toleranceR, uncertainty);
}
