/*
 * File Header:
 * 
 * This file performs the final registration function for fitting the sensed
 * data points onto the CAD model points
 * 
 * Workflow diagram:
 * Initialization (set up window size, kdtree, etc)
 *       |
 *       v
 * Tree search (inside the function, transform ptcld_moving by using Xreg then <-
                search)                                                         |
 * Quaternion filter                                                            |
 * Check for convergence   -----------------------------------------------------
 *       |                                   no
 *       v   yes
 * Terminate loop and return current Xreg and Xregsave
 *
 */ 

#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "KDTree.h"
#include "get_changes_in_transformation_estimate.h"
#include "qr_kf.h"
#include "compute_transformed_points.h"

using namespace std;
using namespace Eigen;

#define WINDOW_RATIO 70     // The constant for deciding windowsize
#define DIMENSION 3     // Dimension of data point
#define INLIER_RATIO 0.9
#define MAX_ITERATIONS 70

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

// For the return type
struct tuple2{
    VectorXd Xreg;
    VectorXd Xregsave;
};

/* 
 *  registration_est_kf_rgbd: (workflow explained in the file header)
 *
 *  Outputs (Xreg, Xregsave):
            Xreg is a 1x6 vector
            Xregsave is an nx6 matrix (a record of Xreg value at different iteration)
    Inputs:
            ptcldMoving is one set of point cloud data. This will represent the sensed points
            sizePtcld_moving is size of ptcldMoving data
            ptcldFixed is another set of point cloud data. This will represent CAD model points 
            sizePtcldFixed is size of ptcldFixed data 
 */

struct tuple2 registration_est_kf_rgbd(PointCloud ptcldMoving, PointCloud ptcldFixed) {

    if (ptcldMoving.rows() != DIMENSION || ptcldFixed.rows() != DIMENSION)
        call_error("Invalid point dimension");
    
    //************ Initialization **************
    
    struct tuple2 result;
    int sizePtcldMoving = ptcldMoving.cols();
    int sizePtcldFixed = ptcldFixed.cols();
    int treeSize = sizePtcldFixed;
    KDTree cloudTree = NULL;    // Generated kd tree from ptcldFixed
    Vector2d tolerance;
    
    clock_t begin = clock();    // For timing the tree construction

    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < treeSize; i++) {
        cloudTree = insert(ptcldFixed.col(i), cloudTree);
    }
    
    clock_t end = clock();  
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    cout << "Tree construction time: " << elapsed_secs << " seconds." << endl;

    int windowsize = sizePtcldMoving / WINDOW_RATIO;

    tolerance << 0.0001 , .009; // Tolerance = [.0001 .009]

    VectorXd Xreg = VectorXd::Zero(6);  //Xreg: 1x6

    // Xregsave.row(0) saves the initialized value. The Xreg output from each
    // iteration is stored there (dimensionL (MAX_ITERATIONS + 1) x 6)
    
    MatrixXd Xregsave = MatrixXd::Zero(6,MAX_ITERATIONS + 1);
    Vector4d Xk = Vector4d::Zero(); //Xk = [1; 0; 0; 0]
    Xk(0) = 1;
    Matrix4d Pk = Matrix4d::Identity(); // Pk = [0    0    0    0
                                        //        0    10^7 0    0
                                        //        0    0    10^7 0
                                        //        0    0    0    10^7]
    Pk *= pow(10.0, 7.0);
    Pk(0, 0) = 0;

    PointCloud ptcldMovingNew = ptcldMoving;

    VectorXd Xregprev = VectorXd::Zero(6);

    //********** Loop starts **********

    // If not converge, transform points using Xreg and repeat
    for (int i = 1; i <= min(MAX_ITERATIONS, sizePtcldMoving / windowsize); i++) {
        
        int iOffset = i - 1;    // Armadillo is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the ptcldMoving points.
        MatrixXd targets(windowsize, 3);

        for (int r = windowsize * (iOffset); r < windowsize * i; r++) {
            int rOffset = r - windowsize * (iOffset);
            
            for (int n = 0; n < 3; n++) 
                targets(rOffset, n) = ptcldMovingNew(r, n);
        }

        // kd_search takes subset of ptcldMovingNew, CAD model points, and Xreg
        // from last iteration 
        struct triple1 searchResult = kd_search(targets, windowsize, cloudTree,
                                      sizePtcldFixed, INLIER_RATIO, Xreg);

        PointCloud pc = searchResult.pc;    // set of all closest point
        PointCloud pr = searchResult.pr;    // set of all target points in corresponding order with pc
        double res = searchResult.res;  // mean of all the distances calculated

        // Truncate the windowsize according to INLIER_RATIO
        int truncSize = trunc(windowsize * INLIER_RATIO);

        // If truncSize odd, round down to even so pc and pr have same dimension
        int oddEntryNum = truncSize / 2;    // size of p1c/p1
        int evenEntryNum = oddEntryNum; // size of p2c/p2r
        PointCloud p1c = PointCloud(oddEntryNum, 3);    // odd index points of pc
        PointCloud p2c = PointCloud(evenEntryNum, 3);       // even index points of pc
        PointCloud p1r = PointCloud(oddEntryNum, 3);    // odd index points of pr
        PointCloud p2r = PointCloud(evenEntryNum, 3);       // even index points of pr
        
        double Rmag=.04 + 4 * pow(res / 6, 2);  // Variable that helps calculate the noise 
        
        int p1Count = 0;
        int p2Count = 0;

        // Store odd entries in pc to p1c, pr to p1r
        // Store even entries in pc to p2c, pr to p2r
        for (int n = 1; n <= truncSize - truncSize % 2; n++) {
            int nOffset = n - 1;    // Armadillo is 0-index instead of 1-index
            if (n % 2) {    // If odd index point
                if (p1Count >= oddEntryNum) {
                    call_error("Incorrect number of odd entry.");
                }

                p1c.col(p1Count) = pc.col(nOffset);
                p1r.col(p1Count) = pr.col(nOffset);
                p1Count++;
            }
            else {  // If even index point
                if (p2Count >= evenEntryNum)
                    call_error("Incorrect number of even entry.");
                p2c.col(p2Count) = pc.col(nOffset);
                p2r.col(p2Count) = pr.col(nOffset);

                p2Count++;
            }
        }

        //  Quaternion Filtering:
        //  Takes updated Xk, Pk from last QF, updated Rmag, p1c, p1r, p2c,
        //  p2r from kdsearch
        //  Output updated Xk, Pk, and Xreg for next iteration. 
        
        struct triple2 qfResult = qr_kf(Xk, Pk, Rmag, p1c, p1r, p2c, p2r);  
        Xk = qfResult.Xk;
        Pk = qfResult.Pk;

        // Store curretn Xreg in Xregsave
        Xregsave.col(i) = qfResult.Xreg;    // No offset applied because 
                                            // Xregsave(0) is saved for initial value   

        Xreg = qfResult.Xreg;
        result.Xreg = qfResult.Xreg;
        result.Xregsave = Xregsave;

        //  Check convergence:
        //  Takes in updated Xreg and previous Xreg
        //  Return dR, dT
 
        struct tuple1 convergenceResult = get_changes_in_transformation_estimate(
                                          qfResult.Xreg, Xregsave.col(i-1));

        if (convergenceResult.dT <= tolerance(0) && convergenceResult.dR <= tolerance(0))
        {
            break;  // Break out of loop if convergence met
        }
    }

    return result;
}
// main:
// Read sensed data and CAD data from .txt files and use them as input for
// running the registration function

int main() {
    
    // Read .text files for data points
    ifstream sensedFile;
    ifstream CADFile;

    sensedFile.open("data/data_bunny.txt", ifstream::in);
    CADFile.open("data/model_bunny.txt", ifstream::in);
    
    if (!sensedFile.good() || !CADFile.good()) {
        cout << "File not found" << "\n";
        return 1; // exit if file not found
    }
    
    // Vector for appending points
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> pointVector;
    
    // read sensedFile into ptcldMoving
    while (!sensedFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        sensedFile.getline(buf, MAX_CHARS_PER_LINE);
    
        // parse the line into blank-delimited tokens
        int n = 0; // a for-loop index
    
        // array to store memory addresses of the tokens in buf
        const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0
        // parse the line
        token[0] = strtok(buf, DELIMITER); // first token
        if (token[0]) {// zero if line is blank
            for (n = 1; n < MAX_TOKENS_PER_LINE; n++) {
                token[n] = strtok(0, DELIMITER); // subsequent tokens
                if (!token[n]) 
                    break; // no more tokens
            }
        }

        // Now n is set to # of tokens in each line
        if (n != DIMENSION)
            call_error("Input data doesn't match dimension1");
        
        // process the tokens
        Vector3d temp;
        for (int i = 0; i < n; i++)     // n = #of tokens
            temp(i) = atof(token[i]);
        
        pointVector.push_back(temp);
    }

    PointCloud ptcldMoving(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++){
        ptcldMoving.col(i) = pointVector[i];
    }

    pointVector.clear();

    // read CADFile into ptcldFixed
    while (!CADFile.eof()) {
        // read an entire line into memory
        char buf[MAX_CHARS_PER_LINE];
        CADFile.getline(buf, MAX_CHARS_PER_LINE);
    
        // parse the line into blank-delimited tokens
        int n = 0; // a for-loop index
    
        // array to store memory addresses of the tokens in buf
        const char* token[MAX_TOKENS_PER_LINE] = {}; // initialize to 0
    
        // parse the line
        token[0] = strtok(buf, DELIMITER); // first token
        if (token[0]) { // zero if line is blank
            for (n = 1; n < MAX_TOKENS_PER_LINE; n++) {
                token[n] = strtok(0, DELIMITER); // subsequent tokens
                if (!token[n]) break; // no more tokens
            }
        }

        // Now n is set to # of tokens in each line
        if (n != DIMENSION)
            call_error("Input data doesn't match dimension");
        
        // process the tokens
        Vector3d temp(n);
        for (int i = 0; i < n; i++) // n = #of tokens
            temp(i) = atof(token[i]);
        
        pointVector.push_back(temp);
    }

    PointCloud ptcldFixed(3,pointVector.size());

    for(int i=0; i<pointVector.size(); i++){
        ptcldFixed.col(i) = pointVector[i];
    }
    
    sensedFile.close();
    CADFile.close();

    clock_t begin = clock();    // For timing the performance

    // Run the registration function
    struct tuple2 result = registration_est_kf_rgbd(ptcldMoving, ptcldFixed);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  cout << "Registration runtime is: " << elapsed_secs << " seconds." << endl;
    cout << "Xreg: " << result.Xreg << endl;
    cout << "Xregsave: ";
    //result.Xregsave.print();
    
    return 0;
}

