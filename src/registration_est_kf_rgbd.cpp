//#include <windows.h>
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
#include "registration_est_kf_rgbd.h"

using namespace std;
using namespace Eigen;

//#define WINDOW_RATIO 100     // The constant for deciding window size
#define DIMENSION 3     // Dimension of data point

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

RegistrationResult* registration_est_kf_rgbd(PointCloud *ptcldMoving,
											  PointCloud *ptcldFixed,
											  double inlierRatio,
											  int maxIterations,
											  int minIterations,
											  int windowSize,
											  double toleranceT,
											  double toleranceR,
											  double uncertaintyR) {
    if ((*ptcldMoving).rows() != DIMENSION || (*ptcldFixed).rows() != DIMENSION)
        call_error("Invalid point dimension");
    
    //************ Initialization **************
   
    int sizePtcldMoving = (*ptcldMoving).cols();
    int sizePtcldFixed = (*ptcldFixed).cols();

    KDTree cloudTree = NULL;    // Generated kd tree from ptcldFixed
    Vector2d tolerance;
	tolerance << toleranceT, toleranceR;
    
    //clock_t begin = clock();    // For timing the tree construction

    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < sizePtcldFixed; i++) {
        insert((*ptcldFixed).col(i), &cloudTree);
    }
    
    //clock_t end = clock();  
    //double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    //cout << "Tree construction time: " << elapsed_secs << " seconds." << endl;

    //int windowsize = sizePtcldMoving / WINDOW_RATIO;

    //tolerance << 0.0001 , .009; // Tolerance = [.0001 .009]

    VectorXld Xreg = VectorXld::Zero(6);  //Xreg: 6x1
	double returnedError = 0;

    // Xregsave.row(0) saves the initialized value. The Xreg output from each
    // iteration is stored there (dimensionL (MAX_ITERATIONS + 1) x 6)
    
    MatrixXld Xregsave = MatrixXld::Zero(6, maxIterations + 1);
	Vector4ld Xk;
	Xk << 1, 0, 0, 0;
    Matrix4ld Pk = Matrix4ld::Identity(); // Pk = [0    0    0    0
                                        //        0    10^7 0    0
                                        //        0    0    10^7 0
                                        //        0    0    0    10^7]
    Pk *= pow(10.0, 7.0);
    Pk(0, 0) = 0;

    VectorXld Xregprev = VectorXld::Zero(6);
    
	struct RegistrationResult *result = (struct RegistrationResult*)
										calloc(1, sizeof(struct RegistrationResult));
    //********** Loop starts **********
    // If not converge, transform points using Xreg and repeat
    for (int i = 1; i <= min(maxIterations, sizePtcldMoving / windowSize); i++) {
        int iOffset = i - 1;    // Eigen is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the ptcldMoving points.
        PointCloud targets(3, windowSize);
        
        for (int r = windowSize * (iOffset); r < windowSize * i; r++) {
            int rOffset = r - windowSize * (iOffset);
            for (int n = 0; n < 3; n++) 
                targets(n,rOffset) = (*ptcldMoving)(n, r);
            
        }

        // kd_search takes subset of ptcldMovingNew, CAD model points, and Xreg
        // from last iteration 
        
        struct KdResult *searchResult = kd_search(&targets, cloudTree,
										inlierRatio, &Xreg);

        PointCloud pc = searchResult->pc;    // set of all closest point
        PointCloud pr = searchResult->pr;    // set of all target points in corresponding order with pc
        long double res = searchResult->res;  // mean of all the distances calculated
		returnedError = res;

        // Truncate the windowsize according to INLIER_RATIO
        int truncSize = trunc(windowSize * inlierRatio);

        // If truncSize odd, round down to even so pc and pr have same dimension
        int oddEntryNum = truncSize / 2;    // size of p1c/p1
        int evenEntryNum = oddEntryNum; // size of p2c/p2r

        PointCloud p1c = PointCloud(3, oddEntryNum);    // odd index points of pc
        PointCloud p2c = PointCloud(3, evenEntryNum);       // even index points of pc
        PointCloud p1r = PointCloud(3, oddEntryNum);    // odd index points of pr
        PointCloud p2r = PointCloud(3, evenEntryNum);       // even index points of pr
        
        long double Rmag= .04 + pow(res / 6, 2);  // Variable that helps calculate the noise 
        
        int p1Count = 0;
        int p2Count = 0;
        
        // Store odd entries in pc to p1c, pr to p1r
        // Store even entries in pc to p2c, pr to p2r
        for (int n = 1; n <= truncSize - truncSize % 2; n++) {
            int nOffset = n - 1;    // Eigen is 0-index instead of 1-index
            if (n % 2) {    // If odd index point
                if (p1Count >= oddEntryNum) 
                    call_error("Incorrect number of odd entry.");
                

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
        struct QrKfResult *qfResult = qr_kf(&Xk, &Pk, Rmag, &p1c, &p1r, &p2c, &p2r); 
        
        Xk = qfResult->Xk;
        Pk = qfResult->Pk;
        // Store curretn Xreg in Xregsave
        Xregsave.col(i) = qfResult->Xreg;    //bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb  No offset applied because 
                                            // Xregsave(0) is saved for initial value   
        
		Xreg = qfResult->Xreg;

        
        //  Check convergence:
        //  Takes in updated Xreg and previous Xreg
        //  Return dR, dT
        
        struct DeltaTransform *convergenceResult = get_changes_in_transformation_estimate(
                                                  qfResult->Xreg, Xregsave.col(i-1));

        if (i >= minIterations && convergenceResult->dT <= tolerance(0) && convergenceResult->dR <= tolerance(1)) {
            cout << "CONVERGED" << endl;
            break;  // Break out of loop if convergence met
        }
    }

	result->Xreg = Xreg;
	result->Xregsave = Xregsave;
	result->error = returnedError;
    return result;
}