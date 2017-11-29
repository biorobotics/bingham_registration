/*
 * File Header for registration_est_kf_rgbd.cpp
 * 
 * This file performs the final registration function for fitting the sensed
 * data points onto the CAD model points
 * 
 * Workflow diagram
 * Initialization (set up window size, kdtree, etc)
 *       |
 *       v
 * Tree search (inside the function, transform ptcldMoving and normalMoving (if provided)
                by using regParams then search)   < --------------------------------
                                                                                 |
        |
        v                                                                        |
 * Quaternion Filtering                                                          |
        |
        v                                                                        |
 * Check for convergence or max iteration reached --------------------------------
 *       |                                   no
 *       v   yes
 * Terminate loop and return current regParams and regHistory
 *
 */ 

#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>
#include "kd_tree.h"
#include "bingham_filter.h"
#include "get_changes_in_transformation_estimate.h"
#include "registration_estimation.h"
#include "type_defs.h"

//#define WINDOW_RATIO 100     // The constant for deciding window size
#define DIMENSION 3     // Dimension of data point

/* 
 *  registration_est_kf_rgbd: (for registration without normals)
 *
 *  Outputs (regParams, regHistory):
            regParams is a 6x1 vector
            regHistory is an 6xn matrix (a record of regParams value at different iteration)
    Inputs:
            ptcldMoving (3xn) is one set of point cloud data. This will represent the sensed points
            ptcldFixed (3xn) is another set of point cloud data. This will represent CAD model points 
 */

RegistrationResult registration_est_kf_rgbd(const PointCloud& ptcldMoving, 
                                            const PointCloud& ptcldFixed,
                                            double inlierRatio,
                                            int maxIterations,
                                            int windowSize,
                                            double toleranceT,
                                            double toleranceR,
                                            double uncertaintyR,
                                            KDTree tree){
    

    if (ptcldMoving.rows() != DIMENSION ||
        ptcldFixed.rows() != DIMENSION) {
        std::cerr << "Invalid point dimension";
        exit(1);
    }

    //************ Initialization **************
    RegistrationResult result;

    int sizePtcldMoving = ptcldMoving.cols();
    
    Eigen::Vector2d tolerance;
    tolerance << toleranceT , toleranceR;

    KDTree cloudTree;
    bool treeProvided = tree;
    if(!treeProvided){
        // Generate kd tree from fixed cloud if none was provided
        cloudTree = tree_from_point_cloud(ptcldFixed);
    } else{
        // Otherwise use the one provided
        cloudTree = tree;
    }

    VectorXld regParams = VectorXld::Zero(6);  //regParams: 6x1

    // regHistory.row(0) saves the initialized value. The regParams output from each
    // iteration is stored there (dimension (maxIterations + 1) x 6)
    
    MatrixXld regHistory = MatrixXld::Zero(6, maxIterations + 1); //regHistory: 6xn

    //Quaterniond Xk_quat = eul2quat(regParams.segment(3,3));
    
    Vector4ld Xk;   //Xk: 4x1
    Xk << 1, 0, 0, 0;

    Matrix4ld Mk= MatrixXld::Identity(4, 4);

    Matrix4ld Zk = MatrixXld::Zero(4, 4);

    for(int i = 1; i <= 3; i++) 
        Zk(i, i) = -1 * pow((long double)10, (long double)-uncertaintyR);
    
    long double BinghamKFSum = 0;  // for timing Bingham_kf
    
    //********** Loop starts **********
    // If not converge, transform points using regParams and repeat
    for (int i = 1; i <= std::min(maxIterations, sizePtcldMoving / windowSize); i++) {
        int iOffset = i - 1;    // Eigen is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the pftcldMoving points according to window size
        PointCloud targets(3, windowSize);
        
        for (int r = windowSize * (iOffset); r < windowSize * i; r++) {
            int rOffset = r - windowSize * (iOffset);
            for (int n = 0; n < 3; n++) 
                targets(n,rOffset) = ptcldMoving(n, r);
        }

        // kd_search takes subset of ptcldMovingNew, CAD model points, and regParams
        // from last iteration according to window size 

        KdResult searchResult = kd_search(targets, cloudTree, inlierRatio, regParams);

        PointCloud pc = searchResult.pc;    // set of all closest point
        PointCloud pr = searchResult.pr;    // set of all target points in 
                                             // corresponding order with pc
        long double res = searchResult.res;  // mean of all the distances calculated
        result.error = res;

        // Truncate the windowSize according to window size and inlier ratio
        int truncSize = trunc(windowSize * inlierRatio);

        // If truncSize odd, round down to even so pc and pr have same dimension
        int oddEntryNum = truncSize / 2;    // size of p1c/p1
        int evenEntryNum = oddEntryNum; // size of p2c/p2r

        PointCloud p1c = PointCloud(3, oddEntryNum);    // odd index points of pc
        PointCloud p2c = PointCloud(3, evenEntryNum);   // even index points of pc
        PointCloud p1r = PointCloud(3, oddEntryNum);    // odd index points of pr
        PointCloud p2r = PointCloud(3, evenEntryNum);   // even index points of pr
        
        long double Rmag= .04 + pow(res / 6, 2);  // Variable that helps calculate the noise 
        
        int p1Count = 0;
        int p2Count = 0;
        
        // Store odd entries in pc to p1c, in pr to p1r
        // Store even entries in pc to p2c, in pr to p2r
        for (int n = 1; n <= truncSize - truncSize % 2; n++) {
            int nOffset = n - 1;    // Eigen is 0-index instead of 1-index
            
            if (n % 2) {    // If odd index point
                if (p1Count >= oddEntryNum){
                    std::cerr << "Incorrect number of odd entry.";
                    exit(1);
                }
                p1c.col(p1Count) = pc.col(nOffset);
                p1r.col(p1Count) = pr.col(nOffset);
                p1Count++;
            }
            else {  // If even index point
                if (p2Count >= evenEntryNum) {
                    std::cerr << "Incorrect number of even entry.";
                    exit(1);
                }
                p2c.col(p2Count) = pc.col(nOffset);
                p2r.col(p2Count) = pr.col(nOffset);
                p2Count++;
            }
        }
        
        //  Quaternion Filtering:
        //  Takes updated Xk, Mk, Zk from last QF, updated Rmag, p1c, p1r, p2c,
        //  p2r from kdsearch
        //  Output updated Xk, Mk, Zk, and regParams for next iteration. 
        BinghamKFResult QFResult = bingham_filter(&Xk, &Mk, &Zk, Rmag, &p1c, &p1r, &p2c, &p2r); 

        Xk = QFResult.Xk;
        Mk = QFResult.Mk;
        Zk = QFResult.Zk;

        // Store current regParams in regHistory
        regHistory.col(i) = QFResult.regParams;    // No offset applied because 
                                             // regHistory(0) is saved for initial value   
        regParams = QFResult.regParams;

        //  Check convergence:
        //  Takes in updated regParams and previous regParams
        //  Return dR, dT
        DeltaTransform convergenceResult = get_changes_in_transformation_estimate(QFResult.regParams,
                                                                                  regHistory.col(i-1));
        if (convergenceResult.dT <= tolerance(0) && 
            convergenceResult.dR <= tolerance(1)) {
            std::cout << "CONVERGED" << std::endl;
            break;  // Break out of loop if convergence met
        }
    }
    if(!treeProvided){ free_tree(cloudTree); }
    result.regParams = regParams;
    result.regHistory = regHistory;
    return result;
}