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
 * Tree search (inside the function, transform ptcldMoving by using Xreg then <-
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
#include "registration_est_bingham_kf_rgbd.h"
#include "ros/ros.h"

using namespace std;
using namespace Eigen;

#define WINDOW_RATIO 100     // The constant for deciding windowsize
#define DIMENSION 3     // Dimension of data point
#define INLIER_RATIO 1
#define MAX_ITERATIONS 100
#define MIN_ITERATIONS 20

/* 
 *  registration_est_kf_rgbd: (workflow explained in the file header)
 *
 *  Outputs (Xreg, Xregsave):
            Xreg is a 1x6 vector
            Xregsave is an nx6 matrix (a record of Xreg value at different iteration)
    Inputs:
            ptcldMoving is one set of point cloud data. This will represent the sensed points
            sizePtcldMoving is size of ptcldMoving data
            ptcldFixed is another set of point cloud data. This will represent CAD model points 
            sizePtcldFixed is size of ptcldFixed data 
 */

struct RegistrationResult registration_est_bingham_kf_rgbd(PointCloud ptcldMoving, PointCloud ptcldFixed) {
    

    if (ptcldMoving.rows() != DIMENSION || ptcldFixed.rows() != DIMENSION)
        call_error("Invalid point dimension");
    
    //************ Initialization **************
    
    struct RegistrationResult result;
    int sizePtcldMoving = ptcldMoving.cols();
    int sizePtcldFixed = ptcldFixed.cols();
    int treeSize = sizePtcldFixed;
    KDTree cloudTree = NULL;    // Generated kd tree from ptcldFixed
    Vector2d tolerance;

    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < treeSize; i++) 
        insert(ptcldFixed.col(i), &cloudTree);

    int windowsize = 20;
    //int windowsize = sizePtcldMoving / WINDOW_RATIO;

    tolerance << .0001 , .009;

    ArrayXld Xreg = VectorXld::Zero(6);  //Xreg: 1x6

    // Xregsave.row(0) saves the initialized value. The Xreg output from each
    // iteration is stored there (dimensionL (MAX_ITERATIONS + 1) x 6)
    
    MatrixXld Xregsave = MatrixXld::Zero(6,MAX_ITERATIONS + 1);

    //Quaterniond Xk_quat = eul2quat(Xreg.segment(3,3));
    
    // Convert Xk to Vector4ld for later computation
    Vector4ld Xk;
    Xk << 1, 0, 0, 0;
    /*Xk(1) = Xk_quat.x() / Xk_quat. w();
    Xk(2) = Xk_quat.y() / Xk_quat. w();
    Xk(3) = Xk_quat.z() / Xk_quat. w();
    Xk(0) = 1;*/

    Matrix4ld Mk= MatrixXld::Identity(4, 4);

    Matrix4ld Zk = MatrixXld::Zero(4, 4);

    for(int i = 1; i <= 3; i++) 
        Zk(i, i) = -1 * pow((long double)10, (long double)-100);
    

    PointCloud ptcldMovingNew = ptcldMoving;

    VectorXld Xregprev = VectorXld::Zero(6);
    
    long double BinghamKFSum = 0;  // for timing Bingham_kf


    //********** Loop starts **********
    // If not converge, transform points using Xreg and repeat
    for (int i = 1; i <= min(MAX_ITERATIONS, sizePtcldMoving / windowsize); i++) {
        int iOffset = i - 1;    // Eigen is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the pftcldMoving points.
        PointCloud targets(3, windowsize);
        
        for (int r = windowsize * (iOffset); r < windowsize * i; r++) {
            int rOffset = r - windowsize * (iOffset);
            for (int n = 0; n < 3; n++) 
                targets(n,rOffset) = ptcldMovingNew(n, r);
        }

        // kd_search takes subset of ptcldMovingNew, CAD model points, and Xreg
        // from last iteration 
        struct KdResult *searchResult = kd_search(&targets, windowsize, cloudTree,
                                       sizePtcldFixed, INLIER_RATIO, &Xreg);

        PointCloud pc = searchResult->pc;    // set of all closest point
        PointCloud pr = searchResult->pr;    // set of all target points in corresponding order with pc
        long double res = searchResult->res;  // mean of all the distances calculated

        /*cout << "Start reporting kd_search result: " << endl << endl;
        cout << "pc is: " << setprecision(18) << pc << endl;
        cout << "pr is: " << setprecision(18) << pr << endl;
        cout << "res is: " << setprecision(18) << res << endl;*/
        // Truncate the windowsize according to INLIER_RATIO
        int truncSize = trunc(windowsize * INLIER_RATIO);

        // If truncSize odd, round down to even so pc and pr have same dimension
        int oddEntryNum = truncSize / 2;    // size of p1c/p1
        int evenEntryNum = oddEntryNum; // size of p2c/p2r

        PointCloud p1c = PointCloud(3, oddEntryNum);    // odd index points of pc
        PointCloud p2c = PointCloud(3, evenEntryNum);   // even index points of pc
        PointCloud p1r = PointCloud(3, oddEntryNum);    // odd index points of pr
        PointCloud p2r = PointCloud(3, evenEntryNum);   // even index points of pr
        
        long double Rmag= .01 + pow(res / 6, 2);  // Variable that helps calculate the noise 
        
        //cout << "Rmag in new is: " << setprecision(18) << Rmag << endl;
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
        //  Takes updated Xk, Mk, Zk from last QF, updated Rmag, p1c, p1r, p2c,
        //  p2r from kdsearch
        //  Output updated Xk, Mk, Zk, and Xreg for next iteration. 

        clock_t start_2 = clock();
        struct BinghamKFResult QFResult = bingham_kf(Xk, Mk, Zk, Rmag, p1c, p1r, p2c, p2r); 
        clock_t end_2 = clock(); 
        long double elapsed_secs = (long double)(end_2 - start_2) / CLOCKS_PER_SEC; 
        BinghamKFSum += elapsed_secs;

        Xk = QFResult.Xk;
        Mk = QFResult.Mk;
        Zk = QFResult.Zk;
        // Store curretn Xreg in Xregsave
        Xregsave.col(i) = QFResult.Xreg;    // No offset applied because 
                                            // Xregsave(0) is saved for initial value   
        
        Xreg = QFResult.Xreg;
        result.Xreg = QFResult.Xreg;
        result.Xregsave = Xregsave;
        
        //  Check convergence:
        //  Takes in updated Xreg and previous Xreg
        //  Return dR, dT
        
        struct DeltaTransform convergenceResult = get_changes_in_transformation_estimate(
                                                  QFResult.Xreg, Xregsave.col(i-1));

        if (i >= MIN_ITERATIONS && convergenceResult.dT <= tolerance(0) && 
            convergenceResult.dR <= tolerance(1)) {
            cout << "CONVERGED" << endl;
            break;  // Break out of loop if convergence met
        }

    }

    free_tree(cloudTree);

    return result;
}