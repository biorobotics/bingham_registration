#include <windows.h>
/*
 * File Header for registration_est_bingham_kf_rgbd.cpp
 * 
 * This file performs the final registration function for fitting the sensed
 * data points onto the CAD model points
 * 
 * Workflow diagram
 * Initialization (set up window size, kdtree, etc)
 *       |
 *       v
 * Tree search (inside the function, transform ptcldMoving and normalMoving (if provided)
                by using Xreg then search)   < --------------------------------
                                                                                 |
        |
        v                                                                        |
 * Quaternion Filtering                                                          |
        |
        v                                                                        |
 * Check for convergence or max iteration reached --------------------------------
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

using namespace std;
using namespace Eigen;

//#define WINDOW_RATIO 100     // The constant for deciding window size
#define DIMENSION 3     // Dimension of data point
#define INLIER_RATIO 1
#define MAX_ITERATIONS 100  //100
#define MIN_ITERATIONS 20  //20
#define WINDOW_SIZE 20

/* 
 *  registration_est_bingham_kf_rgbd: (for registration without normals)
 *
 *  Outputs (Xreg, Xregsave):
            Xreg is a 6x1 vector
            Xregsave is an 6xn matrix (a record of Xreg value at different iteration)
    Inputs:
            ptcldMoving (3xn) is one set of point cloud data. This will represent the sensed points
            ptcldFixed (3xn) is another set of point cloud data. This will represent CAD model points 
 */

extern "C" struct RegistrationResult* registration_est_bingham_kf_rgbd(PointCloud *ptcldMoving, 
                                                            PointCloud *ptcldFixed) {
    
    if ((*ptcldMoving).rows() != DIMENSION || (*ptcldFixed).rows() != DIMENSION)
        call_error("Invalid point dimension");
    
    //************ Initialization **************
    int sizePtcldMoving = (*ptcldMoving).cols();
    int sizePtcldFixed = (*ptcldFixed).cols();

    KDTree cloudTree = NULL;    // Generated kd tree from ptcldFixed
    Vector2d tolerance;
    tolerance << .001 , .009;

    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < sizePtcldFixed; i++) 
        insert((*ptcldFixed).col(i), &cloudTree);

    //int windowSize = sizePtcldMoving / WINDOW_RATIO;

    VectorXld Xreg = VectorXld::Zero(6);  //Xreg: 6x1

    // Xregsave.row(0) saves the initialized value. The Xreg output from each
    // iteration is stored there (dimension (MAX_ITERATIONS + 1) x 6)
    
    MatrixXld Xregsave = MatrixXld::Zero(6,MAX_ITERATIONS + 1); //Xregsave: 6xn

    //Quaterniond Xk_quat = eul2quat(Xreg.segment(3,3));
    
    Vector4ld Xk;   //Xk: 4x1
    Xk << 1, 0, 0, 0;

    Matrix4ld Mk= MatrixXld::Identity(4, 4);

    Matrix4ld Zk = MatrixXld::Zero(4, 4);

    for(int i = 1; i <= 3; i++) 
        Zk(i, i) = -1 * pow((long double)10, (long double)-300);

    VectorXld Xregprev = VectorXld::Zero(6);
    
    long double BinghamKFSum = 0;  // for timing Bingham_kf

    struct RegistrationResult *result = (struct RegistrationResult*)
                                        calloc(1,sizeof(struct RegistrationResult));

    //********** Loop starts **********
    // If not converge, transform points using Xreg and repeat
    for (int i = 1; i <= min(MAX_ITERATIONS, sizePtcldMoving / WINDOW_SIZE); i++) {
        int iOffset = i - 1;    // Eigen is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the pftcldMoving points according to window size
        PointCloud targets(3, WINDOW_SIZE);
        
        for (int r = WINDOW_SIZE * (iOffset); r < WINDOW_SIZE * i; r++) {
            int rOffset = r - WINDOW_SIZE * (iOffset);
            for (int n = 0; n < 3; n++) 
                targets(n,rOffset) = (*ptcldMoving)(n, r);
        }

        // kd_search takes subset of ptcldMovingNew, CAD model points, and Xreg
        // from last iteration according to window size 

        struct KdResult *searchResult = kd_search(&targets, cloudTree,
                                        INLIER_RATIO, &Xreg);

        PointCloud pc = searchResult->pc;    // set of all closest point
        PointCloud pr = searchResult->pr;    // set of all target points in 
                                             // corresponding order with pc
        long double res = searchResult->res;  // mean of all the distances calculated

        // Truncate the WINDOW_SIZE according to window size and inlier ratio
        int truncSize = trunc(WINDOW_SIZE * INLIER_RATIO);

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
        struct BinghamKFResult *QFResult = bingham_kf(&Xk, &Mk, &Zk, Rmag, &p1c, &p1r, &p2c, &p2r); 

        Xk = QFResult->Xk;
        Mk = QFResult->Mk;
        Zk = QFResult->Zk;

        // Store current Xreg in Xregsave
        Xregsave.col(i) = QFResult->Xreg;    // No offset applied because 
                                             // Xregsave(0) is saved for initial value   
        Xreg = QFResult->Xreg;

        result->Xreg = QFResult->Xreg;
        result->Xregsave = Xregsave;
        
        //  Check convergence:
        //  Takes in updated Xreg and previous Xreg
        //  Return dR, dT
        struct DeltaTransform *convergenceResult = get_changes_in_transformation_estimate(
                                                   QFResult->Xreg, Xregsave.col(i-1));

        if (i >= MIN_ITERATIONS && convergenceResult->dT <= tolerance(0) && 
            convergenceResult->dR <= tolerance(1)) {
            cout << "CONVERGED" << endl;
            break;  // Break out of loop if convergence met
        }
    }

    //free_tree(cloudTree);

    return result;
}

/* 
 *  registration_est_bingham_normal: (for registration normals)
 *
 *  Outputs (Xreg, Xregsave):
            Xreg is a 6x1 vector
            Xregsave is an 6xn matrix (a record of Xreg value at different iteration)
    Inputs:
            ptcldMoving (3xn) is one set of point cloud data. This will represent the sensed points
            ptcldFixed (3xn) is another set of point cloud data. This will represent CAD model points 
            normalMoving (3xn) is one set of normal data. This will represent the sensed normals
            normalFixed (3xn) is another set of point normal data. This will represent CAD model normals 
 */
extern "C" struct RegistrationResult *registration_est_bingham_normal(PointCloud *ptcldMoving, 
                                                           PointCloud *ptcldFixed,
                                                           PointCloud *normalMoving, 
                                                           PointCloud *normalFixed) {
    
    if ((*ptcldMoving).rows() != DIMENSION || (*ptcldFixed).rows() != DIMENSION)
        call_error("Invalid point dimension");
    
    //************ Initialization **************
    struct RegistrationResult *result = (struct RegistrationResult*)
                                        calloc(1, sizeof(struct RegistrationResult));
    
    int sizePtcldMoving = (*ptcldMoving).cols();
    int sizePtcldFixed = (*ptcldFixed).cols();

    KDNormalTree cloudTree = NULL;    // Generated kd tree from ptcldFixed
    
    Vector2d tolerance;
    tolerance << .001 , .009;

    // Construct the kdtree from ptcldFixed
    for (int i = 0; i < sizePtcldFixed; i++) 
        insert_normal((*ptcldFixed).col(i), i, &cloudTree);

    //int WINDOW_SIZE = sizePtcldMoving / WINDOW_RATIO;
    VectorXld Xreg = VectorXld::Zero(6);  //Xreg: 6x1

    // Xregsave.row(0) saves the initialized value. The Xreg output from each
    // iteration is stored there (dimensionL (MAX_ITERATIONS + 1) x 6)
    MatrixXld Xregsave = MatrixXld::Zero(6, MAX_ITERATIONS + 1); //Xregsave: 6xn
    
    // Convert Xk to Vector4ld for later computation
    Vector4ld Xk;          //Xk: 4x1
    Xk << 1, 0, 0, 0;

    Matrix4ld Mk= MatrixXld::Identity(4, 4);

    Matrix4ld Zk = MatrixXld::Zero(4, 4);

    for(int i = 1; i <= 3; i++) 
        Zk(i, i) = -1 * pow((long double)10, (long double)-300);

    VectorXld Xregprev = VectorXld::Zero(6);
    
    long double BinghamKFSum = 0;  // for timing Bingham_kf

    //********** Loop starts **********
    // If not converge, transform points using Xreg and repeat
    for (int i = 1; i <= min(MAX_ITERATIONS, sizePtcldMoving / WINDOW_SIZE); i++) {
        int iOffset = i - 1;    // Eigen is 0-index instead of 1-index
        
        // Tree search
        // Send as input a subset of the ptcldMoving and normalMoving points.
        PointCloud targets(3, WINDOW_SIZE);
        PointCloud normalTargets(3, WINDOW_SIZE);

        for (int r = WINDOW_SIZE * (iOffset); r < WINDOW_SIZE * i; r++) {
            int rOffset = r - WINDOW_SIZE * (iOffset);
            for (int n = 0; n < 3; n++) {
                targets(n,rOffset) = (*ptcldMoving)(n, r);
                normalTargets(n,rOffset) = (*normalMoving)(n, r);
            }
        }

        /* kd_search takes subset of ptcldMoving, of normalMoving, 
         * CAD model points (in kdtree form), normalFixed, and Xreg from last iteration
         */
        struct KDNormalResult *searchResult = kd_search_normals(&targets,
                                              cloudTree, INLIER_RATIO, 
                                              &Xreg, &normalTargets, normalFixed);

        PointCloud pc = searchResult->pc;    // set of all closest point
        PointCloud pr = searchResult->pr;    // set of all target points in corresponding 
                                             // order with pc

        // Truncate the window size according to inlier ratio
        int truncSize = trunc(WINDOW_SIZE * INLIER_RATIO);

        // If truncSize odd, round down to even so pc and pr have same dimension
        int oddEntryNum = truncSize / 2;    // size of p1c/p1
        int evenEntryNum = oddEntryNum; // size of p2c/p2r

        PointCloud p1c = PointCloud(3, oddEntryNum);    // odd index points of pc
        PointCloud p2c = PointCloud(3, evenEntryNum);   // even index points of pc
        PointCloud p1r = PointCloud(3, oddEntryNum);    // odd index points of pr
        PointCloud p2r = PointCloud(3, evenEntryNum);   // even index points of pr
        
        long double Rmag= .04 + pow(searchResult->res1 / 6, 2);  // Variable that helps 
                                                                 // calculate the noise 
        long double Qmag = .04 + pow(searchResult->res2 / 6, 2);

        int p1Count = 0;
        int p2Count = 0;
        
        // Store odd entries in pc to p1c, in pr to p1r
        // Store even entries in pc to p2c, in pr to p2r
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
        //  p2r, normalc, normalr from kdsearch
        //  Output updated Xk, Mk, Zk, and Xreg for next iteration. 
        struct BinghamKFResult *QFResult = bingham_normal_kf(&Xk, &Mk, &Zk, Rmag, Qmag, 
                                                 &p1c, &p1r, &p2c, &p2r, &(searchResult->normalc), 
                                                 &(searchResult->normalr)); 
        
        Xk = QFResult->Xk;
        Mk = QFResult->Mk;
        Zk = QFResult->Zk;
        // Store curretn Xreg in Xregsave
        Xregsave.col(i) = QFResult->Xreg;    // No offset applied because 
                                            // Xregsave(0) is saved for initial value   
        
        Xreg = QFResult->Xreg;
        result->Xreg = QFResult->Xreg;
        result->Xregsave = Xregsave;
        
        //  Check convergence:
        //  Takes in updated Xreg and previous Xreg
        //  Return dR, dT
        struct DeltaTransform *convergenceResult = get_changes_in_transformation_estimate(
                                                  QFResult->Xreg, Xregsave.col(i-1));

        if (i >= MIN_ITERATIONS && convergenceResult->dT <= tolerance(0) && 
            convergenceResult->dR <= tolerance(1)) {
            cout << "CONVERGED" << endl;
            break;  // Break out of loop if convergence met
        }
    }

    //free_tree(cloudTree);

    return result;
}