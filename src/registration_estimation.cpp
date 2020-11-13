/*
 * File Header for registration_estimation.cpp
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
#include "conversions.h"
#include "sort_indexes.h"
#include "type_defs.h"

//#define WINDOW_RATIO 100     // The constant for deciding window size
#define DIMENSION 3     // Dimension of data point

/* compute_transformed_points:
 *      Input: ptcld moving, regParams from previous iteration
        Output: ptcld moving after being transformed 
 */
PointCloud compute_transformed_points(const PointCloud& ptcldMoving, const ArrayXld& regParams) {
    Matrix4ld testimated = reg_params_to_transformation_matrix (regParams.segment(0,6));
    Affine3ld t(testimated);
    return t.cast<float>()*ptcldMoving;
}

/* 
 *  registration_estimation: (for registration without normals)
 *
 *  Outputs (regParams, regHistory):
            regParams is a 6x1 vector
            regHistory is an 6xn matrix (a record of regParams value at different iteration)
    Inputs:
            ptcldMoving (3xn) is one set of point cloud data. This will represent the sensed points
            ptcldFixed (3xn) is another set of point cloud data. This will represent CAD model points 
 */

RegistrationResult registration_estimation(const PointCloud& ptcldMoving,
                                            SearchTree cloudTree,
                                            double inlierRatio,
                                            int maxIterations,
                                            int windowSize,
                                            double toleranceT,
                                            double toleranceR,
                                            double uncertaintyR){
    
    // Make sure input makes sense
    if (ptcldMoving.rows() != DIMENSION) {
        std::cerr << "Invalid point dimension";
        exit(1);
    }
    if(uncertaintyR >= 0){
        uncertaintyR = std::numeric_limits<double>::min() * -1;
    }

    //************ Initialization **************
    RegistrationResult result;

    int sizePtcldMoving = ptcldMoving.cols();
    
    Eigen::Vector2d tolerance;
    tolerance << toleranceT , toleranceR;

    VectorXld regParams = VectorXld::Zero(6);  //regParams: 6x1

    // regHistory.row(0) saves the initialized value. The regParams output from each
    // iteration is stored there (dimension (maxIterations + 1) x 6)
    
    MatrixXld regHistory = MatrixXld::Zero(6, maxIterations + 1); //regHistory: 6xn

    // Refer to paper for definition of Xk, Mk and Zk
    Vector4ld Xk;   //Xk: 4x1 
    Xk << 1, 0, 0, 0;

    Matrix4ld Mk= MatrixXld::Identity(4, 4);

    Matrix4ld Zk = MatrixXld::Zero(4, 4);

    for(int i = 1; i <= 3; i++) 
        Zk(i, i) = uncertaintyR;
    
    if(windowSize > sizePtcldMoving){
        windowSize = sizePtcldMoving;
        std::cout << "Warning: Window size is larger than point cloud. Using window size of ";
        std::cout << windowSize << std::endl;
    }

        
    int inlierSize = trunc(windowSize * inlierRatio);   // Round down to int
    
    PointCloud sortedResultTargets(3, inlierSize);
    PointCloud sortedResultMatches(3, inlierSize);
    PointCloud targetsTransformed(3, windowSize);


    //********** Loop starts **********
    // If not converge, transform points using regParams and repeat
    for (int i = 0; i <= maxIterations - 1; i++) {
        
        // Tree search
        // Send as input a subset of the pftcldMoving points according to window size
        PointCloud targets(3, windowSize);
        
        for (int r = 0; r < windowSize; r++) {
            int rOffset = windowSize * i;
            for (int n = 0; n < 3; n++) 
                targets(n, r) = ptcldMoving(n, (r + rOffset) % sizePtcldMoving);
        }

        // tree_search takes subset of ptcldMovingNew, CAD model points, and regParams
        // from last iteration according to window size 

        // Transform the target points before searching
        targetsTransformed = compute_transformed_points(targets, regParams);

        SearchResult searchResult = tree_search(targetsTransformed, cloudTree);

        // Get indexes sorted by distance
        Eigen::VectorXi sortIndex = sort_indexes<Eigen::VectorXf>(searchResult.distances, true);
        
        for (int count = 0; count < inlierSize; count++) {
            sortedResultMatches.col(count) = searchResult.matches.col(sortIndex[count]);
            sortedResultTargets.col(count) = targets.col(sortIndex[count]);
        }

        result.error = searchResult.distances.sum() / searchResult.distances.cols();

        long double Rmag = .04 + pow(result.error / 6, 2);  // Variable that helps calculate the noise 
        
        //  Quaternion Filtering:
        //  Takes updated Xk, Mk, Zk from last QF, updated Rmag, p1c, p1r, p2c,
        //  p2r from kdsearch
        //  Output updated Xk, Mk, Zk, and regParams for next iteration. 
        BinghamKFResult filterResult = bingham_filter(&Xk, &Mk, &Zk, Rmag, &sortedResultMatches, &sortedResultTargets);

        Xk = filterResult.Xk;
        Mk = filterResult.Mk;
        Zk = filterResult.Zk;

        // Store current regParams in regHistory
        regHistory.col(i+1) = filterResult.regParams; // regHistory(0) is saved for initial value
                                                
        regParams = filterResult.regParams;

        //  Check convergence:
        //  Takes in updated regParams and previous regParams
        //  Return dR, dT
        DeltaTransform convergenceResult = get_changes_in_transformation_estimate(filterResult.regParams,
                                                                                  regHistory.col(i));
        if (convergenceResult.dT <= tolerance(0) && 
            convergenceResult.dR <= tolerance(1)) {
            std::cout << "CONVERGED" << std::endl;
            break;  // Break out of loop if convergence met
        }
    }
    result.regParams = regParams;
    result.regHistory = regHistory;
    return result;
}

RegistrationResult registration_estimation(const PointCloud& ptcldMoving, const PointCloud& ptcldFixed,
                                           double inlierRatio, int maxIterations, int windowSize,
                                           double toleranceT, double toleranceR, double uncertaintyR){
    SearchTree cloudTree = tree_from_point_cloud(ptcldFixed);
    RegistrationResult result = registration_estimation(ptcldMoving, cloudTree, inlierRatio,
                                                        maxIterations, windowSize,
                                                        toleranceT, toleranceR, uncertaintyR);
    free_tree(cloudTree);
    return result;
}