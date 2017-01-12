/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */
#ifndef COMPUTE_TRANSFORMED_POINTS
#define COMPUTE_TRANSFORMED_POINTS

#include <Eigen/Dense>
using namespace Eigen;
using namespace std;

typedef Matrix<double, 3, Dynamic> PointCloud;

Matrix4d reg_params_to_transformation_matrix(ArrayXd params);

PointCloud compute_transformed_points(PointCloud ptcldMoving, ArrayXd Xreg);

#endif