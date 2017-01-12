/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */
#ifndef COMPUTE_TRANSFORMED_POINTS
#define COMPUTE_TRANSFORMED_POINTS

#include <Eigen/Dense>
#include <long_double_def.h>

using namespace Eigen;
using namespace std;

typedef Matrix<long double, 3, Dynamic> PointCloud;

Matrix4ld reg_params_to_transformation_matrix(ArrayXld params);

PointCloud compute_transformed_points(PointCloud ptcldMoving, ArrayXld Xreg);

#endif