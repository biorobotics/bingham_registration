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

typedef Matrix<float, 3, Dynamic> pointCloud;

Matrix4f eul2rotm(ArrayXf eul);

Matrix4f reg_params_to_transformation_matrix(ArrayXf params);

pointCloud compute_transformed_points(pointCloud ptcldMoving, ArrayXf Xreg);

#endif