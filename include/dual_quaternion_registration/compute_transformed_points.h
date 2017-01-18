/*
 * File Header for compute_transformed_points.cpp:
 * 		compute_transformed_points takes in sensed points and Xreg
 * 		Return transformed sensed points 
 * 		
 */
#ifndef COMPUTE_TRANSFORMED_POINTS
#define COMPUTE_TRANSFORMED_POINTS

#include <Eigen/Dense>
#include <long_double_def.h>

using namespace Eigen;
using namespace std;

typedef Matrix<long double, 3, Dynamic> PointCloud;

/* reg_params_to_transformation_matrix:
 *		Input: registration parameters in array
 		Output: transformation matrix after conversion 
 */
Matrix4ld reg_params_to_transformation_matrix(ArrayXld params);

/* compute_transformed_points:
 *		Input: ptcld moving, Xreg from previous iteration
 		Output: ptcld moving after being transformed 
 */
PointCloud compute_transformed_points(PointCloud ptcldMoving, ArrayXld Xreg);

#endif