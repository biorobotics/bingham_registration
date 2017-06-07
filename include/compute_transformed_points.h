/*
 * File Header for compute_transformed_points.cpp:
 * 		compute_transformed_points takes in sensed points and Xreg
 * 		Return transformed sensed points 
 * 		
 */
#ifndef COMPUTE_TRANSFORMED_POINTS
#define COMPUTE_TRANSFORMED_POINTS
#include <Eigen/Dense>
#include <type_defs.h>

/* compute_transformed_points:
 *		Input: ptcld moving, Xreg from previous iteration
 		Output: ptcld moving after being transformed 
 */
PointCloud compute_transformed_points(const PointCloud& ptcldMoving, const ArrayXld& Xreg);

#endif