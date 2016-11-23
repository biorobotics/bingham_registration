/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */
#ifndef COMPUTE_TRANSFORMED_POINTS
#define COMPUTE_TRANSFORMED_POINTS

using namespace arma;

mat eul2rotm(mat eul);

mat reg_params_to_transformation_matrix(rowvec params);

mat compute_transformed_points(mat ptcldMoving, rowvec Xreg);

#endif