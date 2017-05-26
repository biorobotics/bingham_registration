/*
 * File Header for compute_transformed_points.cpp:
 * 		compute_transformed_points takes in sensed points and Xreg
 * 		Return transformed sensed points 
 * 		
 */
#include "compute_transformed_points.h"
/* eul2rotm:
 *		Input: euler angle in array (for the use of cos, sin function), in ZYX order
 		Output: quaternion after conversion 
 */
Matrix4ld eul2rotm(const Array3ld& eul) { // ZYX order
	Matrix4ld R = Matrix4ld::Identity(4, 4);	// Since n_slices is just 1, make 
												// Matrix4ld instead
	Array3ld ct = cos(eul);
	Array3ld st = sin(eul);

	//     The rotation matrix R can be construted (as follows by
    //     ct = [cz cy cx] and st =[sz sy sx]
  	//
    //       R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
    //              cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
    //                -sy            cy*sx             cy*cx] 

	R(0,0) = ct(1) * ct(0);
	R(0,1) = st(2) * st(1) * ct(0) - ct(2) * st(0);
	R(0,2) = ct(2) * st(1) * ct(0) + st(2) * st(0);
	R(1,0) = ct(1) * st(0);
	R(1,1) = st(2) * st(1) * st(0) + ct(2) * ct(0);
	R(1,2) = ct(2) * st(1) * st(0) - st(2) * ct(0);
	R(2,0) = -st(1);
	R(2,1) = st(2) * ct(1);
	R(2,2) = ct(2) * ct(1);	
	return R;
}

/* reg_params_to_transformation_matrix:
 *		Input: registration parameters in array
 		Output: transformation matrix after conversion 
 */
Matrix4ld reg_params_to_transformation_matrix(const ArrayXld& params) {
	Matrix4ld R, U, V;
	Matrix4ld T = Matrix4ld::Identity(4, 4);

	for (int r = 0; r < 3; r++) {
		T(r, 3) = params(r);
	}

	Array3ld temp;
	temp(0) = params(3);
	temp(1) = params(4);
	temp(2) = params(5);

	R = eul2rotm (temp);

	Eigen::JacobiSVD<Matrix4ld> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
	
	R = svd.matrixU()*svd.matrixV().transpose();

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++)
			T(r, c) = R(r, c);
	}

	return T;
}

/* compute_transformed_points:
 *		Input: ptcld moving, Xreg from previous iteration
 		Output: ptcld moving after being transformed 
 */
PointCloud compute_transformed_points(const PointCloud& ptcldMoving, const ArrayXld& Xreg) {
	Vector3ld point;
	Matrix4ld testimated = reg_params_to_transformation_matrix (Xreg.segment(0,6));
	Affine3ld t;
	t.matrix() = testimated;
	int numPoints = ptcldMoving.cols();
	PointCloud ptcldMovingTransformed(3,numPoints);
	for (int r = 0; r < numPoints; r++) {
		point = ptcldMoving.col(r);
		ptcldMovingTransformed.col(r) = t*point;
	}
	return ptcldMovingTransformed;
}
