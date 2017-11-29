/*
 * File Header for compute_transformed_points.cpp:
 * 		compute_transformed_points takes in sensed points and regParams
 * 		Return transformed sensed points 
 * 		
 */
#include "conversions.h"
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

/* eul2quat:
 *		Input: euler angle in vector
 		Output: quaternion after conversion 
 */
Quaternionld eul2quat(const Vector3ld& eul) {
	Array3ld eulHalf = eul.array() / 2;

	Array3ld c = eulHalf.cos();
	Array3ld s = eulHalf.sin();
	
	Quaternionld q;
	q.w() = c(0) * c(1) * c(2) + s(0) * s(1) * s(2);
	q.x() = c(0) * c(1) * s(2) - s(0) * s(1) * c(2);
	q.y() = c(0) * s(1) * c(2) + s(0) * c(1) * s(2);
	q.z() = s(0) * c(1) * c(2) - c(0) * s(1) * s(2);
	return q.normalized();
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