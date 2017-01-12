/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */
#include <compute_transformed_points.h>

Matrix4d eul2rotm(Array3d eul) { // ZYX order
	Matrix4d R = Matrix4d::Identity(4, 4);	// Since n_slices is just 1, make Matrix4d instead
	Array3d ct = cos(eul);
	Array3d st = sin(eul);

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

Matrix4d reg_params_to_transformation_matrix(ArrayXd params) {
	Matrix4d R, U, V;
	Matrix4d T = Matrix4d::Identity(4, 4);

	for (int r = 0; r < 3; r++) {
		T(r, 3) = params(r);
	}

	Array3d temp;
	temp(0) = params(3);
	temp(1) = params(4);
	temp(2) = params(5);

	R = eul2rotm (temp);

	JacobiSVD<Matrix4d> svd(R, ComputeFullU | ComputeFullV);
	
	R = svd.matrixU()*svd.matrixV().transpose();

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++)
			T(r, c) = R(r, c);
	}

	return T;
}

PointCloud compute_transformed_points(PointCloud ptcldMoving, ArrayXd Xreg) {
	Vector3d point;
	Matrix4d testimated = reg_params_to_transformation_matrix (Xreg.segment(0,6));
	Affine3d t;
	t.matrix() = testimated;
	int numPoints = ptcldMoving.cols();
	PointCloud ptcldMovingTransformed(3,numPoints);
	for (int r = 0; r < numPoints; r++) {
		point = ptcldMoving.col(r);
		ptcldMovingTransformed.col(r) = t*point;
	}
	return ptcldMovingTransformed;
}