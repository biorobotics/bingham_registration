/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */
#include <compute_transformed_points.h>

Matrix4f eul2rotm(Array3f eul) {
	Matrix4f R = Matrix4f::Identity(4, 4);	// Since n_slices is just 1, make Matrix4f instead
	Array3f ct = cos(eul);
	Array3f st = sin(eul);

	/*
	 *     The rotation matrix R can be construted (as follows by
     *     ct = [cx cy cz] and st =[sx sy sz]
  	 *
     *       R = [  cy*cz   sy*sx*cz-sz*cx    sy*cx*cz+sz*sx
     *              cy*sz   sy*sx*sz+cz*cx    sy*cx*sz-cz*sx
     *                -sy            cy*sx             cy*cx]
	 */ 

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

Matrix4f reg_params_to_transformation_matrix(ArrayXf params) {
	Matrix4f R, U, V;
	Matrix4f T = Matrix4f::Identity(4, 4);

	for (int r = 0; r < 3; r++) {
		T(r, 3) = params(r);
	}

	Array3f temp;
	temp(0) = params(3);
	temp(1) = params(4);
	temp(2) = params(5);

	R = eul2rotm (temp);

	JacobiSVD<Matrix4f> svd(R, ComputeFullU | ComputeFullV);
	
	R = svd.matrixU()*svd.matrixV().transpose();

	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++)
			T(r, c) = R(r, c);
	}

	return T;
}

pointCloud compute_transformed_points(pointCloud ptcldMoving, ArrayXf Xreg) {
	Vector3f point;
	Matrix4f testimated = reg_params_to_transformation_matrix (Xreg.segment(0,5));
	Affine3f t;
	t.matrix() = testimated;
	
	pointCloud ptcldMovingTransformed = ptcldMoving;
	
	int numPoints = ptcldMoving.size() / 3;

	for (int r = 0; r < numPoints; r++) {
		Vector3f point = ptcldMoving.col(r);
		ptcldMovingTransformed.col(r) = t*point;
	}
	
	return ptcldMovingTransformed;
}