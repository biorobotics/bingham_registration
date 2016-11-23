/*
 * File Header:
 * compute_transformed_points takes in sensed points and Xreg
 * Return transformed sensed points 
 * 		
 */

#include <armadillo>

using namespace arma;

mat eul2rotm(rowvec eul) {
	mat R = mat(3, 3);	// Since n_slices is just 1, make mat instead
	rowvec ct = cos(eul);
	rowvec st = sin(eul);

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

mat reg_params_to_transformation_matrix(rowvec params) {
	mat T = eye(4, 4);
	mat R, U, V;
	vec s;
	for (int r = 0; r < 3; r++) {
		T(r, 3) = params(r);
	}
	rowvec temp = rowvec(3);
	temp(0) = params(3);
	temp(1) = params(4);
	temp(2) = params(5);
	 
	R = eul2rotm (temp);


	svd(U, s, V, R);
 
	R = U * V.t();
 
	for (int r = 0; r < 3; r++) {
		for (int c = 0; c < 3; c++)
			T(r, c) = R(r, c);
	}
 
	return T;
}

mat compute_transformed_points(mat ptcldMoving, rowvec Xreg) {
	vec point;
	mat testimated = reg_params_to_transformation_matrix (Xreg.subvec(0, 5));
	mat rot = testimated.submat(0, 0, 2, 2);
	vec translate = testimated.submat(0, 3, 2, 3);
	mat ptcldMovingTransformed = mat(size(ptcldMoving));

	for (int r = 0; r < ptcldMoving.n_rows; r++) {
		point = ptcldMoving.row(r).t();
		ptcldMovingTransformed.row(r) = (rot * point + translate).t();
	}
	if (ptcldMoving.n_rows != ptcldMovingTransformed.n_rows)
	{
		cout << "Dimension doesn't match" << endl;
		exit(-1);
	}
	return ptcldMovingTransformed;
}