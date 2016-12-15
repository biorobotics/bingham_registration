/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */

#include <iostream>
#include <get_changes_in_transformation_estimate.h>

using namespace std;
using namespace Eigen;


// return rowvec q of dimension 1 x 4
Quaterniond eul2quat(Vector3d eul) {
	Array3d eulHalf = eul / 2;
	Array3d c = eulHalf.cos();
	Array3d s = eulHalf.sin();
	
	Quaterniond q;
	q.w() = c(0) * c(1) * c(2) + s(0) * s(1) * s(2);
	q.x() = c(0) * c(1) * s(2) - s(0) * s(1) * c(2);
	q.y() = c(0) * s(1) * c(2) + s(0) * c(1) * s(2);
	q.z() = s(0) * c(1) * c(2) - c(0) * s(1) * s(2);
	return q;
}

struct DeltaTransform get_changes_in_transformation_estimate(VectorXd Xreg, VectorXd Xregprev) {
	struct DeltaTransform result;
	Quaterniond qs = eul2quat(Xreg.segment(3,3));
	Quaterniond qsPrev = eul2quat(Xregprev.segment(3,3));
	// Rotation difference in radians
	double dR = acos(qsPrev.dot(qs));
	
	// Euclidean difference
	double dT = (Xregprev.segment(0,3) - Xreg.segment(0,3)).norm();
	result.dR = dR;
	result.dT = dT;
	return result;
}