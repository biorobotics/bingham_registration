/*
 *	File Header for get_changes_in_transformation_estimate.cpp:
 *  	get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  	Xreg, outputs dT and dR for checking if convergence is met.
 */

#include <iostream>
#include "get_changes_in_transformation_estimate.h"


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

/* get_changes_in_transformation_estimate:
 *		Input: pose from last iteration, a record of the poses from earlier iterations
 		Output: dR and dT to check wheter to stop the iteration
 */
extern "C" struct DeltaTransform get_changes_in_transformation_estimate(const VectorXld& Xreg, const VectorXld& Xregprev) {
	Quaternionld qs = eul2quat(Xreg.segment(3,3));
	Quaternionld qsPrev = eul2quat(Xregprev.segment(3,3));
	// Rotation difference in radians
	long double dR = acos(qsPrev.dot(qs));
	
	// Euclidean difference
	long double dT = (Xregprev.segment(0,3) - Xreg.segment(0,3)).norm();

	DeltaTransform result;
	result.dR = dR;
	result.dT = dT;
	return result;
}