/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */

#include <iostream>
#include <armadillo>
#include <cmath>

using namespace std;
using namespace arma;

struct tuple1{
	double dR;
	double dT;
};

// return rowvec q of dimension 1 x 4
rowvec eul2quat(rowvec eul) {
	rowvec c = cos(eul / 2);
	rowvec s = sin(eul / 2);

	rowvec q = rowvec(4);
	q(0) = c(0) * c(1) * c(2) + s(0) * s(1) * s(2);
	q(1) = c(0) * c(1) * s(2) - s(0) * s(1) * c(2);
	q(2) = c(0) * s(1) * c(2) + s(0) * c(1) * s(2);
	q(3) = s(0) * c(1) * c(2) - c(0) * s(1) * s(2);
	return q;
}

struct tuple1 get_changes_in_transformation_estimate(rowvec Xreg, rowvec Xregprev) {
	struct tuple1 result;
	rowvec qs = eul2quat(Xreg.subvec(3, 6));
	rowvec qsPrev = eul2quat(Xregprev.subvec(3, 6));
	// Rotation difference in radians
	double dR = acos(dot(qsPrev, qs));
	// Euclidean difference
	double dT = std::sqrt(sum(square(Xregprev.subvec(0, 2) - Xreg.subvec(0, 2))));
	result.dR = dR;
	result.dT = dT;
	return result;
}