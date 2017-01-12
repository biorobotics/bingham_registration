/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */
#ifndef GET_CHANGES_IN_TRANSFORMATION_ESTIMATE
#define GET_CHANGES_IN_TRANSFORMATION_ESTIMATE

#include <Eigen/Dense>
#include <long_double_def.h>

struct DeltaTransform{
	long double dR;
	long double dT;
};

Quaternionld eul2quat(Vector3ld eul);

struct DeltaTransform get_changes_in_transformation_estimate(VectorXld Xreg, VectorXld Xregprev);

#endif