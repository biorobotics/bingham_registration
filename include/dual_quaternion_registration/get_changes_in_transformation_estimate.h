/*
 *	File Header:
 *  get_changes_in_transformation_estimate takes in the updated Xreg and previous
 *  Xreg, outputs dT and dR for checking if convergence is met.
 */
#ifndef GET_CHANGES_IN_TRANSFORMATION_ESTIMATE
#define GET_CHANGES_IN_TRANSFORMATION_ESTIMATE
#include <Eigen/Dense>
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Eigen::VectorXd;

struct DeltaTransform{
	double dR;
	double dT;
};

Quaterniond eul2quat(Vector3d eul);

struct DeltaTransform get_changes_in_transformation_estimate(VectorXd Xreg, VectorXd Xregprev);

#endif