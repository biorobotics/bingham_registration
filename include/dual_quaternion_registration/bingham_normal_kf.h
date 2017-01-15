/*
 * File Header:
 * This file contains functions for performing quaternion filtering with the consideration 
 * of normals.
 */

#ifndef BINGHAM_NORMAL_KF
#define BINGHAM_NORMAL_KF

#include <Eigen/Dense>
#include <long_double_def.h>


struct BinghamNormalKFResult {
	Vector4ld Xk;  // 4 * 1. Used as an updated Xk in the next loop
    Matrix4ld Mk;  // 4 * 4. Used as an updated Mk in the next loop
    Matrix4ld Zk;    // 4 * 4. Used as an updated Zk in the next loop
    VectorXld Xreg;  // 6 * 1. The updated Xreg 
};

Vector3ld quat2eul(Quaternionld q);

struct BinghamNormalKFResult bingham_normal_kf(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
								  long double Rmag, long double Qmag, PointCloud *p1c, PointCloud *p1r, 
								  PointCloud *p2c, PointCloud *p2r, PointCloud *normalc,
								  PointCloud *normalr);

#endif