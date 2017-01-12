/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#ifndef BINGHAM_KF
#define BINGHAM_KF

#include <Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Array3d;
using Eigen::Array4d;
using Eigen::ArrayXd;
using Eigen::Quaterniond;

struct BinghamKFResult {
	Vector4d Xk;  // 4 * 1. Used as an updated Xk in the next loop
    Matrix4d Mk;  // 4 * 4. Used as an updated Mk in the next loop
    Matrix4d Zk;    // 4 * 4. Used as an updated Zk in the next loop
    VectorXd Xreg;  // 6 * 1. The updated Xreg 
};

Vector3d quat2eul(Quaterniond q);

struct BinghamKFResult bingham_kf(Vector4d Xk, Matrix4d Mk, Matrix4d Zk, 
								  double Rmag, PointCloud p1c, PointCloud p1r, 
								  PointCloud p2c, PointCloud p2r);

#endif