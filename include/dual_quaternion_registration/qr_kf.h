/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#ifndef QR_KF
#define QR_KF

#include <Eigen/Dense>
using Eigen::Matrix4d;
using Eigen::Array3d;
using Eigen::Array4d;
using Eigen::ArrayXd;
using Eigen::Quaterniond;

struct QrKfResult {
    Array4d Xk;   // 1x4
    Matrix4d Pk;  // 4x4
    ArrayXd Xreg; // 1x6
};

Array3d quat2eul(Quaterniond q);

struct QrKfResult qr_kf(Vector4d Xk, Matrix4d Pk, double Rmag, PointCloud p1c,
                     	PointCloud p1r, PointCloud p2c, PointCloud p2r);

#endif