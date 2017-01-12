/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#ifndef QR_KF
#define QR_KF

#include <Eigen/Dense>
#include <long_double_def.h>

struct QrKfResult {
    Array4ld Xk;   // 1x4
    Matrix4ld Pk;  // 4x4
    ArrayXld Xreg; // 1x6
};

Array3ld quat2eul(Quaternionld q);

struct QrKfResult qr_kf(Vector4ld Xk, Matrix4ld Pk, long double Rmag, PointCloud p1c,
                     	PointCloud p1r, PointCloud p2c, PointCloud p2r);

#endif