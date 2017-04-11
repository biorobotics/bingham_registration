/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#ifndef QR_KF
#define QR_KF

#include <Eigen/Dense>
#include <long_double_def.h>
#include "compute_transformed_points.h"

struct QrKfResult {
    Vector4ld Xk;   // 4x1
    Matrix4ld Pk;  // 4x4
    VectorXld Xreg; // 6x1
};

/*  quat2eul:
*		Input: quaternion
Output: euler angle in vector after conversion
*/
Vector3ld quat2eul(Quaternionld &q);

extern "C" struct QrKfResult *qr_kf(Vector4ld *Xk, Matrix4ld *Pk, long double Rmag, PointCloud *p1c,
                     	PointCloud *p1r, PointCloud *p2c, PointCloud *p2r);

#endif