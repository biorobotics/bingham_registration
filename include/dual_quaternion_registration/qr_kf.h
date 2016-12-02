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

struct triple2;

Array3d quat2eul(Quaterniond q);



/*
mat quat2rotm(rowvec q);

void call_error(string msg);

rowvec quat2eul(rowvec q);

struct triple2 qr_kf(vec Xk, mat Pk, double Rmag, mat p1c, mat p1r, mat p2c, mat p2r);
*/
#endif