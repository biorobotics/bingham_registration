/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#ifndef QR_KF
#define QR_KF

struct triple2{
    mat Xk;
    mat Pk;
    rowvec Xreg;
};

mat quat2rotm(rowvec q);

void call_error(string msg);

rowvec quat2eul(rowvec q);

struct triple2 qr_kf(vec Xk, mat Pk, double Rmag, mat p1c, mat p1r, mat p2c, mat p2r);

#endif