/*
 * File Header for bingham_filter.h:
 * 		This file contains functions for performing Bingham quaternion filtering
 */

#ifndef BINGHAM_FILTER
#define BINGHAM_FILTER

#include "type_defs.h"

struct BinghamKFResult {	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4ld Xk;  // 4 * 1. Used as an updated Xk in the next loop
    Matrix4ld Mk;  // 4 * 4. Used as an updated Mk in the next loop
    Matrix4ld Zk;    // 4 * 4. Used as an updated Zk in the next loop
    VectorXld Xreg;  // 6 * 1. The updated Xreg 
};

/* bingham_filter:
 *		Input: previous Xk, Mk, Zk, Rmag, p1c, p1r, p2c, p2r
 		Output: Updated Xk, Mk, Zk, Xreg
 */
BinghamKFResult bingham_filter(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
							   long double Rmag, PointCloud *p1c, PointCloud *p1r, 
							   PointCloud *p2c, PointCloud *p2r, long double Qmag = 0,
							   PointCloud *normalc = NULL, PointCloud *normalr = NULL);
#endif