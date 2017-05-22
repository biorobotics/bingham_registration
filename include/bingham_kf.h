/*
 * File Header for bingham_kf.h:
 * 		This file contains functions for performing Bingham quaternion filtering
 */

#ifndef BINGHAM_KF
#define BINGHAM_KF

#include <type_defs.h>
#include <registration_tools.h>


struct BinghamKFResult {	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4ld Xk;  // 4 * 1. Used as an updated Xk in the next loop
    Matrix4ld Mk;  // 4 * 4. Used as an updated Mk in the next loop
    Matrix4ld Zk;    // 4 * 4. Used as an updated Zk in the next loop
    VectorXld Xreg;  // 6 * 1. The updated Xreg 
};

/* bingham_kf:
 *		Input: previous Xk, Mk, Zk, Rmag, p1c, p1r, p2c, p2r
 		Output: Updated Xk, Mk, Zk, Xreg
 */
BinghamKFResult bingham_kf(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
							long double Rmag, PointCloud *p1c, PointCloud *p1r, 
							PointCloud *p2c, PointCloud *p2r);

/* bingham_normal_kf: (the version of bingham_kf with the aid of normals)
 *		Input: previous Xk, Mk, Zk, Rmag, Qmag, p1c, p1r, p2c, p2r, normalc,
 		Output: Updated Xk, Mk, Zk, Xreg
 */
BinghamKFResult bingham_normal_kf(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
								  long double Rmag, long double Qmag, PointCloud *p1c, 
								  PointCloud *p1r, PointCloud *p2c, PointCloud *p2r, 
								  PointCloud *normalc, PointCloud *normalr);
#endif