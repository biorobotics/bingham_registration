/*
 * File Header for bingham_kf.cpp:
 * 		This file contains functions for performing Bingham quaternion filtering
 */

#include <iostream>
#include <limits>
#include "sort_indexes.h"
#include "bingham_filter.h"

/* quat2eul:
*		Input: quaternion
Output: euler angle in vector after conversion
*/
Vector3ld quat2eul(Quaternionld q) {
	// Normalize the quaternions
	Quaternionld temp = q.normalized();

	long double qw = temp.w();
	long double qx = temp.x();
	long double qy = temp.y();
	long double qz = temp.z();

	Vector3ld eul(3);

	eul(0) = atan2(2 * (qx * qy + qw * qz), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
	eul(1) = asin(-2 * (qx * qz - qw * qy));
	eul(2) = atan2(2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));

	return eul;
}

Matrix4ld measurementFunctionJacobian(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2) {
    /*p1, p2 is of size 1x3
    *H is of size 4x4 */
    Matrix4ld H;

    H(0,0) = (long double)0;
    H(0,1) = p2(0)-p1(0);
    H(0,2) = p2(1)-p1(1);
    H(0,3) = p2(2)-p1(2);

    H(1,0) = p1(0)-p2(0);
    H(1,1) = (long double)0;
    H(1,2) = -(p1(2)+p2(2));
    H(1,3) = p1(1)+p2(1);

    H(2,0) = p1(1)-p2(1);
    H(2,1) = p1(2)+p2(2);
    H(2,2) = (long double)0;
    H(2,3) = -(p1(0)+p2(0));

    H(3,0) = p1(2)-p2(2);
    H(3,1) = -(p1(1)+p2(1));
    H(3,2) = p1(0)+p2(0);
    H(3,3) = (long double)0;

    return H;
}

/* bingham_kf:
 *		Input: previous Xk, Mk, Zk, Rmag, pointsClosest, pointsTarget,
 		Output: Updated Xk, Mk, Zk, regParams
 */
BinghamKFResult bingham_filter(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
						   	   long double Rmag, PointCloud *pointsClosest,
						   	   PointCloud *pointsTarget) {
	
	// Check for input dimensions 
    if (Xk->size() != 4){
        std::cerr << "Xk has wrong dimension. Should be 4x1\n";
        exit(1);
    }
    if (Mk->rows() != 4 || Mk->cols() != 4){
        std::cerr << "Mk has wrong dimension. Should be 4x4\n";
        exit(1);
    }
    if (Zk->rows() != 4 || Zk->cols() != 4){
        std::cerr << "Mk has wrong dimension. Should be 4x4\n";
        exit(1);
    }
    if (pointsClosest->cols() != pointsTarget->cols()) {
        std::cerr << "point clouds are not equal in size\n";
        exit(1);
	}

	// Truncate the windowSize according to window size and inlier ratio
    int truncSize = pointsClosest->cols();

    // If truncSize odd, round down to even so pc and pr have same dimension
    int oddEntryNum = truncSize / 2;    // size of p1c/p1
    int evenEntryNum = oddEntryNum; // size of p2c/p2r

    // Separate odd and even rows
	Eigen::Map<PointCloud,0,Eigen::OuterStride<> >
		p1c(pointsClosest->data(),
			pointsClosest->rows(),
			(pointsClosest->cols())/2,
			Eigen::OuterStride<>(pointsClosest->outerStride()*2));

	Eigen::Map<PointCloud,0,Eigen::OuterStride<> >
		p2c(pointsClosest->data() + pointsClosest->rows(),
			pointsClosest->rows(),
			(pointsClosest->cols())/2,
			Eigen::OuterStride<>(pointsClosest->outerStride()*2));

	Eigen::Map<PointCloud,0,Eigen::OuterStride<> >
		p1r(pointsTarget->data(),
			pointsTarget->rows(),
			(pointsTarget->cols())/2,
			Eigen::OuterStride<>(pointsTarget->outerStride()*2));

	Eigen::Map<PointCloud,0,Eigen::OuterStride<> >
		p2r(pointsTarget->data() + pointsTarget->rows(),
			pointsTarget->rows(),
			(pointsTarget->cols())/2,
			Eigen::OuterStride<>(pointsTarget->outerStride()*2));
	
	// Subtract even and odd to create vectors to align
	PointCloud pc = p1c - p2c;
	PointCloud pr = p1r - p2r;
	
	/*c is the smallest diagonal value of Zk. remember Zk is a diagonal matrix
	 with all diagonal elements being negative except for first entry which is
	 0 */
	long double c = Zk->minCoeff();

	Matrix4ld I = MatrixXld::Identity(4,4);

	Matrix4ld temp = (*Mk) * ((*Zk) + c*I)*((*Mk).transpose());

	Matrix4ld tempInv;

	tempInv = temp.inverse();

	Matrix4ld Pk = -0.5 * tempInv;

	Matrix4ld Nk = (*Xk) * (*Xk).transpose() + Pk;

	Matrix4ld RTmp = Rmag * (Nk.trace()*I - Nk);

	Eigen::EigenSolver<Matrix4ld> es(RTmp);
	
	VectorXld s = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXld U = es.eigenvectors().real();	// A matrix whose column vectors are eigen vectors of RTmp
	
	// Normalize U
	for (int i = 0; i < U.cols(); i++)
		U.col(i).norm();

	// If any elements in s <=10^-4, then set it to be equal to 1
	for (int i = 0; i < s.size(); i++) {
		if (s(i) <= .0001)
			s(i) = 1;
	}

	// s.^-1 is essentially takes s=[a,b,c,d] and returns [1/a, 1/b, 1/c, 1/d]
	Matrix4ld RInvTmp = U * ((s.array().pow(-1)).matrix().asDiagonal()) * U.transpose();

	Matrix4ld D1 = MatrixXld::Zero(4,4);	// Initialize D1 to zero matrix
	for (int i = 0; i < pc.cols(); i++) {
		Matrix4ld G_tmp = measurementFunctionJacobian(pc.col(i), pr.col(i));
		D1 = D1 + G_tmp.transpose()*RInvTmp*G_tmp;
	}

	Matrix4ld D2 = MatrixXld::Zero(4,4);	// Initialize D2 to zero matrix

	Matrix4ld DStar = -0.5*D1 - 0.5*D2 + (*Mk)*(*Zk)*(*Mk).transpose();
	
	es.compute(DStar, true);
	
	VectorXld ZTmp = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXld MTmp = es.eigenvectors().real();	// A matrix whose column vectors are eigen 
												// vectors of RTmp
	
	// Normalize MTmp
	for (int i = 0; i < U.cols(); i++)
		MTmp.col(i).norm();

	// Sort ZTmp
	Eigen::VectorXi indx = sort_indexes<VectorXld>(ZTmp, false);
	VectorXld ZTmpSorted(ZTmp.size());
	
	for (int i = ZTmp.size()-1; i > 0; i--)
		ZTmpSorted(i) = ZTmp(indx[i]) - ZTmp(indx[0]);

	// This step should ensure that Zk has first diagonal element = 0
	ZTmpSorted(0) = 0;

	*Zk = ZTmpSorted.asDiagonal();
	
	// Since we sorted and changed he order of the elements in Zk, we have to do the same
	// change of orders for columns in Mk
	(*Mk).col(0) = MTmp.col(indx[0]);
	*Xk = (*Mk).col(0);	// Update Xk

	for (int i = 1; i < ZTmp.size(); i++)
		(*Mk).col(i) = MTmp.col(indx[i]);
	
	// Calculate translation vector from rotation estimate
    // quat2rotm converts quaternion to rotation matrix.
    Vector3ld centroid;
    Vector3ld centroidRotated;
    for(int i=0; i<3; i++) {
        centroid(i) = (p1c.row(i).mean() + p2c.row(i).mean()) / 2.0;
        centroidRotated(i) = (p1r.row(i).mean() + p2r.row(i).mean()) / 2.0;
    }
    Quaternionld XkQuat = Quaternionld((*Xk)(0), (*Xk)(1), (*Xk)(2), (*Xk)(3)).normalized();
    
    centroidRotated = XkQuat.toRotationMatrix() * centroidRotated;
    Vector3ld eulerRotation = quat2eul(XkQuat);
    Vector3ld centroidDifference = centroid - centroidRotated;

    VectorXld regParams(6);
    regParams.segment(0,3) = centroidDifference;
    regParams.segment(3,3) = eulerRotation;
    
    BinghamKFResult result;

    result.Xk = *Xk;
    result.regParams = regParams;
    result.Mk = *Mk;
    result.Zk = *Zk;

    return result;
}