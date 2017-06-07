/*
 * File Header for bingham_kf.cpp:
 * 		This file contains functions for performing Bingham quaternion filtering
 */

#include <iostream>
#include <limits>
#include <bingham_kf.h>
#include <registration_tools.h>

using namespace std;
using namespace Eigen;

Vector4ld qr_kf_measurementFunction(const Vector4ld& Xk, const Vector3ld& p1, const Vector3ld& p2) {
    /* Xk if size 4x1
    * p1, p2 is of size 3x1
    * g is of size 4x1
    */
    Vector4ld g = Vector4ld::Zero();
    g(0) = Xk(1)*(p2(0)-p1(0))+Xk(2)*(p2(1)-p1(1))+Xk(3)*(p2(2)-p1(2));
    g(1) = Xk(0)*(p1(0)-p2(0))-Xk(2)*(p1(2)+p2(2))+Xk(3)*(p1(1)+p2(1));
    g(2) = Xk(0)*(p1(1)-p2(1))+Xk(1)*(p1(2)+p2(2))-Xk(3)*(p1(0)+p2(0));
    g(3) = Xk(0)*(p1(2)-p2(2))-Xk(1)*(p1(1)+p2(1))+Xk(2)*(p1(0)+p2(0));
    return g;
}

Matrix4ld qr_kf_measurementFunctionJacobian(const Vector3ld& p1, const Vector3ld& p2) {
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
 *		Input: previous Xk, Mk, Zk, Rmag, p1c, p1r, p2c, p2r
 		Optional input: Qmag, normalc, normalr
 		Output: Updated Xk, Mk, Zk, Xreg
 */
BinghamKFResult bingham_kf(Vector4ld *Xk, Matrix4ld *Mk, Matrix4ld *Zk, 
						   long double Rmag, PointCloud *p1c, PointCloud *p1r, 
						   PointCloud *p2c, PointCloud *p2r, long double Qmag,
						   PointCloud *normalc, PointCloud *normalr) {
	// Check for input dimensions 
    if ((*Xk).size() != 4)
        cerr << "Xk has wrong dimension. Should be 4x1\n";
    if ((*Mk).rows() != 4 || (*Mk).cols() != 4)
        cerr << "Mk has wrong dimension. Should be 4x4\n";
    if ((*Zk).rows() != 4 || (*Zk).cols() != 4)
        cerr << "Mk has wrong dimension. Should be 4x4\n";
    if ((*p1c).cols() != (*p1r).cols() || (*p1c).cols() != (*p2c).cols() 
    	 || (*p1c).cols() != (*p2c).cols())
        cerr << "point clouds are not equal in size\n";

	int i;
	PointCloud pc = *p1c - *p2c;
	PointCloud pr = *p1r - *p2r;
	
	/*c is the smallest diagonal value of Zk. remember Zk is a diagonal matrix
	 with all diagonal elements being negative except for first entry which is
	 0 */
	long double c = (*Zk).minCoeff();

	Matrix4ld I = MatrixXld::Identity(4,4);

	Matrix4ld temp = (*Mk) * ((*Zk) + c*I)*((*Mk).transpose());

	Matrix4ld tempInv;

	/* An alternate calculation to prevent overflow cases that show up when we
		 * use dynamically linked library
		 */
	if (abs(temp(0, 0)) < pow(10, -300))
		tempInv = (temp / (pow(10, -300))).inverse()*(pow(10, 300));
	else if (abs(temp(0, 0)) < pow(10, -250))
		tempInv = (temp / (pow(10, -250))).inverse()*(pow(10, 250));
	else if (abs(temp(0, 0)) < pow(10, -200))
		tempInv = (temp / (pow(10, -200))).inverse()*(pow(10, 200));
	else if (abs(temp(0, 0)) < pow(10, -150))
		tempInv = (temp / (pow(10, -150))).inverse()*(pow(10, 150));
	else if (abs(temp(0, 0)) < pow(10, -100))
		tempInv = (temp / (pow(10, -100))).inverse()*(pow(10, 100));
	else if (abs(temp(0, 0)) < pow(10, -50))
		tempInv = (temp / (pow(10, -50))).inverse()*(pow(10, 50));
	else if (abs(temp(0, 0)) > pow(10, 300))
		tempInv = (temp / (pow(10, 300))).inverse()*(pow(10, -300));
	else if (abs(temp(0, 0)) > pow(10, 250))
		tempInv = (temp / (pow(10, 250))).inverse()*(pow(10, -250));
	else if (abs(temp(0, 0)) > pow(10, 200))
		tempInv = (temp / (pow(10, 200))).inverse()*(pow(10, -200));
	else if (abs(temp(0, 0)) > pow(10, 150))
		tempInv = (temp / (pow(10, 150))).inverse()*(pow(10, -150));
	else if (abs(temp(0, 0)) > pow(10, 100))
		tempInv = (temp / (pow(10, 100))).inverse()*(pow(10, -100));
	else if (abs(temp(0, 0)) > pow(10, 50))
		tempInv = (temp / (pow(10, 50))).inverse()*(pow(10, -50));
	else
		tempInv = temp.inverse();

	Matrix4ld Pk = -0.5 * tempInv;

	Matrix4ld Nk = (*Xk) * (*Xk).transpose() + Pk;

	Matrix4ld RTmp = Rmag * (Nk.trace()*I - Nk);

	EigenSolver<Matrix4ld> es(RTmp);
	
	VectorXld s = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXld U = es.eigenvectors().real();	// A matrix whose column vectors are eigen vectors of RTmp
	
	// Normalize U
	for (i = 0; i < U.cols(); i++)
		U.col(i).norm();

	// If any elements in s <=10^-4, then set it to be equal to 1
	for (i = 0; i < s.size(); i++) {
		if (s(i) <= .0001)
			s(i) = 1;
	}

	// s.^-1 is essentially takes s=[a,b,c,d] and returns [1/a, 1/b, 1/c, 1/d]
	Matrix4ld RInvTmp = U * ((s.array().pow(-1)).matrix().asDiagonal()) * U.transpose();

	Matrix4ld D1 = MatrixXld::Zero(4,4);	// Initialize D1 to zero matrix
	for (i = 0; i < pc.cols(); i++) {
		Matrix4ld G_tmp = qr_kf_measurementFunctionJacobian(pc.col(i), pr.col(i));
		D1 = D1 + G_tmp.transpose()*RInvTmp*G_tmp;
	}

	Matrix4ld D2 = MatrixXld::Zero(4,4);	// Initialize D2 to zero matrix

	if(normalc != NULL && normalr != NULL) {
		Matrix4ld QInvTmp = RInvTmp * Rmag / Qmag;
		for (i = 0; i < (*normalc).cols(); i++) {
			Matrix4ld H_tmp = qr_kf_measurementFunctionJacobian((*normalc).col(i), 
																(*normalr).col(i));
			D2 = D2 + H_tmp.transpose()*QInvTmp*H_tmp;
		}
	}

	Matrix4ld DStar = -0.5*D1 - 0.5*D2 + (*Mk)*(*Zk)*(*Mk).transpose();
	
	es.compute(DStar, true);
	
	VectorXld ZTmp = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXld MTmp = es.eigenvectors().real();	// A matrix whose column vectors are eigen 
												// vectors of RTmp
	
	// Normalize MTmp
	for (i = 0; i < U.cols(); i++)
		MTmp.col(i).norm();

	// Convert ZTmp to std::vector so we can call the sort function
	vector<long double> ZTmpSTD(ZTmp.data(), ZTmp.data() + ZTmp.size());

	vector<unsigned int> indx = sort_indexes(ZTmpSTD, false);
	
	VectorXld ZTmpSorted(ZTmp.size());

	for (i = ZTmp.size()-1; i > 0; i--)
		ZTmpSorted(i) = ZTmp(indx[i]) - ZTmp(indx[0]);

	// This step should ensure that Zk has first diagonal element = 0
	ZTmpSorted(0) = 0;

	*Zk = ZTmpSorted.asDiagonal();
	
	// Since we sorted and changed he order of the elements in Zk, we have to do the same
	// change of orders for columns in Mk
	(*Mk).col(0) = MTmp.col(indx[0]);
	*Xk = (*Mk).col(0);	// Update Xk

	for (i = 1; i < ZTmp.size(); i++)
		(*Mk).col(i) = MTmp.col(indx[i]);
	
	// Calculate translation vector from rotation estimate
    // quat2rotm converts quaternion to rotation matrix.
    Vector3ld centroid;
    Vector3ld centroidRotated;
    for(int i=0; i<3; i++) {
        centroid(i) = ((*p1c).row(i).mean() + (*p2c).row(i).mean()) / 2.0;
        centroidRotated(i) = ((*p1r).row(i).mean() + (*p2r).row(i).mean()) / 2.0;
    }
    Quaternionld XkQuat = Quaternionld((*Xk)(0), (*Xk)(1), (*Xk)(2), (*Xk)(3)).normalized();
    
    centroidRotated = XkQuat.toRotationMatrix() * centroidRotated;
    Vector3ld eulerRotation = quat2eul(XkQuat);
    Vector3ld centroidDifference = centroid - centroidRotated;

    VectorXld Xreg(6);
    Xreg.segment(0,3) = centroidDifference;
    Xreg.segment(3,3) = eulerRotation;
    
    BinghamKFResult result;

    result.Xk = *Xk;
    result.Xreg = Xreg;
    result.Mk = *Mk;
    result.Zk = *Zk;

    return result;
}