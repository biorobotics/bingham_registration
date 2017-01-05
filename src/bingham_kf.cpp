/*
 * File Header:
 * This file contains functions for performing Bingham quaternion filtering
 */

#include <iostream>
#include <KDTree.h>
#include <bingham_kf.h>
#include <algorithm>
#include <iterator>

using namespace std;
using namespace Eigen;


Vector3d quat2eul(Quaterniond q) {
    // Normalize the quaternions

    Quaterniond temp = q.normalized();
    q = temp;
    
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    Vector3d eul(3);

    eul(0) = atan2(2 * (qx * qy + qw * qz), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
    eul(1)= asin(-2 * (qx * qz - qw * qy));
    eul(2) = atan2(2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));

    return eul;
}

Vector4d qr_kf_measurementFunction(Vector4d Xk, Vector3d p1, Vector3d p2) {
    /* Xk if size 4x1
    * p1, p2 is of size 1x3
    * g is of size 4x1
    */

    Vector4d g = Vector4d::Zero();
    g(0) = Xk(1)*(p2(0)-p1(0))+Xk(2)*(p2(1)-p1(1))+Xk(3)*(p2(2)-p1(2));
    g(1) = Xk(0)*(p1(0)-p2(0))-Xk(2)*(p1(2)+p2(2))+Xk(3)*(p1(1)+p2(1));
    g(2) = Xk(0)*(p1(1)-p2(1))+Xk(1)*(p1(2)+p2(2))-Xk(3)*(p1(0)+p2(0));
    g(3) = Xk(0)*(p1(2)-p2(2))-Xk(1)*(p1(1)+p2(1))+Xk(2)*(p1(0)+p2(0));
    return g;
}

Matrix4d qr_kf_measurementFunctionJacobian(Vector3d p1, Vector3d p2) {
    /*p1, p2 is of size 1x3
    *H is of size 4x4 */
    Matrix4d H;

    H(0,0) = (double)0;
    H(0,1) = p2(0)-p1(0);
    H(0,2) = p2(1)-p1(1);
    H(0,3) = p2(2)-p1(2);

    H(1,0) = p1(0)-p2(0);
    H(1,1) = (double)0;
    H(1,2) = -(p1(2)+p2(2));
    H(1,3) = p1(1)+p2(1);

    H(2,0) = p1(1)-p2(1);
    H(2,1) = p1(2)+p2(2);
    H(2,2) = (double)0;
    H(2,3) = -(p1(0)+p2(0));

    H(3,0) = p1(2)-p2(2);
    H(3,1) = -(p1(1)+p2(1));
    H(3,2) = p1(0)+p2(0);
    H(3,3) = (double)0;

    return H;
}

struct BinghamKFResult bingham_kf(Vector4d Xk, Matrix4d Mk, Matrix4d Zk, 
								  double Rmag, PointCloud p1c, PointCloud p1r, 
								  PointCloud p2c, PointCloud p2r) {

/*	if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	{
	cout << "Xk = : " << Xk << endl;
	cout << "Mk = : " << Mk << endl;
	cout << "Zk = : " << Zk << endl;
	cout << "Rmag = : " << Rmag << endl;
	cout << "p1c = : " << p1c << endl;
	cout << "p1r = : " << p1r << endl;
	cout << "p2c = : " << p2c << endl;
	cout << "p2r = : " << p2r << endl;
}*/



	int i;
	PointCloud pc = p1c - p2c;
	PointCloud pr = p1r - p2r;
	/*c is the smallest diagonal value of Zk. remember Zk is a diagonal matrix
	 with all diagonal elements being negative except for first entry which is
	 0 */
	double c = Zk.minCoeff();
	//if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	//cout << "c = : " << c << endl; 
	Matrix4d temp = Mk * (Zk + c*MatrixXd::Identity(4,4))*(Mk.transpose());
/*if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	cout << "temp = : " << temp << endl;*/

	Matrix4d Pk = -0.5 * (temp.inverse());
/*	if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	cout << "Pk = : " << Pk << endl;*/

	//cout << "Pk = : " << Pk << endl;
	Matrix4d Nk = Xk * Xk.transpose() + Pk;
/*		if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	cout << "Nk = : " << Nk << endl;*/
	Matrix4d RTmp = Rmag * (Nk.trace()*MatrixXd::Identity(4,4) - Nk);
/*		if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	cout << "RTmp = : " << RTmp << endl;*/


	EigenSolver<Matrix4d> es(RTmp);
	
	VectorXd s = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXd U = es.eigenvectors().real();	// A matrix whose column vectors are eigen vectors of RTmp
	

	// Normalize U
	/*for (i = 0; i < U.cols(); i++)
	{
		U.col(i).norm();
	}*/

	// A step to make sure every column in U is unit vector
/*	for (i = 0; i < U.cols(); i++) {
		U.col(i) = 
	}*/

    //cout << "s = : " << s << endl;
    //cout << "U = : " << U << endl;

	// If any elements in s <=10^-4, then set it to be equal to 1

	for (i = 0; i < s.size(); i++) {
		if (s(i) <= .0001)
			s(i) = 1;
	}

	// s.^-1 is essentially takes s=[a,b,c,d] and returns [1/a, 1/b, 1/c, 1/d]

//	clock_t begin = clock();
	Matrix4d RInvTmp = U * ((s.array().pow(-1)).matrix().asDiagonal()) * U.transpose();
/*	clock_t end = clock();  
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   	cout << "Before " << elapsed_secs << " seconds." << endl;*/

/*	if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
	cout << "RInvTmp = : " << RInvTmp << endl; */

	//clock_t begin = clock();
	Matrix4d D1 = MatrixXd::Zero(4,4);	// Initialize D1 to zero matrix
	for (i = 0; i < pc.cols(); i++) {
		Matrix4d G_tmp = qr_kf_measurementFunctionJacobian(pc.col(i), pr.col(i));
		D1 = D1 + G_tmp.transpose()*RInvTmp*G_tmp;
	}


	Matrix4d DStar = -0.5 * D1 + Mk * Zk * Mk.transpose();
	
	es.compute(DStar, true);
	// This part might have problem
	VectorXd ZTmp = es.eigenvalues().real();  // A vector whose entries are eigen values of RTmp
	MatrixXd MTmp = es.eigenvectors().real();	// A matrix whose column vectors are eigen vectors of RTmp
	
	// Normalize MTmp
/*	for (i = 0; i < U.cols(); i++)
	{
		MTmp.col(i).norm();
	}*/

	// Convert ZTmp to std::vector so we can call the sort function
	vector<double> ZTmpSTD(ZTmp.data(), ZTmp.data() + ZTmp.size());
	
		//cout << "ZTmpSTD: " << endl;
		//std::copy(ZTmpSTD.begin(), ZTmpSTD.end(), std::ostream_iterator<double>(std::cout, " "));
	
	vector<size_t> indx = sort_indexes(ZTmpSTD, false);
	
		//cout << "indx is: " << endl;
		//std::copy(indx.begin(), indx.end(), std::ostream_iterator<size_t>(std::cout, " "));
	
	VectorXd ZTmpSorted(ZTmp.size());


	for (i = ZTmp.size()-1; i >= 0; i--)
	{
		ZTmpSorted(i) = ZTmp(indx[i]) - ZTmp(indx[0]);
	}

	//cout << "ZTmpSorted is " << ZTmpSorted << endl;
	
	// This step should ensure that Zk has first diagonal element = 0
	ZTmpSorted(0) = 0;
	
	//cout << "ZTmpSorted after is " << ZTmpSorted << endl;
	Zk = ZTmpSorted.asDiagonal();
	
	//cout << "Updated Zk is " << Zk << endl;
	// Since we sorted and changed he order of the elements in Zk, we have to do the same
	// change of orders for columns in Mk
	
	for (i = 0; i < ZTmp.size(); i++)
		Mk.col(i) = MTmp.col(indx[i]);

	//	//cout << "Updated Mk is " << Mk << endl;
	Xk = Mk.col(0);	// Update Xk

	// Calculate translation vector from rotation estimate
    // quat2rotm converts quaternion to rotation matrix.
    Vector3d centroid;
    for(int i=0; i<3; i++) {
        centroid(i) = (p1c.row(i).mean() + p2c.row(i).mean())/2.0;
    }
    Quaterniond XkQuat = Quaterniond(Xk(0),Xk(1),Xk(2),Xk(3)).normalized();
    
    Vector3d centroidRotated;
    for(int i=0; i<3; i++) {
        centroidRotated(i) = (p1r.row(i).mean() + p2r.row(i).mean())/2.0;
    }
    centroidRotated = XkQuat.toRotationMatrix() * centroidRotated;
    Vector3d eulerRotation = quat2eul(XkQuat);
    Vector3d centroidDifference = centroid - centroidRotated;
    ////cout << "t' is " << centroidDifference << endl;
 //   //cout << "eulerRotation is " << eulerRotation << endl;
    // Estimated pose parameters  (x,y,z,alpha,beta,gamma)
    VectorXd Xreg(6);
    Xreg.segment(0,3) = centroidDifference;
    Xreg.segment(3,3) = eulerRotation;
    
    struct BinghamKFResult result;
    result.Xk = Xk;
    result.Xreg = Xreg;
    result.Mk = Mk;
    result.Zk = Zk;
 /*   if (Rmag >  7.28719 - .1 && Rmag < 7.28719 + .1)
    {
     cout << "Updated Xk is " << result.Xk << endl;
     cout << "Updated Xreg is " << result.Xreg << endl;
     cout << "Updated Mk is " << result.Mk << endl;
     cout << "Updated Zk is " << result.Zk << endl;
}*/
    return result;
}
