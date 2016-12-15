/*
 * File Header:
 * This file contains functions for performing quaternion filtering
 */

#include <iostream>
#include <KDTree.h>
#include <qr_kf.h>

using namespace std;
using namespace Eigen;

Array3d quat2eul(Quaterniond q) {
    // Normalize the quaternions

    Quaterniond temp = q.normalized();
    q = temp;
    
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    Array3d eul(3);

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

struct QrKfResult qr_kf(Vector4d Xk, Matrix4d Pk, double Rmag, PointCloud p1c,
                        PointCloud p1r, PointCloud p2c, PointCloud p2r) {
    /*Xk is of size 4x1  
     *Pk is of size 4x4
     *Rmag is a constant scalar
     *p1c, p1r, p2c, p2r are points of size 3xn
     *Xreg is of size 1 x 6 */
    
    // Check for input dimensions 
    if (Xk.size() != 4) {
        cerr << "Xk has wrong dimension. Should be 4x1\n";
    }
    if (Pk.rows() != 4 || Pk.cols() != 4) {
        cerr << "Pk has wrong dimension. Should be 4x4\n";
    }
    if (p1c.cols() != p1r.cols() || p1c.cols() != p2c.cols() || p1c.cols() != p2c.cols()) {
        cerr << "pxx are not equal in size\n";
    }
    int nPoints = p1c.cols();
    PointCloud pc = p1c - p2c;
    PointCloud pr = p1r - p2r;
    //dim is the total number of point pairs that we have
    int dim = 4 * nPoints;

    // Add process uncertainty based on correpondence uncertainty. Details yet
    // to be published
    Matrix4d XkSquare = Xk*Xk.transpose();
    Matrix4d temp4x4 = Pk + Rmag * (Matrix4d::Identity() - XkSquare);
    Pk = temp4x4;
    Pk.row(0) << 0, 0, 0, 0;
    Pk.col(0) << 0, 0, 0, 0;
    
    // Scaled pseudo-measurement uncertainty. Details in RSS 2016 paper
    // M has dimension 4 x 4
    Matrix4d M =  XkSquare + Pk;
    // z is pseudo measurement
    MatrixXd z = MatrixXd::Zero(dim, 1);
    // R is pseudo measurement uncertainty
    MatrixXd R = MatrixXd::Zero(dim, dim);
    // g is estimated measurement
    VectorXd g = VectorXd::Zero(dim);
    // G is measurement Jacobian. Which is trivial for linear functions
    MatrixXd G = MatrixXd::Zero(dim,4);

    Vector3d pcPoint;
    Vector3d prPoint;

    // find estimated measurements and uncertainties over all sensed points
    // considered. In future avoid for loop
    for (int i=1; i<=nPoints; i++){
        pcPoint = pc.col(i-1);
        prPoint = pr.col(i-1);
        int idx = 4*i-4;
        g.segment(idx,4) = qr_kf_measurementFunction(Xk, pcPoint, prPoint);
        G.block(idx, 0, 4, 4) = qr_kf_measurementFunctionJacobian(pcPoint, prPoint);
        R.block(idx, idx, 4, 4) = Rmag * (M.trace() * Matrix4d::Identity() - M);
    }
    
    // Kalman gain computation
    MatrixXd K = Pk * G.transpose() * (G*Pk*G.transpose() + R).inverse();

    // state update
    Array4d temp4x1 = Xk.matrix() + K*(z-g);
    Xk = temp4x1;

    // check for double covering of quaternions. This step can actually be avoided
    if (Xk(0) < 0) {
        Xk = -Xk;
    }

    // uncertainty update
    Pk = Pk - K*G*Pk;
    
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
    
    // Estimated pose parameters  (x,y,z,alpha,beta,gamma)
    VectorXd Xreg(6);
    Xreg.segment(0,3) = centroidDifference;
    Xreg.segment(3,3) = eulerRotation;
    
    struct QrKfResult result;
    result.Xk = Xk;
    result.Pk = Pk;
    result.Xreg = Xreg;

    return result;

}