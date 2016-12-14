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

    q = q.normalized();
    
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

Array4d qr_kf_measurementFunction(Vector4d Xk, Vector3d p1, Vector3d p2) {
    /* Xk if size 4x1
    * p1, p2 is of size 1x3
    * g is of size 4x1
    */

    Array4d g;

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

struct triple2 qr_kf(Vector4d Xk, Matrix4d Pk, double Rmag, PointCloud p1c,
                     PointCloud p1r, PointCloud p2c, PointCloud p2r) {
    /*Xk is of size 4x1  
     *Pk is of size 4x4
     *Rmag is a constant scalar
     *p1c, p1r, p2c, p2r are points of size 3xn
     *Xreg is of size 1 x 6 */
    
    // Check for input dimensions 
    if (Xk.size() != 4)
        cerr << "Xk has wrong dimension. Should be 4x1";
    if (Pk.rows() != 4 || Pk.cols() != 4)
        cerr << "Pk has wrong dimension. Should be 4x4";
    if (p1c.cols() == p1r.cols() || p1c.cols() != p2c.cols() || p1c.cols() != p2c.cols())
        cerr << "pxx are not equal in size";
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
    MatrixXd z = MatrixXd::Zero(1, dim);
    // R is pseudo measurement uncertainty
    MatrixXd R = MatrixXd::Zero(dim, dim);
    // g is estimated measurement
    MatrixXd g = MatrixXd::Zero(1, dim);
    // G is measurement Jacobian. Which is trivial for linear functions
    MatrixXd G = MatrixXd::Zero(4, dim);

    // find estimated measurements and uncertainties over all sensed points
    // considered. In future avoid for loop
    for (int i=1; i<=nPoints; i++){
        Vector3d pcPoint = pc.col(i-1);
        Vector3d prPoint = pr.col(i-1);
        int idx = 4*i-4;
        g.block(0, idx, 4, 1) = qr_kf_measurementFunction(Xk, pcPoint, prPoint);
        G.block(0, idx, 4, 4) = qr_kf_measurementFunctionJacobian(pcPoint, prPoint);
        R.block(idx, idx, 3, 3) = Rmag * (M.sum() * Matrix4d::Identity() - M);
    }

    // Kalman gain computation
    MatrixXd K = Pk * G * (G.transpose()*Pk*G + R).inverse();    

    // state update
    Array4d temp4x1 = Xk.matrix() + K*(z-g).transpose();
    Xk = temp4x1;
    
    // check for double covering of quaternions. This step can actually be avoided
    if (Xk(0) < 0)
        Xk = -Xk;
    // uncertainty update
    Pk = Pk - K*G.transpose()*Pk;
    
    // Calculate translation vector from rotation estimate
    // quat2rotm converts quaternion to rotation matrix.
    Vector3d centroid;
    for(int i=0; i<3; i++) {
        centroid(i) = (p1r.row(i).mean() + p2r.row(i).mean())/2.0;
    }
    Quaterniond XkQuat = Quaterniond(Xk(0),Xk(1),Xk(2),Xk(3));
    
    Vector3d centroidTransformed = XkQuat.matrix() * centroid;
    Vector3d eulerRotation = quat2eul(XkQuat);
    Vector3d centroidDifference = centroid - centroidTransformed;
    
    // Estimated pose parameters  (x,y,z,alpha,beta,gamma)
    VectorXd Xreg(6);
    Xreg.block(0,0,3,1) = centroidDifference;
    Xreg.block(3,0,3,1) = eulerRotation;
    
    struct triple2 result;
    result.Xk = Xk;
    result.Pk = Pk;
    result.Xreg = Xreg;

    return result;
}

#if 0
using namespace std;
using namespace arma;

// For return type
struct triple2 {
    mat Xk;
    mat Pk;
    rowvec Xreg;
};

mat quat2rotm(rowvec q) {
    // Normalize and transpose the quaternions
    vec qT = vec(q.n_elem);

    for (int i = 0; i < q.n_elem; i++)    // Vector transpose 
        qT(i) = q(i);

    qT = qT / norm(q);    // Turns q into a colvec

    // Reshape the quaternions in the depth dimension
    // q = reshape(q, 4, 1, q.n_cols);

    double s = qT(0);  
    double x = qT(1);
    double y = qT(2);
    double z = qT(3);
   
    mat R = mat(3, 3);
    R(0, 0) = 1 - 2 * (pow(y, 2) + pow(z, 2));
    R(0, 1) = 2 * (x * y - s * z);
    R(0, 2) = 2 * (x * z + s * y);
    R(1, 0) = 2 * (x * y + s * z);
    R(1, 1) = 1 - 2 * (pow(x, 2) + pow(z, 2));
    R(1, 2) = 2 * (y * z - s * x);
    R(2, 0) = 2 * (x * z - s * y);
    R(2, 1) = 2 * (y * z + s * x);
    R(2, 2) = 1 - 2 * (pow(x, 2) + pow(y, 2));

    return R;
}

rowvec quat2eul(rowvec q) {
    // Normalize the quaternions

    q = q / norm(q, 2);

    double qw = q(0);
    double qx = q(1);
    double qy = q(2);
    double qz = q(3);

    rowvec eul = rowvec(3);

    eul(0) = atan2(2 * (qx * qy + qw * qz), pow(qw, 2) + pow(qx, 2) - pow(qy, 2) - pow(qz, 2));
    eul(1)= asin(-2 * (qx * qz - qw * qy));
    eul(2) = atan2(2 * (qy * qz + qw * qx), pow(qw, 2) - pow(qx, 2) - pow(qy, 2) + pow(qz, 2));

    return eul;
}

vec qr_kf_measurementFunction(vec Xk, rowvec p1, rowvec p2) {
    /* Xk if size 4x1
    * p1, p2 is of size 1x3
    * g is of size 4x1
    */

    vec g = vec(4, fill::zeros);

    g(0) = Xk(1)*(p2(0)-p1(0))+Xk(2)*(p2(1)-p1(1))+Xk(3)*(p2(2)-p1(2));
    g(1) = Xk(0)*(p1(0)-p2(0))-Xk(2)*(p1(2)+p2(2))+Xk(3)*(p1(1)+p2(1));
    g(2) = Xk(0)*(p1(1)-p2(1))+Xk(1)*(p1(2)+p2(2))-Xk(3)*(p1(0)+p2(0));
    g(3) = Xk(0)*(p1(2)-p2(2))-Xk(1)*(p1(1)+p2(1))+Xk(2)*(p1(0)+p2(0));
    return g;
}

mat qr_kf_measurementFunctionJacobian(rowvec p1, rowvec p2) {
    /*p1, p2 is of size 1x3
    *H is of size 4x4 */
    mat H = mat(4, 4, fill::zeros);

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

struct triple2 qr_kf(vec Xk, mat Pk, double Rmag, mat p1c, mat p1r, mat p2c, mat p2r) {
	/*Xk is of size 4x1  
     *Pk is of size 4x4
     *Rmag is a constant scalar
     *p1c, p1r, p2c, p2r are points of size nx3
     *Xreg is of size 1 x 6 */

    // Check for input dimensions 
    if (Xk.n_elem != 4)
        call_error("Xk has wrong dimension. Should be 4x1");
    if (Pk.n_rows != 4 || Pk.n_cols != 4)
        call_error("Pk has wrong dimension. Should be 4x4");
    if (p1c.n_rows != p1r.n_rows || p2c.n_rows != p2r.n_rows || p1c.n_rows != p2c.n_rows 
        || p1c.n_cols != 3 || p1r.n_cols != 3 || p2c.n_cols != 3 || p2r.n_cols != 3)
        call_error("pxx has wrong dimension. Should be nx3");
	int n = p1c.n_rows;
	mat pc = p1c - p2c;
	mat pr = p1r - p2r;

	//dim is the total number of point pairs that we have
	int dim = 4 * n;

	// Add process uncertainty based on correpondence uncertainty. Details yet to
    // be published
	mat identity = mat(4, 4, fill::eye);
	Pk = Pk + Rmag * (identity - Xk*Xk.t());
	Pk.row(0) = rowvec(4, fill::zeros);
	Pk.col(0) = colvec(4, fill::zeros);

	// Scaled pseudo-measurement uncertainty. Details in RSS 2016 paper
    // M has dimension 4 x 4
    mat temp = Xk * Xk.t();
    mat M = Xk * Xk.t() + Pk;

    /* z is pseudo measurement */
    mat z = mat(dim, 1, fill::zeros);

    /* R is pseudo measurement uncertainty */
    mat R = mat(dim, dim, fill::zeros);

    /* g is estimated measurement */
    vec g = vec(dim, fill::zeros);

    /* G is measurement Jacobian. Which is trivial for linear functions */
    mat G = mat(dim, 4, fill::zeros);


    /* find estimated measurements and uncertainties over all sensed points
     considered. In future avoid for loop */
    for (int i=1; i<=pc.n_rows; i++){
    	g.subvec(4*i-4, 4*i-1) = qr_kf_measurementFunction(Xk, pc.row(i-1), pr.row(i-1));
    	G.submat(4*i-4, 0, 4*i-1, G.n_cols-1) = 
            qr_kf_measurementFunctionJacobian(pc.row(i-1), pr.row(i-1));
    	R.submat(4*i-4, 4*i-4, 4*i-1, 4*i-1) = Rmag * (trace(M) * identity - M);
    }

    /* Kalman gain computation */
    mat K = Pk * G.t() * inv(G*Pk*G.t() + R);
    /* state update */
    Xk = Xk + K*(z-g);

    /* check for double covering of quaternions. This step can actually be avoided */
    if (Xk(0) < 0)
    	Xk = -Xk;
    /* uncertainty update */
    Pk = Pk - K*G*Pk;

    // Calculate translation vector from rotation estimate
    // quat2rotm converts quaternion to rotation matrix.
    vec temp1 = mean(join_cols(p1c, p2c)).t();  // temp1 is colvec
    rowvec XkT = rowvec(Xk.n_elem);
    
    for (int i = 0; i < Xk.n_elem; i++) {   // Vector transpose 
        XkT(i) = Xk(i);
    }
    vec temp2 = quat2rotm(XkT) * (mean(join_cols(p1r, p2r)).t());    // temp2 is colvec

    rowvec temp3 = quat2eul(XkT);
    vec t = temp1 - temp2;
    
    // Estimated pose parameters  (x,y,z,alpha,beta,gamma)
    rowvec Xreg = rowvec(t.n_elem + temp3.n_elem);
    rowvec tTranspose = t.t();
    rowvec eul = quat2eul(Xk.t());

    for (int i = 0; i < t.n_elem; i++) {
        Xreg(i) = tTranspose(i);
    }

    for (int i = 0; i < temp3.n_elem; i++) {
        Xreg(i + t.n_elem) = eul(i);
    }

    struct triple2 result;
    result.Xk = Xk;
    result.Pk = Pk;
    result.Xreg = Xreg;

    return result;
}
#endif