#ifndef TYPE_DEFS
#define TYPE_DEFS
#include <Eigen/Dense>
typedef Eigen::Matrix<float, 3, Eigen::Dynamic> PointCloud;
typedef Eigen::Matrix<long double, Eigen::Dynamic, Eigen::Dynamic> MatrixXld;
typedef Eigen::Matrix<long double, 4, 4> Matrix4ld;
typedef Eigen::Matrix<long double, 3, 3> Matrix3ld;
typedef Eigen::Array<long double, Eigen::Dynamic, 1> ArrayXld;
typedef Eigen::Array<long double, 4, 1> Array4ld;
typedef Eigen::Array<long double, 3, 1> Array3ld;
typedef Eigen::Matrix<long double, Eigen::Dynamic, 1> VectorXld;
typedef Eigen::Matrix<long double, 3, 1> Vector3ld;
typedef Eigen::Matrix<long double, 4, 1> Vector4ld;
typedef Eigen::Quaternion<long double> Quaternionld;
typedef Eigen::Transform<long double, 3, Eigen::Affine> Affine3ld;
#endif
