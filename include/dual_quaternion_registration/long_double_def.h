/*
 * File header:
 		These are customized definitions for long double type application in Eigen
 */

#ifndef LONG_DOUBLE_DEF
#define LONG_DOUBLE_DEF

#include <Eigen/Dense>
using namespace Eigen;


typedef Matrix<long double, Dynamic, Dynamic> MatrixXld;
typedef Matrix<long double, 4, 4> Matrix4ld;
typedef Matrix<long double, 3, 3> Matrix3ld;
typedef Array<long double, Dynamic, 1> ArrayXld;
typedef Array<long double, 4, 1> Array4ld;
typedef Array<long double, 3, 1> Array3ld;
typedef Matrix<long double, Dynamic, 1> VectorXld;
typedef Matrix<long double, 3, 1> Vector3ld;
typedef Matrix<long double, 4, 1> Vector4ld;
typedef Quaternion<long double> Quaternionld;
typedef Transform<long double, 3, Affine> Affine3ld;

#endif