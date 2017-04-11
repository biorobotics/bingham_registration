#ifndef REGISTRATION_EST_KF_RGBD
#define REGISTRATION_EST_KF_RGBD

#include <Eigen/Dense>
#include <type_defs.h>

// For the return type
struct RegistrationResult{
    VectorXld Xreg;
    MatrixXld Xregsave;
	double error;
};

extern "C" struct RegistrationResult *registration_est_kf_rgbd(PointCloud *ptcldMoving, PointCloud *ptcldFixed,
															   double inlierRatio, int maxIterations, int windowSize,
															   double toleranceT, double toleranceR,
															   double uncertaintyR);

extern "C" struct RegistrationResult *registration_est_bingham_kf_rgbd(PointCloud *ptcldMoving, PointCloud *ptcldFixed,
	double inlierRatio, int maxIterations, int windowSize,
	double toleranceT, double toleranceR,
	double uncertaintyR);

extern "C" struct RegistrationResult *registration_est_bingham_normal(PointCloud *ptcldMoving, PointCloud *ptcldFixed,
	PointCloud *normalMoving, PointCloud *normalFixed,
	double inlierRatio, int maxIterations, int windowSize,
	double toleranceT, double toleranceR,
	double uncertaintyR);
#endif