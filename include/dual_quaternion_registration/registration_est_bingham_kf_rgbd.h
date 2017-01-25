#ifndef REGISTRATION_EST_BINGHAM_KF_RGBD
#define REGISTRATION_EST_BINGHAM_KF_RGBD

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <kd_tree.h>
#include <get_changes_in_transformation_estimate.h>
#include <bingham_kf.h>
#include <compute_transformed_points.h>

#include <long_double_def.h>

struct RegistrationResult{
    VectorXld Xreg;
    MatrixXld Xregsave;
};

extern "C" struct RegistrationResult *registration_est_bingham_kf_rgbd(PointCloud *ptcldMoving, PointCloud *ptcldFixed);

extern "C" struct RegistrationResult *registration_est_bingham_normal(PointCloud *ptcldMoving, PointCloud *ptcldFixed,
														   PointCloud *normalMoving, PointCloud *normalFixed);
#endif