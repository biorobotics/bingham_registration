#ifndef REGISTRATION_EST_BINGHAM_NORMAL
#define REGISTRATION_EST_BINGHAM_NORMAL

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "kd_normal_tree.h"
#include "get_changes_in_transformation_estimate.h"
#include "bingham_normal_kf.h"
#include "compute_transformed_points.h"

#include <long_double_def.h>

// For the return type
struct RegistrationResult{
    VectorXld Xreg;
    MatrixXld Xregsave;
};

struct RegistrationResult registration_est_bingham_normal(PointCloud ptcldMoving, PointCloud ptcldFixed,
														   PointCloud normalMoving, PointCloud normalFixed);

#endif