#include <Eigen/Dense>
#include <Eigen/StdVector>
#include "KDTree.h"
#include "get_changes_in_transformation_estimate.h"
#include "qr_kf.h"
#include "compute_transformed_points.h"

#include <long_double_def.h>

// For the return type
struct RegistrationResult{
    VectorXld Xreg;
    MatrixXld Xregsave;
};

struct RegistrationResult registration_est_kf_rgbd(PointCloud ptcldMoving, PointCloud ptcldFixed);