#ifndef REGISTRATION_EST_BINGHAM_KF_RGBD
#define REGISTRATION_EST_BINGHAM_KF_RGBD

#include "type_defs.h"
#include "kd_tree.h"

struct RegistrationResult{
    VectorXld Xreg; //6x1 vector containing x, y, z position followed by x,y,z rotation
    MatrixXld Xregsave;
    double error;
};

RegistrationResult bingham_registration(const PointCloud& ptcldMoving, const PointCloud& ptcldFixed,
                                            double inlierRatio, int maxIterations, int windowSize,
                                            double toleranceT, double toleranceR,
                                            double uncertaintyR, KDTree tree = NULL);
#endif