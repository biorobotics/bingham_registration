#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <Eigen/Dense>
#include <iostream>
#include "type_defs.h"
#include "registration_estimation.h"
#include "conversions.h"
#include "bingham_registration/RegistrationService.h"

PointCloud pclToEigen(const pcl::PCLPointCloud2& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(input, *cloudPtr);
  Eigen::MatrixXf mat = cloudPtr->getMatrixXfMap(3,4,0); // Get just XYZ point
  PointCloud points(3,mat.cols());
  points << mat.cast<long double>();
  return points;
}

Affine3ld poseMsgToEigen(const geometry_msgs::Pose &msg){
  Affine3ld tEigen;
  // // Set rotation
  Quaternionld quat = Quaternionld( msg.orientation.x,
                                    msg.orientation.y,
                                    msg.orientation.z,
                                    msg.orientation.w );
  tEigen.rotate(quat);
  // Set position
  tEigen.translate(Vector3ld( msg.position.x, msg.position.y, msg.position.z ));
  return tEigen;
}

geometry_msgs::Pose eigenToPoseMsg(const Affine3ld &tEigen){
  geometry_msgs::Pose msg;
  // Set position
  msg.position.x = tEigen.translation().x();
  msg.position.y = tEigen.translation().y();
  msg.position.z = tEigen.translation().z();
  // Set rotation
  Quaternionld quat = Quaternionld(tEigen.rotation());
  msg.orientation.x = quat.x();
  msg.orientation.y = quat.y();
  msg.orientation.z = quat.z();
  msg.orientation.w = quat.w();
  return msg;
}

/* affineFromXreg:
 *    Input: Xreg 6x1 array containing x,y,z position and x,y,z euler rotation
 *    Output: Eigen::Affine3 representing Xreg
 */
Affine3ld affineFromXreg(const ArrayXld& Xreg) {
  Matrix4ld tMatrix = reg_params_to_transformation_matrix (Xreg.segment(0,6));
  Affine3ld t(tMatrix);
  return t;
}

/* loadMesh
 *    loads an stl file specified at filePath as a PointCloud (Eigen::Matrix)
 */
PointCloud loadMesh(const std::string &filePath)
{
  // Load in STL file into the ptcldFixed variable
  cout << "Loading stl from " << filePath << endl << endl;
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFileSTL(filePath, mesh) == 0)
  {
    PCL_ERROR("Failed to load STL file\n");
  }
  return pclToEigen(mesh.cloud);
}


class RosRegistration
{

private:
  PointCloud ptcldFixed;
  PointCloud ptcldSensed;
  PointCloud ptcldTransformed;
  Affine3ld lastTransform;
  double lastError;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool process(bingham_registration::RegistrationService::Request &req,
               bingham_registration::RegistrationService::Response &res)
  {
    // Get sensed data
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(req.PtCldSensed,pcl_pc2);
    ptcldSensed = pclToEigen(pcl_pc2);
    // Get fixed data
    if(req.StlFile != "")
      ptcldFixed = loadMesh(req.StlFile);
    else{
      pcl_conversions::toPCL(req.PtCldFixed,pcl_pc2);
      PointCloud ptcldNew = pclToEigen(pcl_pc2);
      if(ptcldNew.cols() != 0)
        ptcldFixed = ptcldNew;
    }
    // Check for no data
    if(ptcldSensed.cols() == 0 || ptcldFixed.cols() == 0)
      return false;

    // Get starting transform
    Affine3ld lastTransform;
    if(req.Reset == false)
      lastTransform = poseMsgToEigen(req.StartPose);
    else
      lastTransform = guessBestTransform();
    // Transform fixed data
    ptcldTransformed = lastTransform * ptcldFixed;

    // Get error
    lastError = req.Uncertainty;

    update();

    // return
    res.Error = lastError;
    res.FinalPose = eigenToPoseMsg(lastTransform);

    return true;
  }

  void update(){
    double uncertainty = 10*std::min(1.0,lastError);
    // Perform registration
    RegistrationResult result = registration_est_kf_rgbd(&ptcldSensed,
                                                         &ptcldTransformed,
                                                         1, 100, 20, .00001,
                                                         .00001, uncertainty);
    // Get error
    lastError = result.error;
    // Get transform
    lastTransform = affineFromXreg(result.Xreg).inverse() * lastTransform;
  }

  Affine3ld guessBestTransform()
  {
    // Find a suitable initial rotation
    if(ptcldSensed.cols() >= 2)
    {      
      double sqrt = 1/std::sqrt(2.0);
      const int numQuats = 24;
      double quats[numQuats][4] =
      { { sqrt,  0.00,  0.00, sqrt},
        {-sqrt,  0.00,  0.00, sqrt},
        { 0.00,  sqrt,  0.00, sqrt},
        { 0.00, -sqrt,  0.00, sqrt},
        { 0.00,  0.00,  sqrt, sqrt},
        { 0.00,  0.00, -sqrt, sqrt},
        { 0, 0, 0, 1},
        { 1, 0, 0, 0},
        { 0, 1, 0, 0},
        { 0, 0, 1, 0},
        {-0.5,-0.5,-0.5, 0.5},
        {-0.5,-0.5, 0.5, 0.5},
        {-0.5, 0.5,-0.5, 0.5},
        {-0.5, 0.5, 0.5, 0.5},
        { 0.5,-0.5,-0.5, 0.5},
        { 0.5,-0.5, 0.5, 0.5},
        { 0.5, 0.5,-0.5, 0.5},
        { 0.5, 0.5, 0.5, 0.5},
        {0,sqrt,sqrt,0},
        {0,sqrt,-sqrt,0},
        {sqrt,0,sqrt,0},
        {sqrt,0,-sqrt,0},
        {sqrt,sqrt,0,0},
        {sqrt,-sqrt,0,0}
      };

      lastError = 1;
      double bestError = 1;
      int numTrials = 2;
      Affine3ld bestTransform(Affine3ld::Identity());
      for(int i=0; i<numQuats; i++){
        Quaternionld rot(quats[i][0], quats[i][1], quats[i][2], quats[i][3]);
        lastTransform = rot*Affine3ld::Identity();
        lastTransform.translate(Vector3ld(0, 0, 10));
        for(int j=0; j<numTrials; j++)
          update();
        if(lastError<bestError){
          bestError = lastError;
          bestTransform = lastTransform;
        }
      }
      return bestTransform;
    }
  }
};

main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "bingham_registration_server");
  ros::NodeHandle n;
  std::string filepath = "/home/biomed/registration_ws/src/dvrk_vision/scripts/fixed.stl";
  PointCloud stlPoints;
  RosRegistration registration;
  ros::ServiceServer service = n.advertiseService("bingham_registration",
                                                  &RosRegistration::process,
                                                  &registration);
  // Spin
  ros::spin ();
}