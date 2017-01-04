/*
  File header:
    Implementation of a qf_registration server. Usage: run the executable and 
    wait for a client to make request.
*/

#include "ros/ros.h"
#include "dual_quaternion_registration/QFRegistration.h"
#include "registration_est_kf_rgbd.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*
  bool process:
     This function provides the service for performing the qf_registration on the inputs in 
     the request. Then it adds the result to the response and send back to the client.

     Return true if success
*/
bool process(dual_quaternion_registration::QFRegistration::Request &req,
             dual_quaternion_registration::QFRegistration::Response &res)
{
  int i;

  // Convert sensor_msgs::PointCloud2 data passed via request to Eigen::MatrixXd, so 
  // the registration functio can take them as arguments.  
  pcl::PointCloud<pcl::PointXYZ> ptcld_moving_cloud;
  pcl::PointCloud<pcl::PointXYZ> ptcld_fixed_cloud;

  pcl::fromROSMsg(req.ptcld_moving, ptcld_moving_cloud);
  pcl::fromROSMsg(req.ptcld_fixed, ptcld_fixed_cloud);

  MatrixXf temp1 = ptcld_moving_cloud.getMatrixXfMap(3, 4, 0);
  MatrixXf temp2 = ptcld_fixed_cloud.getMatrixXfMap(3, 4, 0);

  MatrixXd temp1_d = temp1.cast<double>();
  MatrixXd temp2_d = temp2.cast<double>();

  // Run the registration function
  struct RegistrationResult result = registration_est_kf_rgbd(temp1_d, temp2_d);                                                      

  // Convert Eigen::VectorXd data returned by the registration function to std::vector<double>,
  // so it can be stored in the response to pass back to the client
  VectorXd xreg_eigen = result.Xreg;
  vector<double> xreg_std;
  xreg_std.resize(xreg_eigen.size());
  VectorXd::Map(&xreg_std[0], xreg_eigen.size()) = xreg_eigen;

  /* Convert Eigen::MatrixXd data returned by the registration function to 
     pcl::PointCloud<pcl::PointXYZ>, so it can be stored in the response to pass back to the client
  
     Note: each Xreg (1*6) vector is split into two pcl::PointXYZ objects (1*3)
  */
  MatrixXd xregsaves_eigen = result.Xregsave.transpose();
  pcl::PointCloud<pcl::PointXYZ> xregsaves_pcl;
  
  for (i=0; i<xregsaves_eigen.rows(); i++)
  {
    // Convert x, y, z of one Xreg
    xregsaves_pcl.push_back(pcl::PointXYZ(xregsaves_eigen(i, 0), 
                                          xregsaves_eigen(i, 1),
                                          xregsaves_eigen(i, 2)));
    // Conert theta_x, theta_y, theta_z of one Xreg into another pcl::PointXYZ.
    xregsaves_pcl.push_back(pcl::PointXYZ(xregsaves_eigen(i, 3), 
                                          xregsaves_eigen(i, 4),
                                          xregsaves_eigen(i, 5)));
  }

  // Store the registration result into the response
  res.xreg = xreg_std;
  pcl::toROSMsg(xregsaves_pcl, res.xreg_saves);
  ROS_INFO("Server received request.");
  return true;
}

/*
  int main:
    This function starts the server and make it spin to wait for client.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "qf_registration_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("qf_registration", process);
  ROS_INFO("Ready to register.");
  ros::spin();

  return 0;
}