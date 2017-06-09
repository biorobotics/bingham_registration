#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <Eigen/Dense>
#include <iostream>
// Registration includes
#include "type_defs.h"
#include "registration_estimation.h"

PointCloud pclToEigen(const pcl::PCLPointCloud2& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(input, *cloudPtr);
  Eigen::MatrixXf mat = cloudPtr->getMatrixXfMap();
  PointCloud points(3,mat.cols());
  points << mat.topLeftCorner(3,mat.cols()).cast<long double>();
  return points;
}

class RosRegistration
{

private:
  PointCloud ptcldFixed;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RosRegistration(const std::string &filePath){
    // Load in STL file into the ptcldFixed variable
    cout << "Loading stl from " << filePath << endl << endl;
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(filePath, mesh) == 0)
    {
      PCL_ERROR("Failed to load STL file\n");
    }
    ptcldFixed = pclToEigen(mesh.cloud);
  }

  void cloudCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
  {
    // Container for original & filtered data
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    PointCloud ptcldMoving = pclToEigen(pcl_pc2);
    cout << ptcldMoving.rows() << ", " << ptcldMoving.cols() << endl << endl;
    // Run the registration function without normals
    RegistrationResult result;
    result = registration_est_kf_rgbd(&ptcldMoving, &ptcldFixed,
                                      1, 100, 20, .00001, .00001, 1);
  }

};

main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "bingham_registration");
  ros::NodeHandle nh;
  std::string filepath = "/home/biomed/registration_ws/src/dvrk_vision/defaults/femur.stl";
  PointCloud stlPoints;
  RosRegistration registration(filepath);
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("stereo/point_cloud", 1,
                                      &RosRegistration::cloudCB,
                                      &registration);

  // Spin
  ros::spin ();
}