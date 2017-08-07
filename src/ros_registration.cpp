#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <resource_retriever/retriever.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <Eigen/Dense>
#include <iostream>
#include <exception>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
// Registration includes
#include "type_defs.h"
#include "registration_estimation.h"
#include "conversions.h"

namespace BinghamRegistration
{      
/* pclToEigen
 *  Converts pcl object to PointCloud (Eigen::Matrix)
 */
PointCloud pclToEigen(const pcl::PCLPointCloud2& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(input, *cloudPtr);
  Eigen::MatrixXf mat = cloudPtr->getMatrixXfMap(3,4,0); // Get just XYZ point
  PointCloud points(3,mat.cols());
  points << mat.cast<long double>();
  return points;
}

/* setMarkerPose
 *  Input:
 *    transform_eigen - Eigen::Affine3 representing desired transformation
 *    msg - visualization_msgs::Marker to modify to match above transformation
 */
void setMarkerPose(const Affine3ld &transform_eigen, visualization_msgs::Marker &msg)
{
    tf::Transform t;
    tf::transformEigenToTF(transform_eigen.cast<double>(), t);
    tf::Quaternion quat = t.getRotation();
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    tf::Vector3 pos = t.getOrigin();
    msg.pose.position.x = pos.x();
    msg.pose.position.y = pos.y();
    msg.pose.position.z = pos.z();
    msg.header.stamp    = ros::Time::now();
}

/* affineFromXreg:
 *    Input: Xreg 6x1 array containing x,y,z position and x,y,z euler rotation
 *    Output: Eigen::Affine3 representing Xreg
 */
Affine3ld affineFromXreg(const ArrayXld& Xreg)
{
  Matrix4ld tMatrix = reg_params_to_transformation_matrix (Xreg.segment(0,6));
  Affine3ld t(tMatrix);
  return t;
}

class RosRegistration
{

private:
  PointCloud ptcldFixed;
  PointCloud ptcldMoving;
  PointCloud ptcldTransformed;
  Affine3ld lastTransform;
  double lastError = 1;
  visualization_msgs::Marker marker;
  ros::Publisher posePub;
  ros::Subscriber cloud_sub;
  ros::Subscriber reset_sub;
  ros::Subscriber active_sub;
  bool active = false;
  ros::NodeHandle nh;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RosRegistration(const std::string& stlPath, const double& stlScale)
  {
    reset();
    loadMesh(stlPath, stlScale);
    posePub = nh.advertise<visualization_msgs::Marker>("registration_marker", 5);
    // Create a ROS subscriber for the input point cloud
    cloud_sub = nh.subscribe ("point_cloud", 1, &RosRegistration::cloudCB, this);
    reset_sub = nh.subscribe ("registration/reset", 10, &RosRegistration::resetCB, this);
    active_sub = nh.subscribe ("registration/toggle", 10, &RosRegistration::activeCB, this);
  }

  std::string cleanResourcePath(const std::string& path){
    std::string newPath = path;
    if(path.find("package://") == 0)
    {
      newPath.erase(0,strlen("package://"));
      size_t pos = newPath.find("/");
      if (pos == std::string::npos)
      {
        ROS_FATAL("%s Could not parse package:// format", path.c_str());
        exit(1);
      }

      std::string package = newPath.substr(0, pos);
      newPath.erase(0, pos);
      std::string package_path = ros::package::getPath(package);

      if (package_path.empty())
      {
        ROS_FATAL("%s Package [%s] does not exist",path.c_str(), package.c_str());
        exit(1);
      }

      newPath = package_path + newPath;
    } else if(path.find("file://") == 0)\
    {
      newPath.erase(0,strlen("file://"));
    }
    return newPath;
  }


  void loadMesh(const std::string& path, const double& scale)
  {
    // Exit if stl_path parameter not set
    // Load in STL file into the ptcldFixed variable

    std::string filePath = cleanResourcePath(path);
    std::cout << "Loading stl from " << filePath << std::endl << std::endl;

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFileSTL(filePath, mesh) == 0)
    {
      PCL_ERROR("Failed to load STL file\n");
    }
    ptcldFixed = pclToEigen(mesh.cloud) * scale;

    marker.mesh_resource = filePath;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.header.frame_id = "/stereo_camera_frame";
    marker.header.stamp    = ros::Time::now();
    marker.id = 0;
    marker.type = 10; // mesh resource
    marker.action = 0;
    marker.color.r = 0;
    marker.color.g = 1.0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
  }

  void cloudCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
  {
    if(!active)
      return;

    // Container for original & filtered data
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
    ptcldMoving = pclToEigen(pcl_pc2);
    if(ptcldMoving.cols() < 2)
      return;
    update();
  }

  void resetCB(const std_msgs::Bool::ConstPtr& msg){
    if(msg->data == true)
      reset();
  }

  void activeCB(const std_msgs::Bool::ConstPtr& msg){
    active = msg->data;
  }

  void update()
  {
    ptcldTransformed = lastTransform*ptcldFixed;
    // Add uncertainty
    double uncertainty = 10*std::min(1.0,lastError);
    // Run the registration function without normals
    RegistrationResult result = registration_est_kf_rgbd(&ptcldMoving, &ptcldTransformed,
                                                         1, 100, 20, .00001, .00001, uncertainty);
    lastError = result.error;
    lastTransform = affineFromXreg(result.Xreg).inverse() * lastTransform;
    setMarkerPose(lastTransform, marker);
    posePub.publish(marker);
  }

  void reset()
  {
    lastTransform = Affine3ld::Identity();
    lastTransform.translate(Vector3ld(0, 0, 1));
    lastError = 1;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Find a suitable initial rotation
    if(ptcldMoving.cols() >= 2)
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
      double bestError = 1;
      int numTrials = 2;
      Affine3ld bestTransform(Affine3ld::Identity());
      for(int i=0; i<numQuats; i++){
        Quaternionld rot(quats[i][0], quats[i][1], quats[i][2], quats[i][3]);
        lastTransform = rot*Affine3ld::Identity();
        lastTransform.translate(Vector3ld(0, 0, 1));
        for(int j=0; j<numTrials; j++)
          update();
        if(lastError<bestError)
        {
          bestError = lastError;
          bestTransform = lastTransform;
        }
      }
      lastTransform = bestTransform;
    }
  }
};

} // Namespace BinghamRegistration

main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "bingham_registration");
  // std::string filepath = "/home/biomed/registration_ws/src/dvrk_vision/scripts/fixed.stl";
  PointCloud stlPoints;

  ros::NodeHandle np("~");
  std::string stlPath;
  if(!np.getParam("stl_path", stlPath))
  {
    ROS_FATAL( "%s parameter stl_path not set. Exiting.",
               ros::this_node::getName().c_str() );
    exit(1);
  }

  double stlScale;
  if(!np.getParam("stl_scale", stlScale))
  {
    ROS_WARN("%s parameter stl_scale not set. Using default of 1",
             ros::this_node::getName().c_str());
  }

  BinghamRegistration::RosRegistration registration(stlPath, stlScale);

  // Spin
  ros::spin ();
}