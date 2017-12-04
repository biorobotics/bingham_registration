#include <ros/ros.h>
#include <list>
#include <limits>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <resource_retriever/retriever.h>
#include <message_filters/subscriber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/filters/filter.h>
#include <Eigen/Dense>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
// Registration includes
#include "type_defs.h"
#include "registration_estimation.h"
#include "kd_tree.h"
#include "conversions.h"

namespace BinghamRegistration
{      
/* pclToEigen
 *  Converts pcl object to PointCloud (Eigen::Matrix)
 */

void shufflePointCloud(PointCloud &points){
  Eigen::PermutationMatrix<Eigen::Dynamic,Eigen::Dynamic> perm(points.cols());
  perm.setIdentity();
  std::random_shuffle(perm.indices().data(), perm.indices().data()+perm.indices().size());
  points = points * perm; // permute columns
}

PointCloud pclToEigen(const pcl::PCLPointCloud2& input)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(input, *cloudPtr);
  // Filter out NaNs
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud (*cloudPtr, *cloudPtr, indices);
  // Get just XYZ point
  Eigen::MatrixXf mat = cloudPtr->getMatrixXfMap(3,4,0);
  PointCloud points(3,mat.cols());
  points << mat.cast<long double>();
  // Randomize points
  shufflePointCloud(points);
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

/* affineFromregParams:
 *    Input: regParams 6x1 array containing x,y,z position and x,y,z euler rotation
 *    Output: Eigen::Affine3 representing regParams
 */
Affine3ld affineFromregParams(const ArrayXld& regParams)
{
  Matrix4ld tMatrix = reg_params_to_transformation_matrix (regParams.segment(0,6));
  Affine3ld t(tMatrix);
  return t;
}

class RosRegistration
{

private:
  double inlier_ratio = 1;
  int iterations = 100;
  int window_size = 20;
  double tolerance_t = 0.00001;
  double tolerance_r = 0.00001;
  PointCloud ptcldFixed;
  PointCloud ptcldMoving;
  PointCloud ptcldTransformed;
  std_msgs::Float32 errorMsg;
  KDTree fixedKDTree = NULL;
  Affine3ld lastTransform;
  std::list<Affine3ld> transformBuffer;
  double lastError = 1;
  visualization_msgs::Marker markerMsg;
  ros::Publisher markerPub;
  geometry_msgs::PoseStamped poseMsg;
  ros::Publisher posePub;
  ros::Publisher errorPub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub;
  ros::Subscriber reset_sub;
  ros::Subscriber active_sub;
  bool active = false;
  ros::NodeHandle nh;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RosRegistration(const std::string& stlPath, const double& stlScale)
  {
    ros::NodeHandle np("~");
    // Get bingham registration parameters
    np.getParam("inlier_ratio", inlier_ratio);
    np.getParam("iterations", iterations);
    np.getParam("window_size", window_size);
    np.getParam("tolerance_t", tolerance_t);
    np.getParam("tolerance_r", tolerance_r);
    reset();
    loadMesh(stlPath, stlScale);
    markerPub = nh.advertise<visualization_msgs::Marker>("registration_marker", 5);
    posePub = nh.advertise<geometry_msgs::PoseStamped>("registration_pose", 5);
    errorPub = nh.advertise<std_msgs::Float32>("rms_error", 5);
    // Create a ROS subscriber for the input point cloud
    cloud_sub.registerCallback(&RosRegistration::cloudCB, this);
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
    // Exit if mesh_path parameter not set
    // Load in STL file into the ptcldFixed variable

    std::string filePath = cleanResourcePath(path);
    std::cout << "Loading mesh from " << filePath << std::endl << std::endl;

    pcl::PolygonMesh mesh;
    if (pcl::io::loadPolygonFile(filePath, mesh) == 0)
    {
      PCL_ERROR("Failed to load mesh file\n");
    }
    ptcldFixed = pclToEigen(mesh.cloud) * scale;
    shufflePointCloud(ptcldFixed);

    free_tree(fixedKDTree);
    fixedKDTree = NULL;
    fixedKDTree = tree_from_point_cloud(ptcldFixed);

    // HACK BECAUSE RVIZ DOESN'T USE OBJ
    size_t idx = path.rfind('.', path.length());
    std::string stlPath = path.substr(0, idx) + ".stl";
    markerMsg.mesh_resource = stlPath;
    markerMsg.type = visualization_msgs::Marker::MESH_RESOURCE;
    markerMsg.header.frame_id = "/stereo_camera_frame";
    markerMsg.header.stamp    = ros::Time::now();
    markerMsg.id = 0;
    markerMsg.type = 10; // mesh resource
    markerMsg.action = 0;
    markerMsg.color.r = 0;
    markerMsg.color.g = 1.0;
    markerMsg.color.b = 0;
    markerMsg.color.a = 1.0;
    markerMsg.scale.x = scale;
    markerMsg.scale.y = scale;
    markerMsg.scale.z = scale;
  }

  void cloudCB(const boost::shared_ptr<const sensor_msgs::PointCloud2>& cloud_msg)
  {
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
    if(active) cloud_sub.subscribe(nh,"point_cloud", 1);
    else cloud_sub.unsubscribe();

  }

  void update()
  {
    ptcldTransformed = lastTransform*ptcldMoving;
    double uncertainty = std::numeric_limits<float>::min() * -1 / lastError;
    // Run the registration function without normals
    RegistrationResult result = registration_estimation(ptcldTransformed, ptcldFixed,
                                                         inlier_ratio, iterations, 
                                                         window_size, tolerance_t,
                                                         tolerance_r, uncertainty,
                                                         fixedKDTree);

    lastError = result.error;
    lastTransform = affineFromregParams(result.regParams) * lastTransform;
    setMarkerPose(lastTransform.inverse(), markerMsg);
    poseMsg.header.stamp = markerMsg.header.stamp;
    poseMsg.header.frame_id = markerMsg.header.frame_id;
    poseMsg.pose = markerMsg.pose;
    markerPub.publish(markerMsg);
    posePub.publish(poseMsg);
    errorMsg.data = lastError;
    errorPub.publish(errorMsg);
  }

  void reset()
  {
    lastTransform = Affine3ld::Identity();
    lastTransform.translate(Vector3ld(0, 0, 10));
    lastError = 1;
    markerMsg.pose.position.x = 0;
    markerMsg.pose.position.y = 0;
    markerMsg.pose.position.z = 0;
    markerMsg.pose.orientation.x = 0;
    markerMsg.pose.orientation.y = 0;
    markerMsg.pose.orientation.z = 0;
    markerMsg.pose.orientation.w = 1.0;

    // Wait for a point cloud
    double secs = ros::Time::now().toSec();
    ros::Rate r(100); // 10 hz
    while(ptcldMoving.cols() < 2 && ros::Time::now().toSec() - secs < .5){
      ros::spinOnce();
      r.sleep();
    }

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
      double bestError = 10000;
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
  if(!np.getParam("mesh_path", stlPath))
  {
    ROS_FATAL( "%s parameter mesh_path not set. Exiting.",
               ros::this_node::getName().c_str() );
    exit(1);
  }

  double stlScale;
  if(!np.getParam("mesh_scale", stlScale))
  {
    ROS_WARN("%s parameter mesh_scale not set. Using default of 1",
             ros::this_node::getName().c_str());
  }

  BinghamRegistration::RosRegistration registration(stlPath,
                                                    stlScale);

  // Spin
  ros::spin ();
}