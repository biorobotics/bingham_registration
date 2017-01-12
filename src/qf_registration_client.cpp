/*
  File header:
  Implementation of a qf_registration client that sends request to qf_registration server
  Usage: run the executable with two arguments. (file paths of ptcld_moving.txt and ptcld_fixed.txt)
*/

#include "ros/ros.h"
#include "dual_quaternion_registration/QFRegistration.h"
#include "registration_est_kf_rgbd.h"
#include <cstdlib>

#include <iostream>
#include <fstream>
#include <cstring>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// For .txt file parsing
const int MAX_CHARS_PER_LINE = 512;     
const int MAX_TOKENS_PER_LINE = 20;
const char* const DELIMITER = " ";

/*
  int main:
    This function starts client and sends request to qf_registration server.
    Return 1 if fail, 0 if success
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "qf_registration_client");
  if (argc != 3)
  {
    ROS_INFO("usage: qf_registration_client sensor_file CAD_file");
    return 1;
  }

  // Read .text files for data points
  std::ifstream sensed_file;
  std::ifstream cad_file;

  std::string moving_file_string = argv[1];
  std::string fixed_file_string = argv[2];

  sensed_file.open(moving_file_string, std::ifstream::in);
  cad_file.open(fixed_file_string, std::ifstream::in);
  
  if (!sensed_file.good() || !cad_file.good()) 
  {
    ROS_ERROR_STREAM("Files " << fixed_file_string << ", " << moving_file_string << " not found" << "\n");
    return 1; // exit if file not found
  } 

  if (!sensed_file.good() || !cad_file.good()) 
  {
    ROS_ERROR_STREAM("Files " << fixed_file_string << ", " << moving_file_string << " not found" << "\n");
    return 1; // exit if file not found
  } 
  
  // Create pcl::PointCloud for storing the request data
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld_moving_p(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld_fixed_p(new pcl::PointCloud<pcl::PointXYZ>);
  ptcld_moving_p->height = 1;
  ptcld_moving_p->is_dense = true;
  ptcld_fixed_p->height = 1;
  ptcld_fixed_p->is_dense = true;

  // read sensed_file into ptcld_moving
  while (!sensed_file.eof()) 
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    sensed_file.getline(buf, MAX_CHARS_PER_LINE);
    
    // store line (which is a point) in a pcl::PointXYZ
    std::istringstream iss(buf);

    pcl::PointXYZ temp;

    iss >> temp.x >> temp.y >> temp.z;

    // Make sure all three were read in correctly
    if(iss.fail()) 
    {
      ROS_ERROR_STREAM(moving_file_string << ": Input data doesn't match dimension (too few per line)");
    }

    // Make sure there are no more to read
    float eofCheck;
    iss >> eofCheck;
    if(iss.good()) 
    {
      ROS_ERROR_STREAM(moving_file_string << ": Input data doesn't match dimension (too many per line)");
    }

    // Add temp to ptcldMoving 
    ptcld_moving_p->points.push_back(temp);
    ptcld_moving_p->width++;
  }

  // read cad_file into ptcld_fixed
  while (!cad_file.eof()) 
  {
    // read an entire line into memory
    char buf[MAX_CHARS_PER_LINE];
    cad_file.getline(buf, MAX_CHARS_PER_LINE);

    // store line in a vector
    std::istringstream iss(buf);
    
    pcl::PointXYZ temp;
    
    iss >> temp.x >> temp.y >> temp.z;
    if(iss.fail()) 
    {
      ROS_ERROR_STREAM(fixed_file_string <<": Input data doesn't match dimension (too few per line)");
    }
    // Make sure there are no more to read
    float eofCheck;
    iss >> eofCheck;
    if(iss.good()) 
    {
      ROS_ERROR_STREAM(moving_file_string << ": Input data doesn't match dimension (too many per line)");
    }
    // Add temp to list of ptcld_fixed
    ptcld_fixed_p->points.push_back(temp);
    ptcld_fixed_p->width++;
  }
  
  sensed_file.close();
  cad_file.close();

  // Create the client node in ROS
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<dual_quaternion_registration::QFRegistration>(
                                                                    "qf_registration");
  dual_quaternion_registration::QFRegistration srv;
  pcl::toROSMsg(*ptcld_moving_p, srv.request.ptcld_moving);
  pcl::toROSMsg(*ptcld_fixed_p, srv.request.ptcld_fixed);

  if (client.call(srv))
  {
    int i;
    ROS_INFO_STREAM("Xreg: (" << srv.response.xreg[0] << ", " << srv.response.xreg[1]
                    << ", " << srv.response.xreg[2] << ", " << srv.response.xreg[3] << ", "
                    << srv.response.xreg[4] << ", " << srv.response.xreg[5]);
    ROS_INFO("Xreg-saves: ");

    pcl::PointCloud<pcl::PointXYZ> xreg_saves_response;

    pcl::fromROSMsg(srv.response.xreg_saves, xreg_saves_response);
    for (i = 0; i < srv.response.xreg_saves.width; i+=2)
    {
      // Note: a Xreg entry (1*6) is split into two pcl::PointXYZ (1*3)
      ROS_INFO("\t(%f, %f, %f, %f, %f, %f)\n", xreg_saves_response.points[i].x,
                                               xreg_saves_response.points[i].y,
                                               xreg_saves_response.points[i].z,
                                               xreg_saves_response.points[i+1].x,
                                               xreg_saves_response.points[i+1].y,
                                               xreg_saves_response.points[i+1].z);
    }
  }

  else
  {
    ROS_ERROR("Error occured when calling service qf_registration");
    return 1;
  }

  return 0;
}