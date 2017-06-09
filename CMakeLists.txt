cmake_minimum_required(VERSION 2.8.3)
project(dual_quaternion_registration)
add_definitions(-std=c++11)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${EIGEN3_INCLUDE_DIRS})

set(CMAKE_BUILD_TYPE Release)

# Add catkin and ros redpendencies if catkin is found
find_package(catkin QUIET COMPONENTS
  rospy
  roscpp
  std_msgs
  genmsg
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_generation
  cmake_modules
)
if(catkin_FOUND)
#   # Declare that this catkin package's runtime dependencies
  catkin_package()
  catkin_python_setup()
endif()


## Declare a C++ library
add_library(dual_quaternion_registration MODULE
   src/sort_indexes.cpp
   src/compute_transformed_points.cpp
   src/kd_tree.cpp
   src/registration_est_kf_rgbd.cpp
   src/get_changes_in_transformation_estimate.cpp
   src/bingham_kf.cpp
   src/dual_quaternion_registration.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(dual_quaternion_registration ${EIGEN3_LIBRARIES})

## Declare a C++ executable

add_executable(registration src/console_registration_main.cpp
                            src/dual_quaternion_registration.cpp
                            src/sort_indexes.cpp
                            src/registration_est_kf_rgbd.cpp
                            src/compute_transformed_points.cpp
                            src/kd_tree.cpp src/bingham_kf.cpp
                            src/get_changes_in_transformation_estimate.cpp
)

#############
## Install ##
#############

# Set target name based on our operating system
IF (WIN32)
set_target_properties(dual_quaternion_registration PROPERTIES OUTPUT_NAME "_qf_registration_windows")
ELSE()
set_target_properties(dual_quaternion_registration PROPERTIES OUTPUT_NAME "_qf_registration_linux")
ENDIF()
set_target_properties(dual_quaternion_registration PROPERTIES SKIP_BUILD_RPATH TRUE)
set_target_properties(dual_quaternion_registration PROPERTIES PREFIX "lib")
# Install the library in 'precompiled_clibs' so that the python GUI can find it
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/precompiled_clibs)

install(TARGETS dual_quaternion_registration DESTINATION ${LIBRARY_OUTPUT_PATH})

if(catkin_FOUND)
  if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(utest test/utest.cpp
                           #src/dual_quaternion_registration.cpp 
                           src/sort_indexes.cpp
                           src/registration_est_kf_rgbd.cpp
                           src/compute_transformed_points.cpp
                           src/kd_tree.cpp src/bingham_kf.cpp
                           src/get_changes_in_transformation_estimate.cpp)
    target_link_libraries(utest ${catkin_LIBRARIES})
  endif()
  ## Mark executables and/or libraries for installation
  install(TARGETS dual_quaternion_registration DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()