cmake_minimum_required(VERSION 2.8.3)
project(dual_quaternion_registration)
add_definitions(-std=c++11 -O3) #${PCL_DEFINITIONS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#find_package(catkin REQUIRED COMPONENTS
#  roscpp
#  std_msgs
#  genmsg
#  geometry_msgs
#  sensor_msgs
#  tf2
#  tf2_ros
#  message_generation
#  cmake_modules
#)
find_package(Eigen3 REQUIRED)
#find_package(PCL REQUIRED)

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

# System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

# For setting up Qt

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
set(CMAKE_AUTOMOC ON)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ###############################################
# # Declare ROS messages, services and actions ##
# ###############################################

# # To declare and build messages, services or actions from within this
# # package, follow these steps:
# # * Let MSG_DEP_SET be the set of packages whose message types you use in
# #   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
# # * In the file package.xml:
# #   * add a build_depend tag for "message_generation"
# #   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
# #   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
# #     but can be declared for certainty nonetheless:
# #     * add a run_depend tag for "message_runtime"
# # * In this file (CMakeLists.txt):
# #   * add "message_generation" and every package in MSG_DEP_SET to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * add "message_runtime" and every package in MSG_DEP_SET to
# #     catkin_package(CATKIN_DEPENDS ...)
# #   * uncomment the add_*_files sections below as needed
# #     and list every .msg/.srv/.action file to be processed
# #   * uncomment the generate_messages entry below
# #   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# # Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   QFRegistration.srv
# )

# # Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# # Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

# ###############################################
# # Declare ROS dynamic reconfigure parameters ##
# ###############################################

# # To declare and build dynamic reconfigure parameters within this
# # package, follow these steps:
# # * In the file package.xml:
# #   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
# # * In this file (CMakeLists.txt):
# #   * add "dynamic_reconfigure" to
# #     find_package(catkin REQUIRED COMPONENTS ...)
# #   * uncomment the "generate_dynamic_reconfigure_options" section below
# #     and list every .cfg file to be processed

# # Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
#  LIBRARIES dual_quaternion_registration
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs tf2 tf2_ros
#  DEPENDS system_lib
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include/dual_quaternion_registration
  #${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  #${PCL_INCLUDE_DIRS}
)
#link_directories(${PCL_LIBRARY_DIRS})


## Declare a C++ library
add_library(dual_quaternion_registration MODULE
   src/registration_tools.cpp
   src/compute_transformed_points.cpp
   src/kd_tree.cpp
   src/registration_est_bingham_kf_rgbd.cpp
   src/get_changes_in_transformation_estimate.cpp
   src/bingham_kf.cpp
   src/dual_quaternion_registration.cpp
)



## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(dual_quaternion_registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(dual_quaternion_registration
  #${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  #${PCL_LIBRARIES}
)

## Declare a C++ executable

add_executable(registration src/dual_quaternion_registration.cpp 
                            src/registration_tools.cpp
                            src/registration_est_bingham_kf_rgbd.cpp
                            src/compute_transformed_points.cpp
                            src/kd_tree.cpp src/bingham_kf.cpp
                            src/get_changes_in_transformation_estimate.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(registration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
#target_link_libraries(registration ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

#add_executable(registration_gui src/registration_gui_main.cpp 
#                            src/registration_est_bingham_kf_rgbd.cpp
#                            src/compute_transformed_points.cpp
#                            src/kd_tree.cpp src/bingham_kf.cpp
#                            src/get_changes_in_transformation_estimate.cpp
#                            ${UI_GENERATED_HEADERS}
#)

#add_dependencies(registration_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(registration_gui ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dual_quaternion_registration DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
set_target_properties(dual_quaternion_registration PROPERTIES OUTPUT_NAME "_qf_registration_linux")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_LIST_DIR}/precompiled_clibs)
install(TARGETS dual_quaternion_registration DESTINATION ${LIBRARY_OUTPUT_PATH})

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dual_quaternion_registration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
