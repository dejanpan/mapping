cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

ROSBUILD_ADD_LIBRARY (pcl_ias_sample_consensus
  src/pcl_ias_sample_consensus/pcl_sac_model_orientation.cpp
)

