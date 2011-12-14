cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

rosbuild_add_executable(pcl_hull_to_polygon src/pcl_hull_to_polygon.cpp)
rosbuild_add_executable(transform_pointcloud_node src/transform_pointcloud.cpp)
rosbuild_add_executable(hull_contract_node src/hull_contract.cpp)
rosbuild_add_executable(pointcloud_minmax_3d_node src/pointcloud_minmax_3d.cpp)
#rosbuild_add_executable(pcl_normal_visualization src/pcl_normal_visualization.cpp)
rosbuild_add_executable(pointcloud_line_pose_node src/pointcloud_line_pose.cpp)
rosbuild_add_executable(detect_circle_3D_node src/detect_circle_3d.cpp)
rosbuild_add_executable(create_xyzinormal_type src/create_xyzinormal_type.cpp)
rosbuild_add_executable(extract_clusters_on_table src/extract_clusters_on_table.cpp)
rosbuild_add_executable(extract_clusters_on_table_server
  src/extract_clusters_on_table_server.cpp)
rosbuild_add_executable(extract_clusters_on_table_client src/extract_clusters_on_table_client.cpp)
rosbuild_add_executable(compute_concave_hull src/compute_concave_hull.cpp)
rosbuild_add_executable(pose_logger src/pose_logger.cpp)
