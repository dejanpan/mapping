cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_add_executable(vtk_exporter src/vtk_exporter.cpp)

rosbuild_add_executable(vtk_to_dxf_exporter src/vtk_to_dxf_exporter.cpp src/dxf_writer.cpp)

rosbuild_add_executable (virtual_scanner src/virtual_scanner.cpp)
rosbuild_link_boost (virtual_scanner filesystem system)
target_link_libraries (virtual_scanner vtkHybrid itpp)

rosbuild_add_executable(pcl_hull_to_polygon src/pcl_hull_to_polygon.cpp)
rosbuild_add_executable(transform_pointcloud_node src/transform_pointcloud.cpp)
