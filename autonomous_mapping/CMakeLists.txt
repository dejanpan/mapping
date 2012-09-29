cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
# rosbuild_add_library(NextBestView src/next_best_view.cpp)
# rosbuild_add_library(AutonomousExploration src/autonomous_exploration.cpp)
# rosbuild_add_library(DistanceFilter src/distance_filter.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

# ------ [ Data
# typedef pcl::PointXYZINormal PointT;
#rosbuild_download_data (http://ias.cs.tum.edu/~goron/data/segmentation/september2010scan_iros2011.pcd data/september2010scan_iros2011.pcd 48b159ee285daf220340c932ce12fa84)
# typedef pcl::PointXYZRGB PointT;
#rosbuild_download_data (http://ias.cs.tum.edu/~goron/data/segmentation/high_res_rgb_right_of_oven.pcd data/high_res_rgb_right_of_oven.pcd d7a3739730a30edea9104417b4012270)
# typedef pcl::PointXYZ PointT;
#rosbuild_download_data (http://ias.cs.tum.edu/~goron/data/segmentation/26_August_ICS_no_downsample_12.pcd data/26_August_ICS_no_downsample_12.pcd 1878e90b51c32030e359c907ea406f3b)

# ------ [ Segmentation
#rosbuild_add_executable (segmentation_of_pointcloud src/segmentation/segmentation_of_pointcloud.cpp)
#rosbuild_add_executable (segment_difference src/segmentation/segment_differences.cpp)
#rosbuild_add_executable (project_segments src/segmentation/project_segments.cpp)
#rosbuild_add_executable (handle_removal src/segmentation/handle_removal.cpp)
#rosbuild_add_executable (visualize_segments src/segmentation/visualize_segments.cpp)

#target_link_libraries(example ${PROJECT_NAME})
