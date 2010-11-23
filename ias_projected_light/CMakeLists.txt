cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(ROS_BUILD_TYPE RelWithDebInfo)

#include_directories(include/ias_projected_light)

rosbuild_gensrv()



rosbuild_add_executable(create_pattern src/create_pattern.cpp)
rosbuild_add_executable(compare_point_clouds src/compare_point_clouds.cpp)
rosbuild_add_executable(adjust_exposure src/adjust_exposure.cpp)
rosbuild_add_executable(blur_estimation src/blur_estimation.cpp)
rosbuild_add_executable(roi_diff_image src/roi_diff_image.cpp)
rosbuild_add_executable(roi_bgfg_codebooks src/roi_bgfg_codebooks.cpp)


