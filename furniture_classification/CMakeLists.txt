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

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

include_directories ("/usr/include/openni/")


rosbuild_add_executable(scan src/scan.cpp)
rosbuild_link_boost(scan system filesystem)
target_link_libraries(scan vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)

rosbuild_add_executable(convert src/convert.cpp)
rosbuild_link_boost(convert system filesystem)


rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest training)

#rosbuild_add_executable(sac_test src/sac_test.cpp)
#target_link_libraries(sac_test vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)

rosbuild_add_executable(region_grow src/region_grow.cpp)
target_link_libraries(region_grow vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)


rosbuild_add_library(furniture_classification src/pcl/classification/PHVObjectClassifier.cpp)
rosbuild_link_boost(furniture_classification system filesystem)
target_link_libraries(furniture_classification yaml-cpp)

rosbuild_add_executable(train src/train.cpp)
target_link_libraries(train furniture_classification)
rosbuild_link_boost(train system filesystem)

rosbuild_add_executable(classify src/classify.cpp)
target_link_libraries(classify furniture_classification)
rosbuild_link_boost(classify system filesystem)

rosbuild_add_executable(classify_live src/classify_live.cpp)
target_link_libraries(classify_live furniture_classification)
target_link_libraries(classify_live vtkCommon vtkIO vtkFiltering vtkGraphics)

