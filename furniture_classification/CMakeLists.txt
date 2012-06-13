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

rosbuild_add_library(training src/training.cpp)
rosbuild_link_boost(training system filesystem)
target_link_libraries(training yaml-cpp)

rosbuild_add_executable(train src/train.cpp)
target_link_libraries(train training)

rosbuild_add_executable(classify src/classify.cpp)
target_link_libraries(classify training)
target_link_libraries(classify vtkCommon vtkIO vtkFiltering vtkGraphics)

rosbuild_add_executable(scan src/scan.cpp)
rosbuild_link_boost(scan system filesystem)
target_link_libraries(scan vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)

rosbuild_add_executable(convert src/convert.cpp)
rosbuild_link_boost(convert system filesystem)

INCLUDE_DIRECTORIES("/usr/include/python2.7")

rosbuild_add_gtest(utest test/utest.cpp)
target_link_libraries(utest training)

rosbuild_add_executable(sac_test src/sac_test.cpp)
target_link_libraries(sac_test vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)

rosbuild_add_executable(region_grow src/region_grow.cpp)
target_link_libraries(region_grow vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)


rosbuild_add_library(furniture_classification src/pcl/classification/PHVObjectClassifier.cpp)
rosbuild_link_boost(furniture_classification system filesystem)
target_link_libraries(furniture_classification yaml-cpp)

rosbuild_add_executable(train_new src/train_new.cpp)
target_link_libraries(train_new training furniture_classification)
rosbuild_link_boost(train_new system filesystem)

rosbuild_add_executable(classify_new src/classify_new.cpp)
target_link_libraries(classify_new training furniture_classification)
rosbuild_link_boost(classify_new system filesystem)

