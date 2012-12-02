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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


add_definitions(-DEIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})


#include_directories(/usr/include/vtk-5.6)
#include_directories ("/usr/include/openni/")

rosbuild_add_executable(scan src/scan.cpp)
rosbuild_link_boost(scan system filesystem)
target_link_libraries(scan vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)
target_link_libraries(scan pcl_common pcl_io pcl_visualization)

rosbuild_add_executable(convert src/convert.cpp)
rosbuild_link_boost(convert system filesystem)
target_link_libraries(convert pcl_common pcl_io pcl_segmentation pcl_surface pcl_filters)

#rosbuild_add_gtest(utest test/utest.cpp)
#target_link_libraries(utest training)


#rosbuild_add_executable(region_grow src/region_grow.cpp)
#target_link_libraries(region_grow vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)


rosbuild_add_library(furniture_classification src/pcl/classification/PHVObjectClassifier.cpp)
rosbuild_link_boost(furniture_classification system filesystem)
target_link_libraries(furniture_classification yaml-cpp pcl_common pcl_io pcl_visualization pcl_segmentation pcl_surface pcl_filters pcl_search pcl_octree pcl_features rostime)

rosbuild_add_executable(train src/train.cpp)
target_link_libraries(train furniture_classification)
rosbuild_link_boost(train system filesystem)

rosbuild_add_executable(classify src/classify.cpp)
target_link_libraries(classify furniture_classification)
rosbuild_link_boost(classify system filesystem)


#rosbuild_add_executable(classify_live src/classify_live.cpp)
#target_link_libraries(classify_live furniture_classification)
#target_link_libraries(classify_live vtkCommon vtkIO vtkFiltering vtkGraphics)
#rosbuild_link_boost(classify_live thread)

rosbuild_add_executable(eval_clustering src/eval_clustering.cpp)
target_link_libraries(eval_clustering furniture_classification)
rosbuild_link_boost(eval_clustering system filesystem)

rosbuild_add_executable(generate_hypothesis_node src/generate_hypothesis_node.cpp)
target_link_libraries(generate_hypothesis_node furniture_classification)

rosbuild_add_executable(fit_models_node src/fit_models_node.cpp)
target_link_libraries(fit_models_node furniture_classification)

rosbuild_add_executable(split_hypothesis_node src/split_hypothesis_node.cpp)
target_link_libraries(split_hypothesis_node furniture_classification)
rosbuild_link_boost(split_hypothesis_node system)

rosbuild_add_executable(scan_test src/scan_test.cpp)
rosbuild_link_boost(scan_test system filesystem)
target_link_libraries(scan_test vtkCommon vtkIO vtkFiltering vtkGraphics vtkRendering)
target_link_libraries(scan_test pcl_common pcl_io pcl_visualization)


rosbuild_add_executable(convert_node src/convert_node.cpp)
rosbuild_link_boost(convert_node system filesystem)
target_link_libraries(convert_node pcl_common pcl_common pcl_io pcl_segmentation pcl_surface pcl_filters)
