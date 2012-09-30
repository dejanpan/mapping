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
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)


############# SiftGPU ###################

set(USE_SIFT_GPU        0)
set(ENV{SIFT_GPU_MODE}		2) #CUDA = 1, GLSL = 2
set(ENV{SIFT_GPU_CUDA_PATH}	/usr/local/cuda)	

IF (${USE_SIFT_GPU})
	SET(ADDITIONAL_SOURCES src/sift_gpu_wrapper.cpp)
	SET(sift_gpu_build_path ${CMAKE_SOURCE_DIR}/external/siftgpu/linux)

	include_directories(${CMAKE_CURRENT_BINARY_DIR} external/siftgpu/src)
	
	MESSAGE("\n------------------------------------------------------------------\n")
	MESSAGE("Compiling SiftGPU...")
	EXECUTE_PROCESS(
		COMMAND make siftgpu
		WORKING_DIRECTORY ${sift_gpu_build_path}
		RESULT_VARIABLE sift_gpu_return
	)
	MESSAGE("\n------------------------------------------------------------------\n")
	
	#on error
	IF (NOT ${sift_gpu_return} EQUAL 0)
		MESSAGE(FATAL_ERROR "SiftGPU cannot be compiled. Returned: ${sift_gpu_return}")
	ENDIF (NOT ${sift_gpu_return} EQUAL 0)
	
	FILE(COPY external/siftgpu/linux/bin/libsiftgpu.so DESTINATION ../lib)
	
	add_definitions(-DUSE_SIFT_GPU)
	add_definitions(-DSIFT_GPU_MODE=$ENV{SIFT_GPU_MODE})
	set(SIFT_GPU_LIB siftgpu)
ENDIF (${USE_SIFT_GPU})

############# Eigen #####################

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

############ OpenCV #####################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

############ Make target #####################

SET(ADDITIONAL_SOURCES 
src/parameter_server.cpp
src/ransac_transformation.cpp
src/RGB_feature_detection.cpp
src/RGB_feature_matcher.cpp
src/pcl_utils.cpp
src/joint_optimize_wrapper.cpp
#src/transformation_estimation_wdf.cpp
 ${ADDITIONAL_SOURCES} 
)

rosbuild_add_executable(joint_optimization src/main.cpp ${ADDITIONAL_SOURCES})


target_link_libraries(joint_optimization ${SIFT_GPU_LIB} ${OpenCV_LIBS})
