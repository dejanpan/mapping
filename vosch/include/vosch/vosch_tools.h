#include <sys/time.h>
#include "c3_hlac/c3_hlac.h"
#include "c3_hlac/c3_hlac_tools.h"
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>

//* GRSD type
#define NR_CLASS 5 
#define NOISE 0 
#define PLANE 1 
#define CYLINDER 2
#define SPHERE 3  
#define EDGE 4 
#define EMPTY 5 

#define NR_DIV 7 // number of normal angle divisions

//#define QUIET 1

//* const variables
const double min_radius_plane_ = 0.066;
const double min_radius_noise_ = 0.030, max_radius_noise_ = 0.050;
const double max_min_radius_diff_ = 0.02;
const double min_radius_edge_ = 0.030;
const double rsd_radius_search = 0.01;
const double normals_radius_search = 0.02; // 0.03;

//const float NORMALIZE_GRSD = NR_CLASS / 52.0; // 52 = 2 * 26
//const float NORMALIZE_GRSD = NR_CLASS / 104.0; // 104 = 2 * 2 * 26
const float NORMALIZE_GRSD = 20.0 / 26; // (bin num) / 26
const int GRSD_LARGE_DIM = 325; // 25 * 13

//-------------
/// \brief read
template <typename T>
bool readPoints( const char *name, pcl::PointCloud<T>& cloud );

//------------------------
/// \brief compute normals
template <typename T1, typename T2>
  void computeNormal( pcl::PointCloud<T1> input_cloud, pcl::PointCloud<T2>& output_cloud );

//--------------------------
/// \brief function for GRSD 
int getType (float min_radius, float max_radius);

//-------------------------
/// \brief extract - GRSD -
template <typename T>
Eigen::Vector3i extractGRSDSignature21(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize = false );

template <typename T>
void extractGRSDSignature21(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize = false );

//------------------------------------------
/// \brief extract - rotation-variant GRSD -
template <typename T>
Eigen::Vector3i extractGRSDSignature325(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize = false );

template <typename T>
void extractGRSDSignature325(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize = false );

//------------------------------
/// \brief extract - PlusGRSD -
template <typename T>
Eigen::Vector3i extractPlusGRSDSignature110(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize = false );

template <typename T>
void extractPlusGRSDSignature110(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const float voxel_size, const bool is_normalize = false );

//--------------------
/// \brief concatenate
const std::vector<float> concVector( const std::vector<float> v1, const std::vector<float> v2 );

//---------------
/// \brief VOSCH
template <typename PointT>
Eigen::Vector3i extractVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector< std::vector<float> > &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize = false );

template <typename PointT>
void extractVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector<float> &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

//-----------------
/// \brief ConVOSCH
template <typename PointT>
Eigen::Vector3i extractConVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector< std::vector<float> > &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0, const bool is_normalize = false );

template <typename PointT>
void extractConVOSCH(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, pcl::PointCloud<PointT> cloud_downsampled, std::vector<float> &feature, int color_threshold_r, int color_threshold_g, int color_threshold_b, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

#include <vosch/vosch_tools.hpp>
