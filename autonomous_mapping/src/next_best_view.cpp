#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/PoseArray.h>

#include "octomap/octomap.h"
#include "octomap_ros/conversions.h"
#include <octomap_ros/OctomapBinary.h>
#include "pcl_to_octree/octree/OcTreePCL.h"
#include "pcl_to_octree/octree/OcTreeNodePCL.h"
#include "pcl_to_octree/octree/OcTreeServerPCL.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl_ros/publisher.h>
#include <pcl_ros/pcl_nodelet.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>

#include <nodelet/nodelet.h>
#include <math.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>
/**
@file

@brief  next best view.

@par Advertises
 - \b /nbv_octree topic

@par Subscribes
 - \b /cloud_pcd topic

@par Parameters
- \b input_cloud_topic
- \b output_octree_topic
 */


namespace autonomous_mapping
{

double pi=3.141;
int counter=0;
typedef struct
{
	double x,y;
	int weight;
}point_2d;

void write_pgm (std::string filename, std::vector<std::vector<int> > cm)
{
	std::ofstream myfile;
	myfile.open (filename.c_str());
	myfile << "P2" << std::endl;
	myfile << (int)cm[0].size() << " " << (int)cm.size() << std::endl;
	int max = 0;
	for (std::vector<std::vector<int> >::iterator it = cm.begin (); it != cm.end (); it++)
		for (std::vector<int>::iterator jt = it->begin(); jt != it->end (); jt++)
			if (max < (*jt))
				max = (*jt);
	myfile << max << std::endl;

	for (std::vector<std::vector<int> >::iterator it = cm.begin (); it != cm.end (); it++)
	{
		for (std::vector<int>::iterator jt = it->begin(); jt != it->end (); jt++)
			myfile << (*jt) << " ";
		myfile << std::endl;
	}
	myfile.close();
}
/*void write_kernel_pgm (std::string filename, std::vector<std::vector<point_2d> > ker)
{
	std::ofstream myfile;
	myfile.open (filename.c_str());
	myfile << "P2" << std::endl;
	myfile << (int)ker[0].size() << " " << (int)ker.size() << std::endl;
	point_2d max;
	for (std::vector<std::vector<point_2d> >::iterator it = ker.begin (); it != ker.end (); it++)
		for (std::vector<point_2d>::iterator jt = it->begin(); jt != it->end (); jt++)
		{
			if (max.x < jt->x)
				max.x = jt->x;
	        if (max.y<jt->y)
	        	max.y-jt->y;
	        if (max.weight<jt->weight)
	        	max.weight=jt->weight;
		}
	myfile << max.x<<max.y << max.weight<<std::endl;

	for (std::vector<std::vector<point_2d> >::iterator it = ker.begin (); it != ker.end (); it++)
	{
		for (std::vector<point_2d>::iterator jt = it->begin(); jt != it->end (); jt++)
			myfile <<jt-it->begin() << " ";
		myfile << std::endl;
	}
	myfile.close();
}
*/
class NextBestView : public pcl_ros::PCLNodelet
{

public:
	void onInit ();
protected:
	using pcl_ros::PCLNodelet::pnh_;
	// parameters:
	// topic namesros/ros.h
	std::string input_cloud_topic_;
	std::string output_octree_topic_;
	std::string output_pose_topic_;
	std::string laser_frame_;
	std::string ogrid_topic_;
	std::string ogrid_sub_topic_;

	// octree parameters
	double octree_res_, octree_maxrange_;
	int level_, free_label_, occupied_label_, unknown_label_, fringe_label_;
	bool check_centroids_;
	bool visualize_octree_;

	// search parameters
	double normal_search_radius_;
	int min_pts_per_cluster_;
	double eps_angle_;
	double tolerance_;
	double boundary_angle_threshold_;

	// costmap related stuff
	int nr_costmap_dirs_;
	double sensor_d_min_;
	double sensor_d_max_;
	double kernel_degree_step_;
	double kernel_radial_step_;
	double sensor_opening_angle_;
	double costmap_grid_cell_size_;
	double sensor_horizontal_angle_;
	double sensor_vertical_angle_;
	double sensor_horizontal_resolution_;
	double sensor_vertical_resolution_;
	double sensor_preferred_opening_angle_;
	double sensor_preferred_d_min_;
	double sensor_preferred_d_max_;
	double nr_pose_samples_;
	bool received_map_;
	int number;
	int fringe_nr_,free_nr_,occupied_nr_;
	int fringe_threashhold_;
	//geometry_msgs::Point min,max,minoc,maxoc;
	geometry_msgs::Pose p;
	//std::vector<std::vector<std::vector<int> > > border_costmap;
	// kernels for every (discretized) direction
	std::vector<std::vector<point_2d> > vis_kernel;

	//objects needed
	tf::TransformListener tf_listener_;

	//datasets
	octomap::OcTreePCL* octree_;
	octomap::ScanGraph* octomap_graph_;

	octomap::KeyRay ray;
	//pcl::PointCloud<pcl::PointXYZ> pointcloud2_pcl_total;
	sensor_msgs::PointCloud2 cloud_in_;
	geometry_msgs::PoseArray nbv_pose_array_;
	visualization_msgs::MarkerArray marker;
	visualization_msgs::Marker marker_bound_;
	visualization_msgs::Marker marker_occ_;
	nav_msgs::OccupancyGrid map_;
	// ROS communications / Publishers / Subscribers
	ros::Subscriber cloud_sub_;
	ros::Subscriber grid_sub_;
	ros::Publisher octree_pub_;
	//pcl_ros::Publisher<pcl::PointXYZ> border_cloud_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher pose_marker_pub_, pose_marker_array_pub_;
	ros::Publisher pose_boundary_pub_;
	ros::Publisher pose_occupied_pub_;
	ros::Publisher ogrid_pub_;
	ros::Publisher marker_bound_pub_;
	ros::Publisher marker_occ_pub_;
	ros::Publisher aggregated_pub_;
	ros::Publisher border_cloud_pub_;
	// Publishes the octree in MarkerArray format so that it can be visualized in rviz
	ros::Publisher octree_marker_array_publisher_;
	/* The following publisher, even though not required, is used because otherwise rviz
	 * cannot visualize the MarkerArray format without advertising the Marker format*/
	ros::Publisher octree_marker_publisher_;
	// Marker array to visualize the octree. It displays the occupied cells of the octree
	visualization_msgs::MarkerArray octree_marker_array_msg_;

	static bool compareClusters(pcl::PointIndices c1, pcl::PointIndices c2) { return (c1.indices.size() < c2.indices.size()); }
	void grid_cb (const nav_msgs::OccupancyGridConstPtr& grid_msg);
	void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
	void createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose);
	void visualizeOctree (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint);
	void castRayAndLabel (pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin);
	void findBorderPoints (pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id);
	void findOccupiedPoints(pcl::PointCloud<pcl::PointXYZ>& occupied_cloud, std::string frame_id);
	void computeBoundaryPoints (pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds);

	void extractClusters (const pcl::PointCloud<pcl::PointXYZ> &cloud,
			const pcl::PointCloud<pcl::Normal> &normals,
			float tolerance,
			const boost::shared_ptr<pcl::KdTree<pcl::PointXYZ> > &tree,
			std::vector<pcl::PointIndices> &clusters, double eps_angle,
			unsigned int min_pts_per_cluster = 1,
			unsigned int max_pts_per_cluster = std::numeric_limits<int>::max ());

	void create_kernels ();
	void find_min_max(pcl::PointCloud<pcl::PointXYZ> border_cloud, geometry_msgs::Point &min, geometry_msgs::Point &max);
	std::vector<std::vector<std::vector<int> > > create_costmap(double x_dim,double y_dim, int nr_dirs, pcl::PointCloud<pcl::PointXYZ> border_cloud,geometry_msgs::Point &min,geometry_msgs::Point &max);
	std::vector<std::vector<std::vector<int> > > create_empty_costmap (double x_dim, double y_dim, int nr_dirs);
	geometry_msgs::Pose find_best_pose(int best_i,int best_j,int best_k,int max_reward,geometry_msgs::Point &min,geometry_msgs::Point &max);
	geometry_msgs::Pose sample_from_costmap (std::vector<std::vector<std::vector<int> > > costmap, int max_reward, int nr_dirs, int x_dim, int y_dim, geometry_msgs::Point min, geometry_msgs::Point max, int &reward, double &score);
	//geometry_msgs::PoseArray computedirections(double x, double y, double theta, int reward);
	double compute_overlap_score(double x, double y, double theta, int reward);
	std::vector<std::vector<std::vector<int> > > reduced_costmap(std::vector<std::vector<std::vector<int> > > costmap, int best_j, int best_k,geometry_msgs::Point &min,geometry_msgs::Point &max,double &x_dim,double &y_dim);
	void find_max_indices (int &best_i, int &best_j, int &best_k,
				int nr_dirs, int x_dim, int y_dim, int &max_reward,
				std::vector<std::vector<std::vector<int > > > costmap);

	// save costmap stack to files
	void save_costmaps (int nr_dirs, std::vector<std::vector<std::vector<int> > > costmap, std::string file_prefix, ros::Time stamp);
	//void save_kernel(int nr_dirs,std::vector<std::vector<point_2d> > kernel, std::string file_prefix, ros::Time stamp);

public:
	NextBestView();

	~NextBestView();
};


NextBestView::NextBestView () : received_map_(false)
{

}

NextBestView::~NextBestView()
{
	ROS_INFO("Shutting down NextBestView!");

	octree_marker_array_publisher_.shutdown();
	octree_marker_publisher_.shutdown();
}

void NextBestView::onInit()
{
	pcl_ros::PCLNodelet::onInit ();
	// costmap params
	pnh_->param("nr_costmap_dirs", nr_costmap_dirs_, 8);
	pnh_->param("sensor_d_min", sensor_d_min_, 1.0);
	pnh_->param("sensor_d_max", sensor_d_max_, 4.0);
	pnh_->param("kernel_radial_step", kernel_radial_step_, 0.1);
	pnh_->param("kernel_degree_step", kernel_degree_step_, 10.0);
	kernel_degree_step_=pi*kernel_degree_step_/180.0;
	pnh_->param("sensor_opening_angle", sensor_opening_angle_, 180.0);
	sensor_opening_angle_=pi*sensor_opening_angle_/180.0;
	pnh_->param("costmap_grid_cell_size", costmap_grid_cell_size_, 0.15);
	pnh_->param("sensor_horizontal_angle", sensor_horizontal_angle_,180.0);//180 degrees
	sensor_horizontal_angle_=pi*sensor_horizontal_angle_/180.0;
	pnh_->param("sensor_vertical_angle", sensor_vertical_angle_,91.71);
	sensor_vertical_angle_=sensor_vertical_angle_*pi/180.0;
	pnh_->param("sensor_horizontal_resolution", sensor_horizontal_resolution_,1.0);//1 degree
	sensor_horizontal_resolution_=sensor_horizontal_resolution_*pi/180.0;
	pnh_->param("sensor_vertical_resolution", sensor_vertical_resolution_,1.0);//1 degree
	sensor_vertical_resolution_=sensor_vertical_resolution_*pi/180.0;
	pnh_->param("sensor_preferred_opening_angle", sensor_preferred_opening_angle_,120.0);
	sensor_preferred_opening_angle_=sensor_preferred_opening_angle_*pi/180.0;
	pnh_->param("sensor_preferred_d_min", sensor_preferred_d_min_,1.0);
	pnh_->param("sensor_preferred_d_max", sensor_preferred_d_max_,3.0);
	pnh_->param("nr_pose_samples", nr_pose_samples_,100.0);//1 degree
	// topic names
	pnh_->param("ogrid_sub_topic", ogrid_sub_topic_, std::string("/map"));
	pnh_->param("ogrid_topic", ogrid_topic_, std::string("/nbv_map"));
	pnh_->param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
	pnh_->param("output_octree_topic", output_octree_topic_, std::string("/nbv_octree"));
	pnh_->param("output_pose_topic", output_pose_topic_, std::string("/nbv_pose"));
	pnh_->param("laser_frame", laser_frame_, std::string("/laser_tilt_link"));
	pnh_->param("octree_resolution", octree_res_, 0.1);

	// octree stuff
	pnh_->param("octree_maxrange", octree_maxrange_, -1.0);
	pnh_->param("level", level_, 0);
	pnh_->param("check_centroids", check_centroids_, false);
	pnh_->param("free_label", free_label_, 0);
	pnh_->param("occupied_label", occupied_label_, 1);
	pnh_->param("unknown_label", unknown_label_, -1);
	pnh_->param("fringe_label", fringe_label_, 2);
	pnh_->param("visualize_octree", visualize_octree_, true);

	// search stuff
	pnh_->param("normal_search_radius", normal_search_radius_, 0.6);
	pnh_->param("min_pts_per_cluster", min_pts_per_cluster_, 10);
	pnh_->param("eps_angle", eps_angle_, 0.25);
	pnh_->param("tolerance", tolerance_, 0.3);
	pnh_->param("boundary_angle_threshold", boundary_angle_threshold_, 2.5);
	//ending criterion stuff
	pnh_->param("fringe_threashold",fringe_threashhold_,100);

	// create subs and pubs
	cloud_sub_ = pnh_->subscribe (input_cloud_topic_, 1, &NextBestView::cloud_cb, this);
	octree_pub_ = pnh_->advertise<octomap_ros::OctomapBinary> (output_octree_topic_, 1);
	ogrid_pub_ = pnh_->advertise<nav_msgs::OccupancyGrid> (ogrid_topic_, 1);
	grid_sub_=pnh_->subscribe(ogrid_sub_topic_,1,&NextBestView::grid_cb,this);
	//border_cloud_pub_ = pcl_ros::Publisher<pcl::PointXYZ> (*pnh_, "border_cloud", 1);
	pose_pub_ = pnh_->advertise<geometry_msgs::PoseArray> (output_pose_topic_, 1);
	pose_marker_pub_=pnh_->advertise<visualization_msgs::Marker>("pose_marker", 100);
	pose_marker_array_pub_=pnh_->advertise<visualization_msgs::MarkerArray>("pose_marker_array", 100);
	pose_boundary_pub_ = pnh_->advertise<geometry_msgs::PoseArray> ("boundary_pose", 1);
	pose_occupied_pub_ = pnh_->advertise<geometry_msgs::PoseArray> ("occupied_pose", 1);
	octree_marker_array_publisher_ = pnh_->advertise<visualization_msgs::MarkerArray>("visualization_marker_array",100);
	octree_marker_publisher_ = pnh_->advertise<visualization_msgs::Marker>("visualization_marker", 100);
	marker_bound_pub_=pnh_->advertise<visualization_msgs::Marker>("visualization_boundary_marker", 100);
	marker_occ_pub_=pnh_->advertise<visualization_msgs::Marker>("visualization_occupancy_marker", 100);
	aggregated_pub_=pnh_->advertise<sensor_msgs::PointCloud2>("aggregated_point_cloud",1);
	border_cloud_pub_=pnh_->advertise<sensor_msgs::PointCloud2>("border_cloud",1);
	octree_ = NULL;
   	// finally, precompute visibility kernel points
	create_kernels ();
}

void NextBestView::find_max_indices (int &best_i, int &best_j, int &best_k,
			int nr_dirs, int x_dim, int y_dim, int &max_reward,
			std::vector<std::vector<std::vector<int > > > costmap)
{
	int bla=0;
	std::ofstream file;
	file.open("/home/ghitzarus/Desktop/rewards.txt");
	for (int i = 0; i < nr_dirs; ++i)
	{
		//		std::stringstream ss;
		//		ss << "costmap_" << border_cloud.header.stamp << "_" << i;
		//		write_pgm (ss.str(), costmap[i]);
		for (int j = 0; j < x_dim; ++j)
			for (int k = 0; k < y_dim; k++)
			{
				int reward = costmap[i][j][k];
				if (reward!=0) bla++;
				file<<reward<<std::endl;
				//ROS_INFO("The reward is: %d",reward);
				if (reward > max_reward)
				{

					max_reward = reward;
					best_i = i;
					best_j = j;
					best_k = k;
				}
			}
	}
	file.close();
	//ROS_INFO("THE NUMBER OF REWARDS DIFFERNT THAN 0 is: %d",bla);
	//ROS_INFO("max revard is :%d ",max_reward);
}
void NextBestView::save_costmaps (int nr_dirs, std::vector<std::vector<std::vector<int> > > costmap, std::string file_prefix, ros::Time stamp)
{
	for (int i = 0; i < nr_dirs; ++i)
	{
		std::stringstream ss;
		ss << file_prefix << "_" << stamp << "_" << i << ".pgm";
		write_pgm (ss.str(), costmap[i]);
	}
}
/*void NextBestView::save_kernel(int nr_dirs,std::vector<std::vector<point_2d> > kernel, std::string file_prefix, ros::Time stamp)
{
     for(int i=0;i<nr_dirs;i++)
     {
			std::stringstream ss;
			ss << file_prefix << "_" << stamp << "_" << i << ".pgm";
			write_kernel_pgm (ss.str(), kernel);
     }
}
*/
void NextBestView::find_min_max(pcl::PointCloud<pcl::PointXYZ> border_cloud,geometry_msgs::Point &min, geometry_msgs::Point &max)
{
		for (unsigned int i=1;i<border_cloud.points.size();i++)
		{
			if (min.x > border_cloud.points[i].x) min.x = border_cloud.points[i].x;
			if (min.y > border_cloud.points[i].y) min.y = border_cloud.points[i].y;
			if (max.x < border_cloud.points[i].x) max.x = border_cloud.points[i].x;
			if (max.y < border_cloud.points[i].y) max.y = border_cloud.points[i].y;
		}
}

std::vector<std::vector<std::vector<int> > > NextBestView::create_empty_costmap (double x_dim, double y_dim, int nr_dirs)
{
	std::vector<std::vector<std::vector<int> > > costmap;
	costmap.resize (nr_dirs);
	for (int i = 0; i < nr_dirs; ++i) {
		costmap[i].resize(x_dim);

		for (int j = 0; j < x_dim; ++j)
			costmap[i][j].resize(y_dim);
	}
	return costmap;
}


std::vector<std::vector<std::vector<int> > > NextBestView::create_costmap (double x_dim, double y_dim, int nr_dirs, pcl::PointCloud<pcl::PointXYZ> cloud,geometry_msgs::Point &min,geometry_msgs::Point &max)
{
	// initialize vector of costmaps
	std::vector<std::vector<std::vector<int> > > costmap = create_empty_costmap (x_dim, y_dim, nr_dirs);

		// compute costmaps -- convolute with the different kernels
		for (int dir=0;dir<nr_dirs;dir++)
		{
			for (unsigned int i=0;i<cloud.points.size();i++)
			{
				for (unsigned int j=0;j<vis_kernel[dir].size();j++)
				{
					double x=(double)cloud.points[i].x + vis_kernel[dir][j].x;
					double y=(double)cloud.points[i].y + vis_kernel[dir][j].y;
					int id_x=(int)(x_dim*((x-min.x)/(max.x-min.x)));
					int id_y=(int)(y_dim*((y-min.y)/(max.y-min.y)));


					if (received_map_)
					{
						//  // NOTE: this assumes the "/map" message had no rotation (0,0,0,1)..
						double min_map_x = (double)map_.info.origin.position.x;
						double min_map_y = (double)map_.info.origin.position.y;
						double max_map_x = (double)(map_.info.origin.position.x + map_.info.resolution * map_.info.width);
						double max_map_y = (double)(map_.info.origin.position.y + map_.info.resolution * map_.info.height);

						int id_x_m=(int)(map_.info.width*((x-min_map_x)/(max_map_x-min_map_x)));
						int id_y_m=(int)(map_.info.height*((y-min_map_y)/(max_map_y-min_map_y)));
						if (map_.data[id_y_m*map_.info.width+id_x_m] == 0)
							if (id_x >= 0 && id_x < x_dim &&
									id_y >= 0 && id_y < y_dim)
								costmap[dir][id_x][id_y]+=vis_kernel[dir][j].weight;
					}
					else
						if (id_x >= 0 && id_x < x_dim &&
								id_y >= 0 && id_y < y_dim)
							costmap[dir][id_x][id_y]+=vis_kernel[dir][j].weight;
				}
			}
		}
		return costmap;
}
geometry_msgs::Pose NextBestView::find_best_pose(int best_i,int best_j,int best_k,int max_reward,geometry_msgs::Point &min,geometry_msgs::Point &max)
{
	    p.position.x = min.x + (best_j + 0.5) * costmap_grid_cell_size_;
		p.position.y = min.y + (best_k + 0.5) * costmap_grid_cell_size_;
		p.position.z = 0;
		btVector3 axis(0, 0, 1);
		btQuaternion quat (axis, pi+((double)best_i)*2.0*pi/(double)nr_costmap_dirs_);
		ROS_INFO("BEST I,J,K::: %i %i %i (reward %i)", best_i, best_j, best_k, max_reward);
		ROS_INFO("BEST robot pose::: x,y = [%f , %f], theta = %f", p.position.x, p.position.y, best_i*2.0*pi/nr_costmap_dirs_);

		geometry_msgs::Quaternion quat_msg;
		tf::quaternionTFToMsg(quat, quat_msg);
		p.orientation = quat_msg;
		return p;
}
std::vector<std::vector<std::vector<int> > >  NextBestView::reduced_costmap(std::vector<std::vector<std::vector<int> > > costmap, int best_j, int best_k,geometry_msgs::Point &min,geometry_msgs::Point &max,double &x_dim,double &y_dim)
{
	  p.position.x = min.x + (best_j + 0.5) * costmap_grid_cell_size_;
	  p.position.y = min.y + (best_k + 0.5) * costmap_grid_cell_size_;
	  p.position.z = 0;
	  	octomap::OcTreeNodePCL *octree_node = octree_->search(p.position.x,p.position.y,p.position.y);
	  	for (int dir=0;dir<nr_costmap_dirs_;dir++)
	  		for (int i=0;i<x_dim;i++)
	  			for (int j=0;j<y_dim;j++)
	  			{
	  				if (octree_node != NULL )
	  				{
	  					if (octree_node->getLabel()!=occupied_label_)
	  					   costmap[dir][i][j]=0;
	  				}
	  			}
	return costmap;
}
/*geometry_msgs::PoseArray NextBestView::computedirections(double x, double y, double theta, int reward)
{
	geometry_msgs::PoseArray ret_poses;
	std::vector<std::vector<int> > node_map;
	double node_map_x=(int)(pi / 0.00436332313)+1;
	double node_map_y=(int)(1.6/0.0174532925)+1;
	node_map.resize (node_map_x);
	for (int i = 0; i<node_map_x; i++)
		node_map[i].resize(node_map_y);

	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	pose.position.z = 1.35;
	octomap::point3d origin (x, y, pose.position.z);
	int hit = 0, miss = 0, fringe = 0, known = 0;
	// given a vector, along x axis.
	btVector3 x_axis (1, 0, 0);
	// rotate it "down" around y:
	btQuaternion rot_down (btVector3(0,1,0), 0.3);
	// rotate it in xy plane:
	btQuaternion rot_in_xy (btVector3(0,0,1), theta);
	int countx,county;
	for (double pid=-pi*0.5,countx=0;pid<pi*0.5,countx<node_map_x;pid+=0.00436332313,countx++)  // 0.25 degrees steps
	{
		btQuaternion rot_in_laser_plane (btVector3 (0,0,1), pid);
		for (double sid=-0.8,county=0;sid<0.8,county<node_map_y;sid+=0.0174532925,county++) // 1.0 degree steps
		{

			btQuaternion rot_laser_plane (btVector3 (0,1,0), sid);
			btVector3 laser_ray (1, 0, 0);
			btMatrix3x3 rot (rot_in_xy * rot_down * rot_laser_plane * rot_in_laser_plane);
			btVector3 dir = rot * laser_ray;
			octomap::point3d direction = octomap::point3d (dir.x(), dir.y(), dir.z());
			octomap::point3d obstacle(0,0,0);

			if (!octree_->castRay(origin, direction, obstacle, true, sensor_d_max_))
			  {
				miss++;
			  }
			else {
				hit++;
				octomap::OcTreeNodePCL *octree_node = octree_->search(obstacle);

				// if occupied voxel
				if (octree_node != NULL)
				{
					if (octree_node->getLabel() == occupied_label_)
					{
						known++;
						node_map[countx][county] = 1;
					}
					else //if (octree_node->getLabel() == fringe_label_)
					{
						fringe++;
						node_map[countx][county] = 2;
					}
				}

			}

			//geometry_msgs::Quaternion dir_tic;
			//tf::quaternionTFToMsg(rot_in_xy * rot_down * rot_laser_plane * rot_in_laser_plane, dir_tic);
			//pose.orientation = dir_tic;
      		//ret_poses.poses.push_back (pose);
		}

	}
	double p_f = (double)fringe/hit;
	double p_k = (double)known/hit;
	double entropy = - p_f * log (p_f) - p_k * log (p_k);
	ROS_INFO ("found %i fringe points, %i occupied points, ratios: %f, %f, Entropy = %f, score = %f",
					fringe, known, p_f, p_k, entropy, reward * entropy);
	return ret_poses;
}
*/
geometry_msgs::Pose NextBestView::sample_from_costmap (std::vector<std::vector<std::vector<int> > > costmap, int max_reward, int nr_dirs, int x_dim, int y_dim, geometry_msgs::Point min, geometry_msgs::Point max, int &reward, double &score)
{
	int dir, x, y;
	int c;
	do
	{
        //ROS_INFO("Infinite loop :(");
		dir = rand () * ((double)nr_dirs / RAND_MAX);
		x = rand () * ((double)x_dim / RAND_MAX);
		y = rand () * ((double)y_dim / RAND_MAX);
        //max_reward=max_reward*0.89;
		c = 0.5 * (double)max_reward + rand () * ((double)max_reward * 0.5 / RAND_MAX);
		c = sqrt((double)max_reward*max_reward - (double)c*c); // quarter circular transfer function
		//c =rand () * ((double)max_reward/ RAND_MAX);
		//c = sqrt((double)max_reward*max_reward/2 - (double)c*c);
		//ROS_INFO("The c value is: %d",c);
	} while (costmap [dir][x][y] < c);
	reward = costmap[dir][x][y];

	geometry_msgs::Pose pose = find_best_pose(dir, x, y, reward, min, max);

	double real_x = min.x + (x + 0.5) * costmap_grid_cell_size_;
	double real_y = min.y + (y + 0.5) * costmap_grid_cell_size_;
	double theta = pi+((double)dir)*2.0*pi/(double)nr_costmap_dirs_;
	score = compute_overlap_score (real_x, real_y, theta, reward);
     pose.position.z = (double)score / 100;
        //pose.position.z=0;
	return (pose);
}
double NextBestView::compute_overlap_score(double x, double y, double theta, int reward)
{
	std::vector<std::vector<int> > node_map;
		double node_map_x=(int)(sensor_horizontal_angle_ / sensor_horizontal_resolution_)+1;
		double node_map_y=(int)(sensor_vertical_angle_/sensor_vertical_resolution_)+1;
		node_map.resize (node_map_x);
		for (int i = 0; i<node_map_x; i++)
			node_map[i].resize(node_map_y);

		geometry_msgs::Pose pose;
		pose.position.x = x;
		pose.position.y = y;
		pose.position.z = 1.35;
		octomap::point3d origin (x, y, pose.position.z);
		int hit = 0, miss = 0, fringe = 0, known = 0;
		// given a vector, along x axis.
		btVector3 x_axis (1, 0, 0);
		// rotate it "down" around y:
		btQuaternion rot_down (btVector3(0,1,0), 0.3);
		// rotate it in xy plane:
		btQuaternion rot_in_xy (btVector3(0,0,1), theta);
		int countx = 0, county;
		for (double pid=-sensor_horizontal_angle_*0.5;pid<sensor_horizontal_angle_*0.5 && countx<node_map_x;pid+=sensor_horizontal_resolution_,countx++)  // 1 degrees steps
		{
			btQuaternion rot_in_laser_plane (btVector3 (0,0,1), pid);
			county = 0;
			for (double sid=-sensor_vertical_angle_/2;sid<sensor_vertical_angle_/2 && county<node_map_y;sid+=sensor_vertical_resolution_,county++) // 1.0 degree steps
			{

				btQuaternion rot_laser_plane (btVector3 (0,1,0), sid);
				btVector3 laser_ray (1, 0, 0);
				btMatrix3x3 rot (rot_in_xy * rot_down * rot_laser_plane * rot_in_laser_plane);
				btVector3 dir = rot * laser_ray;
				octomap::point3d direction = octomap::point3d (dir.x(), dir.y(), dir.z());
				octomap::point3d obstacle(0,0,0);

				if (!octree_->castRay(origin, direction, obstacle, true, sensor_d_max_))
				  {
					miss++;
				  }
				else {
					hit++;
					octomap::OcTreeNodePCL *octree_node = octree_->search(obstacle);

					// if occupied voxel
					if (octree_node != NULL)
					{
						if (octree_node->getLabel() == occupied_label_)
						{
							known++;
							node_map[countx][county] = 1;
						}
						else if (octree_node->getLabel() == fringe_label_)
						{
							fringe++;
							node_map[countx][county] = 2;
						}
					}

				}

				//geometry_msgs::Quaternion dir_tic;
				//tf::quaternionTFToMsg(rot_in_xy * rot_down * rot_laser_plane * rot_in_laser_plane, dir_tic);
				//pose.orientation = dir_tic;
	      		//ret_poses.poses.push_back (pose);
			}

		}
		double p_f = (double)fringe/hit;
		double p_k = (double)known/hit;
		double entropy;
		if (p_f == 0 || p_k == 0)
			entropy = 0;
		else
			entropy = pow((-0.5*p_f * log (p_f) -1.5*p_k * log (p_k)),1);
		ROS_INFO ("found %i fringe points, %i occupied points, ratios: %f, %f, Entropy = %f, score = %f",
				fringe, known, p_f, p_k, entropy, reward * entropy);
		return reward * entropy;
}
void NextBestView::grid_cb(const nav_msgs::OccupancyGridConstPtr& grid_msg)
{
	map_.header = grid_msg->header;
	map_.info = grid_msg->info;
	map_.data.resize(grid_msg->data.size());

	// dilate the map..
	double dilation_radius = 0.5;
	int dilation_radius_grid_cells = (dilation_radius / map_.info.resolution);
	int n = dilation_radius_grid_cells * 2 + 1;

	std::vector<std::vector<int> > dil_kernel;
	ROS_INFO ("creating dilation kernel for map with size %i x %i... ", n, n);
	dil_kernel.resize(n);
	for (int i = 0; i < n; i++)
	{
		dil_kernel[i].resize(n);
		for (int j = 0; j < n; j++)
		{
			double sqr_dist = (i - (dilation_radius_grid_cells+1))*(i - (dilation_radius_grid_cells+1))
					    		+ (j - (dilation_radius_grid_cells+1))*(j - (dilation_radius_grid_cells+1));
			if (sqr_dist < dilation_radius_grid_cells*dilation_radius_grid_cells)
				dil_kernel[i][j] = 1;
			else
				dil_kernel[i][j] = 0;
		}
	}
	ROS_INFO ("done.");


	ROS_INFO ("dilating map... ");
	for (unsigned int p = 0; p < grid_msg->data.size(); p++)
		map_.data[p] = grid_msg->data[p];
	int w = grid_msg->info.width;
	int h = grid_msg->info.height;
	for (unsigned int p = 0; p < grid_msg->data.size(); p++)
	{
		if (grid_msg->data[p] == 0) // free point in map
		{
			// find index in data... set it to 100
			int row = p / w;
			int col = (p - row * w) - (dilation_radius_grid_cells+1);
			row -= (dilation_radius_grid_cells+1);

			bool found_occupied_cell = false;
			for (int i = 0; i < n && !found_occupied_cell; i++)
				for (int j = 0; j < n && !found_occupied_cell; j++)
					if (dil_kernel[i][j] == 1)
					{
						int x = col + i;
						int y = row + j;
						if (x >= 0 && x < w && y >= 0 && y < h &&
								grid_msg->data[y*w+x] != 0)
						{
							map_.data[p] = 100;
							found_occupied_cell = true;
						}
					}
		}
	}
	ROS_INFO ("done.");

	received_map_=true;
}


void NextBestView::create_kernels ()
{
	vis_kernel.resize(nr_costmap_dirs_);
	point_2d ker_pt;
	ROS_INFO ("creating visibility kernels... ");
	for(double d = sensor_d_min_; d < sensor_d_max_; d += kernel_radial_step_)
	{
		for (double phi2 = -sensor_opening_angle_/2; phi2 < sensor_opening_angle_/2; phi2 += kernel_degree_step_)
		{
			double x = d*cos(phi2);
			double y = d*sin(phi2);

			for (int i = 0; i < nr_costmap_dirs_; i++)
			{
				double theta=i*2*pi/nr_costmap_dirs_;
				ker_pt.x = cos(theta)*x - sin(theta)*y;
				ker_pt.y = sin(theta)*x + cos(theta)*y;
				if (d>sensor_preferred_d_min_ && d<sensor_preferred_d_max_
						&& fabs(phi2)<sensor_preferred_opening_angle_/2)
					ker_pt.weight = 2;
				else
					ker_pt.weight = 1;
				vis_kernel[i].push_back(ker_pt);
			}
		}
	}
	ROS_INFO ("done.");
}

/**
 * \brief cloud callback and the core filtering function at the same time
 * \param pointcloud2_msg input point cloud to be processed
 */

void NextBestView::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	fringe_nr_=0;
	free_nr_=0;
	occupied_nr_=0;
	counter++;
	//sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg_total;
	ros::Time start_time = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZ> pointcloud2_pcl;
	pcl::fromROSMsg(*pointcloud2_msg, pointcloud2_pcl);
	//pointcloud2_pcl_total.header.frame_id="/map";
	//pointcloud2_pcl_total.header.stamp=start_time;
	//pointcloud2_pcl_total+=pointcloud2_pcl;
	//number+=pointcloud2_pcl.points.size();
    //pointcloud2_pcl_total.points.resize(number);
//uncommnet if you want to get clouds incrementally
	//if (counter<13)
		//return;


	//pointcloud2_pcl_total.points.resize(number);


    //aggregated_pub_.publish(pointcloud2_pcl_total);
	//ROS_INFO("Number is %d);
	octomap::point3d octomap_point3d;
	ROS_INFO("Received point cloud with number of points %ld",pointcloud2_pcl.points.size());
	//get the latest parameters
	pnh_->getParam("normal_search_radius", normal_search_radius_);
	pnh_->getParam("min_pts_per_cluster", min_pts_per_cluster_);
	pnh_->getParam("eps_angle", eps_angle_);
	pnh_->getParam("tolerance", tolerance_);
	pnh_->getParam("boundary_angle_threshold", boundary_angle_threshold_);

	//get the viewpoint (position of laser) from tf
	tf::StampedTransform transform;
	try {

		ros::Time acquisition_time = pointcloud2_pcl.header.stamp;

		ros::Duration timeout(1.0 / 30);

		tf_listener_.waitForTransform(pointcloud2_pcl.header.frame_id, laser_frame_, acquisition_time, timeout);

		tf_listener_.lookupTransform(pointcloud2_pcl.header.frame_id, laser_frame_, acquisition_time, transform);
	}
	catch (tf::TransformException& ex) {
		ROS_WARN("[next_best_view] TF exception:\n%s", ex.what());
	}
	tf::Point pt = transform.getOrigin();
	octomap::pose6d laser_pose (pt.x(), pt.y(), pt.z(),0,0,0);
	ROS_INFO("viewpoint [%f %f %f]", pt.x(), pt.y(), pt.z());


	//Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
	//pcl::fromROSMsg(*pointcloud2_msg, pointcloud2_pcl);

	// create or update the octree
	if (octree_ == NULL) {
		octomap_graph_ = new octomap::ScanGraph();
		octree_ = new octomap::OcTreePCL(octree_res_);
	}
	createOctree(pointcloud2_pcl, laser_pose);

	/*
	 * assign new points to Leaf Nodes  and cast rays from laser pos to point
	 */
	castRayAndLabel(pointcloud2_pcl, laser_pose);

	/*
	 * find unknown voxels with free neighbors and add them to a pointcloud
	 */
	pcl::PointCloud<pcl::PointXYZ> border_cloud,border_cloud_filtered;

	findBorderPoints(border_cloud,pointcloud2_msg->header.frame_id);
	fringe_nr_=border_cloud.points.size();
    ROS_WARN("Number of free points is: %d and number of fringe points is: %d",free_nr_,fringe_nr_);
	pcl::PointCloud<pcl::PointXYZ> occupied_cloud,occupied_cloud_filtered;
	findOccupiedPoints(occupied_cloud,pointcloud2_msg->header.frame_id);
    ROS_WARN("Number of occupied points is: %d ",occupied_nr_);
    // the fringe_threashold_ value was determined after counting the number of fringe points  from IROS01.bag file (1707),
    // then the number of fringe points from IROS01.bag + IROS02.bag (1224), and in the and the number of fringe points
    // from IROS01.bag + IROS02.bag + IROS.03.bag (955)
    if (fringe_nr_>=fringe_threashhold_)
    {
	// Create the filterin objects
	pcl::PassThrough<pcl::PointXYZ> pass;

	//creating datasets
	pcl::PointCloud<pcl::Normal> border_normals;

	//filter out ceiling
	pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 2.2);
	pass.filter(border_cloud_filtered);
	pass.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (occupied_cloud));
	pass.filter(occupied_cloud_filtered);

    geometry_msgs::Point min,max;
	min.x=FLT_MAX;
	min.y=FLT_MAX;
	max.x=-FLT_MAX;
	max.y=-FLT_MAX;

	find_min_max(border_cloud_filtered,min,max);
	find_min_max(occupied_cloud_filtered,min,max);

	min.x -= sensor_d_max_;
	min.y -= sensor_d_max_;
	max.x += sensor_d_max_;
	max.y += sensor_d_max_;

	double x_dim=(int)abs((max.x-min.x)/costmap_grid_cell_size_)+1;
	double y_dim=(int)abs((max.y-min.y)/costmap_grid_cell_size_)+1;
	ROS_INFO("X,Y min, max: %f, %f, %f, %f", min.x, min.y, max.x, max.y);
	ROS_INFO("X,Y DIMENSIONS OF THE COSTMAP: %f, %f",x_dim,y_dim);
	std::vector<std::vector<std::vector<int> > > boundary_costmap, occupied_costmap;
	boundary_costmap = create_costmap(x_dim, y_dim, nr_costmap_dirs_, border_cloud_filtered, min, max);
	occupied_costmap = create_costmap(x_dim, y_dim, nr_costmap_dirs_, occupied_cloud_filtered, min, max);

	// combine both costmaps using histogram intersection
	std::vector<std::vector<std::vector<int> > > bound_occ_costmap = create_empty_costmap(x_dim, y_dim, nr_costmap_dirs_);
	for (int i = 0; i < nr_costmap_dirs_; ++i)
		for (int j = 0; j < x_dim; ++j)
			for (int k = 0; k < y_dim; k++)
			{
				// should use min function.. :D
				if (boundary_costmap[i][j][k] < occupied_costmap[i][j][k])
					bound_occ_costmap[i][j][k] =  boundary_costmap[i][j][k];
				else
					bound_occ_costmap[i][j][k] =  occupied_costmap[i][j][k];
			}

	//save_costmaps (nr_costmap_dirs_, boundary_costmap, "boundary", pointcloud2_msg->header.stamp);
	//save_costmaps (nr_costmap_dirs_, occupied_costmap, "occupied", pointcloud2_msg->header.stamp);
	//save_costmaps (nr_costmap_dirs_, bound_occ_costmap, "combined", pointcloud2_msg->header.stamp);
	//save_kernel  (nr_costmap_dirs_, vis_kernel, "vis_kernel", pointcloud2_msg->header.stamp);
	// also publish the best found pose(s)
	nbv_pose_array_.header.frame_id = border_cloud_filtered.header.frame_id;
	nbv_pose_array_.header.stamp = ros::Time::now();

	// find best scanning pose -- simply the maximum
	int max_reward = 0;
	int best_combined_i=0, best_combined_j=0, best_combined_k=0;

	find_max_indices (best_combined_i, best_combined_j, best_combined_k, nr_costmap_dirs_, x_dim, y_dim, max_reward, bound_occ_costmap);

	nbv_pose_array_.poses.resize(0);
	//nbv_pose_array_.poses.push_back (find_best_pose(best_combined_i,best_combined_j,best_combined_k,max_reward,min,max));
	//geometry_msgs::Pose combined_bose=find_best_pose(best_combined_i,best_combined_j,best_combined_k,max_reward,min,max);
	std::vector<double> scores;
	int reward;
	double score;

         marker.markers.resize(nr_pose_samples_);
	bound_occ_costmap=reduced_costmap(bound_occ_costmap,best_combined_j,best_combined_k,min,max,x_dim,y_dim);

	for (int i = 0; i < nr_pose_samples_; i++)
	{

		geometry_msgs::Pose pose = sample_from_costmap(bound_occ_costmap, max_reward, nr_costmap_dirs_, x_dim, y_dim, min, max, reward,score);
		if (score == 0)
			continue;
		if (scores.size () == 0)
		{
			scores.insert (scores.begin(), score);
                        
			nbv_pose_array_.poses.insert (nbv_pose_array_.poses.begin(), pose);
		}
		else
		{
		for (unsigned int j = 0; j < scores.size(); j++)
			if (scores[j] < score)
			{
				scores.insert (scores.begin() + j, score);
                              
				nbv_pose_array_.poses.insert (nbv_pose_array_.poses.begin() + j, pose);
				break;
			}
		}
	}

	for (unsigned int i = 0; i < scores.size (); i++)
		ROS_INFO ("pose %i: %f", i, scores[i]);
    double threshold=scores[0] *0.85;
    int p=0;
    for (unsigned j=0;j<scores.size ();j++)
    	if (scores[j]>threshold)
    		p=j+1;
    	else break;
    nbv_pose_array_.poses.resize(p);
    ROS_INFO("resizing marker array for goto poses");
    marker.markers.resize(p);
	ROS_INFO("populating marker array for goto poses");
    for (int i = 0; i < p; i++)
    {
    	marker.markers[i].header.frame_id = pointcloud2_msg->header.frame_id;
    	marker.markers[i].header.stamp=ros::Time::now();
    	marker.markers[i].type = visualization_msgs::Marker::ARROW;
    	marker.markers[i].action = visualization_msgs::Marker::ADD;
    	marker.markers[i].ns="autonomous_mapping";
    	marker.markers[i].pose= nbv_pose_array_.poses[i];
        nbv_pose_array_.poses[i].position.z=0;
    	marker.markers[i].scale.x=0.5;
    	marker.markers[i].scale.y=4.0;
    	marker.markers[i].scale.z=1.0;
    	marker.markers[i].id = i;
    	marker.markers[i].color.r = 0.0f;
    	marker.markers[i].color.g = 0.0f;
    	marker.markers[i].color.b = 1.0f;
    	marker.markers[i].color.a = 1.0f;

    	marker.markers[i].lifetime = ros::Duration::Duration();
    }
	ROS_INFO("populated marker array for goto poses");
    
    pose_pub_.publish(nbv_pose_array_);
	ROS_INFO("publishing marker array for goto poses");
    pose_marker_array_pub_.publish(marker);
    ROS_INFO("published marker array for goto poses,%ld",marker.markers.size());
	/*double x, y, theta;
    x = min.x + (best_combined_j + 0.5) * costmap_grid_cell_size_;
	y = min.y + (best_combined_k + 0.5) * costmap_grid_cell_size_;
	theta = pi+((double)best_combined_i)*2.0*pi/(double)nr_costmap_dirs_;
	//geometry_msgs::PoseArray scan_poses = computedirections(x, y, theta, bound_occ_costmap[best_combined_i][best_combined_j][best_combined_k]);
	scan_poses.header = nbv_pose_array_.header;
	pose_pub_.publish(scan_poses);


	nbv_pose_array_.poses.resize(0);
	nbv_pose_array_.poses.push_back (find_best_pose(best_combined_i,best_combined_j,best_combined_k,max_reward,min,max));
	pose_boundary_pub_.publish(nbv_pose_array_);*/

	max_reward = 0;
	int best_boundary_i=0, best_boundary_j=0, best_boundary_k=0;
	find_max_indices (best_boundary_i, best_boundary_j, best_boundary_k, nr_costmap_dirs_, x_dim, y_dim, max_reward, boundary_costmap);
	nbv_pose_array_.poses.resize(0);
	nbv_pose_array_.poses.push_back (find_best_pose(best_boundary_i,best_boundary_j,best_boundary_k,max_reward,min,max));

	//pose_boundary_pub_.publish(nbv_pose_array_);
	marker_bound_.header.frame_id=pointcloud2_msg->header.frame_id;
	marker_bound_.header.frame_id = pointcloud2_msg->header.frame_id;
	marker_bound_.header.stamp=ros::Time::now();
	marker_bound_.type = visualization_msgs::Marker::ARROW;
	marker_bound_.action = visualization_msgs::Marker::ADD;
	marker_bound_.ns="autonomous_mapping";
	marker_bound_.pose=find_best_pose(best_boundary_i,best_boundary_j,best_boundary_k,max_reward,min,max);
	marker_bound_.scale.x=0.5;
	marker_bound_.scale.y=4.0;
	marker_bound_.scale.z=1.0;
	marker_bound_.id =0;
	marker_bound_.color.r = 0.0f;
	marker_bound_.color.g = 1.0f;
	marker_bound_.color.b = 0.0f;
	marker_bound_.color.a = 1.0f;
	marker_bound_pub_.publish(marker_bound_);
	max_reward = 0;
	int best_occupied_i=0, best_occupied_j=0, best_occupied_k=0;
	find_max_indices (best_occupied_i, best_occupied_j, best_occupied_k, nr_costmap_dirs_, x_dim, y_dim, max_reward, occupied_costmap);
	nbv_pose_array_.poses.resize(0);
	nbv_pose_array_.poses.push_back (find_best_pose(best_occupied_i,best_occupied_j,best_occupied_k,max_reward,min,max));
	//pose_occupied_pub_.publish(nbv_pose_array_);
	marker_occ_.header.frame_id=pointcloud2_msg->header.frame_id;
	marker_occ_.header.frame_id = pointcloud2_msg->header.frame_id;
	marker_occ_.header.stamp=ros::Time::now();
	marker_occ_.type = visualization_msgs::Marker::ARROW;
	marker_occ_.action = visualization_msgs::Marker::ADD;
	marker_occ_.ns="autonomous_mapping";
	marker_occ_.pose=find_best_pose(best_occupied_i,best_occupied_j,best_occupied_k,max_reward,min,max);
	marker_occ_.scale.x=0.5;
	marker_occ_.scale.y=4.0;
	marker_occ_.scale.z=1.0;
	marker_occ_.id =0;
	marker_occ_.color.r = 1.0f;
	marker_occ_.color.g = 0.0f;
	marker_occ_.color.b = 0.0f;
	marker_occ_.color.a = 1.0f;
	marker_occ_pub_.publish(marker_occ_);
	// prepare a occupancy grid to be published to rviz
	nav_msgs::OccupancyGrid ogrid;
	ogrid.header.frame_id = "/map";
	ogrid.info.resolution = costmap_grid_cell_size_;
	ogrid.info.width = x_dim;
	ogrid.info.height = y_dim;
	geometry_msgs::Pose gridmap_pose;
	gridmap_pose.position.x = min.x;
	gridmap_pose.position.y = min.y;
	gridmap_pose.position.z = 0;
	ogrid.info.origin = gridmap_pose;

	// prepare a single costmap...
	std::vector<std::vector<int> > final_costmap;
	final_costmap.resize(x_dim);
	for (int i=0;i<x_dim;i++)
		final_costmap[i].resize(y_dim);

	// assign the best costmap to be the final costmap
	int max_final_reward = 0;
	{
		int k = best_boundary_i;
		for (int i=0;i<x_dim;i++)
			for(int j=0;j<y_dim;j++)
			{
				final_costmap[i][j]+=boundary_costmap[k][i][j];
				if (final_costmap[i][j] > max_final_reward)
					max_final_reward = final_costmap[i][j];
			}
	}

	// fill the occupancy grid message, scaled to 0...255
	for (int j=0;j<y_dim;j++)
		for (int i=0;i<x_dim;i++)
			ogrid.data.push_back (255 * (double)final_costmap[i][j] / (double)max_final_reward);

	ogrid_pub_.publish (ogrid);

	//std::stringstream sss;
	//sss << "final_costmap_" << border_cloud.header.stamp;
	//write_pgm (sss.str(), final_costmap);


	//publish border cloud for visualization
	border_cloud_pub_.publish(border_cloud);

	// publish binary octree
	if (0) {
		octomap_ros::OctomapBinary octree_msg;
		octomap_server::octomapMapToMsg(*octree_, octree_msg);
		octree_pub_.publish(octree_msg);
	}

	ROS_INFO("All computed and published in %f seconds.", (ros::Time::now () - start_time).toSec());
	counter=0;
    }
    else
    	{
         
    	ROS_WARN("The number of fringe points is to small to continue scanning");
    	exit(1);
    	}

	//**********************************************************************************
	//Visualization
	//**********************************************************************************
	if (visualize_octree_)
	{
		geometry_msgs::Point viewpoint;
		viewpoint.x = pt.x();
		viewpoint.y = pt.y();
		viewpoint.z = pt.z();
		visualizeOctree(pointcloud2_msg, viewpoint);
	}
}

void NextBestView::computeBoundaryPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, pcl::PointCloud<pcl::Normal>& border_normals, std::vector<pcl::PointIndices>& clusters, pcl::PointCloud<pcl::PointXYZ>* cluster_clouds) {
	//clear old poses
	nbv_pose_array_.poses.clear();

	if (clusters.size() > 0) {
		ROS_INFO ("%d clusters found.", (int)clusters.size());
		// sort the clusters according to number of points they contain
		std::sort(clusters.begin(), clusters.end(), compareClusters);

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::ExtractIndices<pcl::Normal> nextract;

		for (unsigned int nc = 0; nc < clusters.size(); nc++) {
			//extract maximum of 3 biggest clusters
			if (nc == 3)
				break;

			//extract a cluster
			pcl::PointCloud<pcl::PointXYZ> cluster_cloud;
			extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (border_cloud));
			extract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
			extract.setNegative(false);
			extract.filter(cluster_cloud);
			ROS_INFO ("PointCloud representing the cluster %d: %d data points.", nc, cluster_cloud.width * cluster_cloud.height);

			//extract normals of cluster
			pcl::PointCloud<pcl::Normal> cluster_normals;
			nextract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::Normal> > (border_normals));
			nextract.setIndices(boost::make_shared<pcl::PointIndices> (clusters.back()));
			nextract.setNegative(false);
			nextract.filter(cluster_normals);

			// find boundary points of cluster
			pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree3 = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
			pcl::PointCloud<pcl::Boundary> boundary_cloud;
			pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> be;
			be.setSearchMethod(tree3);
			be.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cluster_cloud));
			be.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (cluster_normals));
			be.setRadiusSearch(.5);
			be.angle_threshold_ = boundary_angle_threshold_;
			be.compute(boundary_cloud);

			geometry_msgs::Pose nbv_pose;
			unsigned int nbp = 0;
			/*for (unsigned int i = 0; i < boundary_cloud.points.size(); ++i) {
				if (boundary_cloud.points[i].boundary_point) {
					nbv_pose.position.x = cluster_cloud.points[i].x;
					nbv_pose.position.y = cluster_cloud.points[i].y;
					nbv_pose.position.z = cluster_cloud.points[i].z;
					btVector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
					btQuaternion quat(axis, axis.length());
					geometry_msgs::Quaternion quat_msg;
					tf::quaternionTFToMsg(quat, quat_msg);
					nbv_pose.orientation = quat_msg;
					nbv_pose_array_.poses.push_back(nbv_pose);
					nbp++;
				}
			}*/
			geometry_msgs::Point minim,maxim,geometric_center;
			bool first_boundary = true;
			minim.x=(double)cluster_cloud.points[0].x;
			minim.y=(double)cluster_cloud.points[0].y;
			minim.z=(double)cluster_cloud.points[0].z;
			maxim.x=minim.x;
			maxim.y=minim.y;
			maxim.z=minim.z;
			for (unsigned int i=1; i<boundary_cloud.points.size();++i)
			{
				if (boundary_cloud.points[i].boundary_point)
				{
					if (first_boundary)
					{
						minim.x=(double)cluster_cloud.points[i].x;
						minim.y=(double)cluster_cloud.points[i].y;
						minim.z=(double)cluster_cloud.points[i].z;
						maxim.x=minim.x;
						maxim.y=minim.y;
						maxim.z=minim.z;
						first_boundary = false;
					}
					else
					{
						if (cluster_cloud.points[i].x<minim.x)
							minim.x=(double)cluster_cloud.points[i].x;
						if (cluster_cloud.points[i].y<minim.y)
							minim.y=(double)cluster_cloud.points[i].y;
						if (cluster_cloud.points[i].z<minim.z)
							minim.z=(double)cluster_cloud.points[i].z;
						if (cluster_cloud.points[i].x>maxim.x)
							maxim.x=(double)cluster_cloud.points[i].x;
						if (cluster_cloud.points[i].y>maxim.y)
							maxim.y=(double)cluster_cloud.points[i].y;
						if (cluster_cloud.points[i].z>maxim.z)
							maxim.z=(double)cluster_cloud.points[i].z;
					}
				}
			}
			geometric_center.x=minim.x+(maxim.x-minim.x)/2.0;
			geometric_center.y=minim.y+(maxim.y-minim.y)/2.0;
			geometric_center.z=minim.z+(maxim.z-minim.z)/2.0;
			ROS_INFO("the coordinates of the geometric center are %f, %f, %f",geometric_center.x,geometric_center.y,geometric_center.z);
			pcl::PointCloud<pcl::PointXYZ> cluster_cloud_2,cluster_cloud_final;
			cluster_cloud_2.header = boundary_cloud.header;
			cluster_cloud_2.points.resize(boundary_cloud.points.size());
			cluster_cloud_final.header = boundary_cloud.header;
			cluster_cloud_final.points.resize(boundary_cloud.points.size());
			double fi,dist;
			double dx, dy;
			double factor = 0.1;

			for (unsigned i=0; i< boundary_cloud.points.size(); ++i)
			{

				if (boundary_cloud.points[i].boundary_point)
				{
					fi=atan2(((double)cluster_cloud.points[i].x-geometric_center.x),
							((double)cluster_cloud.points[i].y-geometric_center.y));
					dist=abs((((double)cluster_cloud.points[i].x-geometric_center.x) / sin(fi)));
					dx = (double)cluster_cloud.points[i].x-geometric_center.x;
					dy = (double)cluster_cloud.points[i].y-geometric_center.y;
					//todo? normalize(dx,dy) to unit length, then factor is in meters..
					cluster_cloud_2.points[i].x = cluster_cloud.points[i].x + factor * dx; //(geometric_center.x+factor*dist*sin(fi));
					cluster_cloud_2.points[i].y = cluster_cloud.points[i].y + factor * dy; //(geometric_center.y+factor*dist*cos(fi));
				}
			}
			//ROS_INFO("TEST MESAGE %f",cluster_cloud_2.points[3].x);
			int pt_counter = 0;
			geometry_msgs::Point d,d_norm;
			for (unsigned int i=0; i<boundary_cloud.points.size();++i)
			{
				if (boundary_cloud.points[i].boundary_point)
				{
					octomap::OcTreeNodePCL * free_node = octree_->search(cluster_cloud_2.points[i].x,cluster_cloud_2.points[i].y,cluster_cloud_2.points[i].y);
					if ((free_node != NULL) && (free_node->getLabel() != occupied_label_))
					{

						nbv_pose.position.x = cluster_cloud_2.points[i].x;
						nbv_pose.position.y = cluster_cloud_2.points[i].y;
						nbv_pose.position.z = cluster_cloud_2.points[i].z;
						cluster_cloud_final.points[pt_counter].x=cluster_cloud_2.points[i].x;
						cluster_cloud_final.points[pt_counter].y=cluster_cloud_2.points[i].y;
						cluster_cloud_final.points[pt_counter].z=cluster_cloud_2.points[i].z;
						pt_counter++;
						//btVector3 axis(0, -cluster_normals.points[i].normal[2], cluster_normals.points[i].normal[1]);
						d.x=cluster_cloud_2.points[i].x-geometric_center.x;
						d.y=cluster_cloud_2.points[i].y-geometric_center.y;
						d.z=cluster_cloud_2.points[i].z-geometric_center.z;
						double norm=sqrt(pow(d.x,2)+pow(d.y,2)+pow(d.z,2));
						d_norm.x=d.x/norm;
						d_norm.y=d.y/norm;
						d_norm.z=d.z/norm;
						btVector3 axis(0, 0, 1);
						btQuaternion quat (axis, acos(d_norm.x));
						//ROS_INFO("ANGLE IS::: %f",cluster_cloud_2.points[i].z-geometric_center.z);
						geometry_msgs::Quaternion quat_msg;
						tf::quaternionTFToMsg(quat, quat_msg);
						nbv_pose.orientation = quat_msg;
						nbv_pose_array_.poses.push_back(nbv_pose);
						nbp++;
					}
				}
			}
			ROS_INFO ("%d boundary points in cluster %d.", nbp, nc);
			cluster_cloud_final.points.resize (pt_counter);
			//save this cluster pointcloud for visualization
			cluster_clouds[nc] = cluster_cloud_final;
			//pop the just used cluster from indices
			clusters.pop_back();
		}
		nbv_pose_array_.header.frame_id = border_cloud.header.frame_id;
		nbv_pose_array_.header.stamp = ros::Time::now();
	}
	else {
		ROS_INFO ("No clusters found!");
	}
}

void NextBestView::castRayAndLabel(pcl::PointCloud<pcl::PointXYZ>& cloud, octomap::pose6d origin) {
	octomap::point3d octomap_point3d;

	BOOST_FOREACH (const pcl::PointXYZ& pcl_pt, cloud.points) {
		octomap_point3d(0) = pcl_pt.x;
		octomap_point3d(1) = pcl_pt.y;
		octomap_point3d(2) = pcl_pt.z;
		octomap::OcTreeNodePCL * octree_end_node = octree_->search(octomap_point3d);
		if (octree_end_node != NULL) {
			// Get the nodes along the ray and label them as free
			if (octree_->computeRayKeys(origin.trans(), octomap_point3d, ray)) {
				for(octomap::KeyRay::iterator it=ray.begin(); it != ray.end(); it++) {
					octomap::OcTreeNodePCL * free_node = octree_->search(*it);
					if (free_node != NULL) {
						if (free_node->getLabel() != occupied_label_)
						{
							free_node->setLabel(free_label_);

						}
					}

					else
						ROS_DEBUG("node in ray not found!");
				}
			}
			else {
				ROS_DEBUG("could not compute ray from [%f %f %f] to [%f %f %f]", origin.x(), origin.y(), origin.z(), pcl_pt.x, pcl_pt.y, pcl_pt.z);
			}
			octree_end_node->set3DPointInliers(0);
			octree_end_node->setLabel(occupied_label_);
		}
		else {
			ROS_DEBUG("ERROR: node at [%f %f %f] not found", pcl_pt.x, pcl_pt.y, pcl_pt.z);
		}
	}




}
void NextBestView::findOccupiedPoints(pcl::PointCloud<pcl::PointXYZ>& occupied_cloud, std::string frame_id)
{
	    occupied_cloud.header.frame_id = frame_id;
		occupied_cloud.header.stamp = ros::Time::now();
		std::list<octomap::OcTreeVolume> leaves;
		octree_->getLeafNodes(leaves);
		BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
			octomap::point3d centroid;
			centroid(0) = vol.first.x(),  centroid(1) = vol.first.y(),  centroid(2) = vol.first.z();
			octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);

			// if occupied voxel
			if (octree_node != NULL && octree_node->getLabel() == occupied_label_)
			{
				occupied_nr_++;
				pcl::PointXYZ occupied_pt (centroid.x(), centroid.y(), centroid.z());
		        occupied_cloud.points.push_back(occupied_pt);
			}
		}
}

void NextBestView::findBorderPoints(pcl::PointCloud<pcl::PointXYZ>& border_cloud, std::string frame_id)
{
	border_cloud.header.frame_id = frame_id;
	border_cloud.header.stamp = ros::Time::now();
	std::list<octomap::OcTreeVolume> leaves;
	octree_->getLeafNodes(leaves);
	BOOST_FOREACH(octomap::OcTreeVolume vol, leaves)
	{
		octomap::point3d centroid;
		centroid(0) = vol.first.x(),  centroid(1) = vol.first.y(),  centroid(2) = vol.first.z();
		octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
		int found = 0;
		// if free voxel -> check for unknown neighbors
		if (octree_node != NULL && octree_node->getLabel() == free_label_)
		{

			free_nr_++;
			for (int x = -1; x <= 1; x++)
			{
				for (int y = -1; y <= 1; y++)
				{
					for (int z = -1; z <= 1; z++)
					{
						if (x == 0 && y == 0 && z == 0)
							continue;
						octomap::point3d neighbor_centroid (
								centroid.x() + x * octree_res_,
								centroid.y() + y * octree_res_,
								centroid.z() + z * octree_res_);

						octomap::OcTreeNodePCL *neighbor = octree_->search(neighbor_centroid);
						if (neighbor != NULL && neighbor->getLabel() == unknown_label_) {
							// add to list of border voxels
							found++;
							if (found > 7)
							{
								pcl::PointXYZ border_pt (centroid.x(), centroid.y(), centroid.z());
								border_cloud.points.push_back(border_pt);
								octree_node->setLogOdds (CLAMPING_THRES_MAX);
								octree_node->setLabel(fringe_label_);

								break;
							}
						}
					}
					if (found > 7)
						break;
				}
				if (found > 7)
					break;
			}
		}
	}
	ROS_INFO("%d points in border cloud", (int)border_cloud.points.size());
}

/**
 * creating an octree from pcl data
 */
void NextBestView::createOctree (pcl::PointCloud<pcl::PointXYZ>& pointcloud2_pcl, octomap::pose6d laser_pose) {

	octomap::point3d octomap_3d_point;
	octomap::Pointcloud octomap_pointcloud;

	//Reading from pcl point cloud and saving it into octomap point cloud
	BOOST_FOREACH (const pcl::PointXYZ& pt, pointcloud2_pcl.points) {
		octomap_3d_point(0) = pt.x;
		octomap_3d_point(1) = pt.y;
		octomap_3d_point(2) = pt.z;
		octomap_pointcloud.push_back(octomap_3d_point);
	}

	// Converting from octomap point cloud to octomap graph
        ROS_INFO ("Number ofpoints in octomap_pointcloud %ld",octomap_pointcloud.size());
	octomap_pointcloud.transform(laser_pose.inv());
	octomap::ScanNode* scan_node = octomap_graph_->addNode(&octomap_pointcloud, laser_pose);

	//ROS_INFO("Number of points in scene graph: %d", octomap_graph_->getNumPoints());

	// Converting from octomap graph to octomap tree (octree)
	octree_->insertScan(*scan_node, octree_maxrange_, false);

	octree_->expand();

	//
	// create nodes that are unknown
	//
	//octomap::point3d min, max;
	double min[3], max[3];
	octree_->getMetricMin(min[0], min[1], min[2]);
	octree_->getMetricMax(max[0], max[1], max[2]);
	//ROS_DEBUG("octree min bounds [%f %f %f]", min(0), min(1), min(2));
	//ROS_DEBUG("octree max bounds [%f %f %f]", max(0), max(1), max(2));

	double x,y,z;
	for (x = min[0]+octree_res_/2; x < max[0]-octree_res_/2; x+=octree_res_) {
		for (y = min[1]+octree_res_/2; y < max[1]-octree_res_/2; y+=octree_res_) {
			for (z = min[2]+octree_res_/2; z < max[2]-octree_res_/2; z+=octree_res_) {
				octomap::point3d centroid (x, y, z);
				if (z > max[2])
					ROS_INFO("ahrg node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
				octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
				if (octree_node != NULL) {
					octree_node->setCentroid(centroid);
				} else {
					//ROS_INFO("creating node at [%f %f %f]", centroid(0), centroid(1), centroid(2));
					//corresponding node doesn't exist yet -> create it
					octomap::OcTreeNodePCL *new_node = octree_->updateNode(centroid, false);
					new_node->setCentroid(centroid);
					new_node->setLabel(unknown_label_);

				}
			}
		}
	}


	if (check_centroids_) {
		//int cnt = 0;
		// get all existing leaves
		std::list<octomap::OcTreeVolume> leaves;
		//octree_->getLeafNodes(leaves);

		//find Leaf Nodes' centroids, assign controid coordinates to Leaf Node
		BOOST_FOREACH(octomap::OcTreeVolume vol, leaves) {
			//ROS_DEBUG("Leaf Node %d : x = %f y = %f z = %f side length = %f ", cnt++, it1->first.x(), it1->first.y(), it1->first.z(), it1->second);
			octomap::point3d centroid;
			centroid(0) = vol.first.x(), centroid(1) = vol.first.y(), centroid(2) = vol.first.z();
			octomap::OcTreeNodePCL *octree_node = octree_->search(centroid);
			if (octree_node != NULL) {
				octomap::point3d test_centroid;
				test_centroid = octree_node->getCentroid();
				if (centroid.distance(test_centroid) > octree_res_/4)
					ROS_INFO("node at [%f %f %f] has a wrong centroid: [%f %f %f]", centroid(0), centroid(1), centroid(2), test_centroid(0), test_centroid(1), test_centroid(2));
			}
			else {
				ROS_INFO("node at [%f %f %f] not found", centroid(0), centroid(1), centroid(2));
			}
		}
	}

}


void NextBestView::visualizeOctree(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg, geometry_msgs::Point viewpoint) {
	// each array stores all cubes of a different size, one for each depth level:
	octree_marker_array_msg_.markers.resize(3);
	double lowestRes = octree_->getResolution();
	//ROS_INFO_STREAM("lowest resolution: " << lowestRes);

	std::list<octomap::OcTreeVolume> all_cells;
	//getting the cells at level 0
	octree_->getLeafNodes(all_cells, 0);
	BOOST_FOREACH(octomap::OcTreeVolume vol, all_cells)
	{
		geometry_msgs::Point cube_center;
		cube_center.x = vol.first.x();
		cube_center.y = vol.first.y();
		cube_center.z = vol.first.z();
		octomap::point3d octo_point (cube_center.x, cube_center.y, cube_center.z);
		octomap::OcTreeNodePCL * node = octree_->search(octo_point);
		if (node != NULL) {
			if (occupied_label_ == node->getLabel())
				octree_marker_array_msg_.markers[0].points.push_back(cube_center);
			else if (free_label_ == node->getLabel())
				octree_marker_array_msg_.markers[1].points.push_back(cube_center);
			else if (unknown_label_ == node->getLabel())
				octree_marker_array_msg_.markers[2].points.push_back(cube_center);

		}
	}

	//octree_marker_array_msg_.markers[3].points.push_back(viewpoint);

	// occupied cells
	octree_marker_array_msg_.markers[0].ns = "Occupied cells";
	octree_marker_array_msg_.markers[0].color.r = 1.0f;
	octree_marker_array_msg_.markers[0].color.g = 0.0f;
	octree_marker_array_msg_.markers[0].color.b = 0.0f;
	octree_marker_array_msg_.markers[0].color.a = 0.5f;

	// free cells
	octree_marker_array_msg_.markers[1].ns ="Free cells";
	octree_marker_array_msg_.markers[1].color.r = 0.0f;
	octree_marker_array_msg_.markers[1].color.g = 1.0f;
	octree_marker_array_msg_.markers[1].color.b = 0.0f;
	octree_marker_array_msg_.markers[1].color.a = 0.5f;

	// unknown cells
	octree_marker_array_msg_.markers[2].ns = "Unknown cells";
	octree_marker_array_msg_.markers[2].color.r = 0.0f;
	octree_marker_array_msg_.markers[2].color.g = 0.0f;
	octree_marker_array_msg_.markers[2].color.b = 1.0f;
	octree_marker_array_msg_.markers[2].color.a = 0.05f;

	// viewpoint
	/*octree_marker_array_msg_.markers[3].ns = "Fringe cells";
	octree_marker_array_msg_.markers[3].color.r = 1.0f;
	octree_marker_array_msg_.markers[3].color.g = 1.0f;
	octree_marker_array_msg_.markers[3].color.b = 0.0f;
	octree_marker_array_msg_.markers[3].color.a = 0.5f;*/

	for (unsigned i = 0; i < octree_marker_array_msg_.markers.size(); ++i)
	{
		octree_marker_array_msg_.markers[i].header.frame_id = pointcloud2_msg->header.frame_id;
		octree_marker_array_msg_.markers[i].header.stamp = ros::Time::now();
		octree_marker_array_msg_.markers[i].id = i;
		octree_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
		octree_marker_array_msg_.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		octree_marker_array_msg_.markers[i].scale.x = lowestRes;
		octree_marker_array_msg_.markers[i].scale.y = lowestRes;
		octree_marker_array_msg_.markers[i].scale.z = lowestRes;

		if (octree_marker_array_msg_.markers[i].points.size() > 0)
			octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
		else
			octree_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
	}

	octree_marker_array_publisher_.publish(octree_marker_array_msg_);

	for (unsigned int i = 0; i < octree_marker_array_msg_.markers.size(); i++)
	{
		if (!octree_marker_array_msg_.markers[i].points.empty())
		{
			octree_marker_array_msg_.markers[i].points.clear();
		}
	}
	octree_marker_array_msg_.markers.clear();
}
}

PLUGINLIB_DECLARE_CLASS(autonomous_mapping, NextBestView, autonomous_mapping::NextBestView, nodelet::Nodelet);
