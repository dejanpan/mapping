#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/pcl_nodelet.h>
#include <vector>
#include <nodelet/nodelet.h>
#include <math.h>
#include "pcl/io/pcd_io.h"
namespace distance_filter
{
class DistanceFilter : public pcl_ros::PCLNodelet
{
public:
	void onInit ();
protected:
	using pcl_ros::PCLNodelet::pnh_;
    std::string input_cloud_topic_;
    std::string output_cloud_topic_;
    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_pub_;
    std::string laser_frame_;
    //float_t threshold_;
    float threshold_;
    int cp;
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);
public:
	DistanceFilter();

	~DistanceFilter();

};
DistanceFilter::DistanceFilter()
{

}
DistanceFilter::~DistanceFilter()
{
	ROS_INFO("Shutting down DistanceFilter");
	//cloud_pub_.shutdown();
}
void DistanceFilter::onInit()
{
	threshold_=6.0;
	cp=0;
	pcl_ros::PCLNodelet::onInit ();
	pnh_->param("input_cloud_topic", input_cloud_topic_, std::string("/nbv_cloud"));
	pnh_->param("output_cloud_topic", output_cloud_topic_, std::string("/output_nbv_cloud"));
	pnh_->param("laser_frame", laser_frame_, std::string("/laser_tilt_link"));
	//pnh_->param("threshold", threshold_, 5.0);
	cloud_sub_=pnh_->subscribe(input_cloud_topic_,1, &DistanceFilter::cloud_cb, this);
	cloud_pub_=pnh_->advertise<sensor_msgs::PointCloud2>(output_cloud_topic_,1);
}
void DistanceFilter::cloud_cb(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg)
{
	ros::Time start_time = ros::Time::now();
	pcl::PointCloud<pcl::PointXYZ> input_pointcloud,output_point_cloud;
	pcl::fromROSMsg(*pointcloud2_msg, input_pointcloud);
	ROS_INFO("number of points in pointcloud is: %d ",input_pointcloud.points.size());
	output_point_cloud.points.resize(input_pointcloud.points.size());
	output_point_cloud.header=input_pointcloud.header;
	geometry_msgs::Point viewpoint;
	viewpoint.x=0.0;
	viewpoint.y=0.0;
	viewpoint.z=0.0;
	for (unsigned int i=0;i<input_pointcloud.points.size();i++)
	{
		float d=sqrt(pow(viewpoint.x-input_pointcloud.points[i].x,2)+pow(viewpoint.y-input_pointcloud.points[i].y,2)+pow(viewpoint.z-input_pointcloud.points[i].z,2));
		if (d<threshold_)
		{
			output_point_cloud.points[cp].x=input_pointcloud.points[i].x;
			output_point_cloud.points[cp].y=input_pointcloud.points[i].y;
			output_point_cloud.points[cp].z=input_pointcloud.points[i].z;
			cp++;

		}
	}
	output_point_cloud.points.resize(cp);
	//pcl::io::savePCDFileASCII ("/home/ghitzarus/Desktop/aggredated_point_cloud.pcd", output_point_cloud);
	cloud_pub_.publish(output_point_cloud);
	ROS_INFO("Number of filtered pointcloud is: %d ",output_point_cloud.points.size());
	/*for (unsigned int i=0;i<output_point_cloud.points.size();i++)
	{
		ROS_INFO("the points from the pointcloud %f, %f, %f",output_point_cloud.points[i].x,output_point_cloud.points[i].y,output_point_cloud.points[i].z);
	}*/
}

}
PLUGINLIB_DECLARE_CLASS(autonomous_mapping, DistanceFilter, distance_filter::DistanceFilter, nodelet::Nodelet);
