#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "LinearMath/btTransform.h"

#include <pcl_ros/publisher.h>

#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include <vector>

/**
   @file pcl_normal_visualization.cpp

   @brief  Visualization of normals (pcl pointcloud).

   @par Advertises
   - \b normals_marker topic

   @par Subscribes
   - \b /points2_out topic

   @par Parameters
   - \b input_cloud_topic
*/

class NormalViz
{
protected:
  ros::NodeHandle nh_;

  //parameters
  std::string input_cloud_topic_;

  sensor_msgs::PointCloud2 cloud_in_;
  visualization_msgs::MarkerArray normals_marker_array_msg_;

  ros::Subscriber cloud_sub_;
  ros::Publisher normals_marker_array_publisher_;
  ros::Publisher normals_marker_publisher_;

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pointcloud2_msg);

public:
  NormalViz(ros::NodeHandle &anode);
  ~NormalViz();
};


NormalViz::NormalViz (ros::NodeHandle &anode) : nh_(anode) {
  nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/points2_out"));

  cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &NormalViz::cloud_cb, this);

  normals_marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("normals_marker_array", 100);
  normals_marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("normals_marker", 100);
}


NormalViz::~NormalViz()
{
  ROS_INFO("Shutting down pcl_normal_visualization node!");

  normals_marker_array_publisher_.shutdown();
  normals_marker_publisher_.shutdown();
}


/**
 * \brief cloud callback
 * \param pointcloud2_msg input point cloud to be processed
 */
void NormalViz::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc2_msg) {

  static unsigned int lastsize = 0;

  ROS_INFO("Received PointCloud message.");

  unsigned int validfields = 0;
  for (unsigned int i = 0; i < pc2_msg->fields.size(); i++) {
    if (!strcmp(pc2_msg->fields[i].name.c_str(), "x"))
      validfields++;
    else if (!strcmp(pc2_msg->fields[i].name.c_str(), "y"))
      validfields++;
    else if (!strcmp(pc2_msg->fields[i].name.c_str(), "z"))
      validfields++;
    else if (!strcmp(pc2_msg->fields[i].name.c_str(), "normal_x"))
      validfields++;
    else if (!strcmp(pc2_msg->fields[i].name.c_str(), "normal_y"))
      validfields++;
    else if (!strcmp(pc2_msg->fields[i].name.c_str(), "normal_z"))
      validfields++;
    ROS_INFO("read field: %s", pc2_msg->fields[i].name.c_str());
  }

  //don't process if neccessary field were not found
  if ( validfields != 6 ) {
    ROS_INFO("PointCloud message does not contain neccessary fields!");
    return;
  }

  //Converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
  pcl::PointCloud<pcl::PointNormal> pcl_cloud;
  pcl::fromROSMsg(*pc2_msg, pcl_cloud);

  unsigned int size = pc2_msg->height * pc2_msg->width;
  ROS_INFO("size: %i", size);
  if (size >= lastsize) {
    normals_marker_array_msg_.markers.resize(size);
  }

  for (unsigned int i = 0; i < size; ++i)
    {
      geometry_msgs::Point pos;
      pos.x = pcl_cloud.points[i].x;
      pos.y = pcl_cloud.points[i].y;
      pos.z = pcl_cloud.points[i].z;
      normals_marker_array_msg_.markers[i].pose.position = pos;
      //axis-angle rotation
      btVector3 axis(pcl_cloud.points[i].normal[0],-pcl_cloud.points[2].normal[0],pcl_cloud.points[i].normal[1]);
      btVector3 marker_axis(0, 0, 1);
      btQuaternion qt(marker_axis.cross(axis), marker_axis.angle(axis));
      geometry_msgs::Quaternion quat_msg;
      tf::quaternionTFToMsg(qt, quat_msg);
      normals_marker_array_msg_.markers[i].pose.orientation = quat_msg;

      normals_marker_array_msg_.markers[i].header.frame_id = pcl_cloud.header.frame_id;
      normals_marker_array_msg_.markers[i].header.stamp = pcl_cloud.header.stamp;
      normals_marker_array_msg_.markers[i].id = i;
      normals_marker_array_msg_.markers[i].ns = "Normals";
      normals_marker_array_msg_.markers[i].color.r = 1.0f;
      normals_marker_array_msg_.markers[i].color.g = 0.0f;
      normals_marker_array_msg_.markers[i].color.b = 0.0f;
      normals_marker_array_msg_.markers[i].color.a = 0.5f;
      normals_marker_array_msg_.markers[i].lifetime = ros::Duration::Duration();
      normals_marker_array_msg_.markers[i].type = visualization_msgs::Marker::ARROW;
      normals_marker_array_msg_.markers[i].scale.x = 0.2;
      normals_marker_array_msg_.markers[i].scale.y = 0.2;
      normals_marker_array_msg_.markers[i].scale.z = 0.2;

      normals_marker_array_msg_.markers[i].action = visualization_msgs::Marker::ADD;
    }

  if (lastsize > size) {
    for (unsigned int i = size; i < lastsize; ++i) {
      normals_marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  lastsize = size;

  normals_marker_array_publisher_.publish(normals_marker_array_msg_);

  ROS_INFO("Published marker array with normals");
}

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "pcl_normal_visualization");
  ros::NodeHandle nh("~");
  NormalViz n (nh);
  ROS_INFO("Node up and running...");
  ros::spin ();

  return (0);
}
