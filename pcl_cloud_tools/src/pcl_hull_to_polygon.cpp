#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/subscriber.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

ros::Subscriber PointCloudSubscriber;
ros::Publisher polygon_pub;

void pointCloudToPolygonCb(const sensor_msgs::PointCloud2ConstPtr &input)
{
  PointCloud cloud;
  pcl::fromROSMsg (*input, cloud);
  geometry_msgs::PolygonStamped polygon;

  polygon.header = cloud.header;
  BOOST_FOREACH(const pcl::PointXYZ &pt, cloud.points)
  {
    geometry_msgs::Point32 out_pt;
    out_pt.x = pt.x;
    out_pt.y = pt.y;
    out_pt.z = pt.z;
    polygon.polygon.points.push_back(out_pt);
  }

  polygon_pub.publish(polygon);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcl_hull_to_polygon");
  ros::NodeHandle nh("~");
  polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("output", 10);
  PointCloudSubscriber = nh.subscribe("input", 10, pointCloudToPolygonCb);

  ros::spin();
}
