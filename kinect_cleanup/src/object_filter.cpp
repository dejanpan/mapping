#include <iostream>
//#include <iterator>
//#include <vector>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>  

#include "kinect_cleanup/FilterObject.h"

// TODO get point index+position+color directly?
struct filter
{
  int row[2], col[2];
  double rgb;
  double plane[4];
};

class ObjectFilter
{
  protected:

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    //pcl_ros::Publisher<pcl::PointXYZRGB> pub_;
    ros::ServiceServer service_;

    std::vector<filter> to_filter;
    double std_noise_;

  public:

    ObjectFilter (ros::NodeHandle &nh, double std_noise) : nh_ (nh), std_noise_ (std_noise)
    {
      sub_ = nh_.subscribe("input", 1, &ObjectFilter::inputCallback, this);
      ROS_INFO ("[ObjectFilter] Subscribed to: %s", sub_.getTopic ().c_str ());
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 0 );
      //pub_ = pcl_ros::Publisher<pcl::PointXYZRGB> (nh_, "output", 1);
      ROS_INFO ("[ObjectFilter] Publishing on: %s", pub_.getTopic ().c_str ());
      service_ = nh_.advertiseService("/filter_object", &ObjectFilter::add2filter, this);
      ROS_INFO ("[ObjectFilter] Advertising service on: %s", service_.getService ().c_str ());

      // -cam "0.005132195795,5.132195795/-0.01565100066,0.03999799863,0.9129999876/-0.00633858571,-0.04486026046,0.1577823364/0.01037906958,-0.9936787337,0.1117803609"
      // -cam "0.00618764,6.18764/0.0325911,-0.0688337,2.01496/0.114809,0.430051,-0.539011/0.0230098,-0.98133,-0.190949/1358,384/8,73"
      //int indices[2];
      //filter f;
      //f.plane[0] = 0;
      //f.plane[1] = 0;
      //f.plane[2] = 1;
      //f.plane[3] = 1;
      // tight: 125080 - 184010
      // loose: 119310 - 199400
      //indices[0] = 119310;
      //indices[1] = 199400;
      //for (int i=0; i<2; i++)
      //{
      //  f.row[i] = indices[i] / 640;
      //  f.col[i] = indices[i] - f.row[i] * 640;
      //}
      //f.rgb = 0.0;
      //to_filter.push_back (f);
    }

    bool add2filter (kinect_cleanup::FilterObject::Request  &req,
                     kinect_cleanup::FilterObject::Response &res)
    {
      filter f;
      f.rgb = req.rgb;
      f.row[0] = req.min_row;
      f.row[1] = req.max_row;
      f.col[0] = req.min_col;
      f.col[1] = req.max_col;
      // TODO simple saving would be OK too?
      for (int i=0; i<4; i++)
        f.plane[i] = req.plane_normal[i];
      to_filter.push_back (f);
      res.error = "filtering " + to_filter.size (); // TODO add out of bounds checks and reflect in result
      return true;
    }

    void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      //sensor_msgs::PointCloud2 cloud_blob;
      //pcl::io::loadPCDFile ("1295257088.795222210.pcd", cloud_blob);
      //pcl::PointCloud<pcl::PointXYZRGB> cloud;
      //pcl::fromROSMsg (cloud_blob, cloud);
      // ... (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));

      // Convert from message to point cloud
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg (*msg, cloud);

      // Update the appropriate points
      for (std::vector<filter>::const_iterator it = to_filter.begin (); it != to_filter.end (); it++)
        for (int r = it->row[0]; r <= it->row[1]; r++)
          for (int c = it->col[0]; c <= it->col[1]; c++)
          {
            int idx = 640*r+c; // TODO iterator?
            if (!std::isnan(cloud.points[idx].x))
            {
              cloud.points[idx].rgb = it->rgb;
              // TODO map to eigen?
              double extend_backwards = it->plane[3]/(cloud.points[idx].x * it->plane[0] + cloud.points[idx].y * it->plane[1] + cloud.points[idx].z * it->plane[2]);
              cloud.points[idx].x *= extend_backwards;
              cloud.points[idx].y *= extend_backwards;
              cloud.points[idx].z *= extend_backwards;
            }
          }

      // Convert to message and publish
      sensor_msgs::PointCloud2 cloud_blob;
      pcl::toROSMsg (cloud, cloud_blob);
      pub_.publish (cloud_blob);

      //pcl::io::savePCDFile ("./1295257088.795222210-fill.pcd", cloud);
    }

};

int main(int argc, char **argv)
{
  double std_noise = 0.0;
  if (argc > 2)
  {
    std::stringstream ss (argv[1]);
    ss >> std_noise;
    std::cerr << "Using a noise level with a STD of " << std_noise << std::endl;
  }

  ros::init(argc, argv, "object_filter");
  ros::NodeHandle nh ("~");
  ObjectFilter f (nh, std_noise);
  ros::spin();
  
  //sensor_msgs::PointCloud2 msg;
  //f.inputCallback (boost::make_shared<sensor_msgs::PointCloud2> (msg));
  
  return 0;
}
