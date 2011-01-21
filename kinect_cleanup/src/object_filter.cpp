#include <iostream>
//#include <iterator>
//#include <vector>
#include <boost/make_shared.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>  

class ObjectFilter
{
  protected:

    /*ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::NodeHandle nh_*/;

    int indices[2], row[2], col[2];
    double plane[4];
    double std_noise_;

  public:

    ObjectFilter (/*ros::NodeHandle nh,*/ double std_noise)
    {
      std_noise_ = std_noise;
      /*
      nh_ = nh;
      sub_ = nh_.subscribe("input", 100, &ObjectFilter::inputCallback, this);
      ROS_INFO ("[ObjectFilter] Subscribed to: %s", sub_.getTopic ().c_str ());
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 0 );
      ROS_INFO ("[ObjectFilter] Publishing on: %s", pub_.getTopic ().c_str ());
      */
      // -cam "0.005132195795,5.132195795/-0.01565100066,0.03999799863,0.9129999876/-0.00633858571,-0.04486026046,0.1577823364/0.01037906958,-0.9936787337,0.1117803609"
      // -cam "0.00618764,6.18764/0.0325911,-0.0688337,2.01496/0.114809,0.430051,-0.539011/0.0230098,-0.98133,-0.190949/1358,384/8,73"
      plane[0] = 0;
      plane[1] = 0;
      plane[2] = 1;
      plane[3] = 1;
      // tight: 125080 - 184010
      // loose: 119310 - 199400
      indices[0] = 119310;
      indices[1] = 199400;
    }

    void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      sensor_msgs::PointCloud2 cloud_blob;
      pcl::io::loadPCDFile ("1295257088.795222210.pcd", cloud_blob);
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::fromROSMsg (cloud_blob, cloud);
     // ... (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));

      for (int i=0; i<2; i++)
      {
        row[i] = indices[i] / 640;
        col[i] = indices[i] - row[i] * 640;
      }
      for (int r = row[0]; r <= row[1]; r++)
        for (int c = col[0]; c <= col[1]; c++)
        {
          int idx = 640*r+c; // TODO iterator
          if (!std::isnan(cloud.points[idx].x))
          {
            cloud.points[idx].rgb = 0.0;
            double extend_backwards = plane[3]/(cloud.points[idx].x*plane[0]+cloud.points[idx].x*plane[1]+cloud.points[idx].z*plane[2]);
            cloud.points[idx].x *= extend_backwards;
            cloud.points[idx].y *= extend_backwards;
            cloud.points[idx].z *= extend_backwards;
          }
        }
      //pub_.publish (msg);
      pcl::io::savePCDFile ("./1295257088.795222210-fill.pcd", cloud);
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

  //ros::init(argc, argv, "object_filter");
  //ros::NodeHandle nh ("~");
  ObjectFilter f (/*nh,*/ std_noise);
  //ros::spin();
  
  sensor_msgs::PointCloud2 msg;
  f.inputCallback (boost::make_shared<sensor_msgs::PointCloud2> (msg));
  
  return 0;
}
