#include <iostream>
//#include <iterator>
//#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class ObjectFilter
{
  protected:

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;

    double std_noise_;

  public:

    ObjectFilter (ros::NodeHandle nh, double std_noise)
    {
      std_noise_ = std_noise;
      nh_ = nh;
      sub_ = nh_.subscribe("input", 100, &ObjectFilter::inputCallback, this);
      ROS_INFO ("[ObjectFilter] Subscribed to: %s", sub_.getTopic ().c_str ());
      pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 0 );
      ROS_INFO ("[ObjectFilter] Publishing on: %s", pub_.getTopic ().c_str ());
    }

    void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
      pub_.publish (msg);
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
  ObjectFilter of (nh, std_noise);
  ros::spin();
  return 0;
}
