#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//boost
#include <boost/thread/mutex.hpp>

//for moving of the head
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>

//for writting to bag
#include <rosbag/bag.h>


#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;

class PointCloudCapturer
{
  //subscribers/publishers
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;

  std::string input_cloud_topic_;
  bool cloud_received_, move_head_;

  //point head client
  PointHeadClient* point_head_client_;
  //move offset
  double move_offset_y_min_, move_offset_y_max_, step_y_; 
  double move_offset_z_min_, move_offset_z_max_, step_z_;
  double move_offset_x_;
  double current_position_x_, current_position_y_, current_position_z_;
  double rate_;
  boost::thread spin_thread_;
  double EPS;
  rosbag::Bag bag_;
  std::string bag_name_;
  tf::TransformListener tf_;
  std::string to_frame_;
  sensor_msgs::PointCloud2 transformed_cloud_;
public:
  PointCloudCapturer(ros::NodeHandle &n)
    :  nh_(n)
  {
    nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/kinect_head/camera/rgb/points"));
    cloud_sub_= nh_.subscribe (input_cloud_topic_, 10, &PointCloudCapturer::cloud_cb, this);
    ROS_INFO("[PointCloudColorizer:] Subscribing to cloud topic %s", input_cloud_topic_.c_str());

    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    //wait for head controller action server to come up 
    while(!point_head_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
      ROS_INFO("Waiting for the point_head_action server to come up");
    }
    cloud_received_ = false;
    nh_.param("move_offset_y_min", move_offset_y_min_, -1.5);
    nh_.param("move_offset_y_max", move_offset_y_max_, 1.5);
    nh_.param("step_y", step_y_, 0.3);
    nh_.param("move_offset_z_min", move_offset_z_min_, 0.3);
    nh_.param("move_offset_z_max", move_offset_z_max_, 1.5);
    nh_.param("step_z", step_z_, 0.3);
    nh_.param("move_offset_x", move_offset_x_, 1.0);
    nh_.param("bag_name", bag_name_, std::string("bosch_kitchen_tr.bag"));
    nh_.param("to_frame", to_frame_, std::string("base_link"));
    nh_.param("rate", rate_, 1.0);
    current_position_x_ = move_offset_x_;
    current_position_y_ = move_offset_y_min_;
    current_position_z_ = move_offset_z_min_;
    move_head("base_link", current_position_x_, current_position_y_, current_position_z_);
    EPS = 1e-5;
    //thread ROS spinner
    spin_thread_ = boost::thread (boost::bind (&ros::spin));
    bag_.open(bag_name_, rosbag::bagmode::Write);
  }

  ~PointCloudCapturer()
  {
    bag_.close();
    delete point_head_client_;
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& pc)
  {
    if (!cloud_received_)
    {
      //transform to target frame
      bool found_transform = tf_.waitForTransform(pc->header.frame_id, to_frame_,
                                                  pc->header.stamp, ros::Duration(10.0));
      if (found_transform)
      {
        //ROS_ASSERT_MSG(found_transform, "Could not transform to camera frame");
        tf::StampedTransform transform;
        tf_.lookupTransform(to_frame_,pc->header.frame_id, pc->header.stamp, transform);
        pcl_ros::transformPointCloud(to_frame_, transform, *pc, transformed_cloud_);
        ROS_DEBUG("[TransformPointcloudNode:] Point cloud published in frame %s", transformed_cloud_.header.frame_id.c_str());
      }
      else
      {
        ROS_ERROR("No transform found!!!");
        return;
      }
      bag_.write(input_cloud_topic_, transformed_cloud_.header.stamp, transformed_cloud_);
      ROS_INFO("Wrote cloud to %s", bag_name_.c_str());
      cloud_received_ = true;
    }
  }

  void move_head (std::string frame_id, double x, double y, double z)
  {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;
    
    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;
    
    //we are pointing the wide_stereo camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "narrow_stereo_optical_frame";
    
    //take at least 2 seconds to get there
    goal.min_duration = ros::Duration(1);
    
    //and go no faster than 0.1 rad/s
    goal.max_velocity = 0.5;
    
    //send the goal
    point_head_client_->sendGoal(goal);
    
    //wait for it to get there
    bool finished_within_time = point_head_client_->waitForResult(ros::Duration(20.0));
    if (!finished_within_time)
    {
      ROS_ERROR("Head did not move to pose: %f %f %f", point.point.x, point.point.y, point.point.z);
    }
    else
    {
      actionlib::SimpleClientGoalState state = point_head_client_->getState();
      bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
      if(success)
        ROS_INFO("Action move head finished: %s position: %f %f %f", state.toString().c_str(), point.point.x, point.point.y, point.point.z);
      else
        ROS_ERROR("Action move head failed: %s", state.toString().c_str());
    }
  }

  void spin ()
    {
      ros::Rate loop_rate(rate_);
      while (ros::ok())
      {
        if (cloud_received_)
        {
          // swing left to right and right to left by increasing y step
          if ( (((move_offset_y_min_ < current_position_y_) && (current_position_y_ < move_offset_y_max_))
                || (fabs(current_position_y_ - move_offset_y_min_) < EPS) || (fabs(current_position_y_ - move_offset_y_max_) < EPS))
               && 
               ((current_position_z_ < move_offset_z_max_) 
                || (fabs(current_position_z_ - move_offset_z_min_) < EPS) || (fabs(current_position_z_ - move_offset_z_max_) < EPS)) )
          {
            current_position_y_ += step_y_;
            ROS_INFO("in left to right");
          }
     
          // increase z step
          else if ( ((current_position_y_ < move_offset_y_min_) || (current_position_y_ > move_offset_y_max_)) 
                    && 
                    (current_position_z_ < move_offset_z_max_) )
          {
            current_position_z_ += step_z_;
            step_y_ = -step_y_;
            current_position_y_ += step_y_;
            ROS_INFO("in increase z");
          }

          // check if we are done
          else if ( ((current_position_y_ < move_offset_y_min_) || (current_position_y_ > move_offset_y_max_)) 
                    && 
                    ((current_position_z_ > move_offset_z_max_) || (fabs(current_position_z_ - move_offset_z_max_) < EPS)) )
          {
            ROS_INFO("DONE - exiting");
            ros::shutdown();
          }
          else
          {
            ROS_INFO("Not an option");
          }
          move_head("base_link", current_position_x_, current_position_y_, current_position_z_);
          cloud_received_ = false; 
        }
        ros::spinOnce();
        loop_rate.sleep();
       }
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_capturer_node");
  ros::NodeHandle n("~");
  PointCloudCapturer pp(n);
  pp.spin();
}
