#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "pcl/common/angles.h"
#include "LinearMath/btQuaternion.h"
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_msgs/SetPeriodicCmd.h>

class AutonomousExploration
{
  public:
    AutonomousExploration();
    ~AutonomousExploration();
    void autonomousExplorationCallBack(const geometry_msgs::Pose& pose_msg);
    void pointcloudCallBack(const sensor_msgs::PointCloud& pointcloud_msg);
    void moveRobot(geometry_msgs::Pose goal_pose);
    void moveTorso(double position, double velocity, std::string direction);
    void setLaserProfile(std::string mode);
    void run();
    void spin();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher pointcloud_publisher_;
    std::string subscribe_pose_topic_;
    boost::thread spin_thread_;
    geometry_msgs::Pose pose_msg_;
    bool get_pointcloud_, received_pose_;
    //! Client for changing laser scan speed
    ros::ServiceClient tilt_laser_client_;
};

AutonomousExploration::AutonomousExploration():nh_("~")
{
  nh_.param("subscribe_pose_topic", subscribe_pose_topic_, std::string("/robot_pose"));
  ROS_INFO("autonomous_exploration node is up and running.");
  get_pointcloud_ = false;
  received_pose_ = false;
  run();
}
AutonomousExploration::~AutonomousExploration()
{
  ROS_INFO("Shutting down autonomous_exploration node.");
}

void AutonomousExploration::run()
{
  pose_subscriber_ = nh_.subscribe(subscribe_pose_topic_, 100, &AutonomousExploration::autonomousExplorationCallBack, this);
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud>("pointcloud", 100);
  pointcloud_subscriber_ = nh_.subscribe("/shoulder_cloud", 100, &AutonomousExploration::pointcloudCallBack, this);
  tilt_laser_client_ = nh_.serviceClient<pr2_msgs::SetPeriodicCmd>("laser_tilt_controller/set_periodic_cmd");
  spin_thread_ = boost::thread (boost::bind (&ros::spin));
  //ros::spin();
}
void AutonomousExploration::spin()
{
  ros::Rate loop(1);
  while(nh_.ok())
  {
    loop.sleep();
    if(received_pose_ )
    {
      //move robot to the received pose
      moveRobot(pose_msg_);
      //rise the spine up to scan
      setLaserProfile("scan");
      moveTorso(0.3, 1.0, "up");
      double angles = 0.0;
      while(angles <= 360)
      {
        geometry_msgs::Pose new_pose;
        btQuaternion q ( pcl::deg2rad(angles), 0, 0);
        new_pose.position.x = pose_msg_.position.x;
        new_pose.position.y = pose_msg_.position.y;
        new_pose.position.z = pose_msg_.position.z;
        new_pose.orientation.x = q.x();
        new_pose.orientation.y = q.y();
        new_pose.orientation.z = q.z();
        new_pose.orientation.w = q.w();
        moveRobot(new_pose);

        //get a point cloud
        //get_pointcloud_ = true;

        //TODO: Use laser service to sync
        //Wait 11 seconds before taking a scan
        sleep(2);
        angles += 30.0;
      }
      received_pose_ = false;
      //lower the spine to navigate some place else
      moveTorso(0.01, 1.0, "down");
      setLaserProfile("navigation");
    }
  }
}
void AutonomousExploration::moveRobot(geometry_msgs::Pose goal_pose)
{
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose = goal_pose;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  ROS_INFO("Pose Position x: %f, y:%f, z:%f Orientation x:%f, y:%f, z:%f, w:%f", goal.target_pose.pose.position.x,
      goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,
      goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot has moved to the goal pose");
  }
  else
  {
    ROS_ERROR("Error in moving robot to goal pose.");
    exit(1);
  }
}

void AutonomousExploration::moveTorso(double position, double velocity, std::string direction)
{
  actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> tc ("torso_controller/position_joint_action", true);

  //wait for the action server to come up
  while(!tc.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the torso action server to come up");
  }
  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = position;  //all the way up is 0.3
  goal.min_duration = ros::Duration(2.0);
  goal.max_velocity = velocity;

  ROS_INFO("Sending '%s' goal", direction.c_str());
  tc.sendGoal(goal);
  tc.waitForResult();

  if(tc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Torso moved %s.", direction.c_str());
  }
  else
  {
    ROS_ERROR("Error in moving torso %s.", direction.c_str());
  }
}

void AutonomousExploration::setLaserProfile(std::string mode)
{
  while ( !ros::service::waitForService("laser_tilt_controller/set_periodic_cmd", ros::Duration(2.0)) && nh_.ok() )
  {
    ROS_INFO("Waiting for tilt laser periodic command service to come up");
  }

  pr2_msgs::SetPeriodicCmd::Request req;
  pr2_msgs::SetPeriodicCmd::Response res;
  req.command.header.frame_id = "/map";
  req.command.header.stamp = ros::Time::now();
  req.command.header.seq = 0;

  req.command.profile = "linear";

  if(mode == "scan")
  {
    req.command.period = 20.0;
    req.command.amplitude = 0.8;
    req.command.offset = 0.3;
  }
  else if(mode == "navigation")
  {
  }
  if(!tilt_laser_client_.call(req,res))
  {
    ROS_ERROR("Tilt laser service call failed.\n");
    exit(1);
  }
  else
  {
    ROS_INFO("Tilt laser service successfully called.\n");
  }

}
void AutonomousExploration::pointcloudCallBack(const sensor_msgs::PointCloud& pointcloud_msg)
{
  if(get_pointcloud_)
  {
    ROS_INFO("pointcloud received");
    pointcloud_publisher_.publish(pointcloud_msg);
    ROS_INFO("pointcloud published");
    get_pointcloud_ = false;
  }
}
void AutonomousExploration::autonomousExplorationCallBack(const geometry_msgs::Pose& pose_msg)
{
  pose_msg_ = pose_msg;
  received_pose_ = true;
  ROS_INFO("Received a Pose");

}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_exploration");
  AutonomousExploration autonomous_exploration;
  autonomous_exploration.spin();
  return 0;
}

