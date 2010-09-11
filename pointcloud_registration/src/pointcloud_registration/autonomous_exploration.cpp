#include "ros/ros.h"
#include "rosbag/bag.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "pcl/common/angles.h"

class AutonomousExploration
{
  public:
    AutonomousExploration();
    ~AutonomousExploration();
    void autonomousExplorationCallBack(const geometry_msgs::Pose& pose_msg);
    void pointcloudCallBack(const sensor_msgs::PointCloud& pointcloud_msg);
    void moveRobot(geometry_msgs::Pose goal_pose);
    void run();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_subscriber_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher pointcloud_publisher_;
    std::string subscribe_pose_topic_;
};

AutonomousExploration::AutonomousExploration():nh_("~")
{
  nh_.param("subscribe_pose_topic", subscribe_pose_topic_, std::string("/robot_pose"));
  ROS_INFO("autonomous_exploration node is up and running.");
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
  ros::spin();
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

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Robot has moved to the goal pose");
  }
  else
  {
    ROS_ERROR("Error in moving robot to goal pose.");
    ROS_BREAK();
  }
}

void AutonomousExploration::pointcloudCallBack(const sensor_msgs::PointCloud& pointcloud_msg)
{
  ROS_INFO("pointcloud received");
  pointcloud_publisher_.publish(pointcloud_msg);
  ROS_INFO("pointcloud published");
}
void AutonomousExploration::autonomousExplorationCallBack(const geometry_msgs::Pose& pose_msg)
{
  double angles = 0.0;
  ROS_INFO("Received a Pose");
  //move robot to the received pose
  moveRobot(pose_msg);

  while(angles != 360)
  {
    geometry_msgs::Pose new_pose;
    new_pose.orientation.w = pcl::deg2rad(angles);
    moveRobot(new_pose);

    //Wait 11 seconds before taking a scan
    sleep(11);
    //get a point cloud
    pointcloud_subscriber_ = nh_.subscribe("/shoulder_cloud", 100, &AutonomousExploration::pointcloudCallBack, this);
    angles += 30.0;
  }
}

int main( int argc, char** argv)
{
  ros::init(argc, argv, "autonomous_exploration");
  AutonomousExploration autonomous_exploration;
  return 0;
}

