#include "turtlebot3_gazebo/CTurtlebot3Odometry.hpp"
 

CTurtlebot3Odometry::CTurtlebot3Odometry()
: Node("turtlebot3_odometry_node"),
  robot_pose_x_(0.0), robot_pose_y_(0.0), robot_pose_yaw_(0.0)
{
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&CTurtlebot3Odometry::OdomCallback, this, std::placeholders::_1)
  );
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 odometry node has been initialised");
}
  

CTurtlebot3Odometry::~CTurtlebot3Odometry()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 odometry node has been terminated");
}

void CTurtlebot3Odometry::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Extract position (x, y) from the odometry message
  robot_pose_x_ = msg->pose.pose.position.x;
  robot_pose_y_ = msg->pose.pose.position.y;

  // Extract orientation (yaw) from the quaternion
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);  // Extract yaw (rotation around z-axis)
  robot_pose_yaw_ = yaw;
  
  RCLCPP_INFO(this->get_logger(), "Odometry - x: %.2f, y: %.2f, yaw: %.2f", robot_pose_x_, robot_pose_y_, robot_pose_yaw_);
}

double CTurtlebot3Odometry::GetPoseYaw() const {
  return robot_pose_yaw_;
}

double CTurtlebot3Odometry::GetPoseX() const {
  return robot_pose_x_;
}

double CTurtlebot3Odometry::GetPoseY() const {
  return robot_pose_y_;
}
