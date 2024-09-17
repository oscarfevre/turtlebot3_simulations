#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_ODOMETRY_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class CTurtlebot3Odometry : public rclcpp::Node {
public:
  CTurtlebot3Odometry();
  ~CTurtlebot3Odometry();

  double GetPoseX() const;  // Get the robot's x position
  double GetPoseY() const;  // Get the robot's y position
  double GetPoseYaw() const;  // Get the robot's yaw (orientation)

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double robot_pose_x_;
  double robot_pose_y_;
  double robot_pose_yaw_;
  
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_ODOMETRY_HPP_
