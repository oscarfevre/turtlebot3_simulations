#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_MOVEMENT_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_MOVEMENT_HPP_

#include <memory>

#include "CTurtlebot3Laser.hpp"
#include "CTurtlebot3Occupancy.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include <chrono>

class CTurtlebot3Movement : public rclcpp::Node {
public:
  CTurtlebot3Movement(std::shared_ptr<CTurtlebot3Laser> laser, std::shared_ptr<CTurtlebot3Occupancy> occupancy);
  ~CTurtlebot3Movement();
  
  
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  enum Turtlebot3State {
    GET_TB3_DIRECTION,
    TB3_DRIVE_FORWARD,
    TB3_RIGHT_TURN,
    TB3_LEFT_TURN,
    TB3_FINISHED
  };
  
  const int DEG2RAD = (M_PI / 180.0);
  const int RAD2DEG = (180.0 / M_PI);
  
  const int ZERO_DEGREES = 0;
  const int NINETY_DEGREES = 1;
  const int TWO_HUNDRED_FIFTY_DEGREES = 2;
  const int TWO_HUNDRED_SEVENTY_DEGREES = 3;
  const int TWO_HUNDRED_NINETY_FIVE_DEGREES = 4;
  
  const int LINEAR_VELOCITY = 0.2;
  const int ANGULAR_VELOCITY = 1.0;
  
  const int STOP_DISTANCE = 0.3;

  // State variables
  uint8_t turtlebot3_state_num;
  double robot_pose_;
  double robot_pose_x;
  double robot_pose_y;
  double prev_robot_pose_;
  std::vector<double> scan_data_;
  bool object_detected_;
  int x_operator;
  int y_operator;

  std::shared_ptr<CTurtlebot3Laser> laser_;
  std::shared_ptr<CTurtlebot3Occupancy> occupancy_;

  void UpdateCallback();  // Main function for stepping through the state machine
  void AdjustTempOperators(int &temp_right_x_operator, int &temp_right_y_operator);
  void Tb3DirectionHelper(uint8_t &state_num, int grid_x, int grid_y, int temp_right_x_operator, int temp_right_y_operator, double check_forward_dist, double check_side_dist, double check_wall_dist);
  void DriveForwardHelper(uint8_t &state_num);
  void RightTurnHelper(uint8_t &state_num, double escape_range);
  void LeftTurnHelper(uint8_t &state_num, double escape_range);
  void FinishedHelper();
  void AdjustForwardDirectionRight();
  void AdjustForwardDirectionLeft();
  void UpdateCmdVel(double linear, double angular);  
  int CustomRound(double value);
};

#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_MOVEMENT_HPP_
