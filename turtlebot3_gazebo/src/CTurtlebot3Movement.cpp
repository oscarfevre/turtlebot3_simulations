#include "turtlebot3_gazebo/CTurtlebot3Movement.hpp"

using namespace std::chrono_literals;

CTurtlebot3Movement::CTurtlebot3Movement(std::shared_ptr<CTurtlebot3Laser> laser, std::shared_ptr<CTurtlebot3Occupancy> occupancy)
: Node("turtlebot3_movement_node"), turtlebot3_state_num(GET_TB3_DIRECTION), 
  robot_pose_(0.0), robot_pose_x(0.0), robot_pose_y(0.0), prev_robot_pose_(0.0), 
  scan_data_(5, 0.0), object_detected_(false), x_operator(1), y_operator(0),
  laser_(laser), occupancy_(occupancy)
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise cmd_vel publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  // Initialise ROS timer
  update_timer_ = this->create_wall_timer(10ms, std::bind(&CTurtlebot3Movement::UpdateCallback, this));

  RCLCPP_INFO(this->get_logger(), "CTurtlebot3Movement node has been initialised");
}

CTurtlebot3Movement::~CTurtlebot3Movement()
{
  RCLCPP_INFO(this->get_logger(), "CTurtlebot3Movement node has been terminated");
}

void CTurtlebot3Movement::UpdateCallback()
{
  const double escape_range = 65.3 * DEG2RAD;
  const double check_forward_dist = 0.9;
  const double check_side_dist = 2;
  const double check_wall_dist = 1.5;
  int temp_right_x_operator;
  int temp_right_y_operator;

  // Adjust temporary turn direction relative to occupancy map
  AdjustTempOperators(temp_right_x_operator, temp_right_y_operator);

  // Converts every robot pose to a grid cell and mark it as visited
  auto [grid_x, grid_y] = occupancy_->WorldToGrid(CustomRound(robot_pose_x), CustomRound(robot_pose_y));
  occupancy_->MarkCellVisited(grid_x, grid_y); 

  RCLCPP_INFO(this->get_logger(), "Turning right to an unvisited cell, x: %i y: %i Cell Status: %d", grid_x, grid_y, occupancy_->IsCellVisited(grid_x + x_operator, grid_y + y_operator));

  // State machine cases
  switch (turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
      Tb3DirectionHelper(turtlebot3_state_num, grid_x, grid_y, temp_right_x_operator, temp_right_y_operator, check_forward_dist, check_side_dist, check_wall_dist);
      break;
    case TB3_DRIVE_FORWARD:
      DriveForwardHelper(turtlebot3_state_num);
      break;
    case TB3_RIGHT_TURN:
      RightTurnHelper(turtlebot3_state_num, escape_range);
      break;
    case TB3_LEFT_TURN:
      LeftTurnHelper(turtlebot3_state_num, escape_range);
      break;
    case TB3_FINISHED:
      FinishedHelper();
      break;
    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  RCLCPP_INFO(this->get_logger(), "Current state num: %i", turtlebot3_state_num);
}

void CTurtlebot3Movement::AdjustTempOperators(int &temp_right_x_operator, int &temp_right_y_operator)
{
  if (x_operator == 0 && y_operator == 1)
  {
    temp_right_x_operator = 1;
    temp_right_y_operator = 0;
  }
  else if (x_operator == 1 && y_operator == 0)
  {
    temp_right_x_operator = 0;
    temp_right_y_operator = -1;
  }
  else if (x_operator == 0 && y_operator == -1)
  {
    temp_right_x_operator = -1;
    temp_right_y_operator = 0;
  }
  else if (x_operator == -1 && y_operator == 0)
  {
    temp_right_x_operator = 0;
    temp_right_y_operator = 1;
  }
}

void CTurtlebot3Movement::Tb3DirectionHelper(uint8_t &state_num, int grid_x, int grid_y, int temp_right_x_operator, int temp_right_y_operator, double check_forward_dist, double check_side_dist, double check_wall_dist)
{
  if (object_detected_)
  {
    state_num = TB3_FINISHED;
    return;
  }

  if (laser_->GetScanData()[ZERO_DEGREES] > check_forward_dist)
  {
    if ((laser_->GetScanData()[TWO_HUNDRED_NINETY_FIVE_DEGREES] > check_side_dist) &&
        (laser_->GetScanData()[TWO_HUNDRED_SEVENTY_DEGREES] > check_side_dist) &&
        (laser_->GetScanData()[TWO_HUNDRED_FIFTY_DEGREES] > check_side_dist) &&
        (laser_->GetScanData()[NINETY_DEGREES] < check_wall_dist) &&
        (!occupancy_->IsCellVisited(grid_x + temp_right_x_operator, grid_y + temp_right_y_operator)))
    {
      prev_robot_pose_ = robot_pose_;
      state_num = TB3_RIGHT_TURN;
    }
    else
    {
      state_num = TB3_DRIVE_FORWARD;
    }
  }
  else
  {
    if (laser_->GetScanData()[TWO_HUNDRED_SEVENTY_DEGREES] < check_wall_dist)
    {
      prev_robot_pose_ = robot_pose_;
      state_num = TB3_LEFT_TURN;
    }
    else
    {
      prev_robot_pose_ = robot_pose_;
      state_num = TB3_RIGHT_TURN;
    }
  }
}


void CTurtlebot3Movement::DriveForwardHelper(uint8_t &state_num)
{
  UpdateCmdVel(LINEAR_VELOCITY, 0.0);
  state_num = GET_TB3_DIRECTION;
}

void CTurtlebot3Movement::RightTurnHelper(uint8_t &state_num, double escape_range)
{
  if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
  {
    AdjustForwardDirectionRight();
    state_num = GET_TB3_DIRECTION;
  }
  else
  {
    UpdateCmdVel(0.0, -1 * ANGULAR_VELOCITY);
  }
}

void CTurtlebot3Movement::LeftTurnHelper(uint8_t &state_num, double escape_range)
{
  if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range)
  {
    AdjustForwardDirectionLeft();
    state_num = GET_TB3_DIRECTION;
  }
  else
  {
    UpdateCmdVel(0.0, ANGULAR_VELOCITY);
  }
}

void CTurtlebot3Movement::FinishedHelper()
{
  RCLCPP_INFO(this->get_logger(), "Moving towards the green object...");
  if (laser_->GetScanData()[ZERO_DEGREES] > STOP_DISTANCE)
  {
    UpdateCmdVel(LINEAR_VELOCITY, 0.0);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Reached the object. Stopping...");
    UpdateCmdVel(0.0, 0.0);
  }
}

void CTurtlebot3Movement::AdjustForwardDirectionRight()
{
  if (x_operator == 0 && y_operator == 1)
  {
    x_operator = 1;
    y_operator = 0;
  }
  else if (x_operator == 1 && y_operator == 0)
  {
    x_operator = 0;
    y_operator = -1;
  }
  else if (x_operator == 0 && y_operator == -1)
  {
    x_operator = -1;
    y_operator = 0;
  }
  else if (x_operator == -1 && y_operator == 0)
  {
    x_operator = 0;
    y_operator = 1;
  }
}

void CTurtlebot3Movement::AdjustForwardDirectionLeft()
{
  if (x_operator == 0 && y_operator == 1)
  {
    x_operator = -1;
    y_operator = 0;
  }
  else if (x_operator == -1 && y_operator == 0)
  {
    x_operator = 0;
    y_operator = -1;
  }
  else if (x_operator == 0 && y_operator == -1)
  {
    x_operator = 1;
    y_operator = 0;
  }
  else if (x_operator == 1 && y_operator == 0)
  {
    x_operator = 0;
    y_operator = 1;
  }
}

int CTurtlebot3Movement::CustomRound(double value)
{
  if (value > 0.1 && value <= 1.0) {
    return 0;
  } else if (value < -0.1 && value >= -1.0) {
    return -1;
  } else {
    return static_cast<int>(std::round(value));
  }
}

void CTurtlebot3Movement::UpdateCmdVel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;
  cmd_vel_pub_->publish(cmd_vel);
}
