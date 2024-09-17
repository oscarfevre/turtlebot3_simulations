#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_OCCUPANCY_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_OCCUPANCY_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class CTurtlebot3Occupancy : public rclcpp::Node
{
public:
  CTurtlebot3Occupancy();
  ~CTurtlebot3Occupancy();
  
  std::pair<int, int> WorldToGrid(int x, int y);
  void MarkCellVisited(int grid_x, int grid_y);
  bool IsCellVisited(int grid_x, int grid_y);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  const int GRID_WIDTH = 11;
  const int GRID_HEIGHT = 11;
  const int CELL_SIZE = 1;
  const int ORIGIN_X = 0;
  const int ORIGIN_Y = 1;
  
  
  // Grid of visited cells
  std::vector<std::vector<bool>> visited_grid_;

  // Map data
  std::vector<int8_t> map_data_;
  int map_width_;
  int map_height_;
  double map_resolution_;
  bool map_received_;

  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
};

#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_OCCUPANCY_HPP_
