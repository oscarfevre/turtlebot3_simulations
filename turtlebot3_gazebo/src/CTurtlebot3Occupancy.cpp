#include "turtlebot3_gazebo/CTurtlebot3Occupancy.hpp"
 
CTurtlebot3Occupancy::CTurtlebot3Occupancy()
: Node("turtlebot3_occupancy_node")
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 occupancy node has been initialised");
}

CTurtlebot3Occupancy::~CTurtlebot3Occupancy()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 occupancy node has been terminated");
}

void CTurtlebot3Occupancy::MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Process the map data
  map_data_ = msg->data;
  map_width_ = msg->info.width;
  map_height_ = msg->info.height;
  map_resolution_ = msg->info.resolution;
  map_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Received map of size %d x %d with resolution %.2f", map_width_, map_height_, map_resolution_);
}

std::pair<int, int> CTurtlebot3Occupancy::WorldToGrid(int x, int y) {
  int grid_x = static_cast<int>((x - ORIGIN_X) / CELL_SIZE + 3);
  int grid_y = static_cast<int>((y - ORIGIN_Y) / CELL_SIZE);
  return {grid_x, grid_y};
}
  
bool CTurtlebot3Occupancy::IsCellVisited(int x, int y) {
  if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
    return visited_grid_[x][y];
  }
  return true; // Out of bounds is treated as visited
}

void CTurtlebot3Occupancy::MarkCellVisited(int x, int y) {
  if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
    visited_grid_[x][y] = true;
  }
}

