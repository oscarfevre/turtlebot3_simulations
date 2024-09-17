#include "rclcpp/rclcpp.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Camera.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Occupancy.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Laser.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Movement.hpp"
#include "turtlebot3_gazebo/CTurtlebot3Odometry.hpp"

int main(int argc, char **argv)
{
  // Initialize the ROS 2 client library
  rclcpp::init(argc, argv);

  // Create instances of all the necessary classes
  auto occupancy_node = std::make_shared<CTurtlebot3Occupancy>();
  auto laser_node = std::make_shared<CTurtlebot3Laser>();
  auto odometry_node = std::make_shared<CTurtlebot3Odometry>();
  auto movement_node = std::make_shared<CTurtlebot3Movement>(laser_node, occupancy_node);
  auto camera_node = std::make_shared<CTurtlebot3Camera>();

  // Create a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  // Add all nodes to the executor
  executor.add_node(occupancy_node);
  executor.add_node(laser_node);
  executor.add_node(odometry_node);
  executor.add_node(movement_node);
  executor.add_node(camera_node);

  // Spin the executor to process callbacks
  executor.spin();

  // Shutdown the ROS 2 client library
  rclcpp::shutdown();
  return 0;
}
