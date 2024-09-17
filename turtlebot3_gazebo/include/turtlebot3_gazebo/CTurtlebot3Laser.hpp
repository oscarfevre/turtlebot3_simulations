#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_LASER_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_LASER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <array>

class CTurtlebot3Laser : public rclcpp::Node {
public:
  CTurtlebot3Laser();
  ~CTurtlebot3Laser();

  std::array<double, 5> GetScanData();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  
  std::array<double, 5> scan_data_;  // Processed laser scan data

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};


#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_LASER_HPP_
