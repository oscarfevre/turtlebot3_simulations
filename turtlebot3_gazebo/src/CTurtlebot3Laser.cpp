#include "turtlebot3_gazebo/CTurtlebot3Laser.hpp"

CTurtlebot3Laser::CTurtlebot3Laser()
: Node("turtlebot3_laser_node"), scan_data_{{0.0, 0.0, 0.0, 0.0, 0.0}}
{
  // Subscribe to LaserScan topic
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),\
    std::bind(&CTurtlebot3Laser::scan_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 laser node has been terminated");
}

CTurtlebot3Laser::~CTurtlebot3Laser()
{
  RCLCPP_INFO(this->get_logger(), "Turtlebot3 laser node has been terminated");
}

void CTurtlebot3Laser::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  uint16_t scan_angle[5] = {0, 90, 250, 270, 295};  // Specified angles to read

  for (int num = 0; num < 5; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = static_cast<double>(msg->range_max);
    } else {
      scan_data_[num] = static_cast<double>(msg->ranges.at(scan_angle[num]));
    }
  }
}

std::array<double, 5> CTurtlebot3Laser::GetScanData() {
  return scan_data_;
}
