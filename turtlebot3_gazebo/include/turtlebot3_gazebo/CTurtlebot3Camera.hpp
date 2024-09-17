#ifndef TURTLEBOT3_GAZEBO__CTURTLEBOT3_CAMERA_HPP_
#define TURTLEBOT3_GAZEBO__CTURTLEBOT3_CAMERA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>  // Use CompressedImage instead of Image

// New Camera Handling Class
class CTurtlebot3Camera : public rclcpp::Node
{
public:
  CTurtlebot3Camera();
  ~CTurtlebot3Camera();

  bool Init();  // Initialize camera node  

private:
  // Camera image subscriber and data publisher
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr object_position_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_camera_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_position_;  // For publishing object position

  // Green detection variables
  const int COLOUR_THRESHOLD = 15000;  // Threshold for green detection
  int green_count;  // Green pixel count
  float object_x_position;  // X position of the green object in the image
  bool object_detected_;
  
  // Function prototypes
  void ImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);  // Image processing callback
  void ObjectPositionCallback();
  void PublishDetection(bool detected, float x_position);  // Publish detection result and object position
};

#endif // TURTLEBOT3_GAZEBO__TURTLEBOT3_CAMERA_HPP_
