
#include "turtlebot3_gazebo/CTurtlebot3Camera.hpp"
/************************************************************
** CTurtlebot3Camera Class: Detect Green Object
************************************************************/

// Constructor
CTurtlebot3Camera::CTurtlebot3Camera()
: Node("turtlebot3_camera_node"), green_count(0), object_x_position(0.0),\
  object_detected_(false)

{
  RCLCPP_INFO(this->get_logger(), "Initializing camera detector node");
  bool ret = Init();
  if (!ret) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera node");
  } else {
    RCLCPP_INFO(this->get_logger(), "Camera node initialized successfully");
  }
}

// Destructor
CTurtlebot3Camera::~CTurtlebot3Camera()
{
  RCLCPP_INFO(this->get_logger(), "Camera shutting down");
  rclcpp::shutdown();
}

// Initialisation function
bool CTurtlebot3Camera::Init()
{
  // Subscribe to compressed image topic
  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/camera/image_raw/compressed", 30,
      std::bind(&CTurtlebot3Camera::ImageCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Image subscriber initialized [SUCCESS]");

  // Publisher for camera detection
  pub_camera_ = this->create_publisher<std_msgs::msg::Bool>("/cameradata", 30);
  pub_position_ = this->create_publisher<geometry_msgs::msg::Point>("/object_position", 30);  // For publishing object position
  RCLCPP_INFO(this->get_logger(), "Publishers initialized [SUCCESS]");

  return true;
}

void CTurtlebot3Camera::ImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
  try {
    // Convert compressed ROS image to OpenCV Mat
    cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED);

    // Convert the image from BGR to HSV
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Define HSV range for detecting green (tuned for the green object)
    cv::Scalar lower_green(40, 50, 50);  // Lower bound for green
    cv::Scalar upper_green(80, 255, 255);  // Upper bound for green

    // Create a mask to detect green pixels in the image
    cv::Mat green_mask;
    cv::inRange(hsv_image, lower_green, upper_green, green_mask);

    // Count non-zero pixels in the mask (i.e., green pixels)
    int green_count = cv::countNonZero(green_mask);

    // Find the centroid of the green region (if any green is detected)
    if (green_count > COLOUR_THRESHOLD) {
      // Compute the moments of the mask image
      cv::Moments m = cv::moments(green_mask, true);
      double object_x_position = m.m10 / m.m00;  // Centroid x-position (horizontal position in the frame)

      // Normalize the x-position to be between -1 and 1 (left to right of the image)
      object_x_position = (object_x_position / image.cols) * 2 - 1;

      // Green object detected
      PublishDetection(true, object_x_position);
      RCLCPP_INFO(this->get_logger(), "Green object detected at x-position: %.2f", object_x_position);
    }
    else {
      // No green object detected
      PublishDetection(false, 0.0);
    }
  }
  catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void CTurtlebot3Camera::ObjectPositionCallback()
{
  // Object detected, set object_detected_ flag to true
  object_detected_ = true;
  RCLCPP_INFO(this->get_logger(), "Green object detected. Moving towards the object...");
}

// Function to publish the detection result and object position
void CTurtlebot3Camera::PublishDetection(bool detected, float x_position)
{
  // Publish detection status
  std_msgs::msg::Bool detection_msg;
  detection_msg.data = detected;
  pub_camera_->publish(detection_msg);

  // Publish position of the detected object (if detected)
  if (detected) {
    geometry_msgs::msg::Point position_msg;
    position_msg.x = x_position;  // Publish the horizontal position
    pub_position_->publish(position_msg);
  }
}
