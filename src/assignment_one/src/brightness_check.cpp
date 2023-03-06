// C++ includes.
#include <string>

// ROS2 Message includes.
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"

// ROS2 internal includes.
#include "assignment_one/brightness_check.hpp"
#include "assignment_one/constants.hpp"

BrightnessCheck::BrightnessCheck(const rclcpp::NodeOptions & options) : Node("brightness_check", options) {
    // Parse parameters.
    parse_parameters();

    // Create subscriber.
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic 'image'");
    sub_image_ = this->create_subscription<image_>("image", 10, std::bind(&BrightnessCheck::image_callback, this, _1));

    // Create publisher.
    pub_brightness_ = this->create_publisher<string_>("brightness", 10);
}

void BrightnessCheck::parse_parameters() {
    // Set 'threshold' parameter description.
    rcl_interfaces::msg::ParameterDescriptor threshold_desc;
    threshold_desc.description = "Determines treshold for what is considered light and dark.";
    
    // Set allowed integer range for 'threshold' parameter.
    rcl_interfaces::msg::IntegerRange threshold_range;
    threshold_range.from_value = assignment_one::THRESHOLD_MIN;
    threshold_range.step = assignment_one::THRESHOLD_STEP;
    threshold_range.to_value = assignment_one::THRESHOLD_MAX;
    threshold_desc.integer_range = {threshold_range};

    // Declare 'threshold' parameter.
    threshold_ = this->declare_parameter("threshold", assignment_one::THRESHOLD_DEFAULT, threshold_desc);
    RCLCPP_INFO(this->get_logger(), "Using threshold: %d", threshold_);
}

void BrightnessCheck::image_callback(const image_::SharedPtr img) {
    // Initialise variable needed during execution.
    double pixel_sum = 0;
    double average = 0;

    // Update the threshold value.
    threshold_ = this->get_parameter("threshold").as_int();
    
    // Process image to get average brightness.
    for (unsigned int i = 0; i < img->data.size(); i++)
    {
      pixel_sum += img->data[i];
    }

    // Calculate average brightness.
    average = pixel_sum / img->data.size();

    // Initialise String message.
    string_ pub_msg;
    pub_msg.data = "It is dark";

    // Check if average meets threshold.
    if (average > threshold_) {
        pub_msg.data = "It is light";
    }

    // Publish message.
    pub_msg.data += " (average brightness: " + std::to_string(average) + ")";
    RCLCPP_INFO(this->get_logger(), "Publishing brightness information.");
    pub_brightness_->publish(pub_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(BrightnessCheck)