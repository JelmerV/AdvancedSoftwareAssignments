#include "assignment_one/brightness_check.hpp"

BrightnessCheck::BrightnessCheck(const rclcpp::NodeOptions & options) : Node("brightness_check", options) {
    // Parse parameters
    parse_parameters();

    // Create subscriber.
    sub_image_ = this->create_subscription<image_>("image", 10, std::bind(&BrightnessCheck::image_callback, this, _1));

    // Create publisher.
    pub_brightness_ = this->create_publisher<string_>("brightness", 1);
}

void BrightnessCheck::parse_parameters() {
    // Declare parameter.
    rcl_interfaces::msg::ParameterDescriptor threshold_desc;
    threshold_desc.description = "Determines treshold for light/dark (value between 0-255)";
    threshold_ = this->declare_parameter("threshold", 100, threshold_desc);
}

void BrightnessCheck::image_callback(const image_::SharedPtr img) const {
    // Initialise variable needed during execution.
    double pixel_sum = 0;
    double average = 0;

    // Process image to get average brightness.
    for (uint i = 0; i < img->data.size(); i++)
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
        pub_msg.data = "There is light";
    }

    // Publish message.
    pub_msg.data += "(average brightness: " + std::to_string(average) + ")";
    pub_brightness_->publish(pub_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(BrightnessCheck)