// C++ includes.
#include <functional>
#include <memory>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// ROS2 Message includes.
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifndef BRIGHTNESS_CHECH_HPP
#define BRIGHTNESS_CHECH_HPP

using std::placeholders::_1;

class BrightnessCheck : public rclcpp::Node {
    public:
        BrightnessCheck(const rclcpp::NodeOptions & options);
    private:
        using image_  = sensor_msgs::msg::Image;
        using string_ = std_msgs::msg::String;

        void parse_parameters();

        void image_callback(const image_::SharedPtr img);

        int threshold_;

        rclcpp::Subscription<image_>::SharedPtr sub_image_;
        rclcpp::Publisher<string_>::SharedPtr pub_brightness_;
};

#endif /* BRIGHTNESS_CHECH_HPP */