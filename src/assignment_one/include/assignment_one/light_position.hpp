// C++ includes.
#include <functional>
#include <memory>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp_components/register_node_macro.hpp"

// ROS2 Message includes.
#include "sensor_msgs/msg/image.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

#ifndef LIGHT_POSITION_HPP
#define LIGHT_POSITION_HPP

using std::placeholders::_1;

class LightPosition : public rclcpp::Node {
    public:
        // LightPosition(const rclcpp::NodeOptions & options);
        LightPosition();
    private:
        using image_  = sensor_msgs::msg::Image;
        using point2_ = asdfr_interfaces::msg::Point2;

        void parse_parameters();

        void image_callback(const image_::SharedPtr img);

        int threshold_;

        rclcpp::Subscription<image_>::SharedPtr sub_img_;
        rclcpp::Publisher<point2_>::SharedPtr pub_img_cog_;
        rclcpp::Publisher<image_>::SharedPtr pub_mono_img_;
};

#endif /* LIGHT_POSITION_HPP */