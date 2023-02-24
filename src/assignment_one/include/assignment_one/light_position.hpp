#include <chrono>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

#ifndef LIGHT_POSITION_HPP
#define LIGHT_POSITION_HPP

using std::placeholders::_1;

class LightPosition : public rclcpp::Node {
    public:
        LightPosition(const rclcpp::NodeOptions & options);
    private:
        using image_  = sensor_msgs::msg::Image;
        using point2_ = asdfr_interfaces::msg::Point2;

        void parse_parameters();

        void image_callback(const image_::SharedPtr img) const;

        int threshold_;

        rclcpp::Subscription<image_>::SharedPtr sub_img_;
        rclcpp::Publisher<point2_>::SharedPtr pub_img_cog_;
        rclcpp::Publisher<image_>::SharedPtr pub_mono_img_;
};

#endif /* LIGHT_POSITION_HPP */