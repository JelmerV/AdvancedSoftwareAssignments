// C++ includes.
#include <functional>
#include <memory>

// ROS2 includes.
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// ROS2 Message includes.
#include "sensor_msgs/msg/image.hpp"
#include "asdfr_interfaces/msg/point2.hpp"

#ifndef JIWY_TRAJECTORY_HPP
#define JIWY_TRAJECTORY_HPP

using std::placeholders::_1;

class JiwyTrajectory : public rclcpp::Node {
    public:
        JiwyTrajectory(const rclcpp::NodeOptions & options);
    private:
        // Define aliases to keep code compact.
        using image_  = sensor_msgs::msg::Image;
        using point2_ = asdfr_interfaces::msg::Point2;

        // Define function to parse parameters on startup.
        void parse_parameters();

        // Define callback functions.
        void timer_callback();
        void image_data_callback(const image_::SharedPtr img);
        void image_cog_callback(const point2_::SharedPtr img_cog);

        // Define parameter variables.
        bool use_camera_;
        int interval_;
        double x_limit_rad_;
        double y_limit_rad_;

        // Define topic value variables.
        int img_mono_width_;
        int img_mono_height_;

        // Define timer.
        rclcpp::TimerBase::SharedPtr timer_;

        // Define subscribers and publishers.
        rclcpp::Subscription<image_>::SharedPtr sub_mono_img_;
        rclcpp::Subscription<point2_>::SharedPtr sub_img_cog_;
        rclcpp::Publisher<point2_>::SharedPtr pub_setpoint_;
};

#endif /* JIWY_TRAJECTORY_HPP */