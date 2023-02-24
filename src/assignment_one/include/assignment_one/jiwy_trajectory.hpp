#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
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
        void timer_callback() const;
        void image_callback(const image_::SharedPtr img) const;

        // Define parameter variables.
        bool use_camera_;
        int threshold_;
        double x_limit_rad_;
        double y_limit_rad_;

        // Define timer.
        rclcpp::TimerBase::SharedPtr timer_;

        // Define subscribers and publishers.
        rclcpp::Subscription<image_>::SharedPtr sub_img_;
        rclcpp::Publisher<point2_>::SharedPtr pub_setpoint_;
        rclcpp::Publisher<image_>::SharedPtr pub_mono_img_;
};

#endif /* JIWY_TRAJECTORY_HPP */