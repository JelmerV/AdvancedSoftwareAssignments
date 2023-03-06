// C++ includes.
#include <chrono>
#include <vector>

// ROS2 Message includes.
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"

// ROS2 internal includes.
#include "assignment_one/jiwy_trajectory.hpp"
#include "assignment_one/constants.hpp"

JiwyTrajectory::JiwyTrajectory(const rclcpp::NodeOptions & options) : Node("jiwy_trajectory", options) {
    // Parse parameters.
    parse_parameters();

    // Create publishers.
    pub_setpoint_ = this->create_publisher<point2_>("setpoint", 1);

    // Check whether the parameter 'use_camera' is true. If not, initialise a wall timer to use for publishing setpoint values.
    if (use_camera_) {
        // Create to subscribers.
        sub_mono_img_ = this->create_subscription<image_>("mono_image", 10, std::bind(&JiwyTrajectory::image_data_callback, this, _1));
        sub_img_cog_  = this->create_subscription<point2_>("image_cog", 10, std::bind(&JiwyTrajectory::image_cog_callback, this, _1));
    } else {
        // Create timer.
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(interval_*1ms, std::bind(&JiwyTrajectory::timer_callback, this));
    }
}

void JiwyTrajectory::parse_parameters() {
    // Set 'use_camera' parameter description.
    rcl_interfaces::msg::ParameterDescriptor use_camera_desc;
    use_camera_desc.description = "Whether to use the camera or a wall timer.";
    
    // Declare 'use_camera' parameter.
    use_camera_ = this->declare_parameter("use_camera", assignment_one::USE_CAMERA_DEFAULT, use_camera_desc);
    RCLCPP_INFO(this->get_logger(), "Using camera: %s", (use_camera_ ? "true" : "false"));

    // Set 'interval' parameter description.
    rcl_interfaces::msg::ParameterDescriptor interval_desc;
    interval_desc.description = "Interval that is used for the wall timer.";
    interval_desc.additional_constraints = "Parameter is only used if 'use_camera' parameter has value 'false'.";
    
    // Set allowed integer range for 'interval' parameter.
    rcl_interfaces::msg::IntegerRange interval_range;
    interval_range.from_value = assignment_one::INTERVAL_MIN;
    interval_range.step = assignment_one::INTERVAL_STEP;
    interval_range.to_value = assignment_one::INTERVAL_MAX;
    interval_desc.integer_range = {interval_range};

    // Declare 'threshold' parameter.
    interval_ = this->declare_parameter("interval", assignment_one::INTERVAL_DEFAULT, interval_desc);
    if (!use_camera_) {
        RCLCPP_INFO(this->get_logger(), "Using interval: %dms", interval_);
    }


    // Set 'x_limit_rad' & 'y_limit_rad' parameters description.
    rcl_interfaces::msg::ParameterDescriptor limit_rad_desc;
    limit_rad_desc.description = "The orientation, in radians, at which an end stop is modelled.";
    
    // Declare 'x_limit_rad' & 'y_limit_rad' parameters.
    x_limit_rad_ = this->declare_parameter("x_limit_rad", assignment_one::X_LIMIT_RAD_DEFAULT, limit_rad_desc);
    y_limit_rad_ = this->declare_parameter("y_limit_rad", assignment_one::Y_LIMIT_RAD_DEFAULT, limit_rad_desc);
    RCLCPP_INFO(this->get_logger(), "Using limits: [x: %f, y: %f]", x_limit_rad_, y_limit_rad_);
}

void JiwyTrajectory::timer_callback()
{
    // Update 'x_limit_rad' and 'y_limit_rad' parameters.
    x_limit_rad_ = this->get_parameter("x_limit_rad").as_double();
    y_limit_rad_ = this->get_parameter("y_limit_rad").as_double();

    // Initialise new Point2 message.
    point2_ pub_msg;
    pub_msg.x = (2 * x_limit_rad_ * (float)rand() / (float)RAND_MAX) - x_limit_rad_;
    pub_msg.y = (2 * y_limit_rad_ * (float)rand() / (float)RAND_MAX) - y_limit_rad_;

    // Publish message.
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: (%f, %f)", pub_msg.x, pub_msg.y);
    pub_setpoint_->publish(pub_msg);
}

void JiwyTrajectory::image_data_callback(const image_::SharedPtr img) {
    // Get the width and height from the mono image.
    img_mono_width_  = img->width;
    img_mono_height_ = img->height;
}

void JiwyTrajectory::image_cog_callback(const point2_::SharedPtr img_cog) {
    // Prevent division by zero if width and height have not been received yet.
    if (img_mono_width_ == 0 || img_mono_height_ == 0) {
        return;
    }

    // Update 'x_limit_rad' and 'y_limit_rad' parameters.
    x_limit_rad_ = this->get_parameter("x_limit_rad").as_double();
    y_limit_rad_ = this->get_parameter("y_limit_rad").as_double();

    // Initialise variables needed during execution.
    int sp_x = 0;
    int sp_y = 0;

    // Convert pixel value to pan/tilt values.
    sp_x = img_cog->x - img_mono_width_ / 2;
    sp_y = img_cog->y - img_mono_height_ / 2;

    // Initialise new Point2 message and apply limits.
    point2_ pub_msg;
    pub_msg.x = sp_x * x_limit_rad_ / (float)(img_mono_width_ / 2);
    pub_msg.y = -sp_y * y_limit_rad_ / (float)(img_mono_height_ / 2);

    // Publish setpoint based on thresholded mono image.
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: (%f, %f)", pub_msg.x, pub_msg.y);
    pub_setpoint_->publish(pub_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JiwyTrajectory)