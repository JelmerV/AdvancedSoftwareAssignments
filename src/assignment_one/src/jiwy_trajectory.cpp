#include "assignment_one/jiwy_trajectory.hpp"

JiwyTrajectory::JiwyTrajectory(const rclcpp::NodeOptions & options) : Node("jiwy_trajectory", options) {
    // Parse parameters.
    parse_parameters();

    // Create publishers.
    pub_setpoint_ = this->create_publisher<point2_>("setpoint", 1);
    pub_mono_img_ = this->create_publisher<image_>("mono_image", 10);

    // Check whether the parameter 'use_camera' is true. If not, initialise a wall timer to use for publishing setpoint values.
    if (use_camera_) {
        // Create subscriber.
        sub_img_ = this->create_subscription<image_>("image", 10, std::bind(&JiwyTrajectory::image_callback, this, _1));
    } else {
        // Create timer.
        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&JiwyTrajectory::timer_callback, this));
    }
}

void JiwyTrajectory::parse_parameters() {
    rcl_interfaces::msg::ParameterDescriptor threshold_desc;
    threshold_desc.description = "Determines treshold for light/dark (value between 0-255)";
    threshold_ = this->declare_parameter("threshold", 100, threshold_desc);
    RCLCPP_INFO(this->get_logger(), "using threshold: %d", threshold_);

    use_camera_ = this->declare_parameter("use_camera", false);
    x_limit_rad_ = this->declare_parameter("x_limit_rad", 0.8);
    y_limit_rad_ = this->declare_parameter("y_limit_rad", 0.6);
}

void JiwyTrajectory::timer_callback() const
  {
    // Initialise new Point2 message.
    point2_ pub_msg;
    pub_msg.x = (float)rand() / (float)RAND_MAX;
    pub_msg.y = (float)rand() / (float)RAND_MAX;

    // Publish message.
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: (%f, %f)", pub_msg.x, pub_msg.y);
    pub_setpoint_->publish(pub_msg);
  }

void JiwyTrajectory::image_callback(const image_::SharedPtr img) const {
    // Initialise variables needed during execution.
    int cog_x = 0;
    int cog_y = 0;
    int count = 0;

    // Initialise new mono image.
    image_ img_mono;
    img_mono.height = img->height;
    img_mono.width = img->width;
    img_mono.encoding = "mono8";
    img_mono.data = std::vector<uint8_t>(img->height * img->width);

    // Process image.
    for (uint i = 0; i < img->height; i++) {
        for (uint j = 0; j < img->width; j++) {
            int index = i * img->width + j;
            int gray = (img->data[index * 3] + img->data[index * 3 + 1] + img->data[index * 3 + 2]) / 3;

            // Make pixel black unless threshold is met.
            img_mono.data[index] = 0;
            if (gray > threshold_) {
                img_mono.data[index] = 255;

                // Add to variables to determine center of gravity.
                cog_x += i;
                cog_y += j;
                count += 1;
            }
        }
    }

    // Make sure we do not divide by 0.
    if (count == 0) {
        cog_x = img_mono.width  / 2;
        cog_y = img_mono.height / 2;
    } else {
        // Calculate COG.
        cog_x /= count;
        cog_y /= count;
    }

    // Convert pixel value to pan/tilt values.
    cog_x = cog_x - img_mono.width / 2;
    cog_y = cog_y - img_mono.height / 2;

    // Initialise new Point2 message and apply limits.
    point2_ pub_msg;
    pub_msg.x = cog_x * x_limit_rad_ / (float)(img_mono.width / 2);
    pub_msg.y = -cog_y * x_limit_rad_ / (float)(img_mono.height / 2);

    // Publish setpoint based on thresholded mono image.
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: (%f, %f)", pub_msg.x, pub_msg.y);
    pub_setpoint_->publish(pub_msg);

    // Publish thresholded mono image.
    RCLCPP_INFO(this->get_logger(), "Publishing mono image.");
    pub_mono_img_->publish(img_mono);
}

RCLCPP_COMPONENTS_REGISTER_NODE(JiwyTrajectory)