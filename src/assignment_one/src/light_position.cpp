// C++ includes.
#include <vector>

// ROS2 Message includes.
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"

// ROS2 internal includes.
#include "assignment_one/light_position.hpp"
#include "assignment_one/constants.hpp"

LightPosition::LightPosition(const rclcpp::NodeOptions & options) : Node("light_position", options) {
    // Parse parameters.
    parse_parameters();

    // Create subscriber.
    sub_img_ = this->create_subscription<image_>("image", 10, std::bind(&LightPosition::image_callback, this, _1));

    // Create publishers.
    pub_img_cog_ = this->create_publisher<point2_>("image_cog", 1);
    pub_mono_img_ = this->create_publisher<image_>("mono_image", 10);
}

void LightPosition::parse_parameters() {
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

void LightPosition::image_callback(const image_::SharedPtr img) {
    // Initialise variables needed during execution.
    int cog_x = 0;
    int cog_y = 0;
    int count = 0;

    // Update the threshold value.
    threshold_ = this->get_parameter("threshold").as_int();

    // Initialise new Image message for mono image.
    image_ img_mono;
    img_mono.height = img->height;
    img_mono.width = img->width;
    img_mono.step = img->width;
    img_mono.header.frame_id = img->header.frame_id;
    img_mono.header.stamp = this->now();
    img_mono.encoding = "mono8";
    img_mono.data = std::vector<uint8_t>(img->height * img->width);

    // Process image to get centor of gravity and mono image.
    for (unsigned int i = 0; i < img->height; i++) {
        for (unsigned int j = 0; j < img->width; j++) {
            int index = i * img->width + j;

            // Calculate grayscale value.
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
        cog_x = img->width  / 2;
        cog_y = img->height / 2;
    } else {
        // Calculate COG.
        cog_x /= count;
        cog_y /= count;
    }

    // Initialise new Point2 message.
    point2_ pub_msg;
    pub_msg.x = cog_x;
    pub_msg.y = cog_y;

    // Publish COG of thresholded mono image.
    RCLCPP_INFO(this->get_logger(), "Publishing: [x: '%d', y: '%d']", cog_x, cog_y);
    pub_img_cog_->publish(pub_msg);

    // Publish thresholded mono image.
    RCLCPP_INFO(this->get_logger(), "Publishing mono image.");
    pub_mono_img_->publish(img_mono);
}

RCLCPP_COMPONENTS_REGISTER_NODE(LightPosition)