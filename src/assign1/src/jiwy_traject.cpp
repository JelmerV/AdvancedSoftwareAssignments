
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "asdfr_interfaces/msg/point2.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class JiwyTrajectory : public rclcpp::Node
{
  // define attributes
  rclcpp::Publisher<asdfr_interfaces::msg::Point2>::SharedPtr pub_setpoint;
  rclcpp::TimerBase::SharedPtr timer;

  // define attributes
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;

public:
  JiwyTrajectory() : Node("jiwy_trajectory")
  {
    this->declare_parameter("threshold", 100);
    this->declare_parameter<double>("x_limit_rad", 0.8);
    this->declare_parameter<double>("y_limit_rad", 0.6);
    RCLCPP_INFO(this->get_logger(), "hello");

    pub_setpoint = this->create_publisher<asdfr_interfaces::msg::Point2>("setpoint", 10);
    //timer = this->create_wall_timer(50ms, std::bind(&JiwyTrajectory::timer_callback, this));
  
    // create a subscription to the topic "image"
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image",                                            // topic name
        10,                                                 // queue size
        std::bind(&JiwyTrajectory::topic_callback, this, _1) // callback function
    );
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("image_thres", 10);
  }

private:
  // callback function receiving a image message
  void timer_callback() const
  {
    // construct message
    auto message = asdfr_interfaces::msg::Point2();
    message.x = (float)rand() / (float)RAND_MAX;
    message.y = (float)rand() / (float)RAND_MAX;
    RCLCPP_INFO(this->get_logger(), "publishing setpoint: (%f, %f)", message.x, message.y);
    pub_setpoint->publish(message);
  }

  // callback function receiving a image message
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    int threshold = this->get_parameter("threshold").as_int();
    RCLCPP_INFO(this->get_logger(), "using threshold: %d", threshold);

    // convert image from bgr8 to thresholded image
    auto img_gray = sensor_msgs::msg::Image();
    img_gray.height = msg->height;
    img_gray.width = msg->width;
    img_gray.encoding = "mono8";
    img_gray.data = std::vector<uint8_t>(msg->height * msg->width);
    
    for (unsigned int i = 0; i < msg->height; i++)
    {
      for (unsigned int j = 0; j < msg->width; j++)
      {
        int index = i * msg->width + j;
        int b = msg->data[index * 3];
        int g = msg->data[index * 3 + 1];
        int r = msg->data[index * 3 + 2];
        int gray = (r + g + b) / 3;

        if (gray > threshold) {
          img_gray.data[index] = 255;
        }
        else {
          img_gray.data[index] = 0;
        }
      }
    }
    // publish the thresholded image
    pub_image->publish(img_gray);

    // find the center of gravity of the image
    float avg_col = 0;
    float avg_row = 0;
    float count = 0;
    for (unsigned int i = 0; i < img_gray.height; i++)
    {
      for (unsigned int j = 0; j < img_gray.width; j++)
      {
        int index = (i * img_gray.width + j);
        int pixel = img_gray.data[index];

        if ( pixel == 255) {
          avg_col += j;
          avg_row += i;
          count += 1;
        }
      }
    }
    if (count == 0) {  //prevent division by 0
      avg_col = img_gray.width / 2;
      avg_row = img_gray.height / 2;
    } else {
      avg_col /= count;
      avg_row /= count;
    }

    //convert pixel value to pan/tilt values
    avg_col = avg_col - img_gray.width / 2;
    avg_row = avg_row - img_gray.height / 2;

    float x_lim = get_parameter("x_limit_rad").as_double();
    float y_lim = get_parameter("x_limit_rad").as_double();

    // construct message
    auto message = asdfr_interfaces::msg::Point2();
    message.x = avg_col * x_lim / (float)(img_gray.width / 2);
    message.y = -avg_row * y_lim / (float)(img_gray.height / 2);

    RCLCPP_INFO(this->get_logger(), "publishing setpoint: (%f, %f)", message.x, message.y);
    pub_setpoint->publish(message);
  }

};


// main function, spin up the node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JiwyTrajectory>();
  RCLCPP_INFO(node->get_logger(), "Starting node jiwy_trajectory");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
