
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

// lightchecker node, takes image from topic "image" and checks the brightness, then publishes the result to topic "light"
class LightPosition : public rclcpp::Node
{
public:
  LightPosition() : Node("light_position")
  {
    this->declare_parameter("threshold", 100);

    // create a subscription to the topic "image"
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image",                                            // topic name
        10,                                                 // queue size
        std::bind(&LightPosition::topic_callback, this, _1) // callback function
    );

    // create a publisher to the topic "lights"
    pub_string = this->create_publisher<std_msgs::msg::String>("light_cog", 10);
    pub_image = this->create_publisher<sensor_msgs::msg::Image>("image_thres", 10);
  }

private:
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
          // msg->data[index] = 255;
          // msg->data[index + 1] = 255;
          // msg->data[index + 2] = 255;
          img_gray.data[index] = 255;
        }
        else {
          // msg->data[index] = 0;
          // msg->data[index + 1] = 0;
          // msg->data[index + 2] = 0;
          img_gray.data[index] = 0;
        }
      }
    }

    // publish the thresholded image
    pub_image->publish(img_gray);

    // find the center of gravity of the image
    int avg_x = 0;
    int avg_y = 0;
    int count = 0;
    for (unsigned int i = 0; i < msg->height; i++)
    {
      for (unsigned int j = 0; j < msg->width; j++)
      {
        int index = (i * msg->width + j);
        int pixel = img_gray.data[index];

        if ( pixel == 255) {
          avg_x += i;
          avg_y += j;
          count += 1;
        }
      }
    }
    if (count == 0) {
      count = 1;
    }
    avg_x /= count;
    avg_y /= count;

    // construct message
    auto message = std_msgs::msg::String();
    message.data = "x: " + std::to_string(avg_x) + " y: " + std::to_string(avg_y);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    pub_string->publish(message);
  }

  // define attributes
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_string;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image;
};


// main function, spin up the node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightPosition>());
  rclcpp::shutdown();
  return 0;
}
