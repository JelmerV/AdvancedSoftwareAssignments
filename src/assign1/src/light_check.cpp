
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

// lightchecker node, takes image from topic "image" and checks the brightness, then publishes the result to topic "light"
class LightsChecker : public rclcpp::Node
{
public:
  LightsChecker() : Node("light_check")
  {
    this->declare_parameter("threshold", 100);

    // create a subscription to the topic "image"
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image",                                            // topic name
        10,                                                 // queue size
        std::bind(&LightsChecker::topic_callback, this, _1) // callback function
    );

    // create a publisher to the topic "lights"
    publisher_ = this->create_publisher<std_msgs::msg::String>("brightness", 10); 
  }

private:
  // callback function receiving a image message
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
  {
    int threshold = this->get_parameter("threshold").as_int();

    // Check the brightness of the image
    int sum = 0;
    for (size_t i = 0; i < msg->data.size(); i++)
    {
      sum += msg->data[i];
    }
    double avg = (double)sum / msg->data.size();

    // construct message
    auto message = std_msgs::msg::String();
    if (avg > threshold)
    {
      message.data = "It is light";
    }
    else
    {
      message.data = "It is dark";
    }

    message.data += " (avg: " + std::to_string(avg) + ")";

    // publish message
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  // define attributes
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};


// main function, spin up the node
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightsChecker>());
  rclcpp::shutdown();
  return 0;
}
