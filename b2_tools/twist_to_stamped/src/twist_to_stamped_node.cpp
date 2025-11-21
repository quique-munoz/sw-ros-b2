#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using std::placeholders::_1;

class TwistToStamped : public rclcpp::Node
{
  public:
    TwistToStamped()
    : Node("twist_to_stamped")
    {
      // Declare parameters
      this->declare_parameter("input_topic", "/twist");
      this->declare_parameter("output_topic", "/twist_stamped");
      this->declare_parameter("frame_id", "base_link");
      
      // Get parameter values
      input_topic = this->get_parameter("input_topic").as_string();
      output_topic = this->get_parameter("output_topic").as_string();
      frame_id = this->get_parameter("frame_id").as_string();

      // Subscribe to twist topic
      twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic, 10, std::bind(&TwistToStamped::twist_callback, this, _1));

      // Initialize publisher
      twist_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic, 10);
    }

  private:
    // -------------------------------------
    // CALLBACKS
    // -------------------------------------
    void twist_callback(const geometry_msgs::msg::Twist & msg)
    {
      geometry_msgs::msg::TwistStamped twist_stamped;
      twist_stamped.header.stamp = this->get_clock()->now();
      twist_stamped.header.frame_id = frame_id;
      twist_stamped.twist = msg;

      twist_stamped_publisher_->publish(twist_stamped);
    }

    // -------------------------------------
    // VARIABLES
    // -------------------------------------
    // Parameters
    std::string input_topic;
    std::string output_topic;
    std::string frame_id;

    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToStamped>());
  rclcpp::shutdown();
  return 0;
}