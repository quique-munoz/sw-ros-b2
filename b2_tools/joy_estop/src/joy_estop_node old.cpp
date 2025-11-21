#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class JoyEStop : public rclcpp::Node
{
  public:
    JoyEStop()
    : Node("joy_estop")
    {
      // Declare parameters
      this->declare_parameter("input_topic", "/joy");
      this->declare_parameter("output_topic", "/e_stop");
      this->declare_parameter("estop_button", 1);
      this->declare_parameter("reset_button", 2);
      
      // Get parameter values
      input_topic = this->get_parameter("input_topic").as_string();
      output_topic = this->get_parameter("output_topic").as_string();
      estop_button = this->get_parameter("estop_button").as_int();
      reset_button = this->get_parameter("reset_button").as_int();

      // Subscribe to joy topic
      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        input_topic, 10, std::bind(&JoyEStop::joy_callback, this, _1));

      // Initialize publisher
      e_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>(output_topic, 10);
    }

  private:
    // -------------------------------------
    // CALLBACKS
    // -------------------------------------
    void joy_callback(const sensor_msgs::msg::Joy & msg)
    {
      if (msg.buttons.size() < estop_button || msg.buttons.size() < reset_button) {
        RCLCPP_WARN(this->get_logger(), "Joy message does not contain enough buttons");
        return;
      }

      if (estop_button == reset_button) {
        RCLCPP_WARN(this->get_logger(), "E-stop and reset buttons cannot be the same");
        return;
      }

      std_msgs::msg::Bool e_stop;
      bool estop_pressed = msg.buttons[estop_button];
      bool reset_pressed = msg.buttons[reset_button];

      if (estop_pressed) {
        e_stop.data = true;
      } else if (reset_pressed) {
        e_stop.data = false;
      }

      if (estop_pressed || reset_pressed) {
        e_stop_publisher_->publish(e_stop);
      }
    }

    // -------------------------------------
    // VARIABLES
    // -------------------------------------
    // Parameters
    std::string input_topic;
    std::string output_topic;
    int estop_button;
    int reset_button;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyEStop>());
  rclcpp::shutdown();
  return 0;
}