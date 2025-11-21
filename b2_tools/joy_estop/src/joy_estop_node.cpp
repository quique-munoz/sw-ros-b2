#include <memory>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "b2_interfaces/srv/mode.hpp"

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
      this->declare_parameter("cmd_vel_topic", "/cmd_vel_estop");
      this->declare_parameter("estop_button", 1);
      this->declare_parameter("reset_button", 2);
      this->declare_parameter("damp_buttons", std::vector<long int>({7, 8}));
      this->declare_parameter("stand_down_buttons", std::vector<long int>({9, 10, 12}));
      this->declare_parameter("stand_up_buttons", std::vector<long int>({9, 10, 11}));
      
      // Get parameter values
      input_topic = this->get_parameter("input_topic").as_string();
      output_topic = this->get_parameter("output_topic").as_string();
      cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
      estop_button = this->get_parameter("estop_button").as_int();
      reset_button = this->get_parameter("reset_button").as_int();
      damp_buttons = this->get_parameter("damp_buttons").as_integer_array();
      stand_down_buttons = this->get_parameter("stand_down_buttons").as_integer_array();
      stand_up_buttons = this->get_parameter("stand_up_buttons").as_integer_array();
      
      // Subscribe to joy topic
      joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        input_topic, 10, std::bind(&JoyEStop::joy_callback, this, _1));

      // Initialize publisher
      e_stop_publisher_ = this->create_publisher<std_msgs::msg::Bool>(output_topic, 10);
      cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

      // Create service client
      mode_client_ = this->create_client<b2_interfaces::srv::Mode>("/mode");

      // Initialize timer
      timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&JoyEStop::timer_callback, this));
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

      // Change the status based on the buttons
      if (estop_pressed) {
        call_mode_service("stop_move");
        locked = true;
      } else if (reset_pressed) {
        locked = false;
      }

      // Check if all the damp buttons are pressed
      if (check_all_buttons_pressed(damp_buttons, msg)) 
      {
        RCLCPP_WARN(this->get_logger(), "Trying to activate damp mode");
        call_mode_service("damp");
        return;
      }

      // Check if all the stand down buttons are pressed
      if (check_all_buttons_pressed(stand_down_buttons, msg)) 
      {
          RCLCPP_WARN(this->get_logger(), "Trying to activate stand down mode");
          call_mode_service("stand_down");
          return;
      }

      // Check if all the stand up buttons are pressed
      if (check_all_buttons_pressed(stand_up_buttons, msg)) 
      {
        RCLCPP_WARN(this->get_logger(), "Trying to activate stand up mode");
        call_mode_service("stand_up");
        return;
      }
    }


    void timer_callback()
    {
      if (locked) 
      {
        // Publish a zero twist
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
        cmd_vel_publisher_->publish(twist);

        // Publish a true e-stop to lock the mux
        std_msgs::msg::Bool e_stop;
        e_stop.data = true;
        e_stop_publisher_->publish(e_stop);
      }
      else
      {
        // Publish a false e-stop to unlock the mux
        std_msgs::msg::Bool e_stop;
        e_stop.data = false;
        e_stop_publisher_->publish(e_stop);
      }
    }

    bool check_all_buttons_pressed(const std::vector<long int> & buttons, const sensor_msgs::msg::Joy & msg)
    {
      if (buttons.size() == 0) {
        return false;
      }

      for (int button : buttons) {
        if (msg.buttons[button] == 0) {
          return false;
        }
      }
      return true;
    }

    void call_mode_service(const std::string & mode)
    {
      auto request = std::make_shared<b2_interfaces::srv::Mode::Request>();
      request->mode = mode;

      if (!mode_client_->wait_for_service(std::chrono::seconds(1))) 
      {
        RCLCPP_ERROR(this->get_logger(), "Mode service not available");
        return;
      }

      auto result = mode_client_->async_send_request(request);
    }

    void mode_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<b2_interfaces::srv::Mode::Request> request,
                                const std::shared_ptr<b2_interfaces::srv::Mode::Response> response)
    {
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Mode service called");
    }

    // -------------------------------------
    // VARIABLES
    // -------------------------------------

    // Variables
    bool locked = false;

    // Parameters
    std::string input_topic;
    std::string output_topic;
    std::string cmd_vel_topic;
    int estop_button;
    int reset_button;
    std::vector<long int> damp_buttons;
    std::vector<long int> stand_down_buttons;
    std::vector<long int> stand_up_buttons;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr e_stop_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer;

    // Service clients
    rclcpp::Client<b2_interfaces::srv::Mode>::SharedPtr mode_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyEStop>());
  rclcpp::shutdown();
  return 0;
}