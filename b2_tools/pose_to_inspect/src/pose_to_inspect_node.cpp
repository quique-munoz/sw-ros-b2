#include <memory>
#include <utility>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "pose_to_inspect_interfaces/action/inspect_poses.hpp"
#include "b2_interfaces/srv/mode.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PoseToInspect : public rclcpp::Node
{
  public:
    using InspectPoses = pose_to_inspect_interfaces::action::InspectPoses;
    using GoalHandleInspectPoses = rclcpp_action::ServerGoalHandle<InspectPoses>;

    PoseToInspect()
    : Node("pose_to_inspect")
    {
      // Declare parameters
      this->declare_parameter("verbose", false);
      this->declare_parameter("output_topic", "/euler");
      this->declare_parameter("velocity", 0.1);
      this->declare_parameter("control_dt_ms", 100);
      
      // Get parameter values
      verbose = this->get_parameter("verbose").as_bool();
      output_topic = this->get_parameter("output_topic").as_string();
      velocity = this->get_parameter("velocity").as_double();
      control_dt_ms = this->get_parameter("control_dt_ms").as_int();

      // Initialize publisher
      euler_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(output_topic, 10);

      // Create service client
      mode_client_ = this->create_client<b2_interfaces::srv::Mode>("/mode");

      // Create action server
      this->action_server_ = rclcpp_action::create_server<InspectPoses>(
        this,
        "inspect_poses",
        std::bind(&PoseToInspect::handle_goal, this, _1, _2),
        std::bind(&PoseToInspect::handle_cancel, this, _1),
        std::bind(&PoseToInspect::handle_accepted, this, _1));

      // Log parameter values
      if(verbose) RCLCPP_INFO(this->get_logger(), "PoseToInspect action server started with parameters:");
      if(verbose) RCLCPP_INFO(this->get_logger(), "  output_topic: %s", output_topic.c_str());
      if(verbose) RCLCPP_INFO(this->get_logger(), "  velocity: %.3f rad/s", velocity);
      if(verbose) RCLCPP_INFO(this->get_logger(), "  control_dt: %d ms", control_dt_ms);
    }

  private:
    // -------------------------------------
    // ACTION SERVER CALLBACKS
    // -------------------------------------
    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const InspectPoses::Goal> goal)
    {
      (void)uuid;
      
      if (is_executing_) {
        if(verbose) RCLCPP_WARN(this->get_logger(), "Goal rejected: Already executing an inspection");
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      // Validate goal parameters
      if (goal->min_pitch >= goal->max_pitch) {
        if(verbose) RCLCPP_WARN(this->get_logger(), "Goal rejected: min_pitch (%.3f) must be less than max_pitch (%.3f)", 
                   goal->min_pitch, goal->max_pitch);
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      if (goal->min_yaw >= goal->max_yaw) {
        if(verbose) RCLCPP_WARN(this->get_logger(), "Goal rejected: min_yaw (%.3f) must be less than max_yaw (%.3f)", 
                   goal->min_yaw, goal->max_yaw);
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      if (goal->max_pitch_step <= 0.0) {
        if(verbose) RCLCPP_WARN(this->get_logger(), "Goal rejected: max_pitch_step (%.3f) must be positive", 
                   goal->max_pitch_step);
        return rclcpp_action::GoalResponse::REJECT;
      }
      
      if(verbose) RCLCPP_INFO(this->get_logger(), "Received goal request - accepting");
      if(verbose) RCLCPP_INFO(this->get_logger(), "  pitch range: [%.3f, %.3f] rad", goal->min_pitch, goal->max_pitch);
      if(verbose) RCLCPP_INFO(this->get_logger(), "  yaw range: [%.3f, %.3f] rad", goal->min_yaw, goal->max_yaw);
      if(verbose) RCLCPP_INFO(this->get_logger(), "  max_pitch_step: %.3f rad", goal->max_pitch_step);
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleInspectPoses> goal_handle)
    {
      if(verbose) RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleInspectPoses> goal_handle)
    {
      // This needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&PoseToInspect::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleInspectPoses> goal_handle)
    {
      if(verbose) RCLCPP_INFO(this->get_logger(), "Executing goal");
      
      // Set executing flag to prevent new goals
      is_executing_ = true;
      
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<InspectPoses::Feedback>();
      auto result = std::make_shared<InspectPoses::Result>();

      // Initialize path and variables
      initialize_path(goal->min_pitch, goal->max_pitch, goal->min_yaw, goal->max_yaw, goal->max_pitch_step);
      current_index = 0;
      current_yaw = 0.0;
      current_pitch = 0.0;

      rclcpp::Rate rate(1000.0 / control_dt_ms); // Convert ms to Hz

      while (rclcpp::ok() && current_index < path.size()) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          result->success = false;
          result->message = "Goal canceled";
          goal_handle->canceled(result);
          if(verbose) RCLCPP_INFO(this->get_logger(), "Goal canceled");
          is_executing_ = false;  // Reset executing flag
          return;
        }

        // Execute one step of the path
        bool path_completed = execute_path_step();

        // Publish feedback
        feedback->current_pose_index = current_index;
        feedback->total_poses = path.size();
        feedback->current_pitch = current_pitch;
        feedback->current_yaw = current_yaw;
        feedback->path_completed = path_completed;
        goal_handle->publish_feedback(feedback);

        if (path_completed) 
        {
          break;
        }

        rate.sleep();
      }

      // Check if goal is still active (not canceled)
      if (rclcpp::ok() && !goal_handle->is_canceling()) {
        result->success = true;
        result->message = "Sweeping inspection completed successfully";
        goal_handle->succeed(result);
        if(verbose) RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }

      // Call mode service
      call_mode_service("stop_move");

      // Reset executing flag
      is_executing_ = false;
    }

    void initialize_path(double min_pitch, double max_pitch, double min_yaw, double max_yaw, double max_pitch_step)
    {
      path.clear();
      int steps = ceil(abs(max_pitch - min_pitch) / max_pitch_step);
      double pitch_step = abs(max_pitch - min_pitch) / double(steps);

      for (int i = 0; i <= steps; i++) {
        double pitch = min_pitch + i * pitch_step;
        if (i % 2 == 0) {
          path.push_back(std::make_pair(min_yaw, pitch));
          path.push_back(std::make_pair(max_yaw, pitch));
        } else {
          path.push_back(std::make_pair(max_yaw, pitch));
          path.push_back(std::make_pair(min_yaw, pitch));
        }
      }

      // Add the home pose
      path.push_back(std::make_pair(0.0, 0.0));
    }

    bool execute_path_step()
    {
      if (current_index >= path.size()) {
        return true; // Path completed
      }

      // Get the goal pitch and yaw
      double goal_yaw = path[current_index].first;
      double goal_pitch = path[current_index].second;

      // Calculate the distance to the goal
      double delta_pitch = goal_pitch - current_pitch;
      double delta_yaw = goal_yaw - current_yaw;
      double total_distance = sqrt(delta_pitch * delta_pitch + delta_yaw * delta_yaw);

      // Check if we're close enough to the goal
      double max_next_step = velocity * double(control_dt_ms) / 1000.0;
      if (total_distance < max_next_step) 
      {
        // Close enough - snap to goal
        current_pitch = goal_pitch;
        current_yaw = goal_yaw;
        current_index++;
      }
      else
      {
        // Move in a straight line towards the goal
        // Calculate the unit vector towards the goal
        double unit_pitch = delta_pitch / total_distance;
        double unit_yaw = delta_yaw / total_distance;
  
        // Move by max_next_step in the direction of the goal
        current_pitch += unit_pitch * max_next_step;
        current_yaw += unit_yaw * max_next_step;
      }

      // Send the message
      geometry_msgs::msg::Vector3 euler;
      euler.x = 0.0;
      euler.y = current_pitch;
      euler.z = current_yaw;

      euler_publisher_->publish(euler);

      return current_index >= path.size(); // Return true if path is completed
    }

    // -------------------------------------
    // SERVICE CALLBACKS
    // -------------------------------------
    void call_mode_service(const std::string & mode)
    {
      auto request = std::make_shared<b2_interfaces::srv::Mode::Request>();
      request->mode = mode;

      if (!mode_client_->wait_for_service(std::chrono::seconds(1))) 
      {
        if(verbose) RCLCPP_ERROR(this->get_logger(), "Mode service not available");
        return;
      }

      auto result = mode_client_->async_send_request(request);
    }

    void mode_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<b2_interfaces::srv::Mode::Request> request,
                                const std::shared_ptr<b2_interfaces::srv::Mode::Response> response)
    {
      response->success = true;
      if(verbose) RCLCPP_INFO(this->get_logger(), "Mode service called");
    }

    // -------------------------------------
    // VARIABLES
    // -------------------------------------
    // Parameters
    bool verbose = false;
    std::string output_topic;
    double velocity; //rad/s
    int control_dt_ms;

    // Variables
    double current_yaw = 0.0;
    double current_pitch = 0.0;
    std::vector<std::pair<double, double>> path;
    int current_index = 0;
    bool is_executing_ = false;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_publisher_;

    // Action server
    rclcpp_action::Server<InspectPoses>::SharedPtr action_server_;

    // Service clients
    rclcpp::Client<b2_interfaces::srv::Mode>::SharedPtr mode_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToInspect>());
  rclcpp::shutdown();
  return 0;
}