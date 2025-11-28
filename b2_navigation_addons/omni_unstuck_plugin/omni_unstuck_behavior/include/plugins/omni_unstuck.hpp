// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2022 Joshua Wallace
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_CUSTOM_BEHAVIORS__PLUGINS__OMNI_UNSTUCK_HPP_
#define NAV2_CUSTOM_BEHAVIORS__PLUGINS__OMNI_UNSTUCK_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "omni_unstuck_interfaces/action/omni_unstuck.hpp"
#include "nav2_util/node_utils.hpp"



namespace omni_unstuck_behavior
{

using namespace nav2_behaviors;

// using ActionT = nav2_msgs::action::DriveOnHeading;
using ActionT = omni_unstuck_interfaces::action::OmniUnstuck;
class OmniUnstuck : public TimedBehavior<ActionT>
{
public:
  OmniUnstuck()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_x_(0.0),
    command_linear_speed_(0.0),
    command_angular_speed_(0.0)
  {
  }

  ~OmniUnstuck() = default;

  // Initialization
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    // Check distance limit
    if (command->distance > max_dist)
    {
      RCLCPP_WARN_ONCE(
        this->logger_,
        "OMNI_UNSTUCK: That's too far. Will only move %.2fm at most", max_dist);
    }

    // Check linear speed limit
    if (command->linear_speed > max_linear_speed)
    {
      RCLCPP_WARN_ONCE(
        this->logger_,
        "That's too fast. Will only move %.2fm/s at most", max_linear_speed);
    }

    // Check angular speed limit
    if (command->angular_speed > max_angular_speed)
    {
      RCLCPP_WARN_ONCE(
        this->logger_,
        "That's too fast. Will only move %.2frad/s at most", max_angular_speed);
    }

    // Get robot pose
    if (!nav2_util::getCurrentPose(
        initial_pose_, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Initial robot pose is not available.");
      return Status::FAILED;
    }

    // Process input
    double command_in = std::min(abs(command->distance), max_dist);
    double linear_speed_in = std::min(abs(command->linear_speed), max_linear_speed);
    double angular_speed_in = std::min(abs(command->angular_speed), max_angular_speed);

    // Save input
    command_x_ = command_in;
    command_linear_speed_ = linear_speed_in;
    command_angular_speed_ = angular_speed_in;

    // Check collision initial pose
    geometry_msgs::msg::Pose2D pose2d;
    bool fetch_data = true;
    double initial_theta = tf2::getYaw(initial_pose_.pose.orientation);
    pose2d.x = initial_pose_.pose.position.x;
    pose2d.y = initial_pose_.pose.position.y;
    pose2d.theta = initial_theta;
    bool init_coll_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);  // This is true only the first time, to fetch data
    fetch_data = false;
    if (init_coll_free) RCLCPP_INFO(this->logger_, "Current position is collision free");

    // Check collision around the robot
    collision_array.clear();
    for (int i = 0; i < collision_array_size; i++)
    {
      double angle = pose2d.theta + double(i)/double(collision_array_size) * 2 * M_PI;
      pose2d.x = initial_pose_.pose.position.x + command_in * cos(initial_theta + angle);
      pose2d.y = initial_pose_.pose.position.y + command_in * sin(initial_theta +angle);
      collision_array.push_back(this->collision_checker_->isCollisionFree(pose2d, fetch_data));
    }

    // Find the longest chain of Trues with wrapping
    best_angle_ = findBestDirection(collision_array);
    
    if (best_angle_ == -999.0) {  // Special value indicating no direction found
      RCLCPP_WARN(this->logger_, "No collision-free direction found");
      return Status::FAILED;
    }

    // Set goal pose based on free areas
    goal_pose2d.x = initial_pose_.pose.position.x + command_in * cos(initial_theta + best_angle_);
    goal_pose2d.y = initial_pose_.pose.position.y + command_in * sin(initial_theta + best_angle_);
    goal_pose2d.theta = best_angle_;

    RCLCPP_INFO(this->logger_, "Current pose: %f, %f, %f", initial_pose_.pose.position.x, initial_pose_.pose.position.y, initial_theta);
    RCLCPP_INFO(this->logger_, "Goal pose: %f, %f, %f", goal_pose2d.x, goal_pose2d.y, goal_pose2d.theta);

    // Initialize state
    has_spinned_ = false;

    // Max time allowed
    command_time_allowance_ = command->time_allowance;
    end_time_ = this->clock_->now() + command_time_allowance_;


    return Status::SUCCEEDED;
  }

  // Loop
  Status onCycleUpdate() override
  {
    rclcpp::Duration time_remaining = end_time_ - this->clock_->now();
    if (time_remaining.seconds() < 0.0 && command_time_allowance_.seconds() > 0.0) {
      this->stopRobot();
      RCLCPP_WARN(
        this->logger_,
        "Exceeded time allowance before reaching the goal - Exiting Unstuck");
      return Status::FAILED;
    }

    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(
        current_pose, *this->tf_, this->global_frame_, this->robot_base_frame_,
        this->transform_tolerance_))
    {
      RCLCPP_ERROR(this->logger_, "Current robot pose is not available.");
      return Status::FAILED;
    }

    // Calculate distance and angle traveled and publish feedback
    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);
    double diff_theta = tf2::getYaw(initial_pose_.pose.orientation) - tf2::getYaw(current_pose.pose.orientation);

    feedback_->distance_traveled = distance;
    feedback_->angle_rotated = diff_theta;
    this->action_server_->publish_feedback(feedback_);

    // Calculate distance to goal
    double diff_x_to_goal = goal_pose2d.x - current_pose.pose.position.x;
    double diff_y_to_goal = goal_pose2d.y - current_pose.pose.position.y;
    double dist_to_goal = hypot(diff_x_to_goal, diff_y_to_goal);
    double angle_to_goal = goal_pose2d.theta - tf2::getYaw(current_pose.pose.orientation);

    if (distance >= command_x_) {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    // Check collision of goal pose
    bool goal_free = this->collision_checker_->isCollisionFree(goal_pose2d, true);
    if (!goal_free)
    {
      this->stopRobot();
        RCLCPP_WARN(this->logger_, "Collision detected - Exiting OmniUnstuck");
        return Status::FAILED;
    }

    // Check if the robot has spun
    double movement_angle = atan2(diff_y_to_goal, diff_x_to_goal) - tf2::getYaw(current_pose.pose.orientation);
    if (movement_angle > M_PI) movement_angle -= 2 * M_PI;
    if (movement_angle < -M_PI) movement_angle += 2 * M_PI;
    if (abs(movement_angle) <= goal_angle_threshold) {
      has_spinned_ = true;
    }

    // Calculate command velocity
    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    if (!has_spinned_) 
    {
      double sign = movement_angle / abs(movement_angle);
      cmd_vel->linear.x = 0.0;
      cmd_vel->linear.y = 0.0;
      cmd_vel->angular.z = sign * std::max(std::min(abs(movement_angle * angular_speed_gain), command_angular_speed_), min_angular_speed);
    }
    else
    {
      cmd_vel->linear.x = command_linear_speed_ * cos(movement_angle);
      cmd_vel->linear.y = command_linear_speed_ * sin(movement_angle);
      cmd_vel->angular.z = 0.0;
    }

    // Send command velocity
    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

protected:
  // Find the best direction based on the longest collision-free chain with wrapping
  double findBestDirection(const std::vector<int> & collision_array)
  {
    int max_length = 0;
    int max_start_index = -1;
    
    // First, find the longest chain without wrapping
    int current_length = 0;
    int start_index = -1;
    
    for (int i = 0; i < collision_array_size; i++) {
      if (collision_array[i]) {  // True means collision-free
        if (current_length == 0) {
          start_index = i;
        }
        current_length++;
      } else {
        if (current_length > max_length) {
          max_length = current_length;
          max_start_index = start_index;
        }
        current_length = 0;
      }
    }
    
    // Check if the longest chain ends at the array end
    if (current_length > max_length) {
      max_length = current_length;
      max_start_index = start_index;
    }
    
    // Now check for wrapping chains
    // If the array starts and ends with Trues, we need to check for wrapping
    if (collision_array[0] && collision_array[collision_array_size - 1]) {
      // Find the length of Trues at the beginning
      int start_zeros = 0;
      for (int i = 0; i < collision_array_size; i++) {
        if (collision_array[i]) {
          start_zeros++;
        } else {
          break;
        }
      }
      
      // Find the length of Trues at the end
      int end_zeros = 0;
      for (int i = collision_array_size - 1; i >= 0; i--) {
        if (collision_array[i]) {
          end_zeros++;
        } else {
          break;
        }
      }
      
      // Check if wrapping creates a longer chain
      int wrapping_length = start_zeros + end_zeros;
      if (wrapping_length > max_length) {
        max_length = wrapping_length;
        max_start_index = collision_array_size - end_zeros;  // Start from the end zeros
      }
    }
    
    // If no collision-free areas found
    if (max_length == 0) {
      return -999.0;  // Special value indicating no direction found
    }
    
    // Calculate the centroid angle of the longest chain
    double chain_center = max_start_index + (max_length - 1) / 2.0;
    double centroid_angle = chain_center * 2.0 * M_PI / collision_array_size;
    
    return centroid_angle;
  }

  // Configuration (parameters could be declared here)
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.max_dist", rclcpp::ParameterValue(1.0));
    node->get_parameter("omni_unstuck.max_dist", max_dist);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.min_linear_speed", rclcpp::ParameterValue(0.4));
    node->get_parameter("omni_unstuck.min_linear_speed", min_linear_speed);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.max_linear_speed", rclcpp::ParameterValue(1.0));
    node->get_parameter("omni_unstuck.max_linear_speed", max_linear_speed);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.min_angular_speed", rclcpp::ParameterValue(0.4));
    node->get_parameter("omni_unstuck.min_angular_speed", min_angular_speed);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.max_angular_speed", rclcpp::ParameterValue(1.0));
    node->get_parameter("omni_unstuck.max_angular_speed", max_angular_speed);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.angular_speed_gain", rclcpp::ParameterValue(2.0));
    node->get_parameter("omni_unstuck.angular_speed_gain", angular_speed_gain);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.collision_array_size", rclcpp::ParameterValue(16));
    node->get_parameter("omni_unstuck.collision_array_size", collision_array_size);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.goal_dist_threshold", rclcpp::ParameterValue(0.1));
    node->get_parameter("omni_unstuck.goal_dist_threshold", goal_dist_threshold);

    nav2_util::declare_parameter_if_not_declared(
      node, "omni_unstuck.goal_angle_threshold", rclcpp::ParameterValue(0.1));
    node->get_parameter("omni_unstuck.goal_angle_threshold", goal_angle_threshold);

    RCLCPP_INFO(this->logger_, "Omni Unstuck configured with parameters: ");
    RCLCPP_INFO(this->logger_, " · collision_array_size: %d", collision_array_size);
    RCLCPP_INFO(this->logger_, " · goal_dist_threshold: %f", goal_dist_threshold);
    RCLCPP_INFO(this->logger_, " · goal_angle_threshold: %f", goal_angle_threshold);
    RCLCPP_INFO(this->logger_, " · and more...");
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  geometry_msgs::msg::Pose2D goal_pose2d;
  double command_x_;
  double command_linear_speed_;
  double command_angular_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;

  // Parameters
  double max_dist;
  double min_linear_speed;
  double max_linear_speed;
  double min_angular_speed;
  double max_angular_speed;
  double angular_speed_gain;
  int collision_array_size;
  double goal_dist_threshold;
  double goal_angle_threshold;

  // Variables 
  std::vector<int> collision_array;
  double best_angle_ = 0.0;
  bool has_spinned_ = false;
};

}  // namespace unstuck_behavior

#endif  // NAV2_CUSTOM_BEHAVIORS__PLUGINS__OMNI_UNSTUCK_HPP_