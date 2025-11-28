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

#ifndef NAV2_CUSTOM_BEHAVIORS__PLUGINS__UNSTUCK_HPP_
#define NAV2_CUSTOM_BEHAVIORS__PLUGINS__UNSTUCK_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "nav2_behaviors/timed_behavior.hpp"
#include "unstuck_interfaces/action/unstuck.hpp"
#include "nav2_util/node_utils.hpp"



namespace unstuck_behavior
{

using namespace nav2_behaviors;

// using ActionT = nav2_msgs::action::DriveOnHeading;
using ActionT = unstuck_interfaces::action::Unstuck;
class Unstuck : public TimedBehavior<ActionT>
{
public:
  Unstuck()
  : TimedBehavior<ActionT>(),
    feedback_(std::make_shared<typename ActionT::Feedback>()),
    command_x_(0.0),
    command_speed_(0.0)
  {
  }

  ~Unstuck() = default;

  // Initialization
  Status onRun(const std::shared_ptr<const typename ActionT::Goal> command) override
  {
    // Check distance limit
    if (command->distance > max_dist)
    {
      RCLCPP_WARN_ONCE(
        this->logger_,
        "UNSTUCK: That's too far. Will only move %.2fm at most", max_dist);
    }

    // Check distance limit
    if (command->speed > max_speed)
    {
      RCLCPP_WARN_ONCE(
        this->logger_,
        "That's too fast. Will only move %.2fm/s at most", max_speed);
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
    double speed_in = std::min(abs(command->speed), max_speed);


    // Check collision initial pose
    geometry_msgs::msg::Pose2D pose2d;
    bool fetch_data = true;
    pose2d.x = initial_pose_.pose.position.x;
    pose2d.y = initial_pose_.pose.position.y;
    pose2d.theta = tf2::getYaw(initial_pose_.pose.orientation);
    bool init_coll_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);  // This is true only the first time, to fetch data
    fetch_data = false;
    if (init_coll_free) RCLCPP_INFO(this->logger_, "Current position is collision free");

    // Check collision behind
    pose2d.x = initial_pose_.pose.position.x - command_in * cos(pose2d.theta);
    pose2d.y = initial_pose_.pose.position.y - command_in * sin(pose2d.theta);
    bool bwd_coll_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    if (!bwd_coll_free) RCLCPP_INFO(this->logger_, "There are obstacles behind the vehicle");

    // Check collision forwards
    pose2d.x = initial_pose_.pose.position.x + command_in * cos(pose2d.theta);
    pose2d.y = initial_pose_.pose.position.y + command_in * sin(pose2d.theta);
    bool fwd_coll_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    if (!fwd_coll_free) RCLCPP_INFO(this->logger_, "There are obstacles in front of the vehicle");


    // Send command based on free areas
    double sign = 0.0;
    if (!bwd_coll_free && !fwd_coll_free)       // No free zones -> failure
      return Status::FAILED;
    else if (bwd_coll_free && !fwd_coll_free)   // BWD free -> BWD
      sign = -1.0;
    else if (!bwd_coll_free && fwd_coll_free)   // FWD free -> FWD
      sign = 1.0;
    else if (bwd_coll_free && fwd_coll_free)    // Both free -> check planner footprint
    {
      // Check planner footprint
      bool fr_free, fl_free, br_free, bl_free;
      if (isPlannerFree(fr_free, fl_free, br_free, bl_free))    // No obstacles in sight, problem must be something else
      {
        RCLCPP_WARN(this->logger_, "No obstacles in sight. Movement might not solve problem.");        
        return Status::FAILED;
      }
      else    // Obstacles in planner footprint, we can solve this
      {
        if (fl_free && fr_free)       // If front is clear, go forward
          sign =  1.0;
        else if (bl_free && br_free)  // If back is clear, go back
          sign = -1.0;
        else                          // In any other case, try something different (we already checked and we can perform both movements)
        {
          if (last_sign > 0.0)
            sign = -1.0;
          else
            sign =  1.0;
          last_sign = sign;
        }
      }
    }

    // Info
    std::string s = sign > 0.0 ? "forward" : "backwards";
    RCLCPP_INFO(this->logger_, "Moving %.2fm %s might solve the problem", command_in, s.c_str()); 

    // Set the command
    command_x_ = sign * command_in;
    command_speed_ = sign * speed_in;
    // Set the goal pose
    goal_pose2d.theta = pose2d.theta;
    goal_pose2d.x = initial_pose_.pose.position.x + command_x_ * cos(goal_pose2d.theta);
    goal_pose2d.y = initial_pose_.pose.position.y + command_x_ * sin(goal_pose2d.theta);

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

    double diff_x = initial_pose_.pose.position.x - current_pose.pose.position.x;
    double diff_y = initial_pose_.pose.position.y - current_pose.pose.position.y;
    double distance = hypot(diff_x, diff_y);

    feedback_->distance_traveled = distance;
    this->action_server_->publish_feedback(feedback_);

    if (distance >= std::fabs(command_x_)) {
      this->stopRobot();
      return Status::SUCCEEDED;
    }

    auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel->linear.y = 0.0;
    cmd_vel->angular.z = 0.0;
    cmd_vel->linear.x = command_speed_;

    // Check collision of goal pose
    bool goal_free = this->collision_checker_->isCollisionFree(goal_pose2d, true);

    if (!goal_free)
    {
      this->stopRobot();
        RCLCPP_WARN(this->logger_, "Collision detected - Exiting Unstuck");
        return Status::FAILED;
    }

    this->vel_pub_->publish(std::move(cmd_vel));

    return Status::RUNNING;
  }

protected:
  // Check collisions in planner footprint
  bool isPlannerFree(bool & fl_free, bool & fr_free, bool & br_free, bool & bl_free)
  {
    // Initialize pose 
    geometry_msgs::msg::Pose2D pose2d;
    bool fetch_data = true;
    pose2d.theta = tf2::getYaw(initial_pose_.pose.orientation);

    // Check Front-Left (+ +) 
    pose2d.x = initial_pose_.pose.position.x + pad_diff * (cos(pose2d.theta) - sin(pose2d.theta));
    pose2d.y = initial_pose_.pose.position.y + pad_diff * (sin(pose2d.theta) + cos(pose2d.theta));
    fl_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    fetch_data = false;
    if (!fl_free) RCLCPP_INFO(this->logger_, "Front-Left has obstacle");

    // Check Front-Right (+ -)
    pose2d.x = initial_pose_.pose.position.x + pad_diff * (cos(pose2d.theta) + sin(pose2d.theta));
    pose2d.y = initial_pose_.pose.position.y + pad_diff * (sin(pose2d.theta) - cos(pose2d.theta));
    fr_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    if (!fr_free) RCLCPP_INFO(this->logger_, "Front-Right has obstacle");

    // Check Back-Right (- -)
    pose2d.x = initial_pose_.pose.position.x + pad_diff * (-cos(pose2d.theta) + sin(pose2d.theta));
    pose2d.y = initial_pose_.pose.position.y + pad_diff * (-sin(pose2d.theta) - cos(pose2d.theta));
    br_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    if (!br_free) RCLCPP_INFO(this->logger_, "Back-Right has obstacle");

    // Check Back-Left (- +)
    pose2d.x = initial_pose_.pose.position.x + pad_diff * (-cos(pose2d.theta) - sin(pose2d.theta));
    pose2d.y = initial_pose_.pose.position.y + pad_diff * (-sin(pose2d.theta) + cos(pose2d.theta));
    bl_free = this->collision_checker_->isCollisionFree(pose2d, fetch_data);
    if (!bl_free) RCLCPP_INFO(this->logger_, "Back-Left has obstacle");

    // Are there obstacles in planner footprint?
    if (fl_free && fr_free && br_free && bl_free)
      return true;
    else 
      return false;
  }

  // Configuration (parameters could be declared here)
  void onConfigure() override
  {
    auto node = this->node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }

    nav2_util::declare_parameter_if_not_declared(
      node,
      "padding_difference", rclcpp::ParameterValue(0.5));
    node->get_parameter("padding_difference", pad_diff);
  }

  typename ActionT::Feedback::SharedPtr feedback_;

  geometry_msgs::msg::PoseStamped initial_pose_;
  geometry_msgs::msg::Pose2D goal_pose2d;
  double command_x_;
  double command_speed_;
  rclcpp::Duration command_time_allowance_{0, 0};
  rclcpp::Time end_time_;
  double pad_diff;

  // Const values
  const double max_dist = 1.0;
  const double max_speed = 1.0;

  // Variables 
  double last_sign = 1.0;
};

}  // namespace unstuck_behavior

#endif  // NAV2_CUSTOM_BEHAVIORS__PLUGINS__UNSTUCK_HPP_