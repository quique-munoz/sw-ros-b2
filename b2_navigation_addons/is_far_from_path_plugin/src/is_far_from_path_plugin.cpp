// Copyright (c) 2021 Samsung Research America
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

// This was copied from a more recent version (better than Humble) of Nav2

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp_v3/condition_node.h"

#include "is_far_from_path_plugin/is_far_from_path_plugin.hpp"

namespace nav2_behavior_tree
{

IsFarFromPath::IsFarFromPath(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf),
  max_distance(1.0),
  max_angle(1.0),
  robot_base_frame("base_link"),
  transform_tolerance(0.2)
{
  getInput("max_distance", max_distance);
  getInput("max_angle", max_angle);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance", transform_tolerance);
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

double IsFarFromPath::angle_difference(const geometry_msgs::msg::PoseStamped & pose1, const geometry_msgs::msg::PoseStamped & pose2)
{
  double yaw1 = tf2::getYaw(pose1.pose.orientation);
  double yaw2 = tf2::getYaw(pose2.pose.orientation);
  double angle_diff = abs(yaw1 - yaw2);   
  angle_diff = angle_diff > M_PI ? 2.0*M_PI - angle_diff : angle_diff;    // Need to consider angle wrapping

  return angle_diff;
}

inline BT::NodeStatus IsFarFromPath::tick()
{
  // Grab the new path
  nav_msgs::msg::Path path;
  getInput("path", path);

  // Transform robot pose to path frame
  geometry_msgs::msg::PoseStamped robot_pose;
  nav2_util::getCurrentPose(robot_pose, *this->tf_, path.header.frame_id, robot_base_frame, 0.2);

  // Find distance from robot to path
  bool at_least_one_valid_point = false;
  for (int i=0; i<path.poses.size(); ++i)
  {
    // Distance from waypoint to robot
    double dist = nav2_util::geometry_utils::euclidean_distance(robot_pose, path.poses[i], false);

    if (dist > max_distance)
      continue;

    // Angle difference from waypoint to robot
    double angle_diff = angle_difference(robot_pose, path.poses[i]);

    if (angle_diff > max_angle)
      continue;

    // Both checks passed
    at_least_one_valid_point = true;
    break;
  }

  // Check the result
  if (at_least_one_valid_point)
  {
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "Vehicle is far from path: None of the path waypoints under distance and angle thresholds");
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsFarFromPath>("IsFarFromPath");
}