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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FAR_FROM_PATH_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FAR_FROM_PATH_CONDITION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"

namespace nav2_behavior_tree
{

class IsFarFromPath : public BT::ConditionNode
{
public:

  IsFarFromPath(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);


  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("max_distance", 1.0, "MaxDistance"),
      BT::InputPort<double>("max_angle", 1.0, "MaxAngle"),
      BT::InputPort<nav_msgs::msg::Path>("path")
    };
  }

private:
  BT::NodeStatus tick() override;
  double angle_difference(const geometry_msgs::msg::PoseStamped & pose1, const geometry_msgs::msg::PoseStamped & pose2);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string robot_base_frame;
  double transform_tolerance;
  double max_distance;
  double max_angle;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__IS_FAR_FROM_PATH_CONDITION_HPP_