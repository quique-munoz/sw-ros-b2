// Copyright (c) 2018 Intel Corporation
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

#include <string>
#include <memory>

#include "omni_unstuck_bt_node.hpp"

namespace omni_unstuck_bt_node
{

using namespace nav2_behavior_tree;

OmniUnstuckAction::OmniUnstuckAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<omni_unstuck_interfaces::action::OmniUnstuck>(xml_tag_name, action_name, conf)
{
  double dist;
  getInput("dist_to_travel", dist);
  double linear_speed;
  getInput("linear_speed", linear_speed);
  double angular_speed;
  getInput("angular_speed", angular_speed);
  double time_allowance;
  getInput("time_allowance", time_allowance);

  // Populate the input message
  goal_.distance = dist;
  goal_.linear_speed = linear_speed;
  goal_.angular_speed = angular_speed;
  goal_.time_allowance = rclcpp::Duration::from_seconds(time_allowance);
}

}  // namespace unstuck_bt_node

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<omni_unstuck_bt_node::OmniUnstuckAction>(
        name, "omni_unstuck", config);
    };

  factory.registerBuilder<omni_unstuck_bt_node::OmniUnstuckAction>("OmniUnstuck", builder);
}