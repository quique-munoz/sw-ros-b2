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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UNSTUCK_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UNSTUCK_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "omni_unstuck_interfaces/action/omni_unstuck.hpp"

namespace omni_unstuck_bt_node
{
using namespace nav2_behavior_tree;

class OmniUnstuckAction : public BtActionNode<omni_unstuck_interfaces::action::OmniUnstuck>
{
public:
  OmniUnstuckAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<double>("dist_to_travel", 0.15, "Distance to travel"),
        BT::InputPort<double>("linear_speed", 0.5, "Speed at which to travel"),
        BT::InputPort<double>("angular_speed", 0.6, "Speed at which to rotate"),
        BT::InputPort<double>("time_allowance", 10.0, "Allowed time for driving on heading")
      });
  }
};

}  // namespace omni_unstuck_bt_node

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__UNSTUCK_ACTION_HPP_