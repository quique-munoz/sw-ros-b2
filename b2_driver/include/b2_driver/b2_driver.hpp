// BSD 3-Clause License
//
// Copyright (c) 2024
// All rights reserved.

#ifndef B2_DRIVER__B2_DRIVER_HPP_
#define B2_DRIVER__B2_DRIVER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/imu_state.hpp"

#include "b2_interfaces/srv/euler.hpp"
#include "b2_interfaces/srv/mode.hpp"
#include "b2_interfaces/srv/speed_level.hpp"
#include "b2_interfaces/srv/body_height.hpp"
#include "b2_interfaces/srv/continuous_gait.hpp"
#include "b2_interfaces/srv/switch_gait.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "tf2_ros/transform_broadcaster.h"

// OJO: ruta relativa al include_directories() del CMakeLists
#include "b2_driver/ros2_b2_sport_client.h"

namespace b2_driver
{

class B2Driver : public rclcpp::Node
{
public:
  B2Driver();

private:
  // ==== Topic callbacks ====
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void eulerCallback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void sportsModeCallback(const unitree_go::msg::SportModeState::SharedPtr msg);
  void lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // ==== Service handlers ====
  void handleMode(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::Mode::Request> request,
    const std::shared_ptr<b2_interfaces::srv::Mode::Response> response);

  void handleEuler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::Euler::Request> request,
    const std::shared_ptr<b2_interfaces::srv::Euler::Response> response);

  void handleSpeedLevel(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::SpeedLevel::Request> request,
    const std::shared_ptr<b2_interfaces::srv::SpeedLevel::Response> response);

  void handleBodyHeight(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::BodyHeight::Request> request,
    const std::shared_ptr<b2_interfaces::srv::BodyHeight::Response> response);

  void handleContinuousGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::ContinuousGait::Request> request,
    const std::shared_ptr<b2_interfaces::srv::ContinuousGait::Response> response);

  void handleSwitchGait(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<b2_interfaces::srv::SwitchGait::Request> request,
    const std::shared_ptr<b2_interfaces::srv::SwitchGait::Response> response);

  void handleAutorecover(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // ==== Wrappers sobre SportClient (API oficial B2) ====
  void callDamp();
  void callBalanceStand();
  void callStopMove();
  void callStandUp();
  void callStandDown();
  void callRecoveryStand();
  void callMove(float vx, float vy, float vyaw);
  void callSpeedLevel(int level);
  void callBodyHeight(float height);
  void callContinuousGait(bool flag);
  void callSwitchGait(int d);
  void callAutoRecoverySet(bool flag);
  void callFreeWalk();
  void callClassicWalk(bool flag);
  void callFastWalk(bool flag);
  void callEuler(float roll, float pitch, float yaw);

  // ==== Publishers ====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr error_code_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<unitree_go::msg::IMUState>::SharedPtr imu_pub_;

  // ==== Subscribers ====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr euler_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_state_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // ==== Services ====
  rclcpp::Service<b2_interfaces::srv::Mode>::SharedPtr mode_srv_;
  rclcpp::Service<b2_interfaces::srv::Euler>::SharedPtr euler_srv_;
  rclcpp::Service<b2_interfaces::srv::SpeedLevel>::SharedPtr speed_level_srv_;
  rclcpp::Service<b2_interfaces::srv::BodyHeight>::SharedPtr body_height_srv_;
  rclcpp::Service<b2_interfaces::srv::ContinuousGait>::SharedPtr continuous_gait_srv_;
  rclcpp::Service<b2_interfaces::srv::SwitchGait>::SharedPtr switch_gait_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr autorecover_srv_;

  // ==== Otros ====
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::msg::Joy joy_state_;

  SportClient sport_client_;   // Envoltorio de la API AI Motion B2

  rclcpp::Time last_move_command_time_;
  double move_cooldown_time_s_;
};

}  // namespace b2_driver

#endif  // B2_DRIVER__B2_DRIVER_HPP_
