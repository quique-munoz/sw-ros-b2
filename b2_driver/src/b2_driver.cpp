#include "b2_driver/b2_driver.hpp"

#include "rmw/qos_profiles.h"

namespace b2_driver
{

B2Driver::B2Driver()
: Node("b2_driver"),
  tf_broadcaster_(this),
  sport_client_(this),
  move_cooldown_time_s_(1.0)
{
  // QoS para odom (transient local como en go2_driver)
  rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // === Publishers ===
  error_code_pub_ = create_publisher<std_msgs::msg::String>("error_code", 10);
  lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", qos_profile);
  imu_pub_ = create_publisher<unitree_go::msg::IMUState>("imu", 10);

  // === Subscribers ===
  sport_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
    "/sportmodestate", 10,
    std::bind(&B2Driver::sportsModeCallback, this, std::placeholders::_1));

  low_state_sub_ = create_subscription<unitree_go::msg::LowState>(
    "lowstate", 10,
    std::bind(&B2Driver::lowStateCallback, this, std::placeholders::_1));

  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 10,
    std::bind(&B2Driver::lidarCallback, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/utlidar/robot_pose", 10,
    std::bind(&B2Driver::poseCallback, this, std::placeholders::_1));

  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&B2Driver::joyCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&B2Driver::cmdVelCallback, this, std::placeholders::_1));

  euler_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "euler", 10,
    std::bind(&B2Driver::eulerCallback, this, std::placeholders::_1));

  // === Services ===
  mode_srv_ = create_service<b2_interfaces::srv::Mode>(
    "mode",
    std::bind(
      &B2Driver::handleMode, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  euler_srv_ = create_service<b2_interfaces::srv::Euler>(
    "euler",
    std::bind(
      &B2Driver::handleEuler, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  speed_level_srv_ = create_service<b2_interfaces::srv::SpeedLevel>(
    "speed_level",
    std::bind(
      &B2Driver::handleSpeedLevel, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  body_height_srv_ = create_service<b2_interfaces::srv::BodyHeight>(
    "body_height",
    std::bind(
      &B2Driver::handleBodyHeight, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  continuous_gait_srv_ = create_service<b2_interfaces::srv::ContinuousGait>(
    "continuous_gait",
    std::bind(
      &B2Driver::handleContinuousGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  switch_gait_srv_ = create_service<b2_interfaces::srv::SwitchGait>(
    "switch_gait",
    std::bind(
      &B2Driver::handleSwitchGait, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  autorecover_srv_ = create_service<std_srvs::srv::SetBool>(
    "autorecover",
    std::bind(
      &B2Driver::handleAutorecover, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // Configuración inicial "segura"
  callAutoRecoverySet(false);
  callSpeedLevel(0);
  last_move_command_time_ = now();

  RCLCPP_INFO(get_logger(), "b2_driver node started");
}

// ============================================================================
// Callbacks de tópicos
// ============================================================================

void B2Driver::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if ((now() - last_move_command_time_).seconds() > move_cooldown_time_s_) {
    // Si llevamos un tiempo sin comandos, resetea movimiento
    callStopMove();
  }

  callMove(msg->linear.x, msg->linear.y, msg->angular.z);
  last_move_command_time_ = now();
}

void B2Driver::eulerCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  // Solo aplica Euler si no estamos recibiendo cmd_vel recientes
  if ((now() - last_move_command_time_).seconds() < move_cooldown_time_s_) {
    return;
  }

  callEuler(msg->x, msg->y, msg->z);
  callBalanceStand();
}

void B2Driver::sportsModeCallback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
  std_msgs::msg::String error_msg;
  error_msg.data = std::to_string(msg->error_code);
  error_code_pub_->publish(error_msg);

}

void B2Driver::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = now();
  joint_state.name = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
  };

  joint_state.position = {
    msg->motor_state[3].q, msg->motor_state[4].q, msg->motor_state[5].q,
    msg->motor_state[0].q, msg->motor_state[1].q, msg->motor_state[2].q,
    msg->motor_state[9].q, msg->motor_state[10].q, msg->motor_state[11].q,
    msg->motor_state[6].q, msg->motor_state[7].q, msg->motor_state[8].q
  };

  joint_state_pub_->publish(joint_state);
}

void B2Driver::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto cloud = *msg;
  cloud.header.stamp = now();
  cloud.header.frame_id = "radar";  
  lidar_pub_->publish(cloud);
}

void B2Driver::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now() + rclcpp::Duration::from_seconds(0.05);
  tf_msg.header.frame_id = "odom";
  tf_msg.child_frame_id = "base_link";

  tf_msg.transform.translation.x = msg->pose.position.x;
  tf_msg.transform.translation.y = msg->pose.position.y;
  tf_msg.transform.translation.z = msg->pose.position.z + 0.025;

  tf_msg.transform.rotation = msg->pose.orientation;

  tf_broadcaster_.sendTransform(tf_msg);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position = msg->pose.position;
  odom.pose.pose.position.z += 0.025;
  odom.pose.pose.orientation = msg->pose.orientation;

  odom_pub_->publish(odom);
}

void B2Driver::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  joy_state_ = *msg;
}

// ============================================================================
// Service handlers
// ============================================================================

void B2Driver::handleEuler(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::Euler::Request> request,
  const std::shared_ptr<b2_interfaces::srv::Euler::Response> response)
{
  (void)request_header;

  // Rangos según documentación Ai Motion B2: [-0.6, 0.6]
  if (request->roll < -0.6 || request->roll > 0.6) {
    response->success = false;
    response->message = "Roll value is out of range [-0.6 ~ 0.6]";
    return;
  } else if (request->pitch < -0.6 || request->pitch > 0.6) {
    response->success = false;
    response->message = "Pitch value is out of range [-0.6 ~ 0.6]";
    return;
  } else if (request->yaw < -0.6 || request->yaw > 0.6) {
    response->success = false;
    response->message = "Yaw value is out of range [-0.6 ~ 0.6]";
    return;
  }

  callEuler(request->roll, request->pitch, request->yaw);
  response->success = true;
  response->message = "Euler command sent";
}

void B2Driver::handleMode(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::Mode::Request> request,
  const std::shared_ptr<b2_interfaces::srv::Mode::Response> response)
{
  (void)request_header;
  const std::string & mode = request->mode;
  bool ok = true;

  if (mode == "damp") {
    callDamp();
  } else if (mode == "balance_stand") {
    callBalanceStand();
  } else if (mode == "stop_move") {
    callStopMove();
  } else if (mode == "stand_up") {
    callStandUp();
  } else if (mode == "stand_down") {
    callStandDown();
  } else if (mode == "recovery_stand") {
    callRecoveryStand();
  } else if (mode == "free_walk") {
    callFreeWalk();
  } else if (mode == "classic_walk") {
    callClassicWalk(true);
  } else if (mode == "fast_walk") {
    callFastWalk(true);
  } else {
    ok = false;
    response->success = false;
    response->message = "Invalid mode: " + mode;
  }

  if (ok) {
    response->success = true;
    response->message = "Mode command '" + mode + "' sent";
  }
}

void B2Driver::handleSpeedLevel(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::SpeedLevel::Request> request,
  const std::shared_ptr<b2_interfaces::srv::SpeedLevel::Response> response)
{
  (void)request_header;

  if (request->level < -1 || request->level > 1) {
    response->success = false;
    response->message = "Speed level is out of range [-1 ~ 1]";
    return;
  }

  callSpeedLevel(request->level);
  response->success = true;
  response->message = "Speed level set";
}

void B2Driver::handleBodyHeight(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::BodyHeight::Request> request,
  const std::shared_ptr<b2_interfaces::srv::BodyHeight::Response> response)
{
  (void)request_header;

  // No hay rango claro en docs; dejamos pasar tal cual y lo controla el firmware
  callBodyHeight(request->height);
  response->success = true;
  response->message = "Body height command sent";
}

void B2Driver::handleContinuousGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::ContinuousGait::Request> request,
  const std::shared_ptr<b2_interfaces::srv::ContinuousGait::Response> response)
{
  (void)request_header;

  callContinuousGait(request->flag);
  response->success = true;
}


void B2Driver::handleSwitchGait(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<b2_interfaces::srv::SwitchGait::Request> request,
  const std::shared_ptr<b2_interfaces::srv::SwitchGait::Response> response)
{
  (void)request_header;

  // d: 0..4 según documentación (locked, walk, continuous, vision, flat)
  if (request->d < 0 || request->d > 4) {
    response->success = false;
    response->message = "Gait id must be in [0..4]";
    return;
  }

  callSwitchGait(request->d);
  response->success = true;
  response->message = "Gait switched";
}

void B2Driver::handleAutorecover(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void)request_header;

  callAutoRecoverySet(request->data);
  response->success = true;
  response->message = std::string("AutoRecovery ") + (request->data ? "enabled" : "disabled");
}

// ============================================================================
// Wrappers SportClient
// ============================================================================

void B2Driver::callDamp()
{
  unitree_api::msg::Request req;
  sport_client_.Damp(req);
}

void B2Driver::callBalanceStand()
{
  unitree_api::msg::Request req;
  sport_client_.BalanceStand(req);
}

void B2Driver::callStopMove()
{
  unitree_api::msg::Request req;
  sport_client_.StopMove(req);
}

void B2Driver::callStandUp()
{
  unitree_api::msg::Request req;
  sport_client_.StandUp(req);
}

void B2Driver::callStandDown()
{
  unitree_api::msg::Request req;
  sport_client_.StandDown(req);
}

void B2Driver::callRecoveryStand()
{
  unitree_api::msg::Request req;
  sport_client_.RecoveryStand(req);
}

void B2Driver::callMove(float vx, float vy, float vyaw)
{
  unitree_api::msg::Request req;
  sport_client_.Move(req, vx, vy, vyaw);
}

void B2Driver::callSpeedLevel(int level)
{
  unitree_api::msg::Request req;
  sport_client_.SpeedLevel(req, level);
}

void B2Driver::callBodyHeight(float height)
{
  unitree_api::msg::Request req;
  sport_client_.BodyHeight(req, height);
}

void B2Driver::callContinuousGait(bool flag)
{
  unitree_api::msg::Request req;
  sport_client_.ContinuousGait(req, flag);
}

void B2Driver::callSwitchGait(int d)
{
  unitree_api::msg::Request req;
  sport_client_.SwitchGait(req, d);
}

void B2Driver::callAutoRecoverySet(bool flag)
{
  unitree_api::msg::Request req;
  sport_client_.AutoRecoverySet(req, flag);
}

void B2Driver::callFreeWalk()
{
  unitree_api::msg::Request req;
  sport_client_.FreeWalk(req);
}

void B2Driver::callClassicWalk(bool flag)
{
  unitree_api::msg::Request req;
  sport_client_.ClassicWalk(req, flag);
}

void B2Driver::callFastWalk(bool flag)
{
  unitree_api::msg::Request req;
  sport_client_.FastWalk(req, flag);
}

void B2Driver::callEuler(float roll, float pitch, float yaw)
{
  unitree_api::msg::Request req;
  sport_client_.Euler(req, roll, pitch, yaw);
}

// ============================================================================
// main()
// ============================================================================

}  // namespace b2_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<b2_driver::B2Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
