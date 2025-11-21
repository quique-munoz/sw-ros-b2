#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class OrbitFrameBroadcastNode : public rclcpp::Node
{
public:
  OrbitFrameBroadcastNode() : Node("orbit_frame_broadcast")
  {
    // Declare parameters
    this->declare_parameter("parent_frame", "base_footprint");
    this->declare_parameter("child_frame", "orbit_frame");
    this->declare_parameter("point_topic", "/reconstruction/centroid");
    this->declare_parameter("orbit_radius", 4.0);
    this->declare_parameter("orbit_center_x", 4.0);
    this->declare_parameter("orbit_center_y", 0.0);
    this->declare_parameter("orbit_center_z", 0.0);
    this->declare_parameter("orbit_pitch", 0.2);
    this->declare_parameter("orbit_speed", 0.2);
    this->declare_parameter("publish_rate", 20.0);
    this->declare_parameter("time_offset", 0.5);
    this->declare_parameter("verbose", false);
    
    // Get parameters
    parent_frame_ = this->get_parameter("parent_frame").as_string();
    child_frame_ = this->get_parameter("child_frame").as_string();
    point_topic_ = this->get_parameter("point_topic").as_string();
    orbit_radius_ = this->get_parameter("orbit_radius").as_double();
    orbit_center_x_ = this->get_parameter("orbit_center_x").as_double();
    orbit_center_y_ = this->get_parameter("orbit_center_y").as_double();
    orbit_center_z_ = this->get_parameter("orbit_center_z").as_double();
    orbit_pitch_ = this->get_parameter("orbit_pitch").as_double();
    orbit_speed_ = this->get_parameter("orbit_speed").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    time_offset_ = this->get_parameter("time_offset").as_double();
    verbose_ = this->get_parameter("verbose").as_bool();
    
    // Create TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create transform broadcaster
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Subscribe to point topic
    if (point_topic_ != "")
    {
      point_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        point_topic_, 1, std::bind(&OrbitFrameBroadcastNode::pointCallback, this, std::placeholders::_1));
    }
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&OrbitFrameBroadcastNode::publishTransform, this));

    // Initialize last time
    last_time = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Orbit Frame Broadcaster started");
  }

private:
  void publishTransform()
  {   
    // Use the point from the point topic if it is available
    if (point_topic_ != "")
    {
      orbit_center_x_ = point_stamped_.point.x;
      orbit_center_y_ = point_stamped_.point.y;
      orbit_center_z_ = point_stamped_.point.z;
    }

    // Calculate current position
    double x = orbit_center_x_ - orbit_radius_ * cos(yaw) * cos(orbit_pitch_);
    double y = orbit_center_y_ - orbit_radius_ * sin(yaw) * cos(orbit_pitch_);
    double z = orbit_center_z_ + orbit_radius_ * sin(orbit_pitch_);

    // Update yaw
    rclcpp::Time now = this->get_clock()->now();
    double dt = (now - last_time).seconds();
    yaw = yaw + orbit_speed_ * dt;
    last_time = now;
    
    // Create quaternion with only yaw rotation
    tf2::Quaternion output_quat;
    output_quat.setRPY(0.0, orbit_pitch_, yaw);
    
    // Populate the transform
    geometry_msgs::msg::TransformStamped output_transform;
    output_transform.header.stamp = now + rclcpp::Duration::from_seconds(time_offset_);
    output_transform.header.frame_id = parent_frame_;
    output_transform.child_frame_id = child_frame_;
    output_transform.transform.translation.x = x;
    output_transform.transform.translation.y = y;
    output_transform.transform.translation.z = z;
    output_transform.transform.rotation.x = output_quat.x();
    output_transform.transform.rotation.y = output_quat.y();
    output_transform.transform.rotation.z = output_quat.z();
    output_transform.transform.rotation.w = output_quat.w();
    
    // Broadcast the transform
    transform_broadcaster_->sendTransform(output_transform);
    
    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(), 
        "Published transform from '%s' to '%s'",
        parent_frame_.c_str(), child_frame_.c_str());
    }
  }

  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    point_stamped_ = *msg;
  }

  // Members
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_subscription_;
  
  // Parameters
  std::string parent_frame_;
  std::string child_frame_;
  std::string point_topic_;
  double orbit_radius_;
  double orbit_center_x_;
  double orbit_center_y_;
  double orbit_center_z_;
  double orbit_pitch_;
  double orbit_speed_;
  double time_offset_;
  bool verbose_;

  // Variables
  double yaw = 0.0;
  rclcpp::Time last_time;
  geometry_msgs::msg::PointStamped point_stamped_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<OrbitFrameBroadcastNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
