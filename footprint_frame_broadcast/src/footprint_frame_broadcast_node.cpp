#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class FootprintFrameBroadcastNode : public rclcpp::Node
{
public:
  FootprintFrameBroadcastNode() : Node("footprint_frame_broadcast")
  {
    // Declare parameters
    this->declare_parameter("source_frame", "odom");
    this->declare_parameter("target_frame", "base_link");
    this->declare_parameter("output_parent_frame", "odom");
    this->declare_parameter("output_child_frame", "base_footprint");
    this->declare_parameter("output_frame_height", 0.35);
    this->declare_parameter("yaw_offset", 0.0);
    this->declare_parameter("publish_rate", 20.0);
    this->declare_parameter("time_offset", 0.5);
    
    // Get parameters
    source_frame_ = this->get_parameter("source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    output_parent_frame_ = this->get_parameter("output_parent_frame").as_string();
    output_child_frame_ = this->get_parameter("output_child_frame").as_string();
    output_frame_height_ = this->get_parameter("output_frame_height").as_double();
    yaw_offset_ = this->get_parameter("yaw_offset").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();
    time_offset_ = this->get_parameter("time_offset").as_double();
    
    // Create TF buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Create transform broadcaster
    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // Create timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&FootprintFrameBroadcastNode::publishTransform, this));
    
    RCLCPP_INFO(this->get_logger(), 
      "Footprint Frame Broadcaster started. Listening for transform from '%s' to '%s', publishing to '%s' -> '%s'",
      source_frame_.c_str(), target_frame_.c_str(), output_parent_frame_.c_str(), output_child_frame_.c_str());
  }

private:
  void publishTransform()
  {
    try {
      // Look up the transform from source to target frame
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        source_frame_, target_frame_, tf2::TimePointZero);
      
      // Create output transform
      geometry_msgs::msg::TransformStamped output_transform;
      output_transform.header.stamp = this->get_clock()->now() + rclcpp::Duration::from_seconds(time_offset_);
      output_transform.header.frame_id = output_parent_frame_;
      output_transform.child_frame_id = output_child_frame_;
      
      // Copy only X, Y, and add the Z height from the parameter
      output_transform.transform.translation.x = transform.transform.translation.x;
      output_transform.transform.translation.y = transform.transform.translation.y;
      output_transform.transform.translation.z = output_frame_height_;
      
      // Extract yaw from input transform and create quaternion with only yaw rotation
      tf2::Quaternion input_quat(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
      );
      
      // Convert to RPY to extract yaw
      double roll, pitch, yaw;
      tf2::Matrix3x3(input_quat).getRPY(roll, pitch, yaw);
      
      // Create new quaternion with only yaw (roll=0, pitch=0)
      tf2::Quaternion output_quat;
      output_quat.setRPY(0.0, 0.0, yaw + yaw_offset_);

      // Set the output quaternion
      output_transform.transform.rotation.x = output_quat.x();
      output_transform.transform.rotation.y = output_quat.y();
      output_transform.transform.rotation.z = output_quat.z();
      output_transform.transform.rotation.w = output_quat.w();
      
      // Broadcast the transform
      transform_broadcaster_->sendTransform(output_transform);
      
      RCLCPP_DEBUG(this->get_logger(), 
        "Published transform from '%s' to '%s' based on '%s' -> '%s'",
        output_parent_frame_.c_str(), output_child_frame_.c_str(), 
        source_frame_.c_str(), target_frame_.c_str());
        
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Could not transform from '%s' to '%s': %s",
        source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string source_frame_;
  std::string target_frame_;
  std::string output_parent_frame_;
  std::string output_child_frame_;  
  double output_frame_height_;
  double yaw_offset_;
  double time_offset_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<FootprintFrameBroadcastNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
