#include <memory>
#include <deque>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/impl/transforms.hpp"

using std::placeholders::_1;

class PointcloudAccumulator : public rclcpp::Node
{
  public:
    PointcloudAccumulator()
    : Node("pointcloud_accumulator")
    {
      // Declare parameters
      this->declare_parameter("verbose", false);
      this->declare_parameter("input_topic", "/pointcloud");
      this->declare_parameter("output_topic", "/pointcloud_accumulated");
      this->declare_parameter("max_buffer_size", 50);
      this->declare_parameter("time_window_seconds", 2.0);
      this->declare_parameter("output_frequency", 10.0);
      this->declare_parameter("target_frame", "radar_flat");
      this->declare_parameter("fixed_frame", "odom");
      this->declare_parameter("tf_timeout_ms", 50);

      // Get parameter values
      verbose = this->get_parameter("verbose").as_bool();
      input_topic = this->get_parameter("input_topic").as_string();
      output_topic = this->get_parameter("output_topic").as_string();
      use_sim_time = this->get_parameter("use_sim_time").as_bool();
      max_buffer_size = this->get_parameter("max_buffer_size").as_int();
      time_window_seconds = this->get_parameter("time_window_seconds").as_double();
      double output_frequency = this->get_parameter("output_frequency").as_double();
      target_frame = this->get_parameter("target_frame").as_string();
      fixed_frame = this->get_parameter("fixed_frame").as_string();
      tf_timeout_ms = this->get_parameter("tf_timeout_ms").as_int();

      // Initialize circular buffers
      pointcloud_buffer.set_capacity(max_buffer_size);
      pointcloud_callback_times.set_capacity(MAX_TIMING_SAMPLES);
      timer_callback_times.set_capacity(MAX_TIMING_SAMPLES);

      // Callback groups
      sub_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // Subscribe to pointcloud topic
      rclcpp::SubscriptionOptions sub_options;
      sub_options.callback_group = sub_callback_group;
      pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&PointcloudAccumulator::pointcloud_callback, this, _1), sub_options);
      
      // Initialize TF2
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

      // Initialize publisher
      rclcpp::QoS pub_qos(1);
      pub_qos.reliable();
      pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, pub_qos);

      // Initialize timer¡
      timer_period_ms = int(1000.0 / output_frequency);
      timer = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms), 
                                    std::bind(&PointcloudAccumulator::timer_callback, this), 
                                    timer_callback_group);
    }

  private:
    // -------------------------------------
    // TIMING HELPER FUNCTIONS
    // -------------------------------------
    void log_timing_stats(const std::string& callback_name, const boost::circular_buffer<double>& times) {
      if (times.empty()) return;
      
      double sum = std::accumulate(times.begin(), times.end(), 0.0);
      double mean = sum / times.size();
      double min_time = *std::min_element(times.begin(), times.end());
      double max_time = *std::max_element(times.begin(), times.end());
      
      RCLCPP_INFO(this->get_logger(), 
                  "%s timing stats - Mean: %.3fms, Min: %.3fms, Max: %.3fms, Samples: %zu",
                  callback_name.c_str(), mean, min_time, max_time, times.size());
    }

    void add_timing_sample(boost::circular_buffer<double>& times, double duration_ms) {
      times.push_back(duration_ms);
    }

    // -------------------------------------
    // CALLBACKS
    // -------------------------------------
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      if (verbose) RCLCPP_INFO(this->get_logger(), "I got pointcloud in frame: %s", msg.header.frame_id.c_str());

      // Update current time
      auto current_time = this->get_clock()->now();

      // Find the transform from the fixed frame to the pointcloud frame
      geometry_msgs::msg::TransformStamped transform;
      try {
        try{
          transform = tf_buffer_->lookupTransform(fixed_frame, msg.header.frame_id, msg.header.stamp, rclcpp::Duration::from_seconds(tf_timeout_ms/1000.));
        } catch (tf2::TransformException & ex) {
          transform = tf_buffer_->lookupTransform(fixed_frame, msg.header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0));
        }
        // transform = tf_buffer_->lookupTransform(fixed_frame, msg.header.frame_id, rclcpp::Time(0));
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform input pointcloud to %s: %s", fixed_frame.c_str(), ex.what());
        return;
      }

      // Transform the pointcloud to the fixed frame
      sensor_msgs::msg::PointCloud2 transformed_pointcloud;
      pcl_ros::transformPointCloud(fixed_frame, transform, msg, transformed_pointcloud);

      if (verbose) RCLCPP_INFO(this->get_logger(), " · current time: %f", current_time.seconds());

      // Add to buffer while the mutex is locked (circular_buffer automatically handles capacity)
      {
        std::lock_guard<std::mutex> lock(pointcloud_buffer_mutex);
        pointcloud_buffer.push_back(transformed_pointcloud);

        // Remove old pointclouds based on time window
        while (!pointcloud_buffer.empty() && 
              (current_time - pointcloud_buffer.front().header.stamp).seconds() > time_window_seconds) 
        {
          pointcloud_buffer.pop_front();
        }
      }

      if (verbose) RCLCPP_INFO(this->get_logger(), " · Buffer has: %d/%d", pointcloud_buffer.size(), pointcloud_buffer.capacity());
      if (pointcloud_buffer.full()) 
      {
        RCLCPP_INFO(this->get_logger(), " · Buffer is full. Consider increasing max_buffer_size, or reducing time_window_seconds");
      }

      if (!pointcloud_buffer.empty() && verbose) {
        RCLCPP_INFO(this->get_logger(), " · First: %d.%d", pointcloud_buffer.front().header.stamp.sec, pointcloud_buffer.front().header.stamp.nanosec);
        RCLCPP_INFO(this->get_logger(), " · Last: %d.%d", pointcloud_buffer.back().header.stamp.sec, pointcloud_buffer.back().header.stamp.nanosec);
      }

      // Measure and log timing
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      double duration_ms = duration.count() / 1000.0;
      add_timing_sample(pointcloud_callback_times, duration_ms);
      
      if (verbose) {
        RCLCPP_INFO(this->get_logger(), " · Pointcloud callback took: %.3fms", duration_ms);
      }
    }

    // -------------------------------------
    // TIMER CALLBACK
    // -------------------------------------
    void timer_callback()
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      
      if (pointcloud_buffer.empty())
        return;

      // Update current time
      auto current_time = this->get_clock()->now();

      // Get the transform from the fixed frame to the target frame
      geometry_msgs::msg::TransformStamped transform;
      try {
        try{
          transform = tf_buffer_->lookupTransform(target_frame, fixed_frame, current_time, rclcpp::Duration::from_seconds(tf_timeout_ms/1000.));
        } catch (tf2::TransformException & ex) {
          transform = tf_buffer_->lookupTransform(target_frame, fixed_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0));
        }
        // transform = tf_buffer_->lookupTransform(target_frame, fixed_frame, current_time); //rclcpp::Time(0));
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform buffer pointcloud to %s: %s", fixed_frame.c_str(), ex.what());
        return;
      }

      // Copy the pointclouds to a local vector while the mutex is locked
      std::vector<sensor_msgs::msg::PointCloud2> local_copy;
      {
        std::lock_guard<std::mutex> lock(pointcloud_buffer_mutex);
        local_copy.assign(pointcloud_buffer.begin(), pointcloud_buffer.end());
      }

      // Accumulate pointclouds
      int num_pointclouds = 0;
      sensor_msgs::msg::PointCloud2 accumulated_pointcloud;
      for (int i = 0; i < local_copy.size(); i++) 
      {
        // Transform the pointcloud
        sensor_msgs::msg::PointCloud2 transformed_pointcloud;
        pcl_ros::transformPointCloud(target_frame, transform, local_copy[i], transformed_pointcloud);

        // Concatenate pointclouds
        // sensor_msgs::msg::PointCloud2 aux_pointcloud = accumulated_pointcloud;
        // pcl::concatenatePointCloud(aux_pointcloud, transformed_pointcloud, accumulated_pointcloud);
        accumulate_pointclouds(accumulated_pointcloud, transformed_pointcloud);
        num_pointclouds++;
      }

      if (verbose) RCLCPP_INFO(this->get_logger(), "Publishing %d accumulated pointclouds", num_pointclouds);

      // Publish accumulated pointcloud
      accumulated_pointcloud.header.frame_id = target_frame;
      accumulated_pointcloud.header.stamp = current_time;
      pointcloud_publisher_->publish(accumulated_pointcloud);

      // Measure and log timing
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      double duration_ms = duration.count() / 1000.0;
      add_timing_sample(timer_callback_times, duration_ms);
      
      if (verbose) {
        RCLCPP_INFO(this->get_logger(), " · Timer callback took: %.3fms", duration_ms);
        log_timing_stats("Timer callback", timer_callback_times);
        log_timing_stats("Pointcloud callback", pointcloud_callback_times);
      }

    }


    // -------------------------------------
    // HELPER FUNCTIONS
    // -------------------------------------
    void accumulate_pointclouds(sensor_msgs::msg::PointCloud2 & accumulated_pointcloud, const sensor_msgs::msg::PointCloud2 & pointcloud)
    {
      // Check if the pointcloud is empty
      if (accumulated_pointcloud.width * accumulated_pointcloud.height == 0)
      {
        accumulated_pointcloud = pointcloud;
        return;
      }
      
      // Check if the number of fields is the same
      if (accumulated_pointcloud.fields.size () != pointcloud.fields.size ())
      {
        PCL_ERROR ("[pcl::concatenatePointCloud] Number of fields in accumulated_pointcloud (%u) != Number of fields in pointcloud (%u)\n", accumulated_pointcloud.fields.size (), pointcloud.fields.size ());
        return;
      }

      // Modify the accumulated pointcloud data
      accumulated_pointcloud.width = accumulated_pointcloud.width * accumulated_pointcloud.height + pointcloud.width * pointcloud.height;
      accumulated_pointcloud.height = 1;
      accumulated_pointcloud.row_step = accumulated_pointcloud.width * accumulated_pointcloud.point_step;
      accumulated_pointcloud.is_dense = accumulated_pointcloud.is_dense && pointcloud.is_dense;

      // Add the points to the accumulated pointcloud
      size_t accumulated_size = accumulated_pointcloud.data.size();
      accumulated_pointcloud.data.resize(accumulated_size + pointcloud.data.size());
      std::memcpy(&accumulated_pointcloud.data[accumulated_size], &pointcloud.data[0], pointcloud.data.size());
    }


    // -------------------------------------
    // VARIABLES
    // -------------------------------------
    // Parameters
    bool verbose;
    std::string input_topic;
    std::string output_topic;
    bool use_sim_time;
    int max_buffer_size;
    double time_window_seconds;
    int timer_period_ms;
    std::string target_frame;
    std::string fixed_frame;
    int tf_timeout_ms;

    // Buffer
    boost::circular_buffer<sensor_msgs::msg::PointCloud2> pointcloud_buffer;
    std::mutex pointcloud_buffer_mutex;

    // Timing variables
    boost::circular_buffer<double> pointcloud_callback_times;
    boost::circular_buffer<double> timer_callback_times;
    static constexpr size_t MAX_TIMING_SAMPLES = 100;

    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr sub_callback_group;
    rclcpp::CallbackGroup::SharedPtr timer_callback_group;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudAccumulator>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}