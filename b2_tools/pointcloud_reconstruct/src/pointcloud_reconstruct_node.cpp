#include <memory>
#include <deque>
#include <boost/circular_buffer.hpp>
#include <chrono>
#include <numeric>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/impl/transforms.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/gicp6d.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/registration/ia_ransac.h>

using std::placeholders::_1;

typedef pcl::PointXYZRGBA PointT;

class PointcloudReconstruct : public rclcpp::Node
{
  public:
    PointcloudReconstruct()
    : Node("pointcloud_reconstruct")
    {
      // Declare parameters
      this->declare_parameter("verbose", false);
      this->declare_parameter("input_topic", "/pointcloud");
      this->declare_parameter("activation_topic", "/reconstruction/activation");
      this->declare_parameter("output_topic", "/pointcloud_reconstructed");
      this->declare_parameter("centroid_topic", "/reconstruction/centroid");
      this->declare_parameter("status_topic", "/reconstruction/status");
      this->declare_parameter("fixed_frame", "odom");
      this->declare_parameter("publish_rate", 5.0);
      this->declare_parameter("tf_timeout_ms", 50);
      this->declare_parameter("max_depth", 1.0);
      this->declare_parameter("max_lateral_distance", 0.5);
      this->declare_parameter("filter_voxel_grid_size", 0.01);
      this->declare_parameter("visualization_voxel_grid_size", 0.01);
      this->declare_parameter("max_accumulated_points", 2000000);

      // Get parameter values
      verbose = this->get_parameter("verbose").as_bool();
      input_topic = this->get_parameter("input_topic").as_string();
      activation_topic = this->get_parameter("activation_topic").as_string();
      output_topic = this->get_parameter("output_topic").as_string();
      centroid_topic = this->get_parameter("centroid_topic").as_string();
      status_topic = this->get_parameter("status_topic").as_string();
      fixed_frame = this->get_parameter("fixed_frame").as_string();
      publish_rate = this->get_parameter("publish_rate").as_double();
      tf_timeout_ms = this->get_parameter("tf_timeout_ms").as_int();
      max_depth = this->get_parameter("max_depth").as_double();
      max_lateral_distance = this->get_parameter("max_lateral_distance").as_double();
      filter_voxel_grid_size = this->get_parameter("filter_voxel_grid_size").as_double();
      visualization_voxel_grid_size = this->get_parameter("visualization_voxel_grid_size").as_double();
      max_accumulated_points = this->get_parameter("max_accumulated_points").as_int();

      // Create callback groups
      pointcloud_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      activation_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      publish_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      // Subscribe to pointcloud topic
      rclcpp::SubscriptionOptions pointcloud_subscription_options;
      pointcloud_subscription_options.callback_group = pointcloud_callback_group_;
      pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 1, std::bind(&PointcloudReconstruct::pointcloud_callback, this, _1), pointcloud_subscription_options);

      // Subscribe to activation topic
      rclcpp::SubscriptionOptions activation_subscription_options;
      activation_subscription_options.callback_group = activation_callback_group_;
      activation_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
        activation_topic, 1, std::bind(&PointcloudReconstruct::activation_callback, this, _1), activation_subscription_options);
      
      // Initialize TF2
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

      // Initialize publisher
      rclcpp::QoS pub_qos(1);
      pub_qos.reliable();
      pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, pub_qos);
      centroid_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(centroid_topic, pub_qos);

      // Initialize status publisher
      status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(status_topic, 10);

      // Initialize accumulated pointcloud
      accumulated_pointcloud = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

      // Initialize timing variables
      pointcloud_callback_times.set_capacity(MAX_TIMING_SAMPLES);

      // Initialize reconstruction active
      reconstruction_active = false;

      // Initialize centroid
      centroid = Eigen::Vector3d::Zero();
      centroid_initialized = false;

      // Initialize timer
      timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)), 
                                            std::bind(&PointcloudReconstruct::publish_callback, this),
                                            publish_callback_group_);
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
    // Activation callback
    void activation_callback(const std_msgs::msg::Bool & msg)
    {
      RCLCPP_INFO(this->get_logger(), "Received activation message: %d", msg.data);

      if (reconstruction_active && !msg.data)
      {
        RCLCPP_INFO(this->get_logger(), "Reconstruction deactivated");
        reconstruction_active = false;
        centroid_initialized = false;
      }
      else if (!reconstruction_active && msg.data)
      {
        RCLCPP_INFO(this->get_logger(), "Reconstruction activated");
        std::lock_guard<std::mutex> lock(pointcloud_mutex);
        reconstruction_active = true;
        centroid_initialized = false;
        accumulated_clouds = 0;
        accumulated_pointcloud->clear();
      }
    }

    // Pointcloud callback
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2 & msg)
    {
      // Check if reconstruction is active
      if (!reconstruction_active) return;

      // Check if the accumulated pointcloud is full
      if (accumulated_pointcloud->size() >= max_accumulated_points)
      {
        RCLCPP_INFO(this->get_logger(), "Accumulated pointcloud is full. Ignoring inputs...");
        return;
      }

      auto start_time = std::chrono::high_resolution_clock::now();
      
      if (verbose) RCLCPP_INFO(this->get_logger(), "I got pointcloud in frame: %s", msg.header.frame_id.c_str());

      // Update current time
      auto current_time = this->get_clock()->now();

      // Convert the pointcloud to PCL
      pcl::PointCloud<PointT>::Ptr input_pointcloud(new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(msg, *input_pointcloud);

      // Filter out points that are too far away in depth
      pcl::PointCloud<PointT>::Ptr filtered_pointcloud(new pcl::PointCloud<PointT>);
      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(input_pointcloud);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.0, max_depth);
      pass.filter(*filtered_pointcloud);

      // Filter out points that are too far away laterally
      pass.setInputCloud(filtered_pointcloud);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-max_lateral_distance, max_lateral_distance);
      pass.filter(*filtered_pointcloud);

      // Filter outliers
      pcl::StatisticalOutlierRemoval<PointT> sorfilter;
      sorfilter.setInputCloud (filtered_pointcloud);
      sorfilter.setMeanK (8);
      sorfilter.setStddevMulThresh (1.0);
      sorfilter.filter (*filtered_pointcloud);

      // Check if the downsampled pointcloud is empty
      if (filtered_pointcloud->size() == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Filtered pointcloud is empty. Ignoring it.");
        return;
      }

      if (verbose) RCLCPP_INFO(this->get_logger(), "Filtered pointcloud size: %d", filtered_pointcloud->size());

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
      Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
      Eigen::Vector3d translation(transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z);
      Eigen::Quaterniond rotation(transform.transform.rotation.w,
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z);
      transform_matrix.block<3,3>(0,0) = rotation.toRotationMatrix();
      transform_matrix.block<3,1>(0,3) = translation;
      pcl::PointCloud<PointT>::Ptr transformed_pointcloud(new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(*filtered_pointcloud, *transformed_pointcloud, transform_matrix);

      // Downsample the pointcloud
      pcl::PointCloud<PointT>::Ptr downsampled_pointcloud(new pcl::PointCloud<PointT>);
      pcl::VoxelGrid<PointT> voxel_grid;
      voxel_grid.setDownsampleAllData(true);
      voxel_grid.setInputCloud(transformed_pointcloud);
      voxel_grid.setLeafSize(filter_voxel_grid_size, filter_voxel_grid_size, filter_voxel_grid_size);
      voxel_grid.filter(*downsampled_pointcloud);

      // Check if the downsampled pointcloud is empty
      if (downsampled_pointcloud->size() == 0)
      {
        RCLCPP_WARN(this->get_logger(), "Downsampled pointcloud is empty after filtering. Ignoring it.");
        return;
      }

      // Register the pointcloud
      pcl::PointCloud<PointT>::Ptr registered_pointcloud(new pcl::PointCloud<PointT>);
      if (accumulated_pointcloud->size() == 0)
      {
        *registered_pointcloud = *downsampled_pointcloud;
      }
      else
      {
        // Downsample the accumulated pointcloud
        pcl::PointCloud<PointT>::Ptr downsampled_accumulated_pointcloud(new pcl::PointCloud<PointT>);
        // voxel_grid.setDownsampleAllData(true);
        // voxel_grid.setInputCloud(accumulated_pointcloud);
        // voxel_grid.setLeafSize(filter_voxel_grid_size, filter_voxel_grid_size, filter_voxel_grid_size);
        // voxel_grid.filter(*downsampled_accumulated_pointcloud);

        #if 1
        // GICP in CTF
        for (int i=2; i>=0; i--)
        {
          double leaf_size = filter_voxel_grid_size * std::pow(2, i);
          
          // Downsample the accumulated pointcloud
          voxel_grid.setInputCloud(accumulated_pointcloud);
          voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
          voxel_grid.filter(*downsampled_accumulated_pointcloud);
          
          // Downsample the pointcloud
          voxel_grid.setInputCloud(transformed_pointcloud);
          voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
          voxel_grid.filter(*downsampled_pointcloud);

          // Check number of points
          if (downsampled_pointcloud->size() < 50 || downsampled_accumulated_pointcloud->size() < 50)
          {
            RCLCPP_WARN(this->get_logger(), "Not enough points to register. Skipping registration.");
            continue;
          }
          
          // Register the pointcloud
          pcl::GeneralizedIterativeClosestPoint6D registration;
          registration.setInputSource(downsampled_pointcloud);
          registration.setInputTarget(downsampled_accumulated_pointcloud);
          registration.setUseReciprocalCorrespondences(true);
          registration.align(*registered_pointcloud);

          // Get the transformation
          Eigen::Matrix4f transformation = registration.getFinalTransformation();
          pcl::transformPointCloud(*transformed_pointcloud, *transformed_pointcloud, transformation);
        }
        *registered_pointcloud = *transformed_pointcloud;
        #elif 0
        // GICP downsampled, but store the full pointcloud
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> registration;
        registration.setInputSource(downsampled_pointcloud);
        registration.setInputTarget(downsampled_accumulated_pointcloud);
        registration.setUseReciprocalCorrespondences(true);
        registration.align(*registered_pointcloud);

        // Get the transformation
        Eigen::Matrix4f transformation = registration.getFinalTransformation();
        pcl::transformPointCloud(*transformed_pointcloud, *registered_pointcloud, transformation);
        voxel_grid.setInputCloud(registered_pointcloud);
        voxel_grid.setLeafSize(visualization_voxel_grid_size, visualization_voxel_grid_size, visualization_voxel_grid_size);
        voxel_grid.filter(*registered_pointcloud);
        #elif 0
        // GICP on downsampled pointclouds
        pcl::GeneralizedIterativeClosestPoint<PointT, PointT> registration;
        registration.setInputSource(downsampled_pointcloud);
        registration.setInputTarget(downsampled_accumulated_pointcloud);
        registration.setUseReciprocalCorrespondences(true);
        registration.align(*registered_pointcloud);
        #else
        // 3D NDT
        pcl::NormalDistributionsTransform<PointT, PointT> registration;
        registration.setInputSource(transformed_pointcloud);
        registration.setInputTarget(accumulated_pointcloud);
        registration.setResolution(filter_voxel_grid_size*2);
        registration.align(*registered_pointcloud);
        #endif

        // if (!registration.hasConverged())
        // {
        //   if (verbose) RCLCPP_ERROR(this->get_logger(), "Registration has not converged");
        //   return;
        // }
        // else
        // {
        //   double score = registration.getFitnessScore();  // Average squared distances from correspondences
        //   if (verbose) RCLCPP_INFO(this->get_logger(), "Registration fitness score: %f", sqrt(score));
        // } 
      }

      {
        std::lock_guard<std::mutex> lock(pointcloud_mutex);
        // Accumulate the pointcloud
        *accumulated_pointcloud += *registered_pointcloud;

        // Downsample the accumulated pointcloud
        voxel_grid.setDownsampleAllData(true);
        voxel_grid.setInputCloud(accumulated_pointcloud);
        voxel_grid.setLeafSize(visualization_voxel_grid_size, visualization_voxel_grid_size, visualization_voxel_grid_size);
        voxel_grid.filter(*accumulated_pointcloud);
      }

      // Increment the accumulated clouds counter
      accumulated_clouds++;

      if (verbose) RCLCPP_INFO(this->get_logger(), "Accumulated %d clouds", accumulated_clouds);
      if (verbose) RCLCPP_INFO(this->get_logger(), "Accumulated pointcloud size: %d", accumulated_pointcloud->size());

      // Initialize the centroid if it is not initialized
      Eigen::Vector4d new_centroid;
      pcl::compute3DCentroid(*accumulated_pointcloud, new_centroid);
      if (!centroid_initialized)
      {
        centroid = new_centroid.head<3>();
        centroid_initialized = true;
      }
      else
      {
        // Update the centroid smoothly
        double centroid_update_k = 0.8;
        centroid = centroid_update_k * centroid + (1.0 - centroid_update_k) * new_centroid.head<3>();
      }

      // Measure and log timing
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
      double duration_ms = duration.count() / 1000.0;
      add_timing_sample(pointcloud_callback_times, duration_ms);
      log_timing_stats("Pointcloud callback", pointcloud_callback_times);
      
      if (verbose) {
        RCLCPP_INFO(this->get_logger(), " · Pointcloud callback took: %.3fms", duration_ms); 
      }
    }

    // Publish callback
    void publish_callback()
    {
      
      // Publish the accumulated pointcloud
      sensor_msgs::msg::PointCloud2 accumulated_pointcloud_msg;
      {   // Lock the pointcloud mutex
        std::lock_guard<std::mutex> lock(pointcloud_mutex);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr converted_accumulated_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*accumulated_pointcloud, *converted_accumulated_pointcloud);
        pcl::toROSMsg(*converted_accumulated_pointcloud, accumulated_pointcloud_msg);
      }
      accumulated_pointcloud_msg.header.stamp = this->get_clock()->now();
      accumulated_pointcloud_msg.header.frame_id = fixed_frame;
      pointcloud_publisher_->publish(accumulated_pointcloud_msg);

      // Publish the centroid of the accumulated pointcloud
      geometry_msgs::msg::PointStamped centroid_msg;
      centroid_msg.header.stamp = this->get_clock()->now();
      centroid_msg.header.frame_id = fixed_frame;
      centroid_msg.point.x = centroid[0];
      centroid_msg.point.y = centroid[1];
      centroid_msg.point.z = centroid[2];
      centroid_publisher_->publish(centroid_msg);

      // Publish the status
      std_msgs::msg::Bool status_msg;
      status_msg.data = reconstruction_active;
      status_publisher_->publish(status_msg);
    }


    // -------------------------------------
    // VARIABLES
    // -------------------------------------
    // Parameters
    bool verbose;
    std::string input_topic;
    std::string activation_topic;
    std::string output_topic;
    std::string centroid_topic;
    std::string status_topic;
    std::string fixed_frame;
    double publish_rate;
    int tf_timeout_ms;
    double max_depth;
    double max_lateral_distance;
    double filter_voxel_grid_size;
    double visualization_voxel_grid_size;
    int max_accumulated_points;

    // Variables
    int accumulated_clouds;
    bool reconstruction_active;
    Eigen::Vector3d centroid;
    bool centroid_initialized;
    pcl::PointCloud<PointT>::Ptr accumulated_pointcloud;

    // Timing variables
    boost::circular_buffer<double> pointcloud_callback_times;
    static constexpr size_t MAX_TIMING_SAMPLES = 100;

    // TF2 components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Callback groups
    std::mutex pointcloud_mutex;
    rclcpp::CallbackGroup::SharedPtr pointcloud_callback_group_;
    rclcpp::CallbackGroup::SharedPtr activation_callback_group_;
    rclcpp::CallbackGroup::SharedPtr publish_callback_group_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr activation_subscription_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr centroid_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudReconstruct>();
  rclcpp::executors::MultiThreadedExecutor executor;
  std::cout << "Executor running with " << executor.get_number_of_threads() << " threads" << std::endl;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
