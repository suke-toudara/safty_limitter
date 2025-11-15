#ifndef SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_
#define SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <memory>
#include <vector>

namespace safety_limiter
{

class SafetyLimiterNode : public rclcpp::Node
{
public:
  explicit SafetyLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void timerCallback();
  bool checkTopicTimeout();

  void predictTrajectory(std::vector<geometry_msgs::msg::Pose> & predicted_poses);

  double checkCollision(
    const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  void publishVisualization(const std::vector<geometry_msgs::msg::Pose> & predicted_poses);

  double getFootprintRadius() const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Data storage
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  geometry_msgs::msg::PolygonStamped footprint_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_cloud_time_;

  // Parameters
  std::string robot_frame_;
  std::string map_frame_;
  double publish_rate_;
  double prediction_time_;
  double prediction_step_;
  double slowdown_margin_;
  double min_velocity_scale_;
  bool enable_visualization_;
  double cmd_vel_timeout_;
  double cloud_timeout_;
};

}  // namespace safety_limiter

#endif  // SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_
