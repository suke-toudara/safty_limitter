#ifndef SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_
#define SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

namespace safety_limiter
{

class SafetyLimiterNode : public rclcpp::Node
{
public:
  explicit SafetyLimiterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Point2D
  {
    double x;
    double y;
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg);
  void timerCallback();

  bool getCurrentPose(geometry_msgs::msg::Pose & pose) const;
  void predictTrajectory(
    const geometry_msgs::msg::Pose & start_pose,
    std::vector<geometry_msgs::msg::Pose> & predicted_poses) const;

  std::vector<Point2D> transformFootprint(
    const geometry_msgs::msg::Pose & pose,
    const std::vector<Point2D> & local_footprint) const;
  std::vector<Point2D> expandFootprint(
    const std::vector<Point2D> & local_footprint, double margin) const;

  bool checkFootprintCollision(
    const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
    const std::vector<Point2D> & local_footprint) const;

  void publishFutureMotionVisualization(
    const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
    bool collision, bool collision_margin) const;
  void publishCmdVel(bool collision);

  static bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon);
  static double polygonCentroidX(const std::vector<Point2D> & polygon);
  static double polygonCentroidY(const std::vector<Point2D> & polygon);

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_margin_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr future_motion_prediction_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr future_motion_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_in_map_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
  std::vector<Point2D> footprint_local_;
  std::vector<Point2D> cloud_points_map_;

  bool has_cmd_vel_;
  bool has_cloud_;
  bool has_footprint_;

  std::string robot_frame_;
  std::string map_frame_;
  std::string cmd_vel_in_topic_;
  std::string cmd_vel_out_topic_;
  std::string cloud_topic_;
  std::string footprint_topic_;
  std::string future_motion_prediction_topic_;
  std::string future_motion_markers_topic_;
  std::string cloud_in_map_topic_;
  std::string collision_topic_;
  std::string collision_margin_topic_;
  double publish_rate_;
  double prediction_time_;
  double prediction_step_;
  double footprint_margin_;
  bool enable_visualization_;
  int visualization_stride_;
};

}  // namespace safety_limiter

#endif  // SAFETY_LIMITER__SAFETY_LIMITER_NODE_HPP_
