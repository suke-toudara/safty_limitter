#include "safety_limiter/safety_limiter_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <cmath>
#include <limits>

namespace safety_limiter
{
SafetyLimiterNode::SafetyLimiterNode(const rclcpp::NodeOptions & options)
: Node("safety_limiter", options),
  has_cmd_vel_(false),
  has_cloud_(false),
  has_footprint_(false)
{
  robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
  prediction_time_ = declare_parameter<double>("prediction_time", 2.0);
  prediction_step_ = declare_parameter<double>("prediction_step", 0.1);
  footprint_margin_ = declare_parameter<double>("footprint_margin", 0.15);
  enable_visualization_ = declare_parameter<bool>("enable_visualization", true);
  visualization_stride_ = declare_parameter<int>("visualization_stride", 3);

  cmd_vel_in_topic_ = declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_in");
  cmd_vel_out_topic_ = declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "cloud");
  footprint_topic_ = declare_parameter<std::string>("footprint_topic", "footprint");
  future_motion_prediction_topic_ = declare_parameter<std::string>(
    "future_motion_prediction_topic", "future_motion_prediction");
  future_motion_markers_topic_ = declare_parameter<std::string>(
    "future_motion_markers_topic", "future_motion_markers");
  cloud_in_map_topic_ = declare_parameter<std::string>("cloud_in_map_topic", "cloud_in_map");
  collision_topic_ = declare_parameter<std::string>("collision_topic", "collision");
  collision_margin_topic_ = declare_parameter<std::string>(
    "collision_margin_topic", "collision_margin");

  latest_cmd_vel_ = std::make_shared<geometry_msgs::msg::Twist>();

  // cmd_vel: pure_pursuit 等 (Reliable, depth=10)
  const rclcpp::QoS cmd_vel_qos(10);
  // cloud: laserscan_to_pointcloud / LiDAR driver (Best Effort, SensorDataQoS)
  const rclcpp::QoS cloud_qos = rclcpp::SensorDataQoS();
  // footprint: footprint_publisher (Transient Local, Reliable)
  const rclcpp::QoS footprint_qos = rclcpp::QoS(1).transient_local().reliable();

  collision_pub_ = create_publisher<std_msgs::msg::Bool>(collision_topic_, rclcpp::QoS(10));
  collision_margin_pub_ = create_publisher<std_msgs::msg::Bool>(
    collision_margin_topic_, rclcpp::QoS(10));
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic_, cmd_vel_qos);
  future_motion_prediction_pub_ = create_publisher<nav_msgs::msg::Path>(
    future_motion_prediction_topic_, rclcpp::QoS(10));
  future_motion_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    future_motion_markers_topic_, rclcpp::QoS(10));
  cloud_in_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    cloud_in_map_topic_, cloud_qos);

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic_, cmd_vel_qos,
    std::bind(&SafetyLimiterNode::cmdVelCallback, this, std::placeholders::_1));
  point_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, cloud_qos,
    std::bind(&SafetyLimiterNode::pointCloudCallback, this, std::placeholders::_1));
  footprint_sub_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    footprint_topic_, footprint_qos,
    std::bind(&SafetyLimiterNode::footprintCallback, this, std::placeholders::_1));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyLimiterNode::timerCallback, this));

  RCLCPP_INFO(get_logger(),
    "Safety limiter started (map=%s, robot=%s)", map_frame_.c_str(), robot_frame_.c_str());
  RCLCPP_INFO(get_logger(),
    "Sub: cmd_vel_in=%s (Reliable), cloud=%s (BestEffort), footprint=%s (TransientLocal)",
    cmd_vel_in_topic_.c_str(), cloud_topic_.c_str(), footprint_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Pub: cmd_vel_out=%s (stop on /%s)",
    cmd_vel_out_topic_.c_str(), collision_topic_.c_str());
  RCLCPP_INFO(get_logger(),
    "Pub: future_motion_prediction=%s, future_motion_markers=%s, cloud_in_map=%s (BestEffort)",
    future_motion_prediction_topic_.c_str(),
    future_motion_markers_topic_.c_str(),
    cloud_in_map_topic_.c_str());
}

void SafetyLimiterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = msg;
  has_cmd_vel_ = true;
}

void SafetyLimiterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Could not transform cloud to %s: %s", map_frame_.c_str(), ex.what());
    return;
  }

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transform);
  latest_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(transformed_cloud);

  cloud_points_map_.clear();
  cloud_points_map_.reserve(transformed_cloud.width * transformed_cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y)) {
      continue;
    }
    cloud_points_map_.push_back({*iter_x, *iter_y});
  }

  has_cloud_ = !cloud_points_map_.empty();
  cloud_in_map_pub_->publish(transformed_cloud);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
    "Cloud received: frame=%s -> %s, valid_points=%zu",
    msg->header.frame_id.c_str(), map_frame_.c_str(), cloud_points_map_.size());
}

void SafetyLimiterNode::footprintCallback(
  const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_local_.clear();
  for (const auto & point : msg->polygon.points) {
    footprint_local_.push_back({point.x, point.y});
  }
  has_footprint_ = !footprint_local_.empty();
}

bool SafetyLimiterNode::getCurrentPose(geometry_msgs::msg::Pose & pose) const
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero);
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Could not get robot pose (%s -> %s): %s",
      map_frame_.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }
}

void SafetyLimiterNode::predictTrajectory(
  const geometry_msgs::msg::Pose & start_pose,
  std::vector<geometry_msgs::msg::Pose> & predicted_poses) const
{
  predicted_poses.clear();
  if (!has_cmd_vel_) {
    return;
  }

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf2::Quaternion(
    start_pose.orientation.x, start_pose.orientation.y,
    start_pose.orientation.z, start_pose.orientation.w)).getRPY(roll, pitch, yaw);

  const double vx = latest_cmd_vel_->linear.x;
  const double vy = latest_cmd_vel_->linear.y;
  const double omega = latest_cmd_vel_->angular.z;

  geometry_msgs::msg::Pose predicted_pose = start_pose;
  double predicted_yaw = yaw;

  for (double t = 0.0; t <= prediction_time_; t += prediction_step_) {
    predicted_pose.position.x +=
      (vx * std::cos(predicted_yaw) - vy * std::sin(predicted_yaw)) * prediction_step_;
    predicted_pose.position.y +=
      (vx * std::sin(predicted_yaw) + vy * std::cos(predicted_yaw)) * prediction_step_;

    predicted_yaw += omega * prediction_step_;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, predicted_yaw);
    predicted_pose.orientation = tf2::toMsg(q);
    predicted_poses.push_back(predicted_pose);
  }
}

std::vector<SafetyLimiterNode::Point2D> SafetyLimiterNode::transformFootprint(
  const geometry_msgs::msg::Pose & pose,
  const std::vector<Point2D> & local_footprint) const
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  std::vector<Point2D> world_footprint;
  world_footprint.reserve(local_footprint.size());
  for (const auto & local : local_footprint) {
    Point2D world;
    world.x = pose.position.x + cos_yaw * local.x - sin_yaw * local.y;
    world.y = pose.position.y + sin_yaw * local.x + cos_yaw * local.y;
    world_footprint.push_back(world);
  }
  return world_footprint;
}

std::vector<SafetyLimiterNode::Point2D> SafetyLimiterNode::expandFootprint(
  const std::vector<Point2D> & local_footprint, double margin) const
{
  if (local_footprint.empty() || margin <= 0.0) {
    return local_footprint;
  }

  const double cx = polygonCentroidX(local_footprint);
  const double cy = polygonCentroidY(local_footprint);

  std::vector<Point2D> expanded;
  expanded.reserve(local_footprint.size());
  for (const auto & point : local_footprint) {
    const double dx = point.x - cx;
    const double dy = point.y - cy;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) {
      expanded.push_back(point);
      continue;
    }
    const double scale = (dist + margin) / dist;
    expanded.push_back({cx + dx * scale, cy + dy * scale});
  }
  return expanded;
}

bool SafetyLimiterNode::pointInPolygon(
  double x, double y, const std::vector<Point2D> & polygon)
{
  if (polygon.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];
    const bool intersect = ((pi.y > y) != (pj.y > y)) &&
      (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y + 1e-12) + pi.x);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

double SafetyLimiterNode::polygonCentroidX(const std::vector<Point2D> & polygon)
{
  double sum = 0.0;
  for (const auto & p : polygon) {
    sum += p.x;
  }
  return sum / static_cast<double>(polygon.size());
}

double SafetyLimiterNode::polygonCentroidY(const std::vector<Point2D> & polygon)
{
  double sum = 0.0;
  for (const auto & p : polygon) {
    sum += p.y;
  }
  return sum / static_cast<double>(polygon.size());
}

bool SafetyLimiterNode::checkFootprintCollision(
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  const std::vector<Point2D> & local_footprint) const
{
  if (predicted_poses.empty() || local_footprint.empty() || cloud_points_map_.empty()) {
    return false;
  }

  for (const auto & pose : predicted_poses) {
    const auto world_footprint = transformFootprint(pose, local_footprint);
    for (const auto & cloud_point : cloud_points_map_) {
      if (pointInPolygon(cloud_point.x, cloud_point.y, world_footprint)) {
        return true;
      }
    }
  }
  return false;
}

void SafetyLimiterNode::publishFutureMotionVisualization(
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  bool collision, bool collision_margin) const
{
  if (!enable_visualization_) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;
  for (const auto & pose : predicted_poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = pose;
    path.poses.push_back(pose_stamped);
  }
  future_motion_prediction_pub_->publish(path);

  visualization_msgs::msg::MarkerArray markers;
  int marker_id = 0;
  const int stride = std::max(1, visualization_stride_);
  const auto expanded_footprint = expandFootprint(footprint_local_, footprint_margin_);

  for (size_t i = 0; i < predicted_poses.size(); i += static_cast<size_t>(stride)) {
    const auto & pose = predicted_poses[i];
    const auto world_footprint = transformFootprint(pose, footprint_local_);
    const auto world_margin = transformFootprint(pose, expanded_footprint);

    visualization_msgs::msg::Marker footprint_marker;
    footprint_marker.header.stamp = now();
    footprint_marker.header.frame_id = map_frame_;
    footprint_marker.ns = "footprint";
    footprint_marker.id = marker_id++;
    footprint_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    footprint_marker.action = visualization_msgs::msg::Marker::ADD;
    footprint_marker.pose.orientation.w = 1.0;
    footprint_marker.scale.x = 0.02;
    footprint_marker.color.r = collision ? 1.0f : 0.0f;
    footprint_marker.color.g = collision ? 0.0f : 1.0f;
    footprint_marker.color.b = 0.0f;
    footprint_marker.color.a = 0.8f;
    footprint_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
    for (const auto & point : world_footprint) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = pose.position.z;
      footprint_marker.points.push_back(p);
    }
    if (!world_footprint.empty()) {
      geometry_msgs::msg::Point p;
      p.x = world_footprint.front().x;
      p.y = world_footprint.front().y;
      p.z = pose.position.z;
      footprint_marker.points.push_back(p);
    }
    markers.markers.push_back(footprint_marker);

    visualization_msgs::msg::Marker margin_marker = footprint_marker;
    margin_marker.ns = "footprint_margin";
    margin_marker.id = marker_id++;
    margin_marker.scale.x = 0.015;
    margin_marker.color.r = collision_margin ? 1.0f : 1.0f;
    margin_marker.color.g = collision_margin ? 0.5f : 1.0f;
    margin_marker.color.b = 0.0f;
    margin_marker.color.a = 0.45f;
    margin_marker.points.clear();
    for (const auto & point : world_margin) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = pose.position.z;
      margin_marker.points.push_back(p);
    }
    if (!world_margin.empty()) {
      geometry_msgs::msg::Point p;
      p.x = world_margin.front().x;
      p.y = world_margin.front().y;
      p.z = pose.position.z;
      margin_marker.points.push_back(p);
    }
    markers.markers.push_back(margin_marker);
  }

  future_motion_markers_pub_->publish(markers);
}

void SafetyLimiterNode::publishCmdVel(bool collision)
{
  geometry_msgs::msg::Twist cmd_vel;
  if (collision || !has_cmd_vel_) {
    cmd_vel_pub_->publish(cmd_vel);
    return;
  }
  cmd_vel_pub_->publish(*latest_cmd_vel_);
}

void SafetyLimiterNode::timerCallback()
{
  std_msgs::msg::Bool collision_msg;
  std_msgs::msg::Bool collision_margin_msg;
  collision_msg.data = false;
  collision_margin_msg.data = false;

  if (!has_footprint_ || !has_cloud_ || !has_cmd_vel_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Waiting for inputs (footprint=%d, cloud=%d, cmd_vel=%d, cloud_points=%zu)",
      has_footprint_, has_cloud_, has_cmd_vel_, cloud_points_map_.size());
    collision_pub_->publish(collision_msg);
    collision_margin_pub_->publish(collision_margin_msg);
    publishCmdVel(false);
    return;
  }

  geometry_msgs::msg::Pose current_pose;
  if (!getCurrentPose(current_pose)) {
    collision_pub_->publish(collision_msg);
    collision_margin_pub_->publish(collision_margin_msg);
    publishCmdVel(false);
    return;
  }

  std::vector<geometry_msgs::msg::Pose> predicted_poses;
  predictTrajectory(current_pose, predicted_poses);

  const bool collision = checkFootprintCollision(predicted_poses, footprint_local_);
  const auto expanded_footprint = expandFootprint(footprint_local_, footprint_margin_);
  const bool collision_margin = checkFootprintCollision(predicted_poses, expanded_footprint);

  collision_msg.data = collision;
  collision_margin_msg.data = collision_margin;
  collision_pub_->publish(collision_msg);
  collision_margin_pub_->publish(collision_margin_msg);

  publishFutureMotionVisualization(predicted_poses, collision, collision_margin);
  publishCmdVel(collision);

  if (collision) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Collision detected in predicted path (footprint)");
  } else if (collision_margin) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "Collision detected in predicted path (expanded footprint)");
  }
}

}  // namespace safety_limiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<safety_limiter::SafetyLimiterNode>());
  rclcpp::shutdown();
  return 0;
}
