#include "safety_limiter/safety_limiter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace safety_limiter
{

SafetyLimiterNode::SafetyLimiterNode(const rclcpp::NodeOptions & options): Node("safety_limiter_node", options)
{
  latest_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Declare and get parameters
  robot_frame_ = this->declare_parameter("robot_frame", "base_link");
  map_frame_ = this->declare_parameter("map_frame", "odom");
  publish_rate_ = this->declare_parameter("publish_rate", 10.0);
  prediction_time_ = this->declare_parameter("prediction_time", 2.0);
  prediction_step_ = this->declare_parameter("prediction_step", 0.1);
  slowdown_margin_ = this->declare_parameter("slowdown_margin", 0.2);
  min_velocity_scale_ = this->declare_parameter("min_velocity_scale", 0.0);
  enable_visualization_ = this->declare_parameter("enable_visualization", true);
  cmd_vel_timeout_ = this->declare_parameter("cmd_vel_timeout", 0.5);
  cloud_timeout_ = this->declare_parameter("cloud_timeout", 1.0);

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("safety_markers", 10);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&SafetyLimiterNode::cmdVelCallback, this, std::placeholders::_1));

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 10,
    std::bind(&SafetyLimiterNode::pointCloudCallback, this, std::placeholders::_1));
  
  footprint_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "footprint", 1,
    std::bind(&SafetyLimiterNode::footprintCallback, this, std::placeholders::_1));

  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyLimiterNode::timerCallback, this));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Safety Limiter Node initialized");
}

void SafetyLimiterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = msg;
  last_cmd_vel_time_ = this->now();
}

void SafetyLimiterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    return;
  } 
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(*msg, transformed_cloud, transform_stamped);
  
  pcl::fromROSMsg(transformed_cloud, *latest_cloud_);
  if (latest_cloud_->points.size() > 0) {
    kdtree_.setInputCloud(latest_cloud_);
  }
  last_cloud_time_ = this->now();
}

bool SafetyLimiterNode::checkTopicTimeout()
{
  rclcpp::Time current_time = this->now();
  bool timeout = false;

  double cmd_vel_age = (current_time - last_cmd_vel_time_).seconds();
  if (cmd_vel_age > cmd_vel_timeout_) timeout = true;

  double cloud_age = (current_time - last_cloud_time_).seconds();
  if (cloud_age > cloud_timeout_) timeout = true;

  return timeout;
}

void SafetyLimiterNode::footprintCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
{
  footprint_ = *msg;
}

void SafetyLimiterNode::timerCallback()
{
  if (checkTopicTimeout()) {
    geometry_msgs::msg::Twist zero_cmd_vel;
    cmd_vel_pub_->publish(zero_cmd_vel);
    return;
  }

  std::vector<geometry_msgs::msg::Pose> predicted_poses;
  predictTrajectory(predicted_poses);

  double velocity_scale = checkCollision(predicted_poses, latest_cloud_);

  geometry_msgs::msg::Twist output_cmd_vel = *latest_cmd_vel_;
  output_cmd_vel.linear.x *= velocity_scale;
  output_cmd_vel.linear.y *= velocity_scale;
  output_cmd_vel.linear.z *= velocity_scale;
  output_cmd_vel.angular.x *= velocity_scale;
  output_cmd_vel.angular.y *= velocity_scale;
  output_cmd_vel.angular.z *= velocity_scale;
  cmd_vel_pub_->publish(output_cmd_vel);

  if (enable_visualization_) {
    publishVisualization(predicted_poses);
  }
}

void SafetyLimiterNode::predictTrajectory(std::vector<geometry_msgs::msg::Pose> & predicted_poses)
{
  predicted_poses.clear();
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(
      map_frame_, robot_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    return;
  }

  // Current pose
  geometry_msgs::msg::Pose current_pose;
  current_pose.position.x = transform_stamped.transform.translation.x;
  current_pose.position.y = transform_stamped.transform.translation.y;
  current_pose.position.z = transform_stamped.transform.translation.z;
  current_pose.orientation = transform_stamped.transform.rotation;

  // Extract yaw from quaternion
  tf2::Quaternion q(
    current_pose.orientation.x,
    current_pose.orientation.y,
    current_pose.orientation.z,
    current_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double vx = latest_cmd_vel_.linear.x;
  double vy = latest_cmd_vel_.linear.y;
  double omega = latest_cmd_vel_.angular.z;

  geometry_msgs::msg::Pose predicted_pose = current_pose;
  double predicted_yaw = yaw;

  double t = 0.0;
  while (t <= prediction_time_) {
    predicted_yaw += omega * prediction_step_;
    predicted_pose.position.x += (vx * std::cos(predicted_yaw) - vy * std::sin(predicted_yaw)) * prediction_step_;
    predicted_pose.position.y += (vx * std::sin(predicted_yaw) + vy * std::cos(predicted_yaw)) * prediction_step_;
    predicted_pose.z += 0.0;  

    tf2::Quaternion pred_q;
    pred_q.setRPY(roll, pitch, predicted_yaw);
    predicted_pose.orientation.x = pred_q.x();
    predicted_pose.orientation.y = pred_q.y();
    predicted_pose.orientation.z = pred_q.z();
    predicted_pose.orientation.w = pred_q.w();
    predicted_poses.push_back(predicted_pose);
    t += prediction_step_;
  }
}

double SafetyLimiterNode::getFootprintRadius() const
{
  if (footprint_.polygon.points.empty()) return 0.5;  // Default

  double max_dist = 0.0;
  for (const auto & point : footprint_.polygon.points) {
    double dist = std::sqrt(point.x * point.x + point.y * point.y);
    max_dist = std::max(max_dist, dist);
  }
  return max_dist;
}

double SafetyLimiterNode::checkCollision(
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  double vel_scale = 1.0;
  if (predicted_poses.empty() || cloud->points.empty()) return vel_scale;

  double radius = getFootprintRadius();

  for (const auto & pose : predicted_poses) {
    pcl::PointXYZ center;
    center.x = pose.position.x;
    center.y = pose.position.y;
    center.z = pose.position.z;

    double search_radius = radius + slowdown_margin_;
    std::vector<int> indices;
    std::vector<float> distances;

    if (kdtree_.radiusSearch(center, indices, distances, search_radius) > 0) {
      for (size_t i = 0; i < indices.size(); ++i) {
        double dist = std::sqrt(distances[i]);

        if (dist <= radius) {
          return min_velocity_scale_;  // Stop
        }

        if (dist <= radius + slowdown_margin_) {
          double scale = (dist - radius) / slowdown_margin_;
          vel_scale = std::min(vel_scale, std::max(min_velocity_scale_, scale));
        }
      }
    }
  }

  return vel_scale;
}

void SafetyLimiterNode::publishVisualization(const std::vector<geometry_msgs::msg::Pose> & predicted_poses)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;
  double radius = getFootprintRadius();

  for (size_t i = 0; i < predicted_poses.size(); i += 3) {  // Every 3rd pose
    const auto & pose = predicted_poses[i];

    // Footprint circle (red)
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = map_frame_;
    marker.header.stamp = this->now();
    marker.ns = "footprint";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = marker.scale.y = radius * 2;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 0.5;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    markers.markers.push_back(marker);

    // Slowdown circle (yellow)
    marker.ns = "slowdown";
    marker.id = id++;
    marker.scale.x = marker.scale.y = (radius + slowdown_margin_) * 2;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.3;
    markers.markers.push_back(marker);
  }

  marker_pub_->publish(markers);
}
}  // namespace safety_limiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_limiter::SafetyLimiterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
