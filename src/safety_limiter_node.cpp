#include "safety_limiter/safety_limiter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace safety_limiter
{

SafetyLimiterNode::SafetyLimiterNode(const rclcpp::NodeOptions & options): Node("safety_limiter_node", options)
{
  latest_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  // Declare parameters
  this->declare_parameter("robot_frame", "base_link");
  this->declare_parameter("map_frame", "odom");
  this->declare_parameter("publish_rate", 10.0);
  this->declare_parameter("prediction_time", 2.0);
  this->declare_parameter("prediction_step", 0.1);
  this->declare_parameter("footprint_x_front", 0.5);
  this->declare_parameter("footprint_x_rear", 0.5);
  this->declare_parameter("footprint_y", 0.4);
  this->declare_parameter("safety_margin", 0.0);
  this->declare_parameter("slowdown_margin", 0.2);
  this->declare_parameter("min_velocity_scale", 0.0);
  this->declare_parameter("enable_visualization", true);
  this->declare_parameter("cmd_vel_timeout", 0.5);
  this->declare_parameter("cloud_timeout", 1.0);

  // Get parameters
  robot_frame_ = this->get_parameter("robot_frame").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  prediction_time_ = this->get_parameter("prediction_time").as_double();
  prediction_step_ = this->get_parameter("prediction_step").as_double();
  footprint_x_front_ = this->get_parameter("footprint_x_front").as_double();
  footprint_x_rear_ = this->get_parameter("footprint_x_rear").as_double();
  footprint_y_ = this->get_parameter("footprint_y").as_double();
  safety_margin_ = this->get_parameter("safety_margin").as_double();
  slowdown_margin_ = this->get_parameter("slowdown_margin").as_double();
  min_velocity_scale_ = this->get_parameter("min_velocity_scale").as_double();
  enable_visualization_ = this->get_parameter("enable_visualization").as_bool();
  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
  cloud_timeout_ = this->get_parameter("cloud_timeout").as_double();

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

  // Predict robot poses
  std::vector<geometry_msgs::msg::Pose> predicted_poses;
  predictTrajectory(predicted_poses);

  // Check collision and get velocity scale (0.0 = stop, 1.0 = full speed)
  double velocity_scale = checkCollision(predicted_poses, latest_cloud_);

  // Apply velocity scale
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

double SafetyLimiterNode::checkCollision(
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  double vel_scale = 1.0;
  if (predicted_poses.empty() || cloud->points.empty()) return vel_scale;

  // Check each predicted pose
  for (const auto & pose : predicted_poses) {
    // Calculate center for efficient search
    pcl::PointXYZ center;
    center.x = pose.position.x;
    center.y = pose.position.y;
    center.z = pose.position.z;

    // Calculate footprint radius: maximum distance from center (base_link) to any corner
    // Corners are at (±footprint_x_front/rear, ±footprint_y)
    double footprint_radius = std::max(
      std::sqrt(footprint_x_front_ * footprint_x_front_ + footprint_y_ * footprint_y_),
      std::sqrt(footprint_x_rear_ * footprint_x_rear_ + footprint_y_ * footprint_y_)
    );

    // Search radius: footprint radius + safety margin + slowdown margin
    double search_radius = footprint_radius + safety_margin_ + slowdown_margin_;

    // Use KD-tree radius search
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    /*
    [1]query   – クエリ点．
    [2]indices – 探索範囲内に存在する近傍点のインデックスが格納されるベクトル．クエリ点までの距離で降順
    [3]dists   – 探索範囲内の近傍点までの距離
    [4]radius  – 探索範囲．
    [5]params  – 探索パラメータ．
    */
    if (kdtree_.radiusSearch(center, point_indices, point_distances, search_radius) > 0) {
      for (size_t i = 0; i < point_indices.size(); ++i) {
        const pcl::PointXYZ & pt = cloud->points[point_indices[i]];
        if (isPointInFootprint(pt, pose, safety_margin_)) {
          return min_velocity_scale_; // Collision detected - stop
        }

        if (isPointInFootprint(pt, pose, safety_margin_ + slowdown_margin_)) {
          // Calculate distance-based scale using footprint radius
          double dx = pt.x - center.x;
          double dy = pt.y - center.y;
          double dist_from_center = std::sqrt(dx * dx + dy * dy);

          // Distance from footprint edge (approximation using footprint radius)
          double distance_to_footprint = dist_from_center - footprint_radius - safety_margin_;

          if (distance_to_footprint < slowdown_margin_) {
            // Linear interpolation: 0 at footprint edge, 1 at slowdown_margin distance
            double scale = distance_to_footprint / slowdown_margin_;
            scale = std::max(min_velocity_scale_, std::min(1.0, scale));
            vel_scale = std::min(vel_scale, scale);
          }
        }
      }
    }
  }

  return vel_scale;
}

bool SafetyLimiterNode::isPointInFootprint(
  const pcl::PointXYZ & point,
  const geometry_msgs::msg::Pose & robot_pose,
  double footprint_margin)
{
  tf2::Quaternion q(robot_pose.orientation.x, robot_pose.orientation.y,
                    robot_pose.orientation.z, robot_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Transform point to robot frame
  double dx = point.x - robot_pose.position.x;
  double dy = point.y - robot_pose.position.y;

  double local_x = dx * std::cos(-yaw) - dy * std::sin(-yaw);
  double local_y = dx * std::sin(-yaw) + dy * std::cos(-yaw);

  // Check if point is inside expanded footprint
  bool in_x = (local_x >= -(footprint_x_rear_ + footprint_margin)) &&
              (local_x <= (footprint_x_front_ + footprint_margin));
  bool in_y = (local_y >= -(footprint_y_ + footprint_margin)) &&
              (local_y <= (footprint_y_ + footprint_margin));

  return in_x && in_y;
}

void SafetyLimiterNode::publishVisualization()
{
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
