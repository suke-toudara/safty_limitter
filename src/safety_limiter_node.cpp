#include "safety_limiter/safety_limiter_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace safety_limiter
{

SafetyLimiterNode::SafetyLimiterNode(const rclcpp::NodeOptions & options)
: Node("safety_limiter_node", options),
  latest_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud_updated_(false)
{
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

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel_in", 10,
    std::bind(&SafetyLimiterNode::cmdVelCallback, this, std::placeholders::_1));

  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", 10,
    std::bind(&SafetyLimiterNode::pointCloudCallback, this, std::placeholders::_1));

  // Create publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", 10);

  // Create timer
  auto period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SafetyLimiterNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "Safety Limiter Node initialized");
}

void SafetyLimiterNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = msg;
}

void SafetyLimiterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Convert ROS PointCloud2 to PCL PointCloud
  pcl::fromROSMsg(*msg, *latest_cloud_);

  // Update KD-tree for nearest neighbor search
  if (latest_cloud_->points.size() > 0) {
    kdtree_.setInputCloud(latest_cloud_);
    cloud_updated_ = true;
  }
}

void SafetyLimiterNode::timerCallback()
{
  if (!latest_cmd_vel_ || !cloud_updated_) {
    return;
  }

  // Predict robot poses
  std::vector<geometry_msgs::msg::Pose> predicted_poses;
  predictRobotPoses(*latest_cmd_vel_, predicted_poses);

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

  // Publish modified cmd_vel
  cmd_vel_pub_->publish(output_cmd_vel);

  if (velocity_scale < 1.0) {
    RCLCPP_WARN(
      this->get_logger(),
      "Velocity scaled to %.2f due to obstacle detection", velocity_scale);
  }
}

void SafetyLimiterNode::predictRobotPoses(
  const geometry_msgs::msg::Twist & cmd_vel,
  std::vector<geometry_msgs::msg::Pose> & predicted_poses)
{
  predicted_poses.clear();

  // Get current robot pose from TF
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

  // Predict future poses
  double t = 0.0;
  while (t <= prediction_time_) {
    geometry_msgs::msg::Pose predicted_pose;

    // Simple motion model (assuming constant velocity)
    double vx = cmd_vel.linear.x;
    double vy = cmd_vel.linear.y;
    double omega = cmd_vel.angular.z;

    if (std::abs(omega) < 1e-6) {
      // Straight line motion
      predicted_pose.position.x = current_pose.position.x +
        (vx * std::cos(yaw) - vy * std::sin(yaw)) * t;
      predicted_pose.position.y = current_pose.position.y +
        (vx * std::sin(yaw) + vy * std::cos(yaw)) * t;
      predicted_pose.position.z = current_pose.position.z;

      tf2::Quaternion pred_q;
      pred_q.setRPY(roll, pitch, yaw);
      predicted_pose.orientation.x = pred_q.x();
      predicted_pose.orientation.y = pred_q.y();
      predicted_pose.orientation.z = pred_q.z();
      predicted_pose.orientation.w = pred_q.w();
    } else {
      // Circular motion
      double radius = vx / omega;
      double angle = omega * t;

      predicted_pose.position.x = current_pose.position.x +
        radius * (std::sin(yaw + angle) - std::sin(yaw));
      predicted_pose.position.y = current_pose.position.y -
        radius * (std::cos(yaw + angle) - std::cos(yaw));
      predicted_pose.position.z = current_pose.position.z;

      tf2::Quaternion pred_q;
      pred_q.setRPY(roll, pitch, yaw + angle);
      predicted_pose.orientation.x = pred_q.x();
      predicted_pose.orientation.y = pred_q.y();
      predicted_pose.orientation.z = pred_q.z();
      predicted_pose.orientation.w = pred_q.w();
    }

    predicted_poses.push_back(predicted_pose);
    t += prediction_step_;
  }
}

double SafetyLimiterNode::checkCollision(
  const std::vector<geometry_msgs::msg::Pose> & predicted_poses,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
{
  if (predicted_poses.empty() || cloud->points.empty()) {
    return 1.0;
  }

  double min_scale = 1.0;

  // Check each predicted pose
  for (const auto & pose : predicted_poses) {
    // Define corners of the footprint rectangle in robot frame
    std::vector<std::pair<double, double>> footprint_corners = {
      {footprint_x_front_, footprint_y_},
      {footprint_x_front_, -footprint_y_},
      {-footprint_x_rear_, footprint_y_},
      {-footprint_x_rear_, -footprint_y_}
    };

    // Extract yaw from quaternion
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Transform footprint corners to map frame
    std::vector<pcl::PointXYZ> transformed_corners;
    for (const auto & corner : footprint_corners) {
      pcl::PointXYZ pt;
      pt.x = pose.position.x + corner.first * std::cos(yaw) - corner.second * std::sin(yaw);
      pt.y = pose.position.y + corner.first * std::sin(yaw) + corner.second * std::cos(yaw);
      pt.z = pose.position.z;
      transformed_corners.push_back(pt);
    }

    // Calculate center and max radius for efficient search
    pcl::PointXYZ center;
    center.x = pose.position.x;
    center.y = pose.position.y;
    center.z = pose.position.z;

    // Search radius: diagonal of footprint + slowdown margin
    double search_radius = std::sqrt(
      std::pow(footprint_x_front_ + footprint_x_rear_, 2) +
      std::pow(2 * footprint_y_, 2)) / 2.0 + slowdown_margin_;

    // Use KD-tree radius search
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    if (kdtree_.radiusSearch(center, search_radius, point_indices, point_distances) > 0) {
      // Check each point in the search radius
      for (size_t i = 0; i < point_indices.size(); ++i) {
        const pcl::PointXYZ & pt = cloud->points[point_indices[i]];

        // Check if point is inside footprint (safety margin)
        if (isPointInFootprint(pt, pose, safety_margin_)) {
          // Collision detected - stop
          return min_velocity_scale_;
        }

        // Check if point is in slowdown zone
        if (isPointInFootprint(pt, pose, safety_margin_ + slowdown_margin_)) {
          // Calculate distance-based scale (0 at footprint, 1 at slowdown margin)
          double distance_to_footprint = 0.0;

          // Simple approximation: distance from robot center
          double dx = pt.x - center.x;
          double dy = pt.y - center.y;
          double dist_from_center = std::sqrt(dx * dx + dy * dy);

          double footprint_size = std::sqrt(
            std::pow(footprint_x_front_, 2) + std::pow(footprint_y_, 2));

          distance_to_footprint = dist_from_center - footprint_size - safety_margin_;

          if (distance_to_footprint < slowdown_margin_) {
            // Linear interpolation: 0 at footprint edge, 1 at slowdown_margin distance
            double scale = distance_to_footprint / slowdown_margin_;
            scale = std::max(min_velocity_scale_, std::min(1.0, scale));
            min_scale = std::min(min_scale, scale);
          }
        }
      }
    }
  }

  return min_scale;
}

bool SafetyLimiterNode::isPointInFootprint(
  const pcl::PointXYZ & point,
  const geometry_msgs::msg::Pose & robot_pose,
  double footprint_margin)
{
  // Extract yaw from quaternion
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

}  // namespace safety_limiter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<safety_limiter::SafetyLimiterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
