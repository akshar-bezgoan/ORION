#pragma once
#include <string>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mecanum_drive
{

class MecanumDriveNode : public rclcpp::Node
{
public:
  explicit MecanumDriveNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void watchdogCallback();

  // Helpers
  void publishWheelVelocities(double vx, double vy, double wz);
  void publishOdometry(double vx, double vy, double wz,
                       const rclcpp::Time & stamp);

  // Parameters
  double wheel_radius_;
  double wheel_x_off_;
  double wheel_y_off_;
  double lx_plus_ly_;
  double max_wheel_vel_;
  double cmd_timeout_s_;
  bool   publish_odom_;
  bool   publish_tf_;
  std::string base_frame_id_;
  std::string odom_frame_id_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          odom_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   joint_state_sub_;

  // Transform
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Odometry state
  double odom_x_;
  double odom_y_;
  double odom_yaw_;
  rclcpp::Time last_odom_time_;

  // Watchdog
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_time_;
  bool cmd_active_;
};

}
