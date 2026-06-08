#include "mecanum_drive/mecanum_drive_node.hpp"

#include <algorithm>   // std::max (initializer-list overload)
#include <cmath>
#include <chrono>

namespace mecanum_drive
{

MecanumDriveNode::MecanumDriveNode(const rclcpp::NodeOptions & options)
: Node("mecanum_drive_node", options)
{
  // ── Parameters ──────────────────────────────────────────────────────────
  wheel_radius_  = declare_parameter("wheel_radius",  0.050);
  wheel_x_off_   = declare_parameter("wheel_x_off",   0.112);
  wheel_y_off_   = declare_parameter("wheel_y_off",   0.195);
  base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));
  odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
  max_wheel_vel_ = declare_parameter("max_wheel_velocity", 50.0);
  cmd_timeout_s_ = declare_parameter("cmd_timeout_sec", 0.5);
  publish_odom_  = declare_parameter("publish_odom", true);
  publish_tf_    = declare_parameter("publish_tf",   true);

  lx_plus_ly_ = wheel_x_off_ + wheel_y_off_;

  RCLCPP_INFO(get_logger(),
    "MecanumDriveNode: r=%.3f lx=%.3f ly=%.3f lx+ly=%.3f",
    wheel_radius_, wheel_x_off_, wheel_y_off_, lx_plus_ly_);

  //Pubs
  wheel_vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/wheel_velocity_controller/commands",
    rclcpp::SystemDefaultsQoS());

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SystemDefaultsQoS());

  //Subs
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumDriveNode::cmdVelCallback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SystemDefaultsQoS(),
    std::bind(&MecanumDriveNode::jointStateCallback, this, std::placeholders::_1));

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  odom_x_   = 0.0;
  odom_y_   = 0.0;
  odom_yaw_ = 0.0;
  last_odom_time_ = now();

  auto period = std::chrono::duration<double>(0.05);  // 20 Hz
  watchdog_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&MecanumDriveNode::watchdogCallback, this));

  last_cmd_time_ = now();
  cmd_active_    = false;
}

void MecanumDriveNode::cmdVelCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_time_ = now();
  cmd_active_    = true;
  publishWheelVelocities(msg->linear.x, msg->linear.y, msg->angular.z);
}

void MecanumDriveNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  double v_fl = 0.0, v_fr = 0.0, v_rl = 0.0, v_rr = 0.0;
  bool fl = false, fr = false, rl = false, rr = false;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->velocity.size() <= i) break;
    const auto & n = msg->name[i];
    const double v = msg->velocity[i];
    if      (n == "front_left_wheel_joint")  { v_fl = v; fl = true; }
    else if (n == "front_right_wheel_joint") { v_fr = v; fr = true; }
    else if (n == "rear_left_wheel_joint")   { v_rl = v; rl = true; }
    else if (n == "rear_right_wheel_joint")  { v_rr = v; rr = true; }
  }

  if (!(fl && fr && rl && rr)) return;

  const double r = wheel_radius_;
  const double lxly = lx_plus_ly_;
  double vx  = r * ( v_fl + v_fr + v_rl + v_rr) / 4.0;
  double vy  = r * (-v_fl + v_fr + v_rl - v_rr) / 4.0;
  double wz  = r * (-v_fl + v_fr - v_rl + v_rr) / (4.0 * lxly);

  auto t_now = now();
  double dt = (t_now - last_odom_time_).seconds();
  last_odom_time_ = t_now;

  if (dt > 0.0 && dt < 0.5) {
    double delta_x   = (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
    double delta_y   = (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;
    double delta_yaw = wz * dt;
    odom_x_   += delta_x;
    odom_y_   += delta_y;
    odom_yaw_ += delta_yaw;
  }

  if (publish_odom_) {
    publishOdometry(vx, vy, wz, t_now);
  }
}

void MecanumDriveNode::watchdogCallback()
{
  if (!cmd_active_) return;
  double age = (now() - last_cmd_time_).seconds();
  if (age > cmd_timeout_s_) {
    publishWheelVelocities(0.0, 0.0, 0.0);
    cmd_active_ = false;
    RCLCPP_DEBUG(get_logger(), "cmd_vel timeout — wheels stopped");
  }
}

void MecanumDriveNode::publishWheelVelocities(double vx, double vy, double wz)
{
  const double r    = wheel_radius_;
  const double lxly = lx_plus_ly_;

  //mecanum wheel formula
  double v_fl = (vx - vy - lxly * wz) / r;
  double v_fr = (vx + vy + lxly * wz) / r;
  double v_rl = (vx + vy - lxly * wz) / r;
  double v_rr = (vx - vy + lxly * wz) / r;

  double max_v = std::max({std::abs(v_fl), std::abs(v_fr),
                           std::abs(v_rl), std::abs(v_rr)});
  if (max_v > max_wheel_vel_) {
    double scale = max_wheel_vel_ / max_v;
    v_fl *= scale; v_fr *= scale;
    v_rl *= scale; v_rr *= scale;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {v_fl, v_fr, v_rl, v_rr};
  wheel_vel_pub_->publish(msg);
}

void MecanumDriveNode::publishOdometry(
  double vx, double vy, double wz, const rclcpp::Time & stamp)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp    = stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id  = base_frame_id_;

  odom.pose.pose.position.x  = odom_x_;
  odom.pose.pose.position.y  = odom_y_;
  odom.pose.pose.position.z  = 0.0;
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist.linear.x  = vx;
  odom.twist.twist.linear.y  = vy;
  odom.twist.twist.angular.z = wz;

  odom.pose.covariance[0]  = 0.001;
  odom.pose.covariance[7]  = 0.001;
  odom.pose.covariance[35] = 0.010;
  odom.twist.covariance[0]  = 0.001;
  odom.twist.covariance[7]  = 0.001;
  odom.twist.covariance[35] = 0.010;

  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = stamp;
    tf.header.frame_id = odom_frame_id_;
    tf.child_frame_id  = base_frame_id_;
    tf.transform.translation.x = odom_x_;
    tf.transform.translation.y = odom_y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation      = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(tf);
  }
}

} 

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mecanum_drive::MecanumDriveNode>());
  rclcpp::shutdown();
  return 0;
}
