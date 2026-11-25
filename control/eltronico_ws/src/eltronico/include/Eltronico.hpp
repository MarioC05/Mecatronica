#ifndef ELTRONICO__ELTRONICO_HPP_
#define ELTRONICO__ELTRONICO_HPP_

#include <array>
#include <string>

#include "ControlCore.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace eltronico
{

class Eltronico : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit Eltronico();
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State &) override;

private:
  void on_joints(sensor_msgs::msg::JointState::ConstSharedPtr message);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_contact(ros_gz_interfaces::msg::Contacts::ConstSharedPtr message, bool left);
  void update_geometry();
  void timer_callback();
  void publish_command(const JointArray & target);
  void publish_hold();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_left_subscription_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr contact_right_subscription_;

  ControlCore core_;
  Feedback feedback_;
  double frequency_ = 50.0;
  int step_number_ = 1;
  bool geometric_clearance_ = false;
  bool walk_open_loop_test_ = false;
  bool have_pose_quaternion_ = false;
  bool started_clock_ = false;
  bool warned_missing_ = false;
  bool warned_fault_ = false;
  bool have_joints_ = false;
  bool have_odom_ = false;
  bool have_left_contact_ = false;
  bool have_right_contact_ = false;
  bool last_left_contact_ = false;
  bool last_right_contact_ = false;
  bool have_command_ = false;
  JointArray last_command_{};
  std::array<double, 4> quaternion_xyzw_{{0.0, 0.0, 0.0, 1.0}};
  rclcpp::Time previous_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_joint_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_left_contact_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_contact_time_{0, 0, RCL_ROS_TIME};
  std::string last_requested_behavior_ = "idle";
  std::string last_reported_detail_;
};

}  // namespace eltronico

#endif  // ELTRONICO__ELTRONICO_HPP_
