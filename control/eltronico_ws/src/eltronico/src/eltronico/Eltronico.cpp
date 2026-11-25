#include "Eltronico.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace eltronico
{
namespace
{

constexpr std::array<const char *, JOINT_COUNT> JOINT_NAMES = {{
  "hip_left", "knee_left", "hip_right", "knee_right", "front_pitch"
}};

// En el mundo SDF la escalera comienza en x=1 m.
constexpr std::array<double, 11> STEP_EDGES = {{
  1.000, 1.299, 1.597, 1.894, 2.191, 2.489, 2.787, 3.085, 3.384, 3.682, 3.981
}};
constexpr std::array<double, 11> STEP_TOPS = {{
  0.179, 0.357, 0.535, 0.713, 0.891, 1.068, 1.245, 1.422, 1.599, 1.780, 1.958
}};
constexpr double CASTER_RADIUS = 0.0047625;
constexpr double CONTACT_AGE_S = 0.5;
constexpr double STATE_AGE_S = 0.5;

// Rotación de un vector desde el sistema base al sistema mundo.
std::array<double, 3> rotate(
  const std::array<double, 4> & q, const std::array<double, 3> & v)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double tx = 2.0 * (y * v[2] - z * v[1]);
  const double ty = 2.0 * (z * v[0] - x * v[2]);
  const double tz = 2.0 * (x * v[1] - y * v[0]);
  return {{
    v[0] + w * tx + y * tz - z * ty,
    v[1] + w * ty + z * tx - x * tz,
    v[2] + w * tz + x * ty - y * tx
  }};
}

// Extrae orientación tridimensional; la odometría es ground truth de Gazebo,
// no una IMU física ni una estimación mediante filtro complementario.
std::array<double, 3> to_rpy(const std::array<double, 4> & q)
{
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double roll = std::atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y));
  const double pitch_sin = 2.0 * (w*y - z*x);
  const double pitch = std::asin(clamp(pitch_sin, -1.0, 1.0));
  const double yaw = std::atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
  return {{roll, pitch, yaw}};
}

bool is_stair_contact(const ros_gz_interfaces::msg::Contacts & msg, double top_z)
{
  for (const auto & contact : msg.contacts) {
    const bool stair = contact.collision1.name.find("escaleras") != std::string::npos ||
                       contact.collision2.name.find("escaleras") != std::string::npos;
    if (!stair) continue;
    // Exigir un contacto cerca de la cara superior, no con la contrahuella.
    for (std::size_t i = 0; i < contact.positions.size(); ++i) {
      const double z = contact.positions[i].z;
      const bool vertical = i < contact.normals.size() &&
        std::abs(contact.normals[i].z) > 0.90;
      if (vertical && z >= top_z - 0.004 && z <= top_z + 0.012) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

Eltronico::Eltronico()
: LifecycleNode("eltronico")
{
  declare_parameter<double>("frequency", 50.0);
  declare_parameter<std::string>("behavior", "idle");
  declare_parameter<int>("step_number", 1);
  // Por defecto se exige un sensor de proximidad real, no una distancia aproximada.
  // Activar solo al experimentar en Gazebo, jamás como sustituto en hardware real.
  declare_parameter<bool>("climb_allow_geometric_clearance", false);
  declare_parameter<bool>("walk_open_loop_test", false);
}

Eltronico::CallbackReturn Eltronico::on_configure(const rclcpp_lifecycle::State &)
{
  get_parameter("frequency", frequency_);
  get_parameter("step_number", step_number_);
  get_parameter("climb_allow_geometric_clearance", geometric_clearance_);
  get_parameter("walk_open_loop_test", walk_open_loop_test_);
  core_.set_walk_open_loop_test(walk_open_loop_test_);
  if (frequency_ < 1.0 || frequency_ > 240.0 || step_number_ < 1 || step_number_ > 11) {
    RCLCPP_ERROR(get_logger(), "Frecuencia [1,240] Hz y peldaño [1,11] obligatorios");
    return CallbackReturn::FAILURE;
  }

  core_.set_behavior(Behavior::IDLE);
  core_.set_step_geometry(STEP_EDGES[step_number_-1], STEP_TOPS[step_number_-1]);
  feedback_ = Feedback{};
  have_joints_ = false;
  have_odom_ = false;
  have_left_contact_ = false;
  have_right_contact_ = false;
  have_command_ = false;
  have_pose_quaternion_ = false;
  started_clock_ = false;
  warned_missing_ = false;
  warned_fault_ = false;
  previous_tick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  last_requested_behavior_ = "idle";
  last_reported_detail_.clear();

  command_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "/forward_position_controller/commands", rclcpp::QoS(10).best_effort());
  const auto sensor_qos = rclcpp::SensorDataQoS();
  joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", sensor_qos,
    std::bind(&Eltronico::on_joints, this, std::placeholders::_1));
  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    "/eltronico/odom", sensor_qos,
    std::bind(&Eltronico::on_odom, this, std::placeholders::_1));
  contact_left_subscription_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
    "/eltronico/caster_left/contacts", sensor_qos,
    [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {on_contact(msg, true);});
  contact_right_subscription_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
    "/eltronico/caster_right/contacts", sensor_qos,
    [this](ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg) {on_contact(msg, false);});

  RCLCPP_INFO(get_logger(), "Configurado: %.1f Hz, peldaño %d, x=%.3f m, z=%.3f m",
              frequency_, step_number_, STEP_EDGES[step_number_-1], STEP_TOPS[step_number_-1]);
  if (geometric_clearance_) {
    RCLCPP_WARN(get_logger(),
      "Proximidad estimada por geometría: SOLO PARA SIMULACIÓN; no equivale a detección de colisión");
  }
  return CallbackReturn::SUCCESS;
}

Eltronico::CallbackReturn Eltronico::on_activate(const rclcpp_lifecycle::State &)
{
  set_parameter(rclcpp::Parameter("behavior", "idle"));
  last_requested_behavior_ = "idle";
  core_.set_behavior(Behavior::IDLE);
  previous_tick_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / frequency_));
  timer_ = create_wall_timer(period, std::bind(&Eltronico::timer_callback, this));
  RCLCPP_INFO(get_logger(), "ACTIVE en IDLE. Para mover: ros2 param set /eltronico behavior walk");
  return CallbackReturn::SUCCESS;
}

void Eltronico::publish_command(const JointArray & targets)
{
  if (!command_publisher_) return;
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(targets.begin(), targets.end());
  command_publisher_->publish(msg);
  last_command_ = targets;
  have_command_ = true;
}

void Eltronico::publish_hold()
{
  if (feedback_.joints_valid) publish_command(feedback_.joints_rad);
  else if (have_command_) publish_command(last_command_);
}

Eltronico::CallbackReturn Eltronico::on_deactivate(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_hold();
  core_.set_behavior(Behavior::IDLE);
  set_parameter(rclcpp::Parameter("behavior", "idle"));
  last_requested_behavior_ = "idle";
  RCLCPP_INFO(get_logger(), "INACTIVE: última consigna de mantenimiento de postura enviada");
  return CallbackReturn::SUCCESS;
}

Eltronico::CallbackReturn Eltronico::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_hold();
  core_.set_behavior(Behavior::IDLE);
  joint_subscription_.reset();
  odom_subscription_.reset();
  contact_left_subscription_.reset();
  contact_right_subscription_.reset();
  command_publisher_.reset();
  feedback_ = Feedback{};
  return CallbackReturn::SUCCESS;
}

Eltronico::CallbackReturn Eltronico::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_hold();
  core_.set_behavior(Behavior::IDLE);
  return CallbackReturn::SUCCESS;
}

Eltronico::CallbackReturn Eltronico::on_error(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  publish_hold();
  core_.set_behavior(Behavior::IDLE);
  RCLCPP_ERROR(get_logger(), "Error de lifecycle: comando de retención enviado");
  return CallbackReturn::SUCCESS;
}

void Eltronico::on_joints(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  if (msg->name.size() != msg->position.size()) return;
  JointArray received{};
  std::array<bool, JOINT_COUNT> found{};
  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    for (std::size_t j = 0; j < JOINT_COUNT; ++j) {
      if (msg->name[i] == JOINT_NAMES[j]) {
        received[j] = msg->position[i];
        found[j] = std::isfinite(msg->position[i]);
      }
    }
  }
  if (std::all_of(found.begin(), found.end(), [](bool value) {return value;})) {
    feedback_.joints_rad = received;
    feedback_.joints_valid = true;
    last_joint_time_ = get_clock()->now();
    have_joints_ = true;
    update_geometry();
  }
}

void Eltronico::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w)) return;
  quaternion_xyzw_ = {{q.x, q.y, q.z, q.w}};
  const auto rpy = to_rpy(quaternion_xyzw_);
  feedback_.roll_rad = rpy[0];
  feedback_.pitch_rad = rpy[1];
  feedback_.yaw_rad = rpy[2];
  feedback_.yaw_rate_rad_s = msg->twist.twist.angular.z;
  feedback_.x_m = p.x;
  feedback_.z_m = p.z;
  feedback_.imu_valid = true;
  feedback_.pose_valid = true;
  last_odom_time_ = get_clock()->now();
  have_odom_ = true;
  have_pose_quaternion_ = true;
  update_geometry();
}

void Eltronico::on_contact(
  ros_gz_interfaces::msg::Contacts::ConstSharedPtr msg, bool left)
{
  const bool supported = is_stair_contact(*msg, STEP_TOPS[step_number_-1]);
  if (left) {
    last_left_contact_ = supported;
    last_left_contact_time_ = get_clock()->now();
    have_left_contact_ = true;
  } else {
    last_right_contact_ = supported;
    last_right_contact_time_ = get_clock()->now();
    have_right_contact_ = true;
  }
}

void Eltronico::update_geometry()
{
  if (!have_pose_quaternion_ || !have_joints_) return;
  const double edge = STEP_EDGES[step_number_-1];
  const double top = STEP_TOPS[step_number_-1];
  const auto left = rotate(quaternion_xyzw_, {{0.112, -0.027, 0.19325}});
  const auto right = rotate(quaternion_xyzw_, {{0.112, 0.027, 0.19325}});
  const double left_x = feedback_.x_m + left[0];
  const double right_x = feedback_.x_m + right[0];
  const double left_z = feedback_.z_m + left[2];
  const double right_z = feedback_.z_m + right[2];
  feedback_.both_casters_above_step =
    left_x >= edge + 0.001 && right_x >= edge + 0.001 &&
    left_z >= top + CASTER_RADIUS * 0.5 &&
    right_z >= top + CASTER_RADIUS * 0.5;
  feedback_.edge_clearance_m = edge - std::max(left_x, right_x);

  // Esta es una aproximación geométrica del motor de rodilla. NO incluye
  // contactos, mallas ni el cálculo de distancia real de PyBullet.
  // Solo se usa cuando el operador la habilita para ensayos de simulación.
  feedback_.knee_motor_clearance_valid = geometric_clearance_;
  if (geometric_clearance_) {
    double nearest_x = -1e9;
    for (const auto hip : {HIP_LEFT, HIP_RIGHT}) {
      const double angle = feedback_.joints_rad[hip];
      // Centro aproximado de la caja del motor expresado en la base.
      const double motor_x = -0.005 + 0.0335 * std::cos(angle) + 0.072365 * std::sin(angle);
      const double motor_z = 0.214 + 0.0335 * std::sin(angle) - 0.072365 * std::cos(angle);
      const auto v = rotate(quaternion_xyzw_, {{motor_x, 0.0, motor_z}});
      nearest_x = std::max(nearest_x, feedback_.x_m + v[0]);
    }
    feedback_.knee_motor_clearance_m = edge - nearest_x;
  }
}

void Eltronico::timer_callback()
{
  const auto now = get_clock()->now();
  if (now.nanoseconds() <= 0) {
    if (!started_clock_) {
      RCLCPP_WARN(get_logger(), "Esperando /clock; no se mandan consignas de marcha");
      started_clock_ = true;
    }
    return;
  }
  if (previous_tick_.nanoseconds() <= 0) {
    previous_tick_ = now;
    return;
  }
  const double dt = (now - previous_tick_).seconds();
  previous_tick_ = now;
  if (dt <= 0.0 || dt > 0.2) return;

  bool next_walk_open_loop = walk_open_loop_test_;
  get_parameter("walk_open_loop_test", next_walk_open_loop);
  if (next_walk_open_loop != walk_open_loop_test_) {
    walk_open_loop_test_ = next_walk_open_loop;
    core_.set_walk_open_loop_test(walk_open_loop_test_);
    RCLCPP_WARN(get_logger(), "WALK sin odometría: %s (DIAGNÓSTICO SOLO EN GAZEBO)",
                walk_open_loop_test_ ? "HABILITADO" : "DESHABILITADO");
  }

  bool next_geometry_flag = geometric_clearance_;
  get_parameter("climb_allow_geometric_clearance", next_geometry_flag);
  if (next_geometry_flag != geometric_clearance_) {
    geometric_clearance_ = next_geometry_flag;
    update_geometry();
    RCLCPP_WARN(get_logger(), "Estimación de proximidad por geometría %s (SOLO simulación)",
                geometric_clearance_ ? "habilitada" : "deshabilitada");
  }
  const bool joints_recent = have_joints_ && (now - last_joint_time_).seconds() < STATE_AGE_S;
  const bool odom_recent = have_odom_ && (now - last_odom_time_).seconds() < STATE_AGE_S;
  feedback_.joints_valid = joints_recent;
  feedback_.imu_valid = odom_recent;
  feedback_.pose_valid = odom_recent;
  feedback_.supports_valid = have_left_contact_ && have_right_contact_ &&
    (now - last_left_contact_time_).seconds() < CONTACT_AGE_S &&
    (now - last_right_contact_time_).seconds() < CONTACT_AGE_S;
  feedback_.caster_left_on_step = feedback_.supports_valid && last_left_contact_;
  feedback_.caster_right_on_step = feedback_.supports_valid && last_right_contact_;

  std::string requested;
  get_parameter("behavior", requested);
  if (requested != last_requested_behavior_) {
    last_requested_behavior_ = requested;
    Behavior next;
    if (!parse_behavior(requested, next)) {
      RCLCPP_ERROR(get_logger(), "Modo desconocido: %s", requested.c_str());
      publish_hold();
      core_.set_behavior(Behavior::IDLE);
    } else if (!core_.set_behavior(next)) {
      RCLCPP_ERROR(get_logger(), "Fallo retenido: cambiar behavior a idle para resetear: %s",
                   core_.fault_reason().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Comportamiento: %s", to_string(core_.behavior()));
      last_reported_detail_.clear();
      warned_missing_ = false;
      warned_fault_ = false;
    }
  }

  const Command cmd = core_.step(dt, feedback_);
  if (cmd.valid) publish_command(cmd.target_rad);
  else if (core_.behavior() != Behavior::IDLE) publish_hold();
  if (!cmd.valid && core_.behavior() != Behavior::IDLE && !warned_missing_) {
    RCLCPP_WARN(get_logger(), "Sin movimiento: %s", cmd.detail.c_str());
    warned_missing_ = true;
  }
  if (core_.faulted() && !warned_fault_) {
    RCLCPP_ERROR(get_logger(), "PARADA: %s. Para rearmar: behavior idle",
                 core_.fault_reason().c_str());
    warned_fault_ = true;
  }
  if (cmd.valid && cmd.detail != last_reported_detail_) {
    last_reported_detail_ = cmd.detail;
    RCLCPP_INFO(get_logger(), "%s", cmd.detail.c_str());
  }
}

}  // namespace eltronico
