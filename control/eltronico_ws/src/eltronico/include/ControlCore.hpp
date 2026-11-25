#ifndef ELTRONICO__CONTROL_CORE_HPP_
#define ELTRONICO__CONTROL_CORE_HPP_

// Control cinemático sin dependencias de ROS. Ángulos en radianes.
// La subida usa el orden del Python original, pero exige estados medidos
// y validaciones de apoyo: ningún temporizador sustituye a un sensor.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace eltronico
{

constexpr double PI = 3.14159265358979323846;
constexpr double DEG = PI / 180.0;

enum class Behavior { IDLE, WALK, AVOID_OBSTACLE, CLIMB };
enum class ClimbPhase {
  APPROACH_STEP, CHECK_SUPPORT, RETRACT_FRONT, ADVANCE,
  LIFT_LEGS, VERIFY_SUPPORT, COMPLETE
};

enum JointIndex : std::size_t {
  HIP_LEFT = 0, KNEE_LEFT = 1, HIP_RIGHT = 2, KNEE_RIGHT = 3,
  FRONT_PITCH = 4, JOINT_COUNT = 5
};
using JointArray = std::array<double, JOINT_COUNT>;

inline double clamp(double x, double min, double max)
{
  return std::max(min, std::min(x, max));
}
inline double wrap(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
inline const char * to_string(Behavior value)
{
  switch (value) {
    case Behavior::IDLE: return "IDLE";
    case Behavior::WALK: return "WALK";
    case Behavior::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
    case Behavior::CLIMB: return "CLIMB";
  }
  return "UNKNOWN";
}
inline const char * to_string(ClimbPhase value)
{
  switch (value) {
    case ClimbPhase::APPROACH_STEP: return "APPROACH_STEP";
    case ClimbPhase::CHECK_SUPPORT: return "CHECK_SUPPORT";
    case ClimbPhase::RETRACT_FRONT: return "RETRACT_FRONT";
    case ClimbPhase::ADVANCE: return "ADVANCE";
    case ClimbPhase::LIFT_LEGS: return "LIFT_LEGS";
    case ClimbPhase::VERIFY_SUPPORT: return "VERIFY_SUPPORT";
    case ClimbPhase::COMPLETE: return "COMPLETE";
  }
  return "UNKNOWN";
}
inline bool parse_behavior(const std::string & str, Behavior & result)
{
  if (str == "idle" || str == "IDLE") result = Behavior::IDLE;
  else if (str == "walk" || str == "WALK") result = Behavior::WALK;
  else if (str == "avoid_obstacle" || str == "AVOID_OBSTACLE") result = Behavior::AVOID_OBSTACLE;
  else if (str == "climb" || str == "CLIMB") result = Behavior::CLIMB;
  else return false;
  return true;
}

struct Feedback
{
  bool joints_valid = false;
  bool imu_valid = false;  // En Gazebo se utiliza la orientación de odometría 3D.
  bool pose_valid = false;
  JointArray joints_rad{};
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;
  double yaw_rate_rad_s = 0.0;
  double x_m = 0.0;
  double z_m = 0.0;

  // Datos de la escalera. Contactos de Gazebo, NO estimaciones por tiempo.
  bool supports_valid = false;
  bool caster_left_on_step = false;
  bool caster_right_on_step = false;
  bool both_casters_above_step = false;
  double edge_clearance_m = 1e6;  // Proyección del centro de ruedas a la contrahuella.

  // Proximidad: se activa solo con un medidor válido (o con aproximación
  // geométrica habilitada explícitamente en el nodo de simulación).
  bool knee_motor_clearance_valid = false;
  double knee_motor_clearance_m = 1e6;
};

struct Command
{
  bool valid = false;
  JointArray target_rad{};
  std::string detail;
};

class ControlCore
{
public:
  Behavior behavior() const {return behavior_;}
  ClimbPhase climb_phase() const {return climb_phase_;}
  bool faulted() const {return faulted_;}

  // Solo diagnóstico en Gazebo: marcha sin realimentación de orientación.
  // No habilitar en el hardware físico.
  void set_walk_open_loop_test(bool enabled) {walk_open_loop_test_ = enabled;}
  const std::string & fault_reason() const {return fault_reason_;}

  void set_step_geometry(double edge_x_m, double top_z_m)
  {
    step_edge_x_m_ = edge_x_m;
    step_top_z_m_ = top_z_m;
  }

  bool set_behavior(Behavior next)
  {
    if (next == Behavior::IDLE) {
      behavior_ = Behavior::IDLE;
      faulted_ = false;
      fault_reason_.clear();
      reset_motion();
      return true;
    }
    if (faulted_) return false;
    if (next != behavior_) {
      behavior_ = next;
      reset_motion();
      climb_phase_ = ClimbPhase::APPROACH_STEP;
    }
    return true;
  }

  Command step(double dt, const Feedback & fb)
  {
    Command cmd;
    if (dt <= 0 || !std::isfinite(dt) || dt > 0.2) {
      cmd.detail = "dt no válido o simulación detenida";
      return cmd;
    }
    if (behavior_ == Behavior::AVOID_OBSTACLE) {
      cmd.detail = "AVOID_OBSTACLE reservado para navegación";
      return cmd;
    }
    if (!fb.joints_valid) {
      cmd.detail = "Esperando /joint_states válido";
      return cmd;
    }
    for (double q : fb.joints_rad) {
      if (!std::isfinite(q)) return fail("Estado articular no finito", fb);
    }
    if (behavior_ == Behavior::IDLE) return hold(fb, "IDLE: conservar postura");
    if (!fb.imu_valid || !fb.pose_valid || !finite_pose(fb)) {
      if (behavior_ == Behavior::WALK && walk_open_loop_test_) {
        // Modo diagnóstico: usar postura neutra y desactivar el control de rumbo.
        // CLIMB continúa exigiendo odometría y apoyos medidos.
        Feedback test_fb = fb;
        test_fb.roll_rad = 0.0;
        test_fb.pitch_rad = 0.0;
        test_fb.yaw_rad = 0.0;
        test_fb.yaw_rate_rad_s = 0.0;
        test_fb.edge_clearance_m = 1e6;
        test_fb.x_m = 0.0;
        test_fb.z_m = 0.0;
        return step_walk(dt, test_fb);
      }
      cmd.detail = "Esperando /eltronico/odom válido (orientación y posición)";
      return cmd;
    }
    if (behavior_ == Behavior::WALK) return step_walk(dt, fb);
    return step_climb(dt, fb);
  }

private:
  static bool finite_pose(const Feedback & fb)
  {
    return std::isfinite(fb.roll_rad) && std::isfinite(fb.pitch_rad) &&
           std::isfinite(fb.yaw_rad) && std::isfinite(fb.yaw_rate_rad_s) &&
           std::isfinite(fb.x_m) && std::isfinite(fb.z_m);
  }

  void reset_motion()
  {
    initialized_ = false;
    climb_initialized_ = false;
    gait_cycles_ = 0.0;
    elapsed_s_ = 0.0;
    phase_elapsed_s_ = 0.0;
    dwell_s_ = 0.0;
    waypoint_elapsed_s_ = 0.0;
    waypoint_index_ = 0;
    waypoints_.clear();
    hold_captured_ = false;
    max_measured_joint_excursion_rad_ = 0.0;
    max_requested_joint_excursion_rad_ = 0.0;
  }

  Command hold(const Feedback & fb, const std::string & detail)
  {
    if (!hold_captured_) {
      held_joints_ = fb.joints_rad;
      hold_captured_ = true;
    }
    return Command{true, held_joints_, detail};
  }

  Command fail(const std::string & reason, const Feedback & fb)
  {
    faulted_ = true;
    fault_reason_ = reason;
    behavior_ = Behavior::IDLE;
    hold_captured_ = false;
    return hold(fb, "FALLO: " + reason);
  }

  void begin_motion(const Feedback & fb)
  {
    initialized_ = true;
    heading_reference_ = fb.yaw_rad;
    roll_reference_ = fb.roll_rad;
    pitch_reference_ = fb.pitch_rad;
    walk_start_joints_ = fb.joints_rad;
    previous_command_ = fb.joints_rad;
    elapsed_s_ = 0.0;
    gait_cycles_ = 0.0;
    hold_captured_ = false;
    max_measured_joint_excursion_rad_ = 0.0;
    max_requested_joint_excursion_rad_ = 0.0;
    walk_folded_ = fb.joints_rad[FRONT_PITCH] >= 80.0 * DEG;
  }

  JointArray gait_targets(double cycles, double left_scale, double right_scale,
                          bool paired, double amplitude) const
  {
    JointArray result{};
    const double hip_amp = 12.0 * DEG * amplitude;
    const double knee_amp = 16.0 * DEG * amplitude;
    const double common = 10.0 * DEG * std::min(1.0, (left_scale + right_scale) * 0.5);
    for (int side = 0; side < 2; ++side) {
      double phase = std::fmod(cycles + (paired ? 0.0 : 0.5 * side) + gait_phase_, 1.0);
      if (phase < 0.0) phase += 1.0;
      double hip = 0.0, knee = 0.0;
      if (phase < stance_) {
        hip = hip_amp * std::cos(PI * phase / stance_);
      } else {
        const double u = (phase - stance_) / (1.0 - stance_);
        hip = -hip_amp * std::cos(PI * u);
        knee = knee_amp * std::sin(PI * u);
      }
      const auto hi = side == 0 ? HIP_LEFT : HIP_RIGHT;
      const auto ki = side == 0 ? KNEE_LEFT : KNEE_RIGHT;
      const double scale = side == 0 ? left_scale : right_scale;
      // En modo de patas emparejadas el Python escala también la rodilla derecha.
      result[hi] = hip * scale - (paired ? common * scale / left_scale : common);
      result[ki] = paired ? knee * scale / left_scale : knee;
    }
    return result;
  }

  Command move_toward(double dt, const Feedback & fb, JointArray desired,
                      double speed_rad_s, const std::string & label)
  {
    if (!initialized_) begin_motion(fb);
    Command cmd;
    cmd.valid = true;
    const double max_delta = speed_rad_s * dt;
    for (std::size_t i = 0; i < JOINT_COUNT; ++i) {
      if (i == FRONT_PITCH) desired[i] = clamp(desired[i], 0.0, PI / 2.0);
      const double delta = wrap(desired[i] - previous_command_[i]);
      cmd.target_rad[i] = previous_command_[i] + clamp(delta, -max_delta, max_delta);
      if (i == FRONT_PITCH) cmd.target_rad[i] = clamp(cmd.target_rad[i], 0.0, PI / 2.0);
    }
    previous_command_ = cmd.target_rad;
    cmd.detail = label;
    return cmd;
  }

  Command make_gait(double dt, const Feedback & fb, bool folded)
  {
    if (!initialized_) begin_motion(fb);
    elapsed_s_ += dt;
    const bool slow = fb.edge_clearance_m < brake_distance_m_ && !folded;
    const double period = slow ? stair_gait_period_s_ : gait_period_s_;
    gait_cycles_ += dt / period;
    const double error = wrap(fb.yaw_rad - heading_reference_);
    const double steering = clamp(-yaw_kp_ * error - yaw_kd_ * fb.yaw_rate_rad_s,
                                  -max_steer_, max_steer_);
    const double left = 1.0 + steering;
    const double right = 1.0 - steering;
    JointArray desired = gait_targets(gait_cycles_, left, right, folded,
                                      folded ? 0.8 : 0.6);
    const double t = clamp(elapsed_s_ / 2.0, 0.0, 1.0);
    const double blend = t * t * t * (10.0 - 15.0 * t + 6.0 * t * t);
    for (std::size_t i = 0; i < FRONT_PITCH; ++i) {
      desired[i] = walk_start_joints_[i] + blend * wrap(desired[i] - walk_start_joints_[i]);
    }
    desired[FRONT_PITCH] = folded ? PI / 2.0 : walk_start_joints_[FRONT_PITCH];
    for (std::size_t i = 0; i < FRONT_PITCH; ++i) {
      max_measured_joint_excursion_rad_ = std::max(
        max_measured_joint_excursion_rad_,
        std::abs(wrap(fb.joints_rad[i] - walk_start_joints_[i])));
      max_requested_joint_excursion_rad_ = std::max(
        max_requested_joint_excursion_rad_,
        std::abs(wrap(desired[i] - walk_start_joints_[i])));
    }
    if (elapsed_s_ > 6.0 && max_requested_joint_excursion_rad_ > 3.0 * DEG &&
        max_measured_joint_excursion_rad_ < 1.0 * DEG) {
      return fail("Articulaciones sin movimiento: revisar gz_ros2_control y URDF", fb);
    }
    return move_toward(dt, fb, desired, joint_speed_rad_s_,
                       folded ? "Marcha emparejada con palanca recogida" :
                       (slow ? "Marcha lenta de aproximación" : "Marcha WALK"));
  }

  Command step_walk(double dt, const Feedback & fb)
  {
    if (!initialized_) begin_motion(fb);
    const double tilt = std::max(std::abs(wrap(fb.roll_rad - roll_reference_)),
                                 std::abs(wrap(fb.pitch_rad - pitch_reference_)));
    if (tilt > 12.0 * DEG) return fail("WALK: inclinación relativa >12 grados", fb);
    return make_gait(dt, fb, walk_folded_);
  }

  void change_phase(ClimbPhase next)
  {
    climb_phase_ = next;
    phase_elapsed_s_ = 0.0;
    dwell_s_ = 0.0;
    hold_captured_ = false;
  }

  void create_lift_waypoints(const Feedback & fb)
  {
    waypoints_.clear();
    const JointArray start = fb.joints_rad;
    for (int hip_deg = 10; hip_deg <= 90; hip_deg += 10) {
      const double knee_deg = std::min(135.0, 1.5 * hip_deg);
      JointArray goal = start;
      goal[HIP_LEFT] -= hip_deg * DEG;
      goal[HIP_RIGHT] -= hip_deg * DEG;
      goal[KNEE_LEFT] += knee_deg * DEG;
      goal[KNEE_RIGHT] += knee_deg * DEG;
      goal[FRONT_PITCH] = PI / 2.0;
      waypoints_.push_back(goal);
    }
    for (double knee_deg : {150.0, 165.0, 180.0}) {
      JointArray goal = start;
      goal[HIP_LEFT] -= 90.0 * DEG;
      goal[HIP_RIGHT] -= 90.0 * DEG;
      goal[KNEE_LEFT] += knee_deg * DEG;
      goal[KNEE_RIGHT] += knee_deg * DEG;
      goal[FRONT_PITCH] = PI / 2.0;
      waypoints_.push_back(goal);
    }
    waypoint_index_ = 0;
    waypoint_elapsed_s_ = 0.0;
    dwell_s_ = 0.0;
    previous_command_ = fb.joints_rad;
    initialized_ = true;
  }

  Command step_climb(double dt, const Feedback & fb)
  {
    if (!climb_initialized_) {
      climb_initialized_ = true;
      climb_start_height_m_ = fb.z_m;
      // El controlador de aproximación se inicializa al primer tick.
    }
    if (std::max(std::abs(fb.roll_rad), std::abs(fb.pitch_rad)) > 10.0 * DEG) {
      return fail("CLIMB: inclinación absoluta >10 grados", fb);
    }
    phase_elapsed_s_ += dt;
    switch (climb_phase_) {
      case ClimbPhase::APPROACH_STEP:
        if (phase_elapsed_s_ > 240.0) return fail("No llega a las ruedas del primer peldaño", fb);
        if (fb.both_casters_above_step) {
          dwell_s_ += dt;
          if (dwell_s_ >= 0.04) {
            change_phase(ClimbPhase::CHECK_SUPPORT);
            return hold(fb, "Ruedas sobre la huella: verificar apoyos");
          }
        } else {
          dwell_s_ = 0.0;
        }
        return make_gait(dt, fb, false);

      case ClimbPhase::CHECK_SUPPORT:
        if (phase_elapsed_s_ > 12.0) return fail("Apoyo de las dos ruedas no confirmado", fb);
        if (fb.supports_valid && fb.caster_left_on_step &&
            fb.caster_right_on_step && fb.both_casters_above_step)
        {
          dwell_s_ += dt;
          if (dwell_s_ >= 0.15) {
            change_phase(ClimbPhase::RETRACT_FRONT);
            initialized_ = false;
            return hold(fb, "Dos apoyos confirmados: iniciar repliegue de la palanca");
          }
        } else {
          dwell_s_ = 0.0;
        }
        return hold(fb, "Esperando contactos reales de ambas ruedas con la huella");

      case ClimbPhase::RETRACT_FRONT: {
        if (phase_elapsed_s_ > 12.0) return fail("Palanca frontal: objetivo 90 grados no alcanzado", fb);
        if (std::abs(fb.joints_rad[FRONT_PITCH] - PI / 2.0) < 8.0 * DEG) {
          dwell_s_ += dt;
          if (dwell_s_ >= 0.20 && fb.caster_left_on_step && fb.caster_right_on_step) {
            change_phase(ClimbPhase::ADVANCE);
            initialized_ = false;
            return hold(fb, "Palanca recogida, iniciar avance con patas emparejadas");
          }
        } else {
          dwell_s_ = 0.0;
        }
        JointArray goal = fb.joints_rad;
        goal[FRONT_PITCH] = PI / 2.0; // Python V4: 90° recogida, 0° extendida.
        if (!initialized_) begin_motion(fb);
        // Congelar las patas mientras se repliega la palanca.
        for (std::size_t i = 0; i < FRONT_PITCH; ++i) goal[i] = walk_start_joints_[i];
        return move_toward(dt, fb, goal, joint_speed_rad_s_, "RETRACT_FRONT: palanca hacia 90 grados");
      }

      case ClimbPhase::ADVANCE:
        if (phase_elapsed_s_ > 90.0) return fail("No se ha detectado el motor junto al escalón", fb);
        if (fb.knee_motor_clearance_valid && fb.knee_motor_clearance_m <= motor_trigger_m_) {
          if (fb.knee_motor_clearance_m < -0.010) {
            return fail("Motores de rodilla han sobrepasado la contrahuella", fb);
          }
          if (!(fb.supports_valid && fb.caster_left_on_step && fb.caster_right_on_step)) {
            return fail("Plegado bloqueado: falta apoyo de ambas ruedas", fb);
          }
          create_lift_waypoints(fb);
          change_phase(ClimbPhase::LIFT_LEGS);
          return hold(fb, "Riesgo de choque de motores: iniciar plegado bilateral");
        }
        return make_gait(dt, fb, true);

      case ClimbPhase::LIFT_LEGS: {
        if (waypoint_index_ >= waypoints_.size()) {
          change_phase(ClimbPhase::VERIFY_SUPPORT);
          return hold(fb, "Articulaciones plegadas; verificar altura y apoyos");
        }
        const auto & goal = waypoints_[waypoint_index_];
        double error = 0.0;
        for (std::size_t i = 0; i < FRONT_PITCH; ++i) {
          error = std::max(error, std::abs(wrap(fb.joints_rad[i] - goal[i])));
        }
        if (error < 5.0 * DEG) {
          dwell_s_ += dt;
          if (dwell_s_ > 0.20) {
            ++waypoint_index_;
            waypoint_elapsed_s_ = 0.0;
            dwell_s_ = 0.0;
            if (waypoint_index_ >= waypoints_.size()) {
              change_phase(ClimbPhase::VERIFY_SUPPORT);
              return hold(fb, "Plegado bilateral completado; verificar apoyo");
            }
          }
        } else {
          dwell_s_ = 0.0;
        }
        waypoint_elapsed_s_ += dt;
        // A 4°/s el recorrido total puede durar más de un minuto.
        if (waypoint_elapsed_s_ > 20.0) {
          return fail("Waypoint de plegado no alcanzado (revisar actuadores y contactos)", fb);
        }
        return move_toward(dt, fb, waypoints_[waypoint_index_], 4.0 * DEG,
                           "LIFT_LEGS: waypoint " + std::to_string(waypoint_index_ + 1) +
                           "/" + std::to_string(waypoints_.size()));
      }

      case ClimbPhase::VERIFY_SUPPORT:
        if (phase_elapsed_s_ > 12.0) return fail("No se ha verificado la subida del torso", fb);
        if (fb.supports_valid && fb.caster_left_on_step && fb.caster_right_on_step &&
            fb.z_m >= climb_start_height_m_ + 0.08)
        {
          dwell_s_ += dt;
          if (dwell_s_ >= 0.30) {
            change_phase(ClimbPhase::COMPLETE);
            return hold(fb, "CLIMB completo: peldaño superado; esperando IDLE");
          }
        } else {
          dwell_s_ = 0.0;
        }
        return hold(fb, "Verificando contactos y altura del torso");

      case ClimbPhase::COMPLETE:
        return hold(fb, "Subida de un peldaño completada; cambiar a IDLE");
    }
    return hold(fb, "Estado de subida desconocido");
  }

  Behavior behavior_ = Behavior::IDLE;
  ClimbPhase climb_phase_ = ClimbPhase::APPROACH_STEP;
  bool initialized_ = false;
  bool climb_initialized_ = false;
  bool faulted_ = false;
  bool hold_captured_ = false;
  bool walk_folded_ = false;
  bool walk_open_loop_test_ = false;
  std::string fault_reason_;
  JointArray held_joints_{};
  JointArray previous_command_{};
  JointArray walk_start_joints_{};
  std::vector<JointArray> waypoints_;
  std::size_t waypoint_index_ = 0;
  double heading_reference_ = 0.0;
  double roll_reference_ = 0.0;
  double pitch_reference_ = 0.0;
  double gait_cycles_ = 0.0;
  double elapsed_s_ = 0.0;
  double phase_elapsed_s_ = 0.0;
  double dwell_s_ = 0.0;
  double waypoint_elapsed_s_ = 0.0;
  double climb_start_height_m_ = 0.0;
  double max_measured_joint_excursion_rad_ = 0.0;
  double max_requested_joint_excursion_rad_ = 0.0;
  double step_edge_x_m_ = 1.0;
  double step_top_z_m_ = 0.179;

  static constexpr double stance_ = 0.62;
  static constexpr double gait_phase_ = 0.92;
  static constexpr double gait_period_s_ = 3.0;
  static constexpr double stair_gait_period_s_ = 6.0;
  static constexpr double brake_distance_m_ = 0.06;
  static constexpr double motor_trigger_m_ = 0.030;
  static constexpr double joint_speed_rad_s_ = 30.0 * DEG;
  static constexpr double yaw_kp_ = 5.0;
  static constexpr double yaw_kd_ = 0.5;
  static constexpr double max_steer_ = 0.35;
};

}  // namespace eltronico

#endif  // ELTRONICO__CONTROL_CORE_HPP_
