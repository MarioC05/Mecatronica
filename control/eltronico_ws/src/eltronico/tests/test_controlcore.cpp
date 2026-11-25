// Pruebas sin ROS: transiciones cinemáticas, fallos y precondiciones de subida.
// Compilar: g++ -std=c++17 -Iinclude tests/test_controlcore.cpp -o /tmp/test_eltronico
#include "ControlCore.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main()
{
  using namespace eltronico;
  constexpr double dt = 0.02;
  ControlCore core;
  Feedback fb;
  fb.joints_valid = true;
  assert(core.set_behavior(Behavior::WALK));
  auto cmd = core.step(dt, fb);
  assert(!cmd.valid);  // Sin odometría no arrancamos inadvertidamente.

  core.set_walk_open_loop_test(true);
  bool hubo_marcha = false;
  for (int i = 0; i < 200; ++i) {
    cmd = core.step(dt, fb);
    assert(cmd.valid);
    for (double a : cmd.target_rad) {
      hubo_marcha |= std::abs(a) > 1e-4;
    }
    // Simulamos seguimiento real de los actuadores para la vigilancia de movimiento.
    fb.joints_rad = cmd.target_rad;
  }
  assert(hubo_marcha);

  core.set_walk_open_loop_test(false);
  assert(core.set_behavior(Behavior::IDLE));
  assert(core.set_behavior(Behavior::CLIMB));
  fb = Feedback{};
  fb.joints_valid = true;
  fb.imu_valid = true;
  fb.pose_valid = true;
  fb.edge_clearance_m = 0.2;
  cmd = core.step(dt, fb);
  assert(cmd.valid);
  assert(core.climb_phase() == ClimbPhase::APPROACH_STEP);

  fb.both_casters_above_step = true;
  for (int i = 0; i < 4; ++i) cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::CHECK_SUPPORT);
  for (int i = 0; i < 15; ++i) cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::CHECK_SUPPORT);  // Sin apoyo real no pasa.

  fb.supports_valid = true;
  fb.caster_left_on_step = true;
  fb.caster_right_on_step = true;
  for (int i = 0; i < 10; ++i) cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::RETRACT_FRONT);
  cmd = core.step(dt, fb);
  assert(cmd.valid);
  fb.joints_rad[FRONT_PITCH] = PI / 2.0;
  for (int i = 0; i < 12; ++i) cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::ADVANCE);

  fb.knee_motor_clearance_valid = true;
  fb.knee_motor_clearance_m = 0.02;
  cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::LIFT_LEGS);

  for (int deg = 10; deg <= 90; deg += 10) {
    fb.joints_rad[HIP_LEFT] = fb.joints_rad[HIP_RIGHT] = -deg * DEG;
    fb.joints_rad[KNEE_LEFT] = fb.joints_rad[KNEE_RIGHT] =
      std::min(135.0, 1.5 * deg) * DEG;
    for (int i = 0; i < 12; ++i) cmd = core.step(dt, fb);
  }
  for (double deg : {150.0, 165.0, 180.0}) {
    fb.joints_rad[HIP_LEFT] = fb.joints_rad[HIP_RIGHT] = -90.0 * DEG;
    fb.joints_rad[KNEE_LEFT] = fb.joints_rad[KNEE_RIGHT] = deg * DEG;
    for (int i = 0; i < 12; ++i) cmd = core.step(dt, fb);
  }
  assert(core.climb_phase() == ClimbPhase::VERIFY_SUPPORT);
  fb.z_m = 0.1;
  for (int i = 0; i < 17; ++i) cmd = core.step(dt, fb);
  assert(core.climb_phase() == ClimbPhase::COMPLETE);
  std::cout << "OK: WALK y las siete fases de CLIMB con feedback simulado.\n";
}
