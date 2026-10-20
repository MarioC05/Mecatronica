#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import pybullet as p
import pybullet_data

ROOT = Path(__file__).resolve().parent
DT = 1.0 / 240.0
OUTPUTS = ("hip_left", "knee_left", "hip_right", "knee_right", "front_pitch")
MOTOR_OUTPUTS = OUTPUTS[:4]
OUTPUT_HOLD_EFFORT = {"hip_left": 0.178, "hip_right": 0.178,
                      "knee_left": 0.178, "knee_right": 0.178,
                      "front_pitch": 0.18}
CONFIG = {
    "front_clearance_m": 0.20,
    "gravity_m_s2": 9.81,
    "position_gain": 0.25,
    "velocity_gain": 0.08,
    "solver_iterations": 150,
    "time_step_s": DT,
    "initial_joints_deg": {"hip_left": 0.0, "knee_left": 0.0,
                            "hip_right": 0.0, "knee_right": 0.0,
                            "front_pitch": 90.0},
}
STAIR_DATA = (
    (0.299, 0.179, 1.950, 0.010), (0.298, 0.178, 1.950, 0.010),
    (0.297, 0.178, 1.950, 0.010), (0.297, 0.178, 1.950, 0.010),
    (0.298, 0.178, 1.950, 0.010), (0.298, 0.177, 1.950, 0.010),
    (0.298, 0.177, 1.950, 0.010), (0.299, 0.177, 1.950, 0.010),
    (0.298, 0.177, 1.950, 0.010), (0.299, 0.181, 1.950, 0.010),
    (0.299, 0.178, 1.950, 0.010),
)


def read_stair_profile() -> list[dict]:
    rows: list[dict] = []
    previous_x = previous_z = 0.0
    for number, (tread, rise, width, chamfer) in enumerate(STAIR_DATA, 1):
        x_acc, z_acc = previous_x + tread, previous_z + rise
        if chamfer <= 0 or chamfer >= min(tread, rise):
            raise ValueError(f"Chaflán inválido en peldaño {number}")
        rows.append({"number": number, "tread": tread, "rise": rise,
                     "width": width, "chamfer": chamfer, "x0": previous_x,
                     "x1": x_acc, "z0": previous_z, "z1": z_acc})
        previous_x, previous_z = x_acc, z_acc
    if not rows:
        raise ValueError("No hay peldaños configurados")

    return rows


def load_stairs(client: int, rows: list[dict], friction: float):
    width = rows[0]["width"]
    bodies = []

    def add_box(half_extents, center):
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents,
                                           physicsClientId=client)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                     rgbaColor=(0.66, 0.69, 0.75, 1.0),
                                     physicsClientId=client)
        body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                                 baseVisualShapeIndex=visual, basePosition=center,
                                 physicsClientId=client)
        p.changeDynamics(body, -1, lateralFriction=friction, restitution=0.0,
                         physicsClientId=client)
        bodies.append(body)

    for row in rows:
        x0, x1 = row["x0"], row["x1"]
        top, c = row["z1"], row["chamfer"]
        first_run = x1 - x0 - c
        if first_run <= 0:
            raise ValueError(f"La huella del peldaño {row['number']} no deja espacio al chaflán")
        add_box((c / 2, width / 2, (top - c) / 2),
                (x0 + c / 2, 0, (top - c) / 2))
        add_box((first_run / 2, width / 2, top / 2),
                (x0 + c + first_run / 2, 0, top / 2))

        verts = [(0, -width / 2, 0), (c, -width / 2, 0),
                 (c, -width / 2, c), (0, width / 2, 0),
                 (c, width / 2, 0), (c, width / 2, c)]
        tris = [0, 1, 2, 5, 4, 3,
                0, 3, 4, 0, 4, 1,
                1, 4, 5, 1, 5, 2,
                2, 5, 3, 2, 3, 0]
        collision = p.createCollisionShape(p.GEOM_MESH, vertices=verts,
                                           physicsClientId=client)
        visual = p.createVisualShape(p.GEOM_MESH, vertices=verts, indices=tris,
                                     rgbaColor=(0.66, 0.69, 0.75, 1.0),
                                     physicsClientId=client)
        wedge = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=collision,
                                  baseVisualShapeIndex=visual,
                                  basePosition=(x0, 0, top - c),
                                  physicsClientId=client)
        p.changeDynamics(wedge, -1, lateralFriction=friction, restitution=0.0,
                         physicsClientId=client)
        bodies.append(wedge)
    return bodies


def load_robot(client: int, urdf_path: Path, config: dict, initial_yaw_deg: float):
    flags = p.URDF_USE_SELF_COLLISION | p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    yaw_q = p.getQuaternionFromEuler((0.0, 0.0, math.radians(initial_yaw_deg)))
    robot = p.loadURDF(str(urdf_path), basePosition=(0, 0, 0),
                       baseOrientation=yaw_q, useFixedBase=False, flags=flags,
                       physicsClientId=client)
    joints: dict[str, int] = {}
    links = {-1: "base"}
    for idx in range(p.getNumJoints(robot, physicsClientId=client)):
        info = p.getJointInfo(robot, idx, physicsClientId=client)
        joints[info[1].decode()] = idx
        links[idx] = info[12].decode()

    pose = CONFIG["initial_joints_deg"]
    for name, degrees in pose.items():
        if name not in joints:
            raise ValueError(f"El URDF seleccionado no tiene la articulación {name!r}")
        p.resetJointState(robot, joints[name], math.radians(float(degrees)), 0.0,
                          physicsClientId=client)

    max_x = p.getAABB(robot, -1, physicsClientId=client)[1][0]
    for idx in range(p.getNumJoints(robot, physicsClientId=client)):
        max_x = max(max_x, p.getAABB(robot, idx, physicsClientId=client)[1][0])
    clearance = CONFIG["front_clearance_m"]
    base_x = -clearance - max_x
    p.resetBasePositionAndOrientation(robot, (base_x, 0, 0), yaw_q,
                                      physicsClientId=client)

    for idx in range(-1, p.getNumJoints(robot, physicsClientId=client)):
        p.changeDynamics(robot, idx, lateralFriction=0.65, restitution=0.0,
                         linearDamping=0.06, angularDamping=0.10,
                         spinningFriction=0.001, rollingFriction=0.001,
                         physicsClientId=client)

    for name in MOTOR_OUTPUTS:
        motor, output = joints["motor_" + name], joints[name]
        for idx in (motor, output):
            p.setJointMotorControl2(robot, idx, p.VELOCITY_CONTROL, targetVelocity=0,
                                    force=0, physicsClientId=client)
        gear = p.createConstraint(robot, motor, robot, output, p.JOINT_GEAR,
                                  jointAxis=(0, 1, 0), parentFramePosition=(0, 0, 0),
                                  childFramePosition=(0, 0, 0), physicsClientId=client)
        p.changeConstraint(gear, gearRatio=3.0, maxForce=1000, erp=0.2,
                           physicsClientId=client)
    for name in ("caster_left_roll", "caster_right_roll"):
        if name in joints:
            p.setJointMotorControl2(robot, joints[name], p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=0, physicsClientId=client)

    def hold():
        for name in OUTPUTS:
            current = p.getJointState(robot, joints[name], physicsClientId=client)[0]
            effort = OUTPUT_HOLD_EFFORT[name]
            p.setJointMotorControl2(robot, joints[name], p.POSITION_CONTROL,
                                    targetPosition=current, targetVelocity=0,
                                    force=effort,
                                    positionGain=CONFIG["position_gain"],
                                    velocityGain=CONFIG["velocity_gain"],
                                    maxVelocity=math.radians(12),
                                    physicsClientId=client)
    hold()
    return robot, joints, links, base_x, max_x, hold


def collect_forces(client: int, robot: int, joints: dict[str, int]) -> dict:
    contacts = p.getContactPoints(bodyA=robot, physicsClientId=client)
    normal = sum(float(c[9]) for c in contacts)
    lateral = sum(math.hypot(float(c[10]), float(c[12])) for c in contacts)
    joint_forces = {}
    joint_reactions = {}
    for name, index in joints.items():
        state = p.getJointState(robot, index, physicsClientId=client)
        joint_reactions[name] = [round(float(v), 6) for v in state[2]]
        joint_forces[name] = round(float(state[3]), 6)
    return {
        "contact_count": len(contacts),
        "contact_normal_force_N": round(normal, 6),
        "contact_lateral_force_N": round(lateral, 6),
        "joint_motor_torque_Nm": joint_forces,
        "joint_reaction_wrench": joint_reactions,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, help="cambia el modelo articulado sin editar el script")
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--direct", action="store_true", help="ejecuta 10 s sin GUI y muestra el desplazamiento")
    args = parser.parse_args()

    urdf_path = (args.urdf or (ROOT / "eltronico.urdf")).expanduser().resolve()
    if not urdf_path.is_file():
        parser.error(f"No existe el URDF requerido: {urdf_path}")
    rows = read_stair_profile()
    if any(abs(row["width"] - rows[0]["width"]) > 1e-9 for row in rows):
        parser.error("Este modelo de escalera requiere ancho constante en el CSV")

    client = p.connect(p.DIRECT if args.direct else p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setTimeStep(CONFIG["time_step_s"], physicsClientId=client)
    p.setGravity(0, 0, -CONFIG["gravity_m_s2"], physicsClientId=client)
    p.setPhysicsEngineParameter(numSolverIterations=CONFIG["solver_iterations"],
                                erp=0.2, contactERP=0.15, physicsClientId=client)
    plane_col = p.createCollisionShape(p.GEOM_PLANE, planeNormal=(0, 0, 1),
                                       physicsClientId=client)
    ground = p.createMultiBody(0, plane_col, physicsClientId=client)
    p.changeDynamics(ground, -1, lateralFriction=0.8, restitution=0.0,
                     physicsClientId=client)
    stairs = load_stairs(client, rows, friction=0.8)
    robot, joints, links, base_x, nose_rel_x, hold = load_robot(client, urdf_path,
                                                                 CONFIG, args.yaw_deg)

    if not args.direct:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=client)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=client)
        target = (1.05, 0, 0.75)
        p.resetDebugVisualizerCamera(cameraDistance=4.3, cameraYaw=45,
                                     cameraPitch=-19, cameraTargetPosition=target,
                                     physicsClientId=client)
        p.addUserDebugText("ESTÁTICO · gravedad + contactos + motores de sujeción · cámara manual",
                           (0, 0, 2.20), textColorRGB=(0.05, 0.12, 0.18),
                           textSize=1.2, lifeTime=0, physicsClientId=client)

    end_x = rows[-1]["x1"]
    end_z = rows[-1]["z1"]
    print(f"Robot: {urdf_path}")
    print("Referencia CAD editable: ../Eltronico.FCStd")
    print(f"Escalera: {len(rows)} peldaños; huellas acumuladas {end_x:.3f} m; "
          f"altura {end_z:.3f} m; ancho {rows[0]['width']:.3f} m; "
          f"chaflán {rows[0]['chamfer'] * 1000:g} mm")
    print(f"Separación nariz del perro → primer peldaño: {abs((base_x + nose_rel_x)):.3f} m "
          "(medida desde AABB de la geometría cargada)")
    print("Dinámica activada: gravedad, colisiones, contacto suelo/escalera y servos de postura. "
          "Sin comandos de movimiento; la cámara queda libre.")

    if args.direct:
        initial_pos, initial_quat = p.getBasePositionAndOrientation(robot, physicsClientId=client)
        max_delta = 0.0
        for _ in range(round(10.0 / CONFIG["time_step_s"])):
            hold()
            p.stepSimulation(physicsClientId=client)
            pos, quat = p.getBasePositionAndOrientation(robot, physicsClientId=client)
            max_delta = max(max_delta, math.dist(initial_pos, pos))
        final_pos, final_quat = p.getBasePositionAndOrientation(robot, physicsClientId=client)
        forces = collect_forces(client, robot, joints)
        print(json.dumps({"base_start_m": initial_pos, "base_end_m": final_pos,
                          "base_displacement_m": math.dist(initial_pos, final_pos),
                          "max_base_displacement_m": max_delta,
                          "rpy_start_deg": [math.degrees(x) for x in p.getEulerFromQuaternion(initial_quat)],
                          "rpy_end_deg": [math.degrees(x) for x in p.getEulerFromQuaternion(final_quat)],
                          "forces_after_10s": forces},
                         indent=2))
        p.disconnect(physicsClientId=client)
        return

    try:
        report_at = 0.0
        while p.isConnected(client):
            hold()
            p.stepSimulation(physicsClientId=client)
            time.sleep(CONFIG["time_step_s"])
            now = time.monotonic()
            if now - report_at > 5:
                pos, quat = p.getBasePositionAndOrientation(robot, physicsClientId=client)
                rpy = [round(math.degrees(x), 2) for x in p.getEulerFromQuaternion(quat)]
                force_data = collect_forces(client, robot, joints)
                print(f"Estático: base=({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:.3f}) m; "
                      f"roll/pitch/yaw={rpy}°; contactos={force_data['contact_count']}; "
                      f"Fnormal={force_data['contact_normal_force_N']:.3f} N; "
                      f"Flat={force_data['contact_lateral_force_N']:.3f} N", flush=True)
                report_at = now
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(client):
            p.disconnect(physicsClientId=client)


if __name__ == "__main__":
    main()
