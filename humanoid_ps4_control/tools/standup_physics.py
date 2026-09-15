"""Uncalibrated, free-base stand-up experiment. Never imports a hardware backend."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from src.config import DIR, PWM_PER_DEG, ROBOT, STANDING, STAND_ANG


def joint_targets(pose: dict[int, int]) -> dict[int, float]:
    delta = {sid: (pose[sid] - STANDING[sid]) / DIR.get(sid, 1) / PWM_PER_DEG for sid in STANDING}
    result = {}
    for side, ids in [('L', (12, 13, 14, 15, 16)), ('R', (21, 20, 19, 18, 17))]:
        hr, hp, knee, ankle, ar = ids
        result.update({hr: STAND_ANG[side + '_hip_abduct'] + delta[hr],
                       hp: STAND_ANG[side + '_hip_pitch'] + delta[hp],
                       knee: STAND_ANG[side + '_knee'] + delta[knee],
                       ankle: STAND_ANG[side + '_ankle'] + delta[ankle], ar: delta[ar]})
    for sid in (9, 10, 11, 22, 23, 24, 25):
        result[sid] = delta[sid]
    return {sid: math.radians(v) for sid, v in result.items()}


def build_model(torque_nm: float = 2.0, friction: float = 0.8, hip_spacing_mm: float = 90.0,
                upper_arm_mm: float = 61.0, forearm_mm: float = 66.0):
    """SI units; axes match the viewer: x lateral, y up, z forward."""
    xml = ET.Element('mujoco', model='H17 uncalibrated standup')
    ET.SubElement(xml, 'compiler', angle='radian')
    ET.SubElement(xml, 'option', timestep='0.002', gravity='0 -9.81 0', integrator='implicitfast', iterations='100')
    default = ET.SubElement(xml, 'default')
    ET.SubElement(default, 'joint', damping='0.03', armature='0.0001')
    ET.SubElement(default, 'geom', type='box', friction=f'{friction} 0.01 0.001',
                  condim='4', solref='0.005 1', solimp='0.95 0.99 0.001')
    world = ET.SubElement(xml, 'worldbody')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='2 2 0.1',
                  quat='0.707106781 -0.707106781 0 0', rgba='0.15 0.19 0.17 1')
    root = ET.SubElement(world, 'body', name='pelvis')
    ET.SubElement(root, 'freejoint', name='free_base')
    actuator = ET.SubElement(xml, 'actuator')
    colors = {'metal': '0.50 0.56 0.53 1', 'servo': '0.18 0.23 0.20 1',
              'L': '0.49 0.79 0.60 1', 'R': '0.88 0.69 0.40 1', 'sole': '0.09 0.12 0.10 1'}

    def box(parent, name, size_mm, pos_mm, mass, color='metal'):
        return ET.SubElement(parent, 'geom', name=name,
                             size=' '.join(str(v / 2000) for v in size_mm),
                             pos=' '.join(str(v / 1000) for v in pos_mm),
                             mass=str(mass), rgba=colors[color])

    def joint(parent, sid, pos_mm, axis):
        body = ET.SubElement(parent, 'body', name=f'servo_{sid}',
                             pos=' '.join(str(v / 1000) for v in pos_mm))
        # Limits cover the command range; actual mechanical stops still need measurement.
        bounds = sorted([joint_targets({**STANDING, sid: pwm})[sid] for pwm in (500, 2500)])
        ET.SubElement(body, 'joint', name=f'j{sid}', type='hinge', axis=axis,
                      range=f'{bounds[0]} {bounds[1]}')
        box(body, f'case_{sid}', (21, 18, 23), (0, 0, 0), 0.045, 'servo')
        ET.SubElement(actuator, 'position', name=f'a{sid}', joint=f'j{sid}', kp='12', kv='0.3',
                      forcerange=f'{-torque_nm} {torque_nm}', ctrlrange=f'{bounds[0]} {bounds[1]}')
        return body

    box(root, 'torso', (95, 82, 40), (0, 56, 0), 0.65)
    box(root, 'pelvis_plate', (hip_spacing_mm + 24, 16, 35), (0, 8, 0), 0.10)
    for side, s, ids in [('L', -1, (12, 13, 14, 15, 16)), ('R', 1, (21, 20, 19, 18, 17))]:
        hr, hp, k, a, ar = ids
        hiproll = joint(root, hr, (s * hip_spacing_mm / 2, 0, 0), f'0 0 {s}')
        hip = joint(hiproll, hp, (0, 0, 0), '-1 0 0')
        box(hip, f'{side}_thigh', (19, ROBOT['upper_leg'] - 18, 18), (0, -ROBOT['upper_leg'] / 2, 0), 0.06)
        knee = joint(hip, k, (0, -ROBOT['upper_leg'], 0), '1 0 0')
        box(knee, f'{side}_shin', (18, ROBOT['lower_leg'] - 18, 16), (0, -ROBOT['lower_leg'] / 2, 0), 0.05)
        ankle = joint(knee, a, (0, -ROBOT['lower_leg'], 0), '-1 0 0')
        foot = joint(ankle, ar, (0, 0, 0), f'0 0 {-s}')
        box(foot, f'{side}_foot', (43, 7, 77), (0, -16, 15), 0.03, side)
        box(foot, f'{side}_sole', (43, 3, 77), (0, -21, 15), 0.02, 'sole')
    for side, s, ids in [('L', -1, (11, 10, 9)), ('R', 1, (22, 23, 24))]:
        shoulder = joint(root, ids[0], (s * (hip_spacing_mm / 2 + 35), 86, 0), f'{-s} 0 0')
        upper = joint(shoulder, ids[1], (0, 0, 0), f'0 0 {s}')
        box(upper, f'{side}_upper_arm', (15, upper_arm_mm-17, 16), (0, -upper_arm_mm/2, 0), 0.025)
        elbow = joint(upper, ids[2], (0, -upper_arm_mm, 0), f'{-s} 0 0')
        box(elbow, f'{side}_forearm', (13, forearm_mm-19, 14), (0, -forearm_mm/2, 0), 0.02)
        box(elbow, f'{side}_palm', (24, 6, 28), (0, -forearm_mm, 7), 0.02, side)
    head = joint(root, 25, (0, 116, 0), '0 1 0')
    box(head, 'head', (35, 37, 32), (0, 23, 0), 0.10)
    # Display-only face marking has no collision response or meaningful mass.
    face = box(head, 'camera', (25, 10, 2), (0, 25, 17), 0.00001, 'L')
    face.set('contype', '0')
    face.set('conaffinity', '0')
    contact = ET.SubElement(xml, 'contact')
    # Co-located hip axes share a housing in this simplified model.
    for sid in (13, 20):
        ET.SubElement(contact, 'exclude', body1='pelvis', body2=f'servo_{sid}')
    for shin, foot in ((14, 16), (19, 17)):
        ET.SubElement(contact, 'exclude', body1=f'servo_{shin}', body2=f'servo_{foot}')
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))


def simulate(commands, *, torque_nm=2.0, friction=0.8, hip_spacing_mm=90.0, initial_pitch_deg=90.0,
             settle_s=0.5, hold_s=2.0, upper_arm_mm=61.0, forearm_mm=66.0,
             gate_arm_release=False, max_gate_wait_s=3.0):
    model = build_model(torque_nm, friction, hip_spacing_mm, upper_arm_mm, forearm_mm)
    state = mujoco.MjData(model)
    root = model.body('pelvis').id
    floor = model.geom('floor').id
    geoms = [g for g in range(model.ngeom) if g != floor]
    servo_ids = list(STANDING)
    qaddrs = [model.jnt_qposadr[model.joint(f'j{s}').id] for s in servo_ids]
    aaddrs = [model.actuator(f'a{s}').id for s in servo_ids]
    initial = joint_targets(commands[0]['pose'])
    for s, q, a in zip(servo_ids, qaddrs, aaddrs):
        state.qpos[q] = state.ctrl[a] = initial[s]
    pitch = math.radians(initial_pitch_deg) / 2
    state.qpos[:7] = (0, 0, 0, math.cos(pitch), math.sin(pitch), 0, 0)
    mujoco.mj_forward(model, state)
    lowest = min(state.geom_xpos[g, 1] - np.abs(state.geom_xmat[g].reshape(3, 3)[1]) @ model.geom_size[g] for g in geoms)
    state.qpos[1] += 0.002 - lowest
    mujoco.mj_forward(model, state)
    # Resetting the free base ends here. All later poses come only from dynamics.
    geometry = [{'name': model.geom(g).name, 'size': (model.geom_size[g] * 2000).tolist(),
                 'color': model.geom_rgba[g, :3].tolist()} for g in geoms]
    output = []
    end_command = commands[-1]['t']
    end_time = settle_s + end_command + hold_s + (max_gate_wait_s if gate_arm_release else 0.0)
    next_sample = 0.0
    command_index = 0
    success_time = 0.0
    success_at_end = False
    max_torque = 0.0
    quat = np.empty(4)
    contact_force = np.empty(6)
    gate_delay = 0.0
    release_blocked = False
    for _ in range(round(end_time / model.opt.timestep) + 1):
        elapsed = float(state.time)
        command_t = max(0.0, elapsed - settle_s - gate_delay)
        waiting = False
        while not release_blocked and command_index + 1 < len(commands) and commands[command_index + 1]['t'] <= command_t + 1e-9:
            if (gate_arm_release and commands[command_index + 1]['phase'] == 'release-arms'
                    and commands[command_index]['phase'] != 'release-arms' and success_time < 0.3):
                waiting = True
                gate_delay += model.opt.timestep
                release_blocked = gate_delay >= max_gate_wait_s
                break
            command_index += 1
        command = commands[command_index]
        targets = joint_targets(command['pose'])
        for s, a in zip(servo_ids, aaddrs):
            state.ctrl[a] += np.clip(targets[s] - state.ctrl[a], -4 * model.opt.timestep, 4 * model.opt.timestep)
        mujoco.mj_forward(model, state)
        tilt = math.degrees(math.acos(float(np.clip(state.xmat[root].reshape(3, 3)[1, 1], -1, 1))))
        contacts = set()
        for contact_index, c in enumerate(state.contact[:state.ncon]):
            if floor in (c.geom1, c.geom2) and c.dist <= 0.001:
                mujoco.mj_contactForce(model, state, contact_index, contact_force)
                if contact_force[0] < 0.05:
                    continue
                g = c.geom2 if c.geom1 == floor else c.geom1
                contacts.add(model.geom(g).name)
        soles = {'L_sole', 'R_sole'}
        feet_only = soles <= contacts and contacts <= {'L_sole', 'R_sole', 'L_foot', 'R_foot'}
        foot_tilts = [math.degrees(math.acos(float(np.clip(state.geom_xmat[model.geom(n).id].reshape(3, 3)[1, 1], -1, 1)))) for n in ('L_sole', 'R_sole')]
        quiet = np.linalg.norm(state.qvel[:3]) < 0.05 and np.linalg.norm(state.qvel[3:6]) < 0.3
        upright = tilt < 15 and state.xpos[root, 1] > 0.13 and feet_only and max(foot_tilts) < 10 and quiet
        success_time = success_time + model.opt.timestep if upright else 0.0
        success_at_end = success_time >= 1.0
        max_torque = max(max_torque, float(np.max(np.abs(state.actuator_force))))
        if elapsed + 1e-9 >= next_sample:
            transforms = []
            for g in geoms:
                mujoco.mju_mat2Quat(quat, state.geom_xmat[g])
                transforms.append([*(state.geom_xpos[g] * 1000).round(5).tolist(),
                                   *quat[[1, 2, 3, 0]].round(8).tolist()])
            phase = 'initial-contact' if elapsed < settle_s else ('standing-check' if command_t > end_command else command['phase'])
            if waiting or release_blocked:
                phase = 'support-not-ready' if release_blocked else 'wait-foot-support'
            output.append(dict(t=round(elapsed, 5), phase=phase, pose=command['pose'], transforms=transforms,
                               actual_deg={s: round(math.degrees(state.qpos[q]), 2) for s, q in zip(servo_ids, qaddrs)},
                               tilt_deg=round(tilt, 2), height_mm=round(float(state.xpos[root, 1] * 1000), 2),
                               contacts=sorted(contacts), foot_tilts=foot_tilts, stable_s=round(success_time, 3),
                               root_mm=(state.xpos[root] * 1000).tolist()))
            next_sample += 0.03
        if not np.isfinite(state.qpos).all() or not np.isfinite(state.qvel).all():
            raise RuntimeError('Physics diverged; no valid result')
        mujoco.mj_step(model, state)
    if np.any(state.warning.number):
        raise RuntimeError(f'MuJoCo warnings: {state.warning.number.tolist()}')
    return dict(frames=output, geometry=geometry,
                result='STANDING IN MODEL' if success_at_end else 'NOT STANDING IN MODEL',
                release_blocked=release_blocked,
                assumptions=dict(engine=f'MuJoCo {mujoco.__version__}', calibrated=False,
                                 mass_kg=round(float(model.body_mass.sum()), 3), torque_nm=torque_nm,
                                 friction=friction, hip_spacing_mm=hip_spacing_mm,
                                 source_hip_spacing_mm=2 * ROBOT['half_hip'], servo_speed_rad_s=4.0,
                                 upper_leg_mm=ROBOT['upper_leg'], lower_leg_mm=ROBOT['lower_leg'],
                                 upper_arm_mm=upper_arm_mm, forearm_mm=forearm_mm,
                                 approximate_parts='torso, arms, feet, mass, joint zeros, actuator response',
                                 excluded_housing_contacts=[['pelvis',13],['pelvis',20],[14,16],[19,17]],
                                 kp=12.0, kv=0.3, dt_s=model.opt.timestep, contact_time_constant_s=0.005,
                                 max_torque_nm=round(max_torque, 3)))
