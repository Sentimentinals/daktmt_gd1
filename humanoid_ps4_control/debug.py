from __future__ import annotations

from dataclasses import dataclass
import math
import time

from src.backends import make_backend
from src.walking_engine import DynamicWalkingEngine, PWM_PER_DEG, ROBOT, STANDING


LEG_SERVO_IDS = [1, 2, 3, 4, 5, 20, 21, 22, 23, 24]
ARM_SERVO_IDS = [6, 7, 8, 17, 18, 19]


@dataclass
class Config:
    port: str = "COM24"
    baudrate: int = 115200
    update_ms: int = 40

    walk_speed: float = 0.30
    turn_cmd: float = 0.0
    side_cmd: float = 0.0

    phase_ms: int = 320
    hold_ms: int = 250
    standing_ms: int = 1000
    auto_phase: bool = False
    support_shift_only: bool = False
    lift_land_only: bool = True
    support_shift_cycles: int = 2
    print_servos: bool = True

    # Algorithm-level gait parameters. Avoid per-phase PWM edits here.
    max_step_len: float = 28.0
    step_height: float = 24.0
    t_step: float = 1.28
    t_dbl: float = 0.04
    zmp_support_ratio: float = 0.88
    hip_abduct_gain: float = 0.35
    ankle_roll_gain: float = -0.38
    swing_ankle_roll_scale: float = 0.0
    step_x_ratio: float = 0.62
    left_swing_x_scale: float = 1.45
    right_swing_x_scale: float = 1.45
    left_step_height_scale: float = 1.25
    right_step_height_scale: float = 1.25
    landing_gap_mm: float = 18.0
    lift_start_phase: float = 0.0
    swing_advance_end_phase: float = 0.60
    lift_end_phase: float = 0.96
    landing_roll_release_start: float = 0.62
    command_rate_limit: float = 24.0

    arm_swing_pwm: int = 260
    arm_right_dir: int = 1
    arm_left_dir: int = 1
    arm_smooth_tau: float = 0.22
    arm_quantum_pwm: int = 5


def build_engine(cfg: Config) -> DynamicWalkingEngine:
    return DynamicWalkingEngine(
        dt=cfg.update_ms / 1000.0,
        t_step=cfg.t_step,
        t_dbl=cfg.t_dbl,
        max_step_len=cfg.max_step_len,
        step_height=cfg.step_height,
        zmp_support_ratio=cfg.zmp_support_ratio,
        hip_abduct_gain=cfg.hip_abduct_gain,
        ankle_roll_gain=cfg.ankle_roll_gain,
        swing_ankle_roll_scale=cfg.swing_ankle_roll_scale,
        step_x_ratio=cfg.step_x_ratio,
        left_swing_x_scale=cfg.left_swing_x_scale,
        right_swing_x_scale=cfg.right_swing_x_scale,
        left_step_height_scale=cfg.left_step_height_scale,
        right_step_height_scale=cfg.right_step_height_scale,
        landing_gap_mm=cfg.landing_gap_mm,
        lift_start_phase=cfg.lift_start_phase,
        swing_advance_end_phase=cfg.swing_advance_end_phase,
        lift_end_phase=cfg.lift_end_phase,
        landing_roll_release_start=cfg.landing_roll_release_start,
        command_rate_limit=cfg.command_rate_limit,
        arm_swing_pwm=cfg.arm_swing_pwm,
        arm_right_dir=cfg.arm_right_dir,
        arm_left_dir=cfg.arm_left_dir,
        arm_smooth_tau=cfg.arm_smooth_tau,
        arm_quantum_pwm=cfg.arm_quantum_pwm,
    )


def servo_summary(pose: dict[int, int]) -> str:
    ids = LEG_SERVO_IDS + (ARM_SERVO_IDS if any(pose[sid] != STANDING[sid] for sid in ARM_SERVO_IDS) else [])
    return " ".join(f"{sid}:{pose[sid]}({pose[sid] - STANDING[sid]:+d})" for sid in ids)


def ankle_delta_summary(prev_pose: dict[int, int], pose: dict[int, int]) -> str:
    return (
        f"ankle-step: 2 {prev_pose[2]}->{pose[2]} ({pose[2] - prev_pose[2]:+d}), "
        f"23 {prev_pose[23]}->{pose[23]} ({pose[23] - prev_pose[23]:+d})"
    )


def pitch_debug_summary(pose: dict[int, int]) -> str:
    right = (
        f"right pitch: ankle 2 {pose[2]}({pose[2] - STANDING[2]:+d}), "
        f"knee 3 {pose[3]}({pose[3] - STANDING[3]:+d}), "
        f"thigh 4 {pose[4]}({pose[4] - STANDING[4]:+d})"
    )
    left = (
        f"left pitch: thigh 21 {pose[21]}({pose[21] - STANDING[21]:+d}), "
        f"knee 22 {pose[22]}({pose[22] - STANDING[22]:+d}), "
        f"ankle 23 {pose[23]}({pose[23] - STANDING[23]:+d})"
    )
    return f"{right}\n{left}"


def support_roll_summary(pose: dict[int, int]) -> str:
    right_roll = (
        f"right support roll: ankle 1 {pose[1]}({pose[1] - STANDING[1]:+d}), "
        f"hip 5 {pose[5]}({pose[5] - STANDING[5]:+d})"
    )
    left_roll = (
        f"left support roll: ankle 24 {pose[24]}({pose[24] - STANDING[24]:+d}), "
        f"hip 20 {pose[20]}({pose[20] - STANDING[20]:+d})"
    )
    pitch_noise = " ".join(
        f"{sid}:{pose[sid] - STANDING[sid]:+d}" for sid in (2, 3, 4, 21, 22, 23)
    )
    return f"{right_roll}\n{left_roll}\npitch/thigh/knee delta check: {pitch_noise}"


def phase_state(engine: DynamicWalkingEngine) -> str:
    return (
        f"phase={engine.last_phase_mode} support={engine.support_leg} swing={engine.last_swing_leg} "
        f"arm={engine.last_arm_role} "
        f"liftF={engine.last_lift_factor:.2f} landT={engine.last_landing_progress:.2f} "
        f"xL={engine.last_foot_L[0]:.1f} zL={engine.last_foot_L[2]:.1f} "
        f"xR={engine.last_foot_R[0]:.1f} zR={engine.last_foot_R[2]:.1f}"
    )


def roll_target_summary(cfg: Config) -> str:
    support_y = ROBOT["half_hip"] * cfg.zmp_support_ratio
    raw_roll_deg = math.degrees(math.atan2(support_y, ROBOT["com_height"]))
    ankle_deg = raw_roll_deg * cfg.ankle_roll_gain
    hip_counter_deg = abs(raw_roll_deg * cfg.hip_abduct_gain)
    ankle_pwm = round(abs(ankle_deg) * PWM_PER_DEG)
    hip_pwm = round(hip_counter_deg * PWM_PER_DEG)
    right_ankle_pwm = round(-ankle_deg * PWM_PER_DEG)
    left_ankle_pwm = round(-ankle_deg * PWM_PER_DEG)
    right_hip_pwm = hip_pwm
    left_hip_pwm = -hip_pwm

    return (
        "Phase-1 nominal roll target:\n"
        f"- support lateral ZMP: {support_y:.1f} mm\n"
        f"- support ankle roll: {abs(ankle_deg):.1f} deg (~{ankle_pwm} PWM)\n"
        f"- support hip counter-roll: {hip_counter_deg:.1f} deg (~{hip_pwm} PWM)\n"
        f"- right support: servo 1 {right_ankle_pwm:+d} PWM, servo 5 {right_hip_pwm:+d} PWM\n"
        f"- left support: servo 24 {left_ankle_pwm:+d} PWM, servo 20 {left_hip_pwm:+d} PWM\n"
        "- note: left/right physical lean is mirrored by servo mounting and DIR mapping; compare real motion, not PWM sign alone\n"
        f"- phase-3 landing gap X: at least {cfg.landing_gap_mm:.1f} mm from support foot\n"
        f"- support roll release starts at {cfg.landing_roll_release_start:.2f} of landing phase\n"
        f"- arm logical envelope: left swing +{cfg.arm_swing_pwm}, right swing -{cfg.arm_swing_pwm}; "
        f"servo dirs 8:{cfg.arm_right_dir:+d}, 17:{cfg.arm_left_dir:+d}\n"
    )


def capture_support_shift_phases(
    cfg: Config,
) -> list[tuple[str, str, dict[int, int]]]:
    engine = build_engine(cfg)
    phases: list[tuple[str, str, dict[int, int]]] = []

    for cycle in range(1, cfg.support_shift_cycles + 1):
        expected_swing_leg = "left" if cycle % 2 == 1 else "right"
        step_phases = capture_next_step(engine, cfg, expected_swing_leg)
        shift_phase = next((phase for phase in step_phases if phase[0].startswith("01 ")), None)
        if shift_phase is None:
            raise RuntimeError(f"Could not capture support-shift phase for cycle {cycle}.")
        phases.append(shift_phase)

    return phases


def capture_next_step(
    engine: DynamicWalkingEngine,
    cfg: Config,
    expected_swing_leg: str,
) -> list[tuple[str, str, dict[int, int]]]:
    phases: dict[str, tuple[str, dict[int, int]]] = {}

    swing_leg = None
    support_leg = None
    shift_candidate: tuple[float, str, dict[int, int]] | None = None
    lift_peak: tuple[float, str, dict[int, int]] | None = None
    landed = False
    after_land_shift: tuple[str, dict[int, int]] | None = None

    for _ in range(320):
        pose = engine.update(cfg.walk_speed, turn_cmd=cfg.turn_cmd, side_cmd=cfg.side_cmd)
        state = phase_state(engine)

        if landed:
            if engine.last_phase_mode == "shift":
                after_land_shift = (state, dict(pose))
                break
            continue

        if swing_leg is None and engine.last_swing_leg in ("left", "right"):
            if engine.last_swing_leg != expected_swing_leg:
                continue
            swing_leg = engine.last_swing_leg
            support_leg = "right" if swing_leg == "left" else "left"

        if swing_leg is None or engine.last_swing_leg != swing_leg:
            continue

        if engine.support_leg == support_leg and engine.last_lift_factor <= 0.02:
            if support_leg == "right":
                score = abs(pose[1] - STANDING[1]) + abs(pose[5] - STANDING[5])
            else:
                score = abs(pose[24] - STANDING[24]) + abs(pose[20] - STANDING[20])
            if shift_candidate is None or score > shift_candidate[0]:
                shift_candidate = (score, state, dict(pose))

        if lift_peak is None or engine.last_lift_factor > lift_peak[0]:
            lift_peak = (engine.last_lift_factor, state, dict(pose))

        if lift_peak is not None and lift_peak[0] > 0.90 and engine.last_lift_factor <= 0.05:
            phases[f"03 land-{swing_leg}-leg"] = (state, dict(pose))
            landed = True
            continue

    if swing_leg is None or support_leg is None:
        raise RuntimeError(
            f"Could not capture {expected_swing_leg} walking step. "
            "Check walk_speed, command_deadzone, and preview queue state."
        )

    if shift_candidate is None and not cfg.lift_land_only:
        raise RuntimeError(f"Could not capture support shift for {swing_leg} swing.")
    if shift_candidate is not None:
        phases[f"01 shift-to-{support_leg}-support"] = (shift_candidate[1], dict(shift_candidate[2]))

    if lift_peak is None:
        raise RuntimeError(f"Could not capture lift peak for {swing_leg} swing.")
    phases[f"02 lift-{swing_leg}-leg"] = (lift_peak[1], lift_peak[2])

    if not landed:
        raise RuntimeError(f"Could not capture landing for {swing_leg} swing.")
    if after_land_shift is not None:
        phases[f"04 after-land-next-shift"] = after_land_shift

    order = [
        f"01 shift-to-{support_leg}-support",
        f"02 lift-{swing_leg}-leg",
        f"03 land-{swing_leg}-leg",
    ]
    if f"04 after-land-next-shift" in phases:
        order.append(f"04 after-land-next-shift")
    if cfg.lift_land_only:
        order = [
            f"02 lift-{swing_leg}-leg",
            f"03 land-{swing_leg}-leg",
        ]
    return [(name, phases[name][0], phases[name][1]) for name in order]


def send_phase_sequence(cfg: Config) -> None:
    with make_backend(mode="serial", port=cfg.port, baudrate=cfg.baudrate) as backend:
        print(f"Sending STANDING for {cfg.standing_ms} ms.")
        backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
        time.sleep(cfg.standing_ms / 1000.0)

        last_sent = dict(STANDING)
        if cfg.support_shift_only:
            phases = capture_support_shift_phases(cfg)
            print("\n=== SUPPORT SHIFT DEBUG ===")
            print("Only support ankle/hip roll should move clearly. Pitch/thigh/knee deltas should stay near 0.")
            try:
                for idx, (name, state, pose) in enumerate(phases, start=1):
                    max_delta = max(abs(pose[sid] - last_sent[sid]) for sid in LEG_SERVO_IDS)
                    actual_ms = cfg.phase_ms

                    print(f"\n{idx:02d} {name}: {state}")
                    if cfg.print_servos:
                        print(servo_summary(pose))
                        print(support_roll_summary(pose))
                    print(f"  duration: {actual_ms}ms (delta={max_delta})")
                    if not cfg.auto_phase:
                        input("Press Enter to send support-shift phase...")
                    backend.send(pose, duration_ms=actual_ms, force=True)
                    time.sleep(actual_ms / 1000.0 + cfg.hold_ms / 1000.0)

                    print("Returning to STANDING before next support-side test.")
                    backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
                    time.sleep(cfg.standing_ms / 1000.0)
                    last_sent = dict(STANDING)
            except KeyboardInterrupt:
                print("\nCtrl+C received. Returning to STANDING.")
            finally:
                backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
                time.sleep(cfg.standing_ms / 1000.0)
            return

        engine = build_engine(cfg)
        cycle = 0
        try:
            while True:
                cycle += 1
                expected_swing_leg = "left" if cycle % 2 == 1 else "right"
                phases = capture_next_step(engine, cfg, expected_swing_leg)
                print(f"\n=== WALK CYCLE {cycle} ===")

                for name, state, pose in phases:
                    max_delta = max(abs(pose[sid] - last_sent[sid]) for sid in LEG_SERVO_IDS)
                    ref_delta = 350
                    scale = max(1.0, max_delta / ref_delta)
                    actual_ms = int(cfg.phase_ms * scale)

                    print(f"\n{name}: {state}")
                    if cfg.print_servos:
                        print(servo_summary(pose))
                        print(support_roll_summary(pose))
                        print(pitch_debug_summary(pose))
                        print(ankle_delta_summary(last_sent, pose))
                    print(f"  duration: {actual_ms}ms (delta={max_delta}, scale={scale:.1f}x)")
                    if not cfg.auto_phase:
                        input("Press Enter to send this phase...")
                    backend.send(pose, duration_ms=actual_ms, force=True)
                    time.sleep(actual_ms / 1000.0 + cfg.hold_ms / 1000.0)
                    last_sent = dict(pose)
        except KeyboardInterrupt:
            print("\nCtrl+C received. Returning to STANDING.")
        finally:
            backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
            time.sleep(cfg.standing_ms / 1000.0)


def run_continuous_walk(cfg: Config) -> None:
    """Real-time continuous walking at update_ms frame rate."""
    engine = build_engine(cfg)
    dt = cfg.update_ms / 1000.0

    with make_backend(mode="serial", port=cfg.port, baudrate=cfg.baudrate) as backend:
        print(f"Sending STANDING for {cfg.standing_ms} ms...")
        backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
        time.sleep(cfg.standing_ms / 1000.0)

        print(f"\nContinuous walk: speed={cfg.walk_speed}, frame={cfg.update_ms}ms")
        print("Press Ctrl+C to stop.\n")

        frame = 0
        prev_swing = "none"
        try:
            while True:
                t_start = time.perf_counter()

                pose = engine.update(
                    cfg.walk_speed,
                    turn_cmd=cfg.turn_cmd,
                    side_cmd=cfg.side_cmd,
                )

                # Print phase transitions
                if engine.last_swing_leg != prev_swing:
                    if engine.last_swing_leg != "none":
                        print(f"[frame {frame}] Swing: {engine.last_swing_leg}")
                    prev_swing = engine.last_swing_leg

                try:
                    backend.send(pose, duration_ms=cfg.update_ms)
                except Exception as exc:
                    print(f"Send error: {exc}")

                frame += 1
                elapsed = time.perf_counter() - t_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print(f"\nStopped at frame {frame}. Returning to STANDING...")
        finally:
            backend.send(STANDING, duration_ms=cfg.standing_ms, force=True)
            time.sleep(cfg.standing_ms / 1000.0)


def main() -> None:
    cfg = Config()

    if cfg.auto_phase:
        # Continuous real-time walking
        print("=== CONTINUOUS WALK MODE ===")
        print(roll_target_summary(cfg))
        run_continuous_walk(cfg)
    else:
        # Step-by-step debug
        print("=== PHASE DEBUG MODE ===")
        print("2-phase mode: lift leg, then land leg.")
        print("Set auto_phase=True for continuous walking.")
        print(roll_target_summary(cfg))
        send_phase_sequence(cfg)


if __name__ == "__main__":
    main()
