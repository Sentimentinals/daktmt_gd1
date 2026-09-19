from __future__ import annotations

import argparse
import ctypes
import os
import time
from dataclasses import dataclass

from src.arm_dance import ArmDanceEngine
from src.backends import MockBackend, SerialRTBackend
from src.config import Config, STANDING
from src.walking_engine import DynamicWalkingEngine


@dataclass(frozen=True)
class Controls:
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    dance: bool = False
    stop: bool = False


class WindowsKeyboard:
    VK_UP = 0x26
    VK_DOWN = 0x28
    VK_LEFT = 0x25
    VK_RIGHT = 0x27
    VK_A = 0x41
    VK_SPACE = 0x20
    VK_ESCAPE = 0x1B

    def __init__(self) -> None:
        if os.name != "nt":
            raise RuntimeError("This COM keyboard test runs on Windows only.")
        self._get_key = ctypes.windll.user32.GetAsyncKeyState

    def _down(self, key: int) -> bool:
        return bool(self._get_key(key) & 0x8000)

    def read(self) -> Controls:
        return Controls(
            forward=self._down(self.VK_UP),
            backward=self._down(self.VK_DOWN),
            left=self._down(self.VK_LEFT),
            right=self._down(self.VK_RIGHT),
            dance=self._down(self.VK_A),
            stop=self._down(self.VK_SPACE),
        )

    def exit_pressed(self) -> bool:
        return self._down(self.VK_ESCAPE)


def _walking_engine(config: Config) -> DynamicWalkingEngine:
    return DynamicWalkingEngine(
        dt=config.update_ms / 1000.0,
        t_step=config.t_step,
        t_dbl=config.t_dbl,
        max_step_len=config.walk_step_length_mm,
        max_turn_step_len=config.max_turn_step_len,
        max_side_step_len=config.max_side_step_len,
        step_height=config.walk_step_height_mm,
        crouch_depth_mm=config.walk_crouch_depth_mm,
        forward_lean_deg=config.walk_forward_lean_deg,
        zmp_support_ratio=config.zmp_support_ratio,
        ankle_roll_gain=config.ankle_roll_gain,
        landing_gap_mm=0.0,
        lift_start_phase=config.walk_lift_start_phase,
        swing_advance_end_phase=config.walk_swing_advance_end_phase,
        lift_end_phase=config.walk_lift_end_phase,
        landing_roll_release_start=config.walk_landing_roll_release_start,
        arm_swing_pwm=config.arm_swing_pwm,
        arm_right_dir=config.arm_right_dir,
        arm_left_dir=config.arm_left_dir,
        crouch_transition_s=config.walk_crouch_transition_s,
    )


class WalkingComTest:
    def __init__(self, config: Config, backend) -> None:
        self.config = config
        self.backend = backend
        self.walking = _walking_engine(config)
        self.dance = ArmDanceEngine(
            dt=config.update_ms / 1000.0,
            period_s=config.dance_period,
            transition_s=config.dance_transition,
            shoulder_pwm=config.dance_shoulder_pwm,
            elbow_pwm=config.dance_elbow_pwm,
            lift_pwm=config.dance_lift_pwm,
            head_pwm=config.dance_head_pwm,
        )
        self._dance_pressed = False

    def reset(self) -> None:
        self.walking.reset()
        self.dance.reset()

    def update(self, controls: Controls) -> tuple[dict[int, int], str]:
        forward = float(controls.forward) - float(controls.backward)
        turn = float(controls.left) - float(controls.right)
        moving = bool(forward or turn)
        dance_tapped = controls.dance and not self._dance_pressed
        self._dance_pressed = controls.dance

        if controls.stop:
            self.reset()
            pose = dict(STANDING)
            status = "STOP / STANDING"
        elif moving:
            if self.dance.running:
                self.dance.reset()
            pose = self.walking.update(
                forward * self.config.walk_speed,
                turn_cmd=turn * self.config.turn_speed,
            )
            directions = []
            if forward:
                directions.append("FORWARD" if forward > 0 else "BACKWARD")
            if turn:
                directions.append("TURN LEFT" if turn > 0 else "TURN RIGHT")
            status = " + ".join(directions)
        elif dance_tapped:
            self.walking.reset()
            self.dance.toggle()
            pose = self.dance.update()
            status = "ARM DANCE" if self.dance.active else "DANCE RETURN"
        elif self.dance.running:
            pose = self.dance.update()
            status = "ARM DANCE" if self.dance.active else "DANCE RETURN"
        else:
            pose = self.walking.update(0.0, turn_cmd=0.0)
            status = "STANDING" if self.walking.is_idle_ready() else "SETTLING"

        self.backend.send(pose, duration_ms=self.config.update_ms)
        return pose, status


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Minimal Windows keyboard test for walking and arm dance."
    )
    parser.add_argument("--servo-port", help="Servo controller COM port, for example COM3")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true")
    options = parser.parse_args()
    if not options.dry_run and not options.servo_port:
        parser.error("--servo-port is required unless --dry-run is used")

    config = Config()
    backend = (
        MockBackend(verbose=False)
        if options.dry_run
        else SerialRTBackend(options.servo_port, options.baudrate)
    )
    keyboard = WindowsKeyboard()
    controller = WalkingComTest(config, backend)
    period_s = config.update_ms / 1000.0
    last_status = ""

    backend.open()
    try:
        backend.send(STANDING, duration_ms=800, force=True)
        time.sleep(0.8)
        print("Arrow keys: Up/Down walk, Left/Right turn")
        print("A: arm dance | Space: stop | Esc: exit")
        while not keyboard.exit_pressed():
            started = time.monotonic()
            _, status = controller.update(keyboard.read())
            if status != last_status:
                print(f"[walking-test] {status}")
                last_status = status
            remaining = period_s - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        controller.reset()
        backend.send(STANDING, duration_ms=500, force=True)
        time.sleep(0.5)
        backend.close()


if __name__ == "__main__":
    main()
