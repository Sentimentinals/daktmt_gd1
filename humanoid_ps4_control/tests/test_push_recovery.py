import unittest

from src.balance import (
    BalanceConfig,
    IMUBalanceController,
    PushRecoveryConfig,
    PushRecoveryController,
    RecoveryState,
    lower_toward_standing,
)
from src.config import STANDING
from src.walking_engine import DynamicWalkingEngine


class PushRecoveryTests(unittest.TestCase):
    def make_controller(self) -> PushRecoveryController:
        return PushRecoveryController(
            PushRecoveryConfig(
                warning_tilt_deg=3.0,
                recovery_tilt_deg=5.0,
                safe_lower_tilt_deg=9.0,
                recovery_rate_deg_s=28.0,
            )
        )

    def test_light_tilt_uses_ankle_and_hip_only(self) -> None:
        controller = self.make_controller()
        decision = controller.update(3.5, 0.0, 0.04, True, True)
        self.assertEqual(decision.state, RecoveryState.ANKLE_HIP)
        self.assertFalse(decision.start_step)

        balance = IMUBalanceController(BalanceConfig(max_correction_deg=2.0))
        corrected = balance.apply(STANDING, roll_deg=3.5, pitch_deg=0.0, dt=0.04)
        changed = {servo_id for servo_id in STANDING if corrected[servo_id] != STANDING[servo_id]}
        self.assertTrue(changed)
        self.assertTrue(changed <= {12, 13, 15, 16, 17, 18, 20, 21})
        self.assertEqual(corrected[14], STANDING[14])
        self.assertEqual(corrected[19], STANDING[19])

    def test_push_starts_one_short_recovery_step(self) -> None:
        controller = self.make_controller()
        controller.update(0.0, 0.0, 0.04, True, True, now=1.0)
        decision = controller.update(0.0, 6.0, 0.04, True, True, now=1.04)
        self.assertEqual(decision.state, RecoveryState.RECOVERY_STEP)
        self.assertTrue(decision.start_step)
        self.assertLess(decision.forward_cmd, 0.0)
        self.assertEqual(decision.side_cmd, 0.0)

    def test_lateral_push_uses_side_step(self) -> None:
        controller = self.make_controller()
        controller.update(0.0, 0.0, 0.04, True, True, now=1.0)
        decision = controller.update(6.0, 0.0, 0.04, True, True, now=1.04)
        self.assertTrue(decision.start_step)
        self.assertEqual(decision.forward_cmd, 0.0)
        self.assertLess(decision.side_cmd, 0.0)

    def test_missing_fsr_blocks_recovery_step(self) -> None:
        controller = self.make_controller()
        controller.update(0.0, 0.0, 0.04, True, True)
        decision = controller.update(0.0, 6.0, 0.04, False, False)
        self.assertEqual(decision.state, RecoveryState.ANKLE_HIP)
        self.assertFalse(decision.start_step)
        self.assertIn("blocked", decision.reason)

    def test_recovery_allows_the_swing_fsr_to_lift(self) -> None:
        controller = self.make_controller()
        controller.update(0.0, 0.0, 0.04, True, True, now=1.0)
        controller.update(0.0, 6.0, 0.04, True, True, now=1.04)
        decision = controller.update(0.0, 6.0, 0.04, False, True, support_leg="right", now=1.08)
        self.assertEqual(decision.state, RecoveryState.RECOVERY_STEP)

    def test_lost_single_support_or_large_tilt_latches_safe_lower(self) -> None:
        controller = self.make_controller()
        decision = controller.update(0.0, 0.0, 0.04, True, False, single_support=True, support_leg="right")
        self.assertTrue(decision.safe_lower)
        self.assertEqual(decision.reason, "support FSR lost")

        controller.reset()
        decision = controller.update(0.0, 9.0, 0.04, True, True)
        self.assertTrue(decision.safe_lower)
        self.assertEqual(decision.reason, "tilt limit")

    def test_recovery_gait_and_lowering_are_bounded(self) -> None:
        engine = DynamicWalkingEngine(
            dt=0.04,
            t_step=1.35,
            max_step_len=34.0,
            max_side_step_len=38.0,
            step_height=12.0,
            command_rate_limit=1000.0,
        )
        engine.stop_extra_steps = 0
        poses = [engine.update(-0.20)]
        poses.extend(engine.update(0.0) for _ in range(120))
        self.assertTrue(all(500 <= pwm <= 2500 for pose in poses for pwm in pose.values()))

        current = dict(STANDING)
        current[14] = 1900
        lowered = lower_toward_standing(current, STANDING, dt=0.04, max_rate_pwm_s=300.0)
        self.assertEqual(lowered[14], 1888)


if __name__ == "__main__":
    unittest.main()
