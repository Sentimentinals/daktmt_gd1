import unittest


class SquatEngineTests(unittest.TestCase):
    def test_squat_is_symmetric_bounded_and_returns_to_calibrated_standing(self) -> None:
        try:
            from src.config import STANDING
            from src.walking_engine import AdaptiveSquatEngine
        except ImportError as exc:
            self.skipTest(str(exc))

        engine = AdaptiveSquatEngine(0.03, 12.0, 40.0, 35.0, 28.0)
        poses = [engine.update(1.0) for _ in range(60)]
        deep = poses[-1]
        poses.extend(engine.update(0.0) for _ in range(65))

        self.assertTrue(all(500 <= value <= 2500 for pose in poses for value in pose.values()))
        self.assertEqual(deep[14] - STANDING[14], STANDING[19] - deep[19])
        self.assertTrue(engine.is_idle())
        self.assertEqual(poses[-1], STANDING)


if __name__ == "__main__":
    unittest.main()
