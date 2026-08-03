import time
import unittest

from src.sensors import DepthObstacleGuard, DepthReading, RobotSensorHub, parse_serial_depth_line


def depth_line(values: list[int]) -> str:
    return "D,1234," + ",".join(str(value) for value in values)


class DepthSensorTests(unittest.TestCase):
    def test_parser_requires_exact_8_by_8_frame(self) -> None:
        reading = parse_serial_depth_line(depth_line([1000] * 64))
        self.assertIsNotNone(reading)
        assert reading is not None
        self.assertEqual(reading.sensor_time_ms, 1234)
        self.assertEqual(reading.center_distance_mm, 1000)
        self.assertEqual(reading.vertical_span_mm, 0)
        self.assertIsNone(parse_serial_depth_line(depth_line([1000] * 63)))
        self.assertIsNone(parse_serial_depth_line(depth_line([5000] * 64)))

    def test_obstacle_distance_uses_small_region_median(self) -> None:
        values = [1200] * 64
        for row in (3, 4):
            for col in (3, 4):
                values[row * 8 + col] = 240
        reading = DepthReading(tuple(values), 1)
        self.assertEqual(reading.obstacle_distance_mm, 240)

    def test_obstacle_guard_has_debounce_and_clearance(self) -> None:
        near = DepthReading(tuple([250] * 64), 1)
        far = DepthReading(tuple([700] * 64), 2)
        guard = DepthObstacleGuard(350, 100, 3)
        self.assertFalse(guard.update(near)[0])
        self.assertFalse(guard.update(near)[0])
        self.assertTrue(guard.update(near)[0])
        self.assertFalse(guard.update(far)[0])

    def test_depth_has_own_stale_timeout(self) -> None:
        hub = RobotSensorHub(timeout_s=0.10, depth_timeout_s=0.30, use_depth=True)
        hub._depth = DepthReading(tuple([900] * 64), 1)
        hub._depth_at = time.monotonic() - 0.20
        self.assertIsNotNone(hub.read().depth)
        hub._depth_at = time.monotonic() - 0.40
        self.assertIsNone(hub.read().depth)

    def test_stairs_require_depth_relief(self) -> None:
        try:
            from src.terrain_control import TerrainModeController
            from src.terrain_vision import TerrainKind, TerrainObservation
        except ImportError as exc:
            self.skipTest(str(exc))
        controller = TerrainModeController(34, 30, 62, 3, 12, 62, 0.58, True, 80)
        observation = TerrainObservation(
            TerrainKind.STAIRS_UP,
            TerrainKind.STAIRS_UP,
            0.9,
            0.5,
            (180, 220, 260),
            0.05,
            True,
        )
        flat_depth = DepthReading(tuple([1000] * 64), 1)
        profile, status = controller.select(observation, flat_depth)
        self.assertIsNone(profile)
        self.assertEqual(status, "STAIR DEPTH UNCONFIRMED")

        relief = tuple(700 + (index // 8) * 40 for index in range(64))
        profile, status = controller.select(observation, DepthReading(relief, 2))
        self.assertIsNotNone(profile)
        self.assertEqual(status, "STAIRS UP")


if __name__ == "__main__":
    unittest.main()
