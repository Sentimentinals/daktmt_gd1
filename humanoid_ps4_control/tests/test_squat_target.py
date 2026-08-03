import unittest

from src.person_follow import ObjectDetection, PersonFrame, SquatTargetController
from src.sensors import DepthReading


class SquatTargetTests(unittest.TestCase):
    def make_controller(self) -> SquatTargetController:
        return SquatTargetController(180, 650, 0.28, 0.20, 0.8)

    @staticmethod
    def frame(box: tuple[int, int, int, int], captured_at: float = 10.0) -> PersonFrame:
        target = ObjectDetection("bottle", box, 0.9, 480, 360)
        return PersonFrame(captured_at=captured_at, objects=(target,))

    def test_fuses_centered_camera_target_and_tof_distance(self) -> None:
        depth = DepthReading(tuple([400] * 64), 1)
        ratio, status = self.make_controller().command(
            depth,
            self.frame((190, 170, 290, 330)),
            now_s=10.1,
        )
        self.assertGreaterEqual(ratio, 0.20)
        self.assertLessEqual(ratio, 1.0)
        self.assertEqual(status, "BOTTLE 400 MM")

    def test_rejects_missing_stale_or_off_center_target(self) -> None:
        controller = self.make_controller()
        depth = DepthReading(tuple([400] * 64), 1)
        self.assertEqual(controller.command(None, self.frame((190, 170, 290, 330)), 10.1)[1], "TOF WAIT")
        self.assertEqual(controller.command(depth, PersonFrame(), 10.1)[1], "OBJECT WAIT")
        self.assertEqual(controller.command(depth, self.frame((190, 170, 290, 330)), 11.0)[1], "OBJECT WAIT")
        self.assertEqual(controller.command(depth, self.frame((0, 170, 80, 330)), 10.1)[1], "CENTER OBJECT")

    def test_rejects_object_outside_reachable_distance(self) -> None:
        controller = self.make_controller()
        frame = self.frame((190, 170, 290, 330))
        self.assertEqual(controller.command(DepthReading(tuple([120] * 64)), frame, 10.1)[1], "OBJECT TOO CLOSE")
        self.assertEqual(controller.command(DepthReading(tuple([900] * 64)), frame, 10.1)[1], "OBJECT TOO FAR")


if __name__ == "__main__":
    unittest.main()
