import math
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import numpy as np

from src.camera import HeadlessCamera
from src.config import Config, DIR, STANDING
from src.fall_safety import PriorityBackend
from src.gait_dashboard import GaitDashboard, WebControlState
from src.imu_bno055 import IMUReading
from src.person_follow import PersonDetection, PersonDetector, PersonFollowController, PersonFrame, PersonObstaclePlanner
from src.sensors import DepthReading, SensorSnapshot
from src.stair_main import run_terrain_auto
from src.stair_motion import StairStepEngine
from src.stair_perception import StairDetection, StairDetector, StairFrame, StairGeometry, estimate_stair_geometry
from src.walking_engine import SquatEngine, compute_pose


def stepper():
    a = Config()
    return StairStepEngine(
        clearance_mm=a.stair_foot_clearance_mm,
        shift_s=a.stair_phase_shift_s, swing_s=a.stair_phase_swing_s,
        transfer_s=a.stair_phase_transfer_s, settle_s=a.stair_phase_settle_s,
        zmp_support_ratio=a.zmp_support_ratio, ankle_roll_gain=a.ankle_roll_gain,
        crouch_depth_mm=a.stair_crouch_depth_mm,
    )


class DetectionTests(unittest.TestCase):
    def setUp(self):
        import cv2
        self.cv2 = cv2
        self.image = np.zeros((240, 320, 3), np.uint8)

    def person_detector(self, every=1):
        d = PersonDetector.__new__(PersonDetector)
        d._cv2 = self.cv2
        d._net = Mock()
        d._net.forward.return_value = np.zeros((1, 1, 0, 7), np.float32)
        d.confidence, d.detect_every_frames = 0.55, every
        d._frame_count, d._last = 0, PersonFrame()
        d._tracks, d._next_track_id, d._max_track_misses = {}, 1, 5
        return d

    def test_person_keeps_capture_time_when_inference_is_slow(self):
        d = self.person_detector()
        clock = [100.0]
        def forward():
            clock[0] += 0.8
            return np.zeros((1, 1, 0, 7), np.float32)
        d._net.forward.side_effect = forward
        with patch('src.person_follow.time.monotonic', side_effect=lambda: clock[0]):
            self.assertEqual(d.detect(self.image).captured_at, 100.0)

    def test_person_cached_result_does_not_refresh_capture_time(self):
        d = self.person_detector(every=2)
        d.detect(self.image, captured_at=10)
        fresh = d.detect(self.image, captured_at=11)
        self.assertIs(d.detect(self.image, captured_at=12), fresh)
        self.assertEqual(fresh.captured_at, 11)
        self.assertEqual(d._net.forward.call_count, 1)

    def test_person_rejects_empty_frame(self):
        with self.assertRaises(ValueError):
            self.person_detector().detect(np.zeros((0, 0, 3), np.uint8))

    def test_tracker_preserves_ids_when_detection_order_changes(self):
        d = self.person_detector()
        people = [PersonDetection((10, 10, 80, 220), .8, 320, 240),
                  PersonDetection((230, 10, 310, 220), .9, 320, 240)]
        first = d._assign_track_ids(people, [None, None])
        second = d._assign_track_ids(people[::-1], [None, None])
        self.assertEqual([p.track_id for p in second], [p.track_id for p in first[::-1]])

    def test_expired_track_is_not_silently_reused(self):
        d = self.person_detector()
        person = PersonDetection((10, 10, 80, 220), .8, 320, 240)
        first = d._assign_track_ids([person], [None])[0]
        for _ in range(6):
            d._assign_track_ids([], [])
        self.assertNotEqual(d._assign_track_ids([person], [None])[0].track_id, first.track_id)

    def test_appearance_uses_bgr(self):
        d = self.person_detector()
        d._cv2 = Mock(wraps=self.cv2)
        d._cv2.COLOR_BGR2HSV = self.cv2.COLOR_BGR2HSV
        d._appearance(self.image, (10, 10, 110, 210))
        self.assertEqual(d._cv2.cvtColor.call_args.args[1], self.cv2.COLOR_BGR2HSV)

    def test_stair_keeps_capture_time_and_cached_result(self):
        d = StairDetector.__new__(StairDetector)
        d.detect_every_frames, d._frame_count = 2, 0
        d._last, d._net = StairFrame(), None
        d._detect_lines = Mock(return_value=None)
        d.detect(self.image, captured_at=10)
        fresh = d.detect(self.image, captured_at=11)
        self.assertEqual(fresh.captured_at, 11)
        self.assertIs(d.detect(self.image, captured_at=12), fresh)

    def test_invalid_onnx_output_keeps_line_fallback(self):
        d = StairDetector.__new__(StairDetector)
        d._cv2, d._net = self.cv2, Mock()
        d.confidence, d.iou_threshold, d.input_size = 0.55, 0.45, 416
        with patch('src.stair_perception.detect_yolo', side_effect=RuntimeError('output shape')):
            self.assertIsNone(d._detect_model(self.image))
        self.assertFalse(d.model_ready)

    def test_stair_model_needs_overlapping_tread_edges(self):
        d = StairDetector.__new__(StairDetector)
        d.detect_every_frames, d._frame_count = 1, 0
        d._last, d._net = StairFrame(), Mock()
        model = StairDetection((20, 100, 140, 230), .9, -.5, 'model')
        d._detect_model = Mock(return_value=model)
        matching = StairDetection((30, 150, 130, 220), .6, -.5, 'lines')
        disjoint = StairDetection((180, 150, 300, 220), .6, .5, 'lines')
        for lines, source in ((None, None), (disjoint, 'lines'), (matching, 'model+lines')):
            with self.subTest(source=source):
                d._detect_lines = Mock(return_value=lines)
                frame = d.detect(self.image, captured_at=10)
                self.assertEqual(frame.captured_at, 10)
                if source is None:
                    self.assertFalse(frame.stairs)
                else:
                    self.assertEqual(frame.primary_stair.source, source)
                    self.assertEqual(frame.primary_stair.box, model.box if source == 'model+lines' else lines.box)

    def test_camera_passes_capture_timestamp_to_detector(self):
        c = HeadlessCamera(320, 240, 12)
        c._detection_frame, c._frame_at, c._detection_sequence = self.image, 10.0, 1
        def detect(frame, *, captured_at):
            c._stop.set()
            return PersonFrame(captured_at=captured_at)
        c._detector = Mock()
        c._detector.detect.side_effect = detect
        c._detect_loop()
        self.assertEqual(c._person_frame.captured_at, 10.0)

    def test_camera_does_not_lock_stale_person_result(self):
        c = HeadlessCamera(320, 240, 12)
        person = PersonDetection((100, 20, 220, 230), 0.9, 320, 240, 1)
        c._frame, c._frame_at = self.image, 100.0
        c._person_stable_frames = 3
        with patch('src.camera.time.monotonic', return_value=100.0):
            for stamp, expected in ((99.4, False), (99.6, True), (100.1, False)):
                c._person_frame = PersonFrame((person,), stamp)
                self.assertEqual(c.person_ready(), expected)

    def test_detection_error_retries_and_discards_old_person(self):
        c = HeadlessCamera(320, 240, 12)
        c._detection_frame, c._frame_at, c._detection_sequence = self.image, 10.0, 1
        person = PersonDetection((100, 20, 220, 230), .9, 320, 240, 1)
        c._person_frame, c._person_stable_frames = PersonFrame((person,), 9), 3
        def detect(frame, *, captured_at):
            if c._detection_sequence == 1:
                raise RuntimeError('temporary inference failure')
            if c._detection_sequence == 2:
                c._detection_sequence += 1
                return PersonFrame((person,), 9)
            c._stop.set()
            return PersonFrame((person,), captured_at)
        c._detector = Mock(detect=Mock(side_effect=detect))
        def retry(_):
            self.assertIsNone(c._person_frame)
            self.assertEqual(c._person_stable_frames, 0)
            c._detection_sequence += 1
        with patch.object(c._stop, 'wait', side_effect=retry):
            c._detect_loop()
        self.assertEqual(c._detector.detect.call_count, 3)
        self.assertIsNone(c.detection_error)
        self.assertEqual(c._person_stable_frames, 1)

    def test_selected_card_detects_while_disarmed_without_reloading_on_arm(self):
        from src.main import main
        args = Config()
        args.sensor_use_imu = args.sensor_use_foot_fsr = args.sensor_use_depth = False
        args.fall_detection_enabled = False
        camera = Mock(detection_error=None)
        dashboard, raw = Mock(), MagicMock()
        states = [('follow', False), ('follow', False), ('follow', True),
                  ('follow', False), ('terrain', False), ('terrain', True), ('manual', False)]
        index = [-1]
        def control():
            index[0] += 1
            if index[0] == 1:
                person.assert_called_once()
            if index[0] >= len(states):
                raise KeyboardInterrupt
            mode, armed = states[index[0]]
            return dict(mode=mode, armed=armed, runtime_status='Ready')
        dashboard.control_payload.side_effect = control
        safety = Mock(active=False, status='FALL OFF')
        with patch('src.main.Config', return_value=args), \
             patch('src.main.make_backend', return_value=raw), \
             patch('src.main.time.sleep'), \
             patch('src.camera.HeadlessCamera', return_value=camera), \
             patch('src.gait_dashboard.GaitDashboard', return_value=dashboard), \
             patch('src.gait_anomaly.GaitHealthMonitor'), \
             patch('src.fall_safety.FallSafety', return_value=safety), \
             patch('src.person_follow.PersonDetector') as person, \
             patch('src.stair_perception.StairDetector') as stair, \
             patch('src.follow_main.run_follow') as follow, \
             patch('src.stair_main.run_terrain_auto') as terrain, \
             patch('src.main.run_manual') as manual:
            main()
        person.assert_called_once()
        stair.assert_called_once()
        self.assertEqual([call.args[0] for call in camera.set_detector.call_args_list],
                         [person.return_value, stair.return_value, None])
        follow.assert_called_once()
        terrain.assert_called_once()
        manual.assert_not_called()
        raw.send.assert_called_once_with(STANDING, duration_ms=1200, force=True)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.c = PersonFollowController(0.1, 100, 100, 700, 0.3)
        self.c.enable(1)
        self.p = PersonDetection((120, 10, 200, 230), 0.9, 320, 240, 1)

    def test_fresh_target_and_tof_allow_forward(self):
        self.assertGreater(self.c.command(PersonFrame((self.p,), 100), 1000, 1, 100)[0], 0)

    def test_lost_stale_or_future_target_never_commands_motion(self):
        for f in (PersonFrame(), PersonFrame((self.p,), 99), PersonFrame((self.p,), 101)):
            self.assertEqual(self.c.command(f, 1000, 1, 100)[:2], (0, 0))

    def test_other_person_does_not_replace_locked_target(self):
        other = PersonDetection((100, 10, 180, 230), 0.99, 320, 240, 2)
        self.assertGreater(self.c.command(PersonFrame((other, self.p), 100), 1000, 1, 100)[0], 0)
        self.assertEqual(self.c.command(PersonFrame((other,), 100), 1000, 2, 100)[:2], (0, 0))
        self.assertEqual(self.c.target_id, 1)

    def test_missing_tof_and_close_range_stop_without_camera_distance(self):
        for distance in (None, 100, 80):
            with self.subTest(distance=distance):
                self.assertEqual(self.c.command(PersonFrame((self.p,), 100), distance, distance, 100)[:2], (0, 0))

    def test_obstacle_waits_when_all_corridors_blocked(self):
        p = PersonObstaclePlanner(100, 100, 3)
        for d in (None, DepthReading((0,)*64, 1), DepthReading((80,)*64, 2)):
            self.assertEqual(p.update(d, 1, 0)[:2], (0, 0))

    def test_avoid_chooses_clear_side_and_counts_only_new_samples(self):
        p = PersonObstaclePlanner(100, 100, 3)
        values = tuple(80 if c in (3, 4) else 700 if c < 3 else 1000
                       for r in range(8) for c in range(8))
        self.assertEqual(p.update(DepthReading(values, 1), 1, 0)[:2], (0, -1))
        for sample_id in (2, 2, 2, 3):
            p.update(DepthReading((800,)*64, sample_id), 1, 0)
            self.assertEqual(p.direction, -1)
        self.assertEqual(p.update(DepthReading((800,)*64, 4), 1, 0)[:2], (1, 0))
        self.assertEqual(p.direction, 0)


class StairGeometryTests(unittest.TestCase):
    def geometry(self, riser):
        # Synthetic axial depths from two known horizontal planes, not robot logs.
        values = [0] * 64
        pitch = math.radians(45)
        tangent = math.tan(math.radians(22.5))
        for row in range(8):
            level = 0 if row >= 6 else riser
            distance = round((220 - level) / (math.sin(pitch) + ((row + .5) / 4 - 1) * tangent * math.cos(pitch)))
            for col in (3, 4):
                values[(7-row)*8+col] = distance
        det = StairDetection((100, 120, 220, 230), .9, 0, 'test')
        return estimate_stair_geometry(det, DepthReading(tuple(values), 1),
            default_riser_mm=20, min_riser_mm=15, max_riser_mm=25,
            mount_height_mm=220, pitch_down_deg=45, vertical_fov_deg=45, flip_vertical=True)

    def test_known_twenty_mm_planes_resolve_up_and_down(self):
        for height, direction in ((20, 'up'), (-20, 'down')):
            g = self.geometry(height)
            self.assertEqual(g.direction, direction)
            self.assertTrue(g.riser_measured)
            self.assertAlmostEqual(g.riser_height_mm, 20, delta=1)
            self.assertGreater(g.edge_uncertainty_mm, 0)

    def test_flat_ground_and_wrong_risers_do_not_unlock_steps(self):
        for height in (0, 5, 40):
            self.assertFalse(self.geometry(height).riser_measured)


class StairMotionTests(unittest.TestCase):
    def test_configured_height_stride_matrix_is_prevalidated(self):
        for direction in ('up', 'down'):
            for leg in ('left', 'right'):
                for height in (15, 20, 25):
                    for stride in (20, 40, 60, 80, 100, 120):
                        with self.subTest(direction=direction, leg=leg, height=height, stride=stride):
                            e = stepper()
                            e.start(direction, height, stride, leg, 100)
                            self.assertTrue(e.active)

    def test_up_down_both_legs_clearance_pwm_and_standing(self):
        for direction in ('up', 'down'):
            for leg in ('left', 'right'):
                e = stepper()
                e.start(direction, 20, 40, leg, 100)
                total = sum(e.durations.values())
                previous = e.update(100)
                max_delta = 0
                max_lift = {'left': 0, 'right': 0}
                for now in np.arange(100.03, 100+total+.04, .03):
                    pose = e.update(float(now))
                    self.assertTrue(all(500 <= v <= 2500 for v in pose.values()))
                    max_delta = max(max_delta, max(abs(pose[k]-previous[k]) for k in pose))
                    previous = pose
                    for side in max_lift:
                        max_lift[side] = max(max_lift[side], e.feet_mm[side][2])
                self.assertLessEqual(max_delta, 40)
                expected = 38 if direction == 'up' else 18
                for value in max_lift.values():
                    self.assertAlmostEqual(value, expected, delta=.2)
                self.assertFalse(e.active)
                self.assertEqual(e.completed_steps, 1)
                self.assertEqual(previous, STANDING)

    def test_pause_excludes_held_time_and_resume_has_no_jump(self):
        e = stepper()
        e.start('up', 20, 40, 'left', 100)
        pose = e.update(102)
        for now in (102, 105, 110):
            self.assertEqual(e.update(now, paused=True), pose)
        self.assertEqual(e.update(111, paused=False), pose)
        self.assertNotEqual(e.update(111.1), pose)

    def test_unreachable_or_invalid_steps_are_rejected(self):
        for height, stride in ((20, 400), (float('nan'), 40), (20, -1)):
            e = stepper()
            with self.assertRaises(ValueError):
                e.start('up', height, stride, 'left', 100)
            self.assertFalse(e.active)


class TerrainRuntimeTests(unittest.TestCase):
    def run_frames(self, readings, controls=None, config=None, camera_frames=None, engine=None, fall=False):
        frames, calls = [], []
        clock = [100.0]
        controls = controls or [{} for _ in readings]
        index = [-1]
        class Dashboard:
            def control_state(self, mode):
                calls.append(mode)
                index[0] += 1
                return WebControlState(armed=index[0] < len(controls), mode='terrain',
                    **(controls[index[0]] if index[0] < len(controls) else {}))
            def publish(self, **kwargs):
                frames.append(kwargs)
            def set_runtime(self, *args):
                pass
            def disarm(self, *args):
                pass
        camera = Mock()
        camera.detection_error = None
        camera.stair_frame.side_effect = lambda: camera_frames[index[0]] if camera_frames else None
        sensors = Mock()
        sensors.read.side_effect = lambda: readings[index[0]]
        raw = Mock()
        backend = PriorityBackend(raw, Config.fall_arm_forward_pwm)
        safety = SimpleNamespace(active=fall, reference=(0, 0), status='FALL READY', reason='test')
        if fall:
            backend.trigger_fall(30)
        fake_time = SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda s: clock.__setitem__(0, clock[0]+s))
        real_factory = StairStepEngine
        with patch('src.stair_main.time', fake_time), \
             patch('src.stair_main.StairStepEngine', side_effect=lambda **kw: engine or real_factory(**kw)):
            run_terrain_auto(config or Config(), Dashboard(), camera, backend, sensors, safety)
        self.assertTrue(all(mode == 'terrain' for mode in calls))
        return frames

    def test_balance_without_fsr_requires_calibrated_imu(self):
        good = IMUReading(4, -5, 0, gyro_cal=3)
        bad = IMUReading(4, -5, 0, gyro_cal=0)
        frames = self.run_frames([SensorSnapshot(x, None, None) for x in (good, bad, None, good)])
        self.assertNotEqual(frames[0]['pose'], STANDING)
        self.assertEqual(frames[1]['pose'], STANDING)
        self.assertEqual(frames[2]['pose'], STANDING)
        self.assertNotEqual(frames[3]['pose'], STANDING)

    def test_uncalibrated_stair_never_starts(self):
        frames = self.run_frames([SensorSnapshot(IMUReading(0, 0, 0, gyro_cal=3), None, None)], [{'stair_toggle': True}])
        self.assertEqual(frames[0]['pose'], STANDING)
        self.assertIn('CALIBRATE', frames[0]['status'])

    def test_stair_confirmation_needs_four_fresh_camera_tof_pairs(self):
        config = Config(stair_geometry_calibrated=True, stair_foot_toe_mm=20,
                        stair_foot_heel_mm=20, stair_foot_width_mm=30)
        geometry = StairGeometry('up', .9, 70, 20, 0, 'test', 5, True)
        det = StairDetection((100, 100, 220, 230), .9, 0, 'test')
        imu = IMUReading(0, 0, 0, gyro_cal=3)
        for fresh in (False, True):
            reads = [SensorSnapshot(imu, None, DepthReading((800,)*64, i+1 if fresh else 1)) for i in range(4)]
            images = [StairFrame((det,), 100+i*.03 if fresh else 100) for i in range(4)]
            with patch('src.stair_main.estimate_stair_geometry', return_value=geometry):
                frames = self.run_frames(reads, [{'stair_toggle': True}, {}, {}, {}],
                                         config=config, camera_frames=images)
            self.assertEqual([f['gait']['terrain']['stable_frames'] for f in frames],
                             [1, 2, 3, 4] if fresh else [1, 1, 1, 1])
            self.assertEqual(frames[-1]['gait']['phase'], 'shift' if fresh else 'terrain-wait')

    def test_stair_selection_does_not_overlay_standing_balance(self):
        frames = self.run_frames([SensorSnapshot(IMUReading(4, -5, 0, gyro_cal=3), None, None)], [{'stair_toggle': True}])
        self.assertEqual(frames[0]['pose'], STANDING)
        self.assertIn('IMU PAUSED', frames[0]['balance_status'])

    def test_missing_sensors_pause_step_and_do_not_auto_resume(self):
        e = stepper()
        e.start('up', 20, 40, 'left', 98)
        good = SensorSnapshot(IMUReading(0, 0, 0, gyro_cal=3), None, DepthReading((800,)*64, 1))
        frames = self.run_frames([SensorSnapshot(None, None, None), good, good],
            camera_frames=[StairFrame(captured_at=100)]*3, engine=e)
        self.assertIn('IMU LOST', frames[0]['status'])
        self.assertIn('PRESS U', frames[1]['status'])
        self.assertEqual(frames[0]['pose'], frames[2]['pose'])
        self.assertTrue(e.paused)

    def test_camera_tof_and_tilt_each_pause_active_step(self):
        imu = IMUReading(0, 0, 0, gyro_cal=3)
        depth = DepthReading((800,)*64, 1)
        for reading, image, reason in (
            (SensorSnapshot(imu, None, None), StairFrame(captured_at=100), 'TOF LOST'),
            (SensorSnapshot(imu, None, depth), None, 'CAMERA LOST'),
            (SensorSnapshot(imu, None, depth), StairFrame(captured_at=99), 'CAMERA LOST'),
            (SensorSnapshot(imu, None, depth), StairFrame(captured_at=101), 'CAMERA LOST'),
            (SensorSnapshot(IMUReading(9, 0, 0, gyro_cal=3), None, depth), StairFrame(captured_at=100), 'TILT LIMIT'),
        ):
            with self.subTest(reason=reason):
                e = stepper()
                e.start('up', 20, 40, 'left', 98)
                frames = self.run_frames([reading], camera_frames=[image], engine=e)
                self.assertIn(reason, frames[0]['status'])
                self.assertTrue(e.paused)

    def test_fall_takes_priority_over_stair_and_balance(self):
        e = stepper()
        e.start('up', 20, 40, 'left', 98)
        frame = self.run_frames([SensorSnapshot(IMUReading(0, 0, 0, gyro_cal=3), None, None)],
                                [{'stair_toggle': True}], engine=e, fall=True)[0]
        self.assertEqual(frame['gait']['phase'], 'fall')
        self.assertEqual(frame['pose'][11], STANDING[11] - Config.fall_arm_forward_pwm)
        self.assertEqual(frame['pose'][22], STANDING[22] + Config.fall_arm_forward_pwm)
        self.assertFalse(e.active)

    def test_terrain_exit_preserves_reset_for_manual(self):
        dashboard = GaitDashboard()
        code, _ = dashboard.update_control({'client_id': 'test-client', 'sequence': 1, 'armed': True,
                                          'mode': 'manual', 'actions': ['reset']})
        self.assertEqual(code, 200)
        backend = PriorityBackend(Mock(), Config.fall_arm_forward_pwm)
        safety = SimpleNamespace(active=False, reference=(0, 0))
        with patch('src.stair_main.time.sleep'):
            run_terrain_auto(Config(), dashboard, Mock(), backend, None, safety)
        self.assertTrue(dashboard.control_state('manual').reset)


class SquatTests(unittest.TestCase):
    def engine(self, **overrides):
        c = Config()
        params = dict(dt=c.update_ms / 1000, depth_mm=c.manual_squat_depth_mm,
                      forward_mm=c.manual_squat_forward_mm,
                      arm_forward_pwm=c.manual_squat_arm_forward_pwm,
                      arm_raise_s=c.manual_squat_arm_raise_s,
                      transition_s=c.manual_squat_transition_s)
        return SquatEngine(**{**params, **overrides})

    def test_arms_raise_before_legs_lower(self):
        e = self.engine()
        e.toggle()
        for _ in range(math.floor(e.arm_raise_s / e.dt)):
            p = e.update()
            self.assertEqual(e.depth_mm, 0)
            self.assertTrue(all(p[sid] == STANDING[sid] for sid in DIR))
        self.assertLess(p[11], STANDING[11])
        self.assertGreater(p[22], STANDING[22])

    def test_forward_offset_uses_the_same_progress_as_depth(self):
        e = self.engine(forward_mm=20)
        e.toggle()
        with patch('src.walking_engine.compute_pose', wraps=compute_pose) as solve:
            for _ in range(math.ceil((e.arm_raise_s + e.transition_s) / e.dt)):
                e.update()
                if e.depth_mm > 0.1:
                    self.assertAlmostEqual(solve.call_args.args[0], 20 * e.depth_mm / e.max_depth_mm)

    def test_toggle_and_reset_return_exact_standing(self):
        for elapsed in (0.1, 0.6, 1.5, 3.0):
            with self.subTest(elapsed=elapsed):
                e = self.engine()
                e.toggle()
                for _ in range(math.ceil(elapsed / e.dt)):
                    p = e.update()
                    self.assertTrue(all(500 <= v <= 2500 for v in p.values()))
                    for left, right in ((13, 20), (14, 19), (15, 18)):
                        self.assertEqual(p[left] + p[right], STANDING[left] + STANDING[right])
                    for sid in (12, 16, 17, 21):
                        self.assertEqual(p[sid], STANDING[sid])
                if elapsed >= e.arm_raise_s + e.transition_s:
                    self.assertEqual(e.phase, 'squat-hold')
                    self.assertEqual(e.depth_mm, e.max_depth_mm)
                    self.assertEqual(e.update(), p)
                e.toggle()
                for _ in range(math.ceil((e.arm_raise_s + e.transition_s) / e.dt) + 2):
                    p = e.update()
                    self.assertTrue(all(500 <= v <= 2500 for v in p.values()))
                self.assertEqual(p, STANDING)
                self.assertFalse(e.active)
                e.toggle()
                for _ in range(40):
                    e.update()
                e.reset()
                self.assertEqual(e.update(), STANDING)


if __name__ == '__main__':
    unittest.main()
