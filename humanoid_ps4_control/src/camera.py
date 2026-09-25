from __future__ import annotations

import threading
import time


class HeadlessCamera:
    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        detector=None,
        stable_frames: int = 3,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self.camera = None
        self._frame = None
        self._frame_at = 0.0
        self.error = None
        self._frame_sequence = 0
        self._jpeg_frame = None
        self._jpeg_sequence = -1
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._capture_thread = None
        self._detector_thread = None
        self._cv2 = None
        self._detector = detector
        self.stable_frames = max(1, stable_frames)
        self._person_frame = None
        self._stair_frame = None
        self._person_stable_frames = 0
        self._last_person_timestamp = None
        self._person_ignored = False
        self._detection_frame = None
        self._detection_sequence = 0

    def start(self) -> bool:
        try:
            import cv2
            from libcamera import Transform
            from picamera2 import Picamera2
        except ImportError as exc:
            self.error = f"Missing camera dependency: {exc.name}"
            print(f"[camera] {self.error}. Install python3-picamera2/python3-opencv via apt "
                  "and enable system-site-packages in the venv.")
            return False

        self._cv2 = cv2
        camera = None
        stage = "enumeration"
        try:
            if not Picamera2.global_camera_info():
                raise RuntimeError("No camera detected by libcamera. Run rpicam-hello --list-cameras.")
            stage = "open"
            camera = Picamera2()
            stage = "configuration"
            camera.configure(
                camera.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.width, self.height)},
                    controls={"FrameRate": self.fps},
                    transform=Transform(hflip=True, vflip=True),
                )
            )
            stage = "start"
            camera.start()
        except Exception as exc:
            self.error = f"Camera {stage}: {exc}"
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            print(f"[camera] {self.error}")
            return False

        self.camera = camera
        self.error = None
        self._stop.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="headless-camera",
            daemon=True,
        )
        self._detector_thread = threading.Thread(
            target=self._detect_loop,
            name="vision-detector",
            daemon=True,
        )
        self._capture_thread.start()
        self._detector_thread.start()
        print("[camera] Headless camera started.")
        return True

    def set_detector(self, detector, stable_frames: int | None = None) -> None:
        with self._lock:
            self._detector = detector
            if stable_frames is not None:
                self.stable_frames = max(1, stable_frames)
            self._person_frame = None
            self._stair_frame = None
            self._person_stable_frames = 0
            self._last_person_timestamp = None
            self._person_ignored = False
            self._jpeg_sequence = -1

    def _capture_loop(self) -> None:
        failures = 0
        while not self._stop.is_set():
            try:
                frame = self.camera.capture_array("main")
            except Exception as exc:
                if self._stop.is_set():
                    break
                failures += 1
                self.error = str(exc)
                if failures == 1 or failures % 20 == 0:
                    print(f"[camera] Capture retry {failures}: {exc}")
                self._stop.wait(min(1.0, 0.1 * failures))
                continue
            failures = 0
            self.error = None
            with self._lock:
                self._frame = frame.copy()
                self._frame_at = time.monotonic()
                self._frame_sequence += 1
                self._detection_frame = self._frame
                self._detection_sequence += 1

    def _detect_loop(self) -> None:
        observed_sequence = -1
        while not self._stop.is_set():
            with self._lock:
                detector = self._detector
                sequence = self._detection_sequence
                frame = self._detection_frame
            if detector is None or frame is None or sequence == observed_sequence:
                time.sleep(0.01)
                continue
            observed_sequence = sequence
            try:
                detection = detector.detect(frame)
            except Exception as exc:
                print(f"[camera] Detection stopped: {exc}")
                self.set_detector(None)
                continue
            with self._lock:
                if detector is not self._detector:
                    continue
                if hasattr(detection, "people"):
                    self._person_frame = detection
                    self._stair_frame = None
                    is_new = detection.captured_at != self._last_person_timestamp
                    if detection.single_person is not None and is_new:
                        self._person_stable_frames += 1
                    elif detection.single_person is None and is_new:
                        self._person_stable_frames = 0
                        self._person_ignored = False
                    self._last_person_timestamp = detection.captured_at
                elif hasattr(detection, "stairs"):
                    self._person_frame = None
                    self._stair_frame = detection
                self._jpeg_sequence = -1

    @property
    def ready(self) -> bool:
        with self._lock:
            return (
                not self._stop.is_set()
                and self._frame is not None
                and time.monotonic() - self._frame_at < 2.0
            )

    def person_frame(self):
        if not self.ready:
            return None
        with self._lock:
            return self._person_frame

    def stair_frame(self):
        if not self.ready:
            return None
        with self._lock:
            return self._stair_frame

    def jpeg_frame(self, quality: int = 68) -> bytes | None:
        if self._cv2 is None or not self.ready:
            return None
        with self._lock:
            if self._jpeg_sequence == self._frame_sequence:
                return self._jpeg_frame
            frame = None if self._frame is None else self._frame.copy()
            person_frame = self._person_frame
            stair_frame = self._stair_frame
            sequence = self._frame_sequence
        if frame is None:
            return None
        if person_frame is not None:
            for person in person_frame.people:
                x1, y1, x2, y2 = person.box
                self._cv2.rectangle(frame, (x1, y1), (x2, y2), (69, 208, 154), 2)
                self._cv2.putText(
                    frame,
                    f"PERSON #{person.track_id} CONF {person.confidence:.0%}",
                    (x1, max(18, y1 - 7)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (69, 208, 154),
                    1,
                    self._cv2.LINE_AA,
                )
        if stair_frame is not None:
            for stair in stair_frame.stairs:
                x1, y1, x2, y2 = stair.box
                self._cv2.rectangle(frame, (x1, y1), (x2, y2), (78, 166, 255), 2)
                label = f"STAIR {stair.confidence:.2f}"
                self._cv2.putText(
                    frame,
                    label,
                    (x1, max(18, y1 - 7)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (78, 166, 255),
                    1,
                    self._cv2.LINE_AA,
                )
        ok, encoded = self._cv2.imencode(
            ".jpg",
            frame,
            [self._cv2.IMWRITE_JPEG_QUALITY, max(40, min(85, quality))],
        )
        if not ok:
            return None
        jpeg = encoded.tobytes()
        with self._lock:
            if sequence >= self._jpeg_sequence:
                self._jpeg_frame = jpeg
                self._jpeg_sequence = sequence
        return jpeg

    def person_ready(self) -> bool:
        if not self.ready:
            return False
        with self._lock:
            return (
                self._person_frame is not None
                and time.monotonic() - self._person_frame.captured_at < 0.8
                and self._person_frame.single_person is not None
                and self._person_stable_frames >= self.stable_frames
                and not self._person_ignored
            )

    def ignore_person(self) -> None:
        with self._lock:
            self._person_ignored = True

    @staticmethod
    def _call_with_timeout(callback, timeout_s: float) -> bool:
        done = threading.Event()

        def run() -> None:
            try:
                callback()
            except Exception:
                pass
            finally:
                done.set()

        threading.Thread(target=run, name="camera-shutdown", daemon=True).start()
        return done.wait(timeout_s)

    def close(self) -> None:
        was_running = self.camera is not None
        self._stop.set()
        camera = self.camera
        stopped = camera is None or self._call_with_timeout(camera.stop, 1.0)
        if camera is not None and not stopped:
            print("[camera] Stop timed out; leaving the stalled pipeline for process exit.")
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        if self._detector_thread is not None:
            self._detector_thread.join(timeout=1.0)
        if camera is not None and stopped and not self._call_with_timeout(camera.close, 1.0):
            print("[camera] Close timed out; continuing shutdown.")
        self.camera = None
        self._capture_thread = None
        self._detector_thread = None
        with self._lock:
            self._frame = None
            self._frame_at = 0.0
            self._detection_frame = None
            self._person_frame = None
            self._stair_frame = None
            self._jpeg_frame = None
            self._jpeg_sequence = -1
        if was_running:
            print("[camera] Headless camera stopped.")
