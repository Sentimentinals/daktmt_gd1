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
            print(f"[camera] Headless camera unavailable: missing {exc.name}.")
            return False

        self._cv2 = cv2
        camera = None
        try:
            camera = Picamera2()
            camera.configure(
                camera.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.width, self.height)},
                    controls={"FrameRate": self.fps},
                    transform=Transform(hflip=True, vflip=True),
                )
            )
            camera.start()
        except Exception as exc:
            if camera is not None:
                try:
                    camera.close()
                except Exception:
                    pass
            print(f"[camera] Headless camera unavailable: {exc}")
            return False

        self.camera = camera
        self._stop.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="headless-camera",
            daemon=True,
        )
        self._detector_thread = threading.Thread(
            target=self._detect_loop,
            name="person-detector",
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
            self._person_stable_frames = 0
            self._last_person_timestamp = None
            self._person_ignored = False
            self._jpeg_sequence = -1

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.camera.capture_array("main")
            except Exception as exc:
                if not self._stop.is_set():
                    print(f"[camera] Capture stopped: {exc}")
                break
            with self._lock:
                self._frame = frame.copy()
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
                person_frame = detector.detect(frame)
            except Exception as exc:
                print(f"[camera] Person detection stopped: {exc}")
                self.set_detector(None)
                continue
            with self._lock:
                if detector is not self._detector:
                    continue
                self._person_frame = person_frame
                is_new = person_frame.captured_at != self._last_person_timestamp
                if person_frame.single_person is not None and is_new:
                    self._person_stable_frames += 1
                elif person_frame.single_person is None and is_new:
                    self._person_stable_frames = 0
                    self._person_ignored = False
                self._last_person_timestamp = person_frame.captured_at
                self._jpeg_sequence = -1

    def person_frame(self):
        with self._lock:
            return self._person_frame

    def jpeg_frame(self, quality: int = 68) -> bytes | None:
        if self._cv2 is None:
            return None
        with self._lock:
            if self._jpeg_sequence == self._frame_sequence:
                return self._jpeg_frame
            frame = None if self._frame is None else self._frame.copy()
            person_frame = self._person_frame
            sequence = self._frame_sequence
        if frame is None:
            return None
        if person_frame is not None:
            for person in person_frame.people:
                x1, y1, x2, y2 = person.box
                self._cv2.rectangle(frame, (x1, y1), (x2, y2), (69, 208, 154), 2)
                self._cv2.putText(
                    frame,
                    f"PERSON {person.confidence:.2f}",
                    (x1, max(18, y1 - 7)),
                    self._cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (69, 208, 154),
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
        with self._lock:
            return (
                self._person_frame is not None
                and self._person_frame.single_person is not None
                and self._person_stable_frames >= self.stable_frames
                and not self._person_ignored
            )

    def ignore_person(self) -> None:
        with self._lock:
            self._person_ignored = True

    def close(self) -> None:
        was_running = self.camera is not None
        self._stop.set()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1.0)
        if self._detector_thread is not None:
            self._detector_thread.join(timeout=1.0)
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass
            try:
                self.camera.close()
            except Exception:
                pass
        self.camera = None
        self._capture_thread = None
        self._detector_thread = None
        with self._lock:
            self._frame = None
            self._jpeg_frame = None
            self._jpeg_sequence = -1
        if was_running:
            print("[camera] Headless camera stopped.")
