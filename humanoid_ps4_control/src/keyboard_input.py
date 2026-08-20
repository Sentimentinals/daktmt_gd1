from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class KeyboardState:
    forward: float = 0.0
    turn: float = 0.0
    side: float = 0.0
    auto_toggle: bool = False
    single_support: bool = False
    dance: bool = False
    getup: bool = False
    getup_back: bool = False
    stop: bool = False
    reset: bool = False
    follow: bool = False
    ignore_person: bool = False
    squat: bool = False
    menu: bool = False
    quit: bool = False


class KeyboardReader:
    """Poll keyboard commands through a focused pygame window."""

    def __init__(
        self,
        poll_rate_hz: int = 50,
        caption: str = "Humanoid Keyboard Control",
        controls: str | None = None,
    ) -> None:
        self.poll_rate_hz = max(1, poll_rate_hz)
        self.caption = caption
        self.controls = controls
        self._pygame_ready = False

    def init(self) -> bool:
        try:
            import pygame
        except ImportError:
            print("[KeyboardReader] pygame not installed. Run: pip install pygame")
            return False

        pygame.init()
        if pygame.display.get_surface() is None:
            pygame.display.set_mode((520, 170))
        pygame.display.set_caption(self.caption)
        self._pygame_ready = True
        if self.controls:
            print(f"[KeyboardReader] {self.controls}")
        return True

    @staticmethod
    def _axis(positive: bool, negative: bool) -> float:
        if positive == negative:
            return 0.0
        return 1.0 if positive else -1.0

    def poll(self):
        if not self._pygame_ready and not self.init():
            yield KeyboardState(quit=True)
            return

        import pygame

        clock = pygame.time.Clock()
        while True:
            try:
                events = pygame.event.get()
            except pygame.error as exc:
                print(f"[KeyboardReader] pygame event error: {exc}")
                yield KeyboardState(quit=True)
                return

            if any(event.type == pygame.QUIT for event in events):
                yield KeyboardState(quit=True)
                return

            keys = pygame.key.get_pressed()
            if keys[pygame.K_q]:
                yield KeyboardState(quit=True)
                return

            yield KeyboardState(
                forward=self._axis(keys[pygame.K_UP], keys[pygame.K_DOWN]),
                turn=self._axis(keys[pygame.K_LEFT], keys[pygame.K_RIGHT]),
                side=self._axis(keys[pygame.K_j], keys[pygame.K_k]),
                auto_toggle=bool(keys[pygame.K_v]),
                single_support=bool(keys[pygame.K_x]),
                dance=bool(keys[pygame.K_l] or keys[pygame.K_m]),
                getup=bool(keys[pygame.K_g]),
                getup_back=bool(keys[pygame.K_b]),
                stop=bool(keys[pygame.K_c]),
                reset=bool(keys[pygame.K_e] or keys[pygame.K_t]),
                follow=bool(keys[pygame.K_y]),
                ignore_person=bool(keys[pygame.K_n]),
                squat=bool(keys[pygame.K_r]),
                menu=bool(keys[pygame.K_o] or keys[pygame.K_ESCAPE]),
            )
            clock.tick(self.poll_rate_hz)

    def quit(self) -> None:
        try:
            import pygame

            pygame.quit()
        except Exception:
            pass


class LiveCameraPreview:
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
        self.screen = None
        self.font = None
        self._frame = None
        self._frame_sequence = 0
        self._jpeg_frame = None
        self._jpeg_sequence = -1
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._detector_thread = None
        self._cv2 = None
        self._pygame = None
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
            import pygame
        except ImportError:
            print("[camera] Live preview unavailable: missing pygame.")
            return False

        if not pygame.get_init():
            pygame.init()
        self._pygame = pygame
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Humanoid Live Control")
        self.font = pygame.font.Font(None, 28)

        try:
            import cv2
            from libcamera import Transform
            from picamera2 import Picamera2
        except ImportError as exc:
            print(f"[camera] Live preview unavailable: missing {exc.name}.")
            self.render(f"CAMERA UNAVAILABLE: {exc.name}")
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
            print(f"[camera] Live preview unavailable: {exc}")
            self.render("CAMERA START FAILED - KEYBOARD READY")
            return False

        self.camera = camera
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="live-camera", daemon=True)
        self._thread.start()
        if self._detector is not None:
            self._detector_thread = threading.Thread(
                target=self._detect_loop,
                name="person-detector",
                daemon=True,
            )
            self._detector_thread.start()
        print("[camera] Live preview started.")
        return True

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
                if self._detector is not None:
                    self._detection_frame = self._frame
                    self._detection_sequence += 1

    def _detect_loop(self) -> None:
        observed_sequence = -1
        while not self._stop.is_set():
            with self._lock:
                sequence = self._detection_sequence
                frame = self._detection_frame
            if frame is None or sequence == observed_sequence:
                time.sleep(0.01)
                continue
            observed_sequence = sequence
            try:
                person_frame = self._detector.detect(frame)
            except Exception as exc:
                print(f"[camera] Person detection stopped: {exc}")
                self._detector = None
                return
            with self._lock:
                self._person_frame = person_frame
                is_new_detection = person_frame.captured_at != self._last_person_timestamp
                if person_frame.single_person is not None and is_new_detection:
                    self._person_stable_frames += 1
                elif person_frame.single_person is None and is_new_detection:
                    self._person_stable_frames = 0
                    self._person_ignored = False
                self._last_person_timestamp = person_frame.captured_at

    def person_frame(self):
        with self._lock:
            return self._person_frame

    def jpeg_frame(self, quality: int = 72) -> bytes | None:
        if self._cv2 is None:
            return None
        with self._lock:
            if self._jpeg_sequence == self._frame_sequence:
                return self._jpeg_frame
            frame = None if self._frame is None else self._frame.copy()
            sequence = self._frame_sequence
        if frame is None:
            return None
        ok, encoded = self._cv2.imencode(
            ".jpg",
            frame,
            [self._cv2.IMWRITE_JPEG_QUALITY, max(40, min(90, quality))],
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

    def render(self, status: str, follow_enabled: bool = False) -> None:
        if self.screen is None or self._pygame is None or self.font is None:
            return
        with self._lock:
            frame = self._frame
        if frame is None:
            self.screen.fill((10, 14, 18))
        elif self._cv2 is not None:
            rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
            surface = self._pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            self.screen.blit(surface, (0, 0))
        person_frame = self.person_frame()
        if person_frame is not None:
            for person in person_frame.people:
                x1, y1, x2, y2 = person.box
                self._pygame.draw.rect(
                    self.screen,
                    (58, 210, 148),
                    self._pygame.Rect(x1, y1, x2 - x1, y2 - y1),
                    2,
                )
                confidence = self.font.render(
                    f"PERSON {person.confidence:.2f}",
                    True,
                    (58, 210, 148),
                )
                self.screen.blit(confidence, (x1, max(2, y1 - 24)))
        label = self.font.render(status, True, (238, 245, 248))
        panel = self._pygame.Surface((label.get_width() + 24, label.get_height() + 12), self._pygame.SRCALPHA)
        panel.fill((10, 14, 18, 205))
        self.screen.blit(panel, (12, 12))
        self.screen.blit(label, (24, 18))
        prompt = None
        prompt_color = (245, 190, 72)
        if follow_enabled:
            prompt = "FOLLOW ON - N/C: STOP"
            prompt_color = (58, 210, 148)
        elif self.person_ready():
            prompt = "PERSON DETECTED - Y: FOLLOW / N: IGNORE"
        elif person_frame is not None and len(person_frame.people) > 1:
            prompt = "MULTIPLE PEOPLE - FOLLOW DISABLED"
        if prompt is not None:
            prompt_label = self.font.render(prompt, True, prompt_color)
            prompt_panel = self._pygame.Surface(
                (prompt_label.get_width() + 24, prompt_label.get_height() + 12),
                self._pygame.SRCALPHA,
            )
            prompt_panel.fill((10, 14, 18, 220))
            y = self.height - prompt_panel.get_height() - 12
            self.screen.blit(prompt_panel, (12, y))
            self.screen.blit(prompt_label, (24, y + 6))
        self._pygame.display.flip()

    def close(self) -> None:
        was_running = self.camera is not None
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
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
        self._thread = None
        self._detector_thread = None
        with self._lock:
            self._frame = None
            self._jpeg_frame = None
            self._jpeg_sequence = -1
        if was_running:
            print("[camera] Live preview stopped.")
