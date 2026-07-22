from __future__ import annotations

import threading
from dataclasses import dataclass


@dataclass(frozen=True)
class KeyboardState:
    forward: float = 0.0
    turn: float = 0.0
    side: float = 0.0
    handshake: bool = False
    single_support: bool = False
    dance: bool = False
    getup: bool = False
    getup_back: bool = False
    stop: bool = False
    reset: bool = False
    menu: bool = False
    quit: bool = False


class KeyboardReader:
    """Poll keyboard commands through a focused pygame window."""

    def __init__(self, poll_rate_hz: int = 50) -> None:
        self.poll_rate_hz = max(1, poll_rate_hz)
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
        pygame.display.set_caption("Humanoid Keyboard Control")
        self._pygame_ready = True
        print(
            "[KeyboardReader] W/S walk, A/D turn, J/K side, V handshake, "
            "X single support, L/M dance, G/B get-up, C stop, E/T reset, O/Esc menu."
        )
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
                forward=self._axis(keys[pygame.K_w] or keys[pygame.K_UP], keys[pygame.K_s] or keys[pygame.K_DOWN]),
                turn=self._axis(keys[pygame.K_a] or keys[pygame.K_LEFT], keys[pygame.K_d] or keys[pygame.K_RIGHT]),
                side=self._axis(keys[pygame.K_j], keys[pygame.K_k]),
                handshake=bool(keys[pygame.K_v]),
                single_support=bool(keys[pygame.K_x]),
                dance=bool(keys[pygame.K_l] or keys[pygame.K_m]),
                getup=bool(keys[pygame.K_g]),
                getup_back=bool(keys[pygame.K_b]),
                stop=bool(keys[pygame.K_c]),
                reset=bool(keys[pygame.K_e] or keys[pygame.K_t]),
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
    def __init__(self, width: int, height: int, fps: int) -> None:
        self.width = width
        self.height = height
        self.fps = max(1, fps)
        self.camera = None
        self.screen = None
        self.font = None
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._cv2 = None
        self._pygame = None

    def start(self) -> bool:
        try:
            import cv2
            import pygame
            from picamera2 import Picamera2
        except ImportError as exc:
            print(f"[camera] Live preview unavailable: missing {exc.name}.")
            return False

        self._cv2 = cv2
        self._pygame = pygame
        camera = None
        try:
            camera = Picamera2()
            camera.configure(
                camera.create_preview_configuration(
                    main={"format": "RGB888", "size": (self.width, self.height)},
                    controls={"FrameRate": self.fps},
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
            return False

        self.camera = camera
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("Humanoid Live Control")
        self.font = pygame.font.Font(None, 28)
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, name="live-camera", daemon=True)
        self._thread.start()
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
                self._frame = frame

    def render(self, status: str) -> None:
        if self.screen is None or self._pygame is None or self._cv2 is None:
            return
        with self._lock:
            frame = self._frame
        if frame is None:
            self.screen.fill((10, 14, 18))
        else:
            rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
            surface = self._pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
            self.screen.blit(surface, (0, 0))
        label = self.font.render(status, True, (238, 245, 248))
        panel = self._pygame.Surface((label.get_width() + 24, label.get_height() + 12), self._pygame.SRCALPHA)
        panel.fill((10, 14, 18, 205))
        self.screen.blit(panel, (12, 12))
        self.screen.blit(label, (24, 18))
        self._pygame.display.flip()

    def close(self) -> None:
        was_running = self.camera is not None
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
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
        if was_running:
            print("[camera] Live preview stopped.")
