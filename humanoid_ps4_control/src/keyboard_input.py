from __future__ import annotations

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
