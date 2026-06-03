"""
DualShock 4 reader via pygame.

Windows DS4 button indices commonly seen by pygame:
    0  Cross      1  Circle     2  Square      3  Triangle
    4  L1         5  R1         6  L2          7  R2
    8  Share      9  Options    10 L3
    11 D-pad Up   12 D-pad Down 13 D-pad Left 14 D-pad Right

Hat input is also supported for Linux/macOS. Keyboard fallback remains active
even when a joystick is connected:
    T/E reset, W/Up forward, S/Down backward, A/Left side left,
    D/Right side right, J/K side walk, L/M arm dance/L1,
    X single-leg support/Cross, G get-up/R1 using selected mode, B get-up back, C stop, Q quit.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple


@dataclass
class ControllerState:
    buttons: Dict[int, bool] = field(default_factory=dict)
    axes: Dict[int, float] = field(default_factory=dict)
    hats: Dict[int, Tuple[int, int]] = field(default_factory=dict)
    quit: bool = False

    def button(self, idx: int) -> bool:
        return self.buttons.get(idx, False)

    def axis(self, idx: int) -> float:
        return self.axes.get(idx, 0.0)

    def hat(self, idx: int = 0) -> Tuple[int, int]:
        return self.hats.get(idx, (0, 0))

    def dpad_up(self) -> bool:
        return self.hat()[1] == 1 or self.buttons.get(11, False)

    def dpad_down(self) -> bool:
        return self.hat()[1] == -1 or self.buttons.get(12, False)

    def dpad_left(self) -> bool:
        return self.hat()[0] == -1 or self.buttons.get(13, False)

    def dpad_right(self) -> bool:
        return self.hat()[0] == 1 or self.buttons.get(14, False)

    def signed_axis(self, idx: int, sign: float = 1.0) -> float:
        return max(-1.0, min(1.0, self.axis(idx) * sign))


class PS4Reader:
    BTN_CROSS = 0
    BTN_CIRCLE = 1
    BTN_SQUARE = 2
    BTN_TRIANGLE = 3
    BTN_L1, BTN_R1, BTN_L2, BTN_R2 = 4, 5, 6, 7
    BTN_SHARE = 8
    BTN_OPTIONS = 9
    BTN_L3 = 10
    BTN_DPAD_UP, BTN_DPAD_DOWN, BTN_DPAD_LEFT, BTN_DPAD_RIGHT = 11, 12, 13, 14
    BTN_GETUP_BACK = 15

    def __init__(
        self,
        joystick_index: int = 0,
        fallback_keys: bool = True,
        poll_rate_hz: int = 50,
        deadzone: float = 0.08,
        debug: bool = False,
    ) -> None:
        self.joystick_index = joystick_index
        self.fallback_keys = fallback_keys
        self.poll_interval = 1.0 / poll_rate_hz
        self.deadzone = deadzone
        self.debug = debug
        self._joystick = None
        self._has_joystick = False
        self._pygame_ready = False

    def init(self) -> bool:
        try:
            import pygame  # type: ignore
        except ImportError:
            print("[PS4Reader] pygame not installed. Run: pip install pygame")
            return False

        pygame.init()
        try:
            pygame.display.set_mode((400, 200))
            pygame.display.set_caption("PS4 Robot Control (click to focus)")
        except Exception:
            pass

        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            if self.fallback_keys:
                print(
                    "[PS4Reader] No joystick. Keyboard fallback active: "
                    "W/Up forward, S/Down backward, A/Left and D/Right side walk, J/K side walk, "
                    "L/M dance/L1, X single-leg support/Cross, G get-up/R1, B get-up back, C stop, Q quit."
                )
            else:
                print("[PS4Reader] No joystick detected.")
                return False
        else:
            if self.joystick_index >= joystick_count:
                print(f"[PS4Reader] joystick_index={self.joystick_index} unavailable; using 0 of {joystick_count}.")
                self.joystick_index = 0
            self._joystick = pygame.joystick.Joystick(self.joystick_index)
            self._joystick.init()
            print(
                f"[PS4Reader] Connected: {self._joystick.get_name()} "
                f"(axes={self._joystick.get_numaxes()}, "
                f"buttons={self._joystick.get_numbuttons()}, hats={self._joystick.get_numhats()})"
            )
            if self.fallback_keys:
                print("[PS4Reader] Keyboard override active.")
            self._has_joystick = True

        self._pygame_ready = True
        return self._has_joystick

    def poll(self):
        """Yield ControllerState at poll_rate_hz until QUIT or Q."""
        if not self._pygame_ready:
            self.init()

        import pygame

        clock = pygame.time.Clock()
        buttons: Dict[int, bool] = {}
        axes: Dict[int, float] = {}
        hats: Dict[int, Tuple[int, int]] = {}
        kb_hat: Tuple[int, int] = (0, 0)

        while True:
            try:
                pygame.event.pump()
            except pygame.error as exc:
                print(f"[PS4Reader] pygame event error: {exc}")
                yield ControllerState(quit=True)
                return

            if self._has_joystick and self._joystick is not None:
                for idx in range(self._joystick.get_numbuttons()):
                    buttons[idx] = bool(self._joystick.get_button(idx))
                for idx in range(self._joystick.get_numaxes()):
                    v = float(self._joystick.get_axis(idx))
                    axes[idx] = 0.0 if abs(v) < self.deadzone else v
                for idx in range(self._joystick.get_numhats()):
                    hats[idx] = self._joystick.get_hat(idx)

            if self.fallback_keys:
                keys = pygame.key.get_pressed()
                if keys[pygame.K_q]:
                    yield ControllerState(quit=True)
                    return
                forward = keys[pygame.K_w] or keys[pygame.K_UP]
                backward = keys[pygame.K_s] or keys[pygame.K_DOWN]
                left = keys[pygame.K_LEFT] or keys[pygame.K_a]
                right = keys[pygame.K_RIGHT] or keys[pygame.K_d]
                x = -1 if left and not right else (1 if right and not left else 0)
                y = 1 if forward and not backward else (-1 if backward and not forward else 0)
                kb_hat = (x, y)
                kb_buttons = {
                    self.BTN_TRIANGLE: keys[pygame.K_t] or keys[pygame.K_e],
                    self.BTN_CROSS: keys[pygame.K_x],
                    self.BTN_CIRCLE: keys[pygame.K_c],
                    self.BTN_L1: keys[pygame.K_l] or keys[pygame.K_m],
                    self.BTN_R1: keys[pygame.K_g],
                    self.BTN_GETUP_BACK: keys[pygame.K_b],
                    self.BTN_L2: keys[pygame.K_j],
                    self.BTN_R2: keys[pygame.K_k],
                    self.BTN_OPTIONS: keys[pygame.K_o] or keys[pygame.K_ESCAPE],
                }
            else:
                kb_buttons = {}

            merged_buttons = dict(buttons)
            for idx, pressed in kb_buttons.items():
                merged_buttons[idx] = merged_buttons.get(idx, False) or pressed

            merged_hats = dict(hats)
            if kb_hat != (0, 0) or 0 not in merged_hats:
                merged_hats[0] = kb_hat

            yield ControllerState(buttons=merged_buttons, axes=dict(axes), hats=merged_hats)
            clock.tick(max(1, int(1.0 / self.poll_interval)))

    def quit(self) -> None:
        try:
            import pygame
            pygame.quit()
        except Exception:
            pass
