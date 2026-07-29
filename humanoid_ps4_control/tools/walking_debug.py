from __future__ import annotations

import argparse
import time

from src.backends import SerialRTBackend
from src.config import STANDING


LEGS = (
    (12, "Left hip roll"),
    (13, "Left hip pitch"),
    (14, "Left knee"),
    (15, "Left ankle pitch"),
    (16, "Left ankle roll"),
    (17, "Right ankle roll"),
    (18, "Right ankle pitch"),
    (19, "Right knee"),
    (20, "Right hip pitch"),
    (21, "Right hip roll"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Interactive leg-servo calibration")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--min-pwm", type=int, default=1350)
    parser.add_argument("--max-pwm", type=int, default=1650)
    return parser.parse_args()


def draw(screen, pygame, font, small_font, selected: int, pose: dict[int, int], minimum: int, maximum: int) -> None:
    screen.fill((18, 22, 28))
    title = font.render("Walking Leg Calibration", True, (238, 242, 246))
    screen.blit(title, (28, 22))
    help_text = (
        "Up/Down select   Left/Right: 5us   Shift+Left/Right: 20us   "
        "Home: 1500us   S: print values   C: all 1500us   Q/Esc: exit"
    )
    screen.blit(small_font.render(help_text, True, (184, 194, 204)), (28, 64))
    for index, (servo_id, label) in enumerate(LEGS):
        y = 104 + index * 42
        active = index == selected
        rect = pygame.Rect(28, y, 584, 32)
        pygame.draw.rect(screen, (43, 104, 92) if active else (39, 45, 54), rect)
        pygame.draw.rect(screen, (100, 210, 174) if active else (75, 84, 96), rect, 2)
        text = f"{servo_id:02d}  {label:<20} {pose[servo_id]:4d} us"
        screen.blit(small_font.render(text, True, (248, 250, 252)), (42, y + 8))
    footer = f"Safe range: {minimum}-{maximum} us. Keep robot supported; this tool controls legs only."
    screen.blit(small_font.render(footer, True, (245, 190, 72)), (28, 548))
    pygame.display.flip()


def print_pose(pose: dict[int, int]) -> None:
    values = ", ".join(f"{servo_id}: {pose[servo_id]}" for servo_id, _ in LEGS)
    print(f"LEG_STANDING = {{{values}}}")


def main() -> None:
    args = parse_args()
    minimum = min(args.min_pwm, args.max_pwm)
    maximum = max(args.min_pwm, args.max_pwm)
    pose = {servo_id: max(minimum, min(maximum, STANDING[servo_id])) for servo_id, _ in LEGS}

    import pygame

    pygame.init()
    screen = pygame.display.set_mode((640, 590))
    pygame.display.set_caption("Humanoid Walking Calibration")
    font = pygame.font.Font(None, 36)
    small_font = pygame.font.Font(None, 24)
    clock = pygame.time.Clock()
    selected = 0

    try:
        with SerialRTBackend(args.port, args.baudrate) as backend:
            backend.send(pose, duration_ms=1200, force=True)
            print(f"[walking-debug] Connected to {args.port}.")
            print_pose(pose)
            running = True
            while running:
                changed = False
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif event.key == pygame.K_UP:
                            selected = (selected - 1) % len(LEGS)
                        elif event.key == pygame.K_DOWN:
                            selected = (selected + 1) % len(LEGS)
                        elif event.key in (pygame.K_LEFT, pygame.K_RIGHT):
                            step = 20 if event.mod & pygame.KMOD_SHIFT else 5
                            if event.key == pygame.K_LEFT:
                                step = -step
                            servo_id = LEGS[selected][0]
                            pose[servo_id] = max(minimum, min(maximum, pose[servo_id] + step))
                            changed = True
                        elif event.key == pygame.K_HOME:
                            pose[LEGS[selected][0]] = 1500
                            changed = True
                        elif event.key == pygame.K_c:
                            for servo_id, _ in LEGS:
                                pose[servo_id] = 1500
                            changed = True
                        elif event.key == pygame.K_s:
                            print_pose(pose)
                if changed:
                    backend.send(pose, duration_ms=350, force=True)
                draw(screen, pygame, font, small_font, selected, pose, minimum, maximum)
                clock.tick(30)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        print("[walking-debug] Closed. Copy LEG_STANDING from the terminal before changing config.")
        time.sleep(0.1)


if __name__ == "__main__":
    main()
