from __future__ import annotations


def run_menu(joystick_index: int = 0) -> str:
    """Show the top-level mode picker and return walking, vision, terrain, or quit."""
    try:
        import pygame
    except ImportError as exc:
        raise ImportError("The function menu requires pygame: pip install pygame") from exc

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((680, 480))
    pygame.display.set_caption("Humanoid Robot Control")
    clock = pygame.time.Clock()
    title_font = pygame.font.Font(None, 46)
    item_font = pygame.font.Font(None, 32)
    detail_font = pygame.font.Font(None, 22)

    joystick = None
    if pygame.joystick.get_count() > 0:
        index = min(max(0, joystick_index), pygame.joystick.get_count() - 1)
        joystick = pygame.joystick.Joystick(index)
        joystick.init()

    entries = [
        ("Walking & Balance", "PS4 / keyboard locomotion", "walking"),
        ("Camera Mimic", "Pi Camera full-body tracking", "vision"),
        ("Terrain Auto", "Continuous ramp and stair adaptation", "terrain"),
        ("Exit", "Return all servos to standing", "quit"),
    ]
    selected = 0
    previous_hat_y = 0
    previous_buttons: dict[int, bool] = {}

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return "quit"
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_UP, pygame.K_w):
                        selected = (selected - 1) % len(entries)
                    elif event.key in (pygame.K_DOWN, pygame.K_s):
                        selected = (selected + 1) % len(entries)
                    elif event.key in (pygame.K_RETURN, pygame.K_SPACE):
                        return entries[selected][2]
                    elif event.key == pygame.K_v:
                        return "vision"
                    elif event.key == pygame.K_t:
                        return "terrain"
                    elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                        return "quit"
                if event.type == pygame.MOUSEMOTION:
                    for index in range(len(entries)):
                        if pygame.Rect(70, 105 + index * 82, 540, 64).collidepoint(event.pos):
                            selected = index
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    rect = pygame.Rect(70, 105 + selected * 82, 540, 64)
                    if rect.collidepoint(event.pos):
                        return entries[selected][2]

            if joystick is not None:
                hat_y = joystick.get_hat(0)[1] if joystick.get_numhats() else 0
                if hat_y != 0 and previous_hat_y == 0:
                    selected = (selected - hat_y) % len(entries)
                previous_hat_y = hat_y

                buttons = {index: bool(joystick.get_button(index)) for index in range(joystick.get_numbuttons())}
                if buttons.get(0, False) and not previous_buttons.get(0, False):
                    return entries[selected][2]
                if buttons.get(2, False) and not previous_buttons.get(2, False):
                    return "vision"
                if buttons.get(3, False) and not previous_buttons.get(3, False):
                    return "terrain"
                if buttons.get(1, False) and not previous_buttons.get(1, False):
                    return "quit"
                previous_buttons = buttons

            screen.fill((18, 22, 28))
            title = title_font.render("Humanoid Robot", True, (238, 242, 246))
            screen.blit(title, (70, 42))

            for index, (label, detail, _) in enumerate(entries):
                rect = pygame.Rect(70, 105 + index * 82, 540, 64)
                active = index == selected
                pygame.draw.rect(screen, (43, 104, 92) if active else (39, 45, 54), rect)
                pygame.draw.rect(screen, (100, 210, 174) if active else (75, 84, 96), rect, 2)
                screen.blit(item_font.render(label, True, (248, 250, 252)), (90, rect.y + 9))
                screen.blit(detail_font.render(detail, True, (184, 194, 204)), (90, rect.y + 38))

            pygame.display.flip()
            clock.tick(30)
    finally:
        pygame.quit()
