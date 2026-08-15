from __future__ import annotations


def _run_picker(title: str, entries: list[tuple[str, str, str]]) -> str:
    try:
        import pygame
    except ImportError as exc:
        raise ImportError("The function menu requires pygame: pip install pygame") from exc

    pygame.init()
    screen = pygame.display.set_mode((680, 480))
    pygame.display.set_caption("Humanoid Robot Control")
    clock = pygame.time.Clock()
    title_font = pygame.font.Font(None, 46)
    item_font = pygame.font.Font(None, 32)
    detail_font = pygame.font.Font(None, 22)

    selected = 0

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
                    elif event.key in (pygame.K_q, pygame.K_ESCAPE):
                        return "back"
                if event.type == pygame.MOUSEMOTION:
                    for index in range(len(entries)):
                        if pygame.Rect(70, 105 + index * 82, 540, 64).collidepoint(event.pos):
                            selected = index
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    rect = pygame.Rect(70, 105 + selected * 82, 540, 64)
                    if rect.collidepoint(event.pos):
                        return entries[selected][2]

            screen.fill((18, 22, 28))
            title_surface = title_font.render(title, True, (238, 242, 246))
            screen.blit(title_surface, (70, 42))

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


def run_menu() -> str:
    return _run_picker(
        "Humanoid Robot",
        [
            ("Locomotion", "Walking, recovery and terrain balance", "locomotion"),
            ("Person Follow", "Detect and follow one person", "follow"),
            ("Pick Up Positioning", "Manual squat positioning", "pickup"),
            ("Exit", "Return all servos to standing", "quit"),
        ],
    )


def run_locomotion_menu() -> str:
    return _run_picker(
        "Locomotion",
        [
            ("Walking & Recovery", "Keyboard gait and push recovery", "walking"),
            ("Terrain Balance", "IMU ankle and hip stabilization", "terrain"),
            ("Back", "Return to function menu", "back"),
        ],
    )
