import heapq
import math
import pygame

# Initialize Pygame
pygame.init()

WIDTH, HEIGHT = 900, 620
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("PAI - Autonomous Lab Robot")

clock = pygame.time.Clock()
font = pygame.font.SysFont("meiryo", 20)
small_font = pygame.font.SysFont("meiryo", 15)

# Colors
BG = (245, 247, 250)
GRID = (225, 230, 235)
DARK = (45, 55, 70)
BLUE = (55, 125, 220)
GREEN = (45, 165, 110)
ORANGE = (235, 145, 55)
WHITE = (255, 255, 255)
GRAY = (180, 190, 200)
BROWN = (190, 175, 155)

# Grid settings
TILE = 30
COLS, ROWS = 28, 15
ORIGIN_X, ORIGIN_Y = 30, 60
ROBOT_RADIUS = 17


def cell_to_pos(cell):
    """Convert a grid cell to screen coordinates."""
    col, row = cell
    return (
        ORIGIN_X + col * TILE + TILE // 2,
        ORIGIN_Y + row * TILE + TILE // 2,
    )


# Laboratory objects
shelf = pygame.Rect(60, 120, 180, 110)
analyzer = pygame.Rect(650, 120, 190, 140)
workbench = pygame.Rect(350, 220, 150, 100)

# The robot must keep a safe distance from each object.
obstacles = [shelf, analyzer, workbench]


def is_blocked(cell):
    col, row = cell

    if not (0 <= col < COLS and 0 <= row < ROWS):
        return True

    x, y = cell_to_pos(cell)

    for obstacle in obstacles:
        safe_obstacle = obstacle.inflate(
            ROBOT_RADIUS * 2 + 2,
            ROBOT_RADIUS * 2 + 2,
        )
        if safe_obstacle.collidepoint(x, y):
            return True

    return False


def astar(start, goal):
    """Find a route using A* search and Manhattan distance."""

    if is_blocked(start) or is_blocked(goal):
        return []

    frontier = []
    heapq.heappush(frontier, (0, start))

    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path

        col, row = current
        neighbors = [
            (col + 1, row),
            (col - 1, row),
            (col, row + 1),
            (col, row - 1),
        ]

        for nxt in neighbors:
            if is_blocked(nxt):
                continue

            new_cost = cost_so_far[current] + 1

            if new_cost < cost_so_far.get(nxt, float("inf")):
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current

                heuristic = (
                    abs(nxt[0] - goal[0])
                    + abs(nxt[1] - goal[1])
                )
                heapq.heappush(
                    frontier,
                    (new_cost + heuristic, nxt),
                )

    return []


# Start, pickup, and delivery locations
START = (4, 12)
PICKUP = (8, 4)
DELIVERY = (19, 7)


def reset_task():
    """Restart the autonomous delivery task."""
    global robot_x, robot_y, route, path_index
    global phase, carrying, status

    robot_x, robot_y = cell_to_pos(START)
    route = [
        cell_to_pos(cell)
        for cell in astar(START, PICKUP)
    ]

    path_index = 1
    phase = "to_sample"
    carrying = False
    status = "AI planned a route to the sample shelf"

    if not route:
        status = "ERROR: No route to sample shelf"


def draw_text(text, x, y, color=DARK, font_obj=font):
    image = font_obj.render(text, True, color)
    screen.blit(image, (x, y))


def move_robot(dt):
    """Move along the planned path and switch delivery stages."""
    global robot_x, robot_y, route, path_index
    global phase, carrying, status

    if phase == "done" or phase == "error":
        return

    if path_index >= len(route):
        if phase == "to_sample":
            carrying = True
            route = [
                cell_to_pos(cell)
                for cell in astar(PICKUP, DELIVERY)
            ]
            path_index = 1
            phase = "to_analyzer"

            if route:
                status = "Sample collected - delivering to analyzer"
            else:
                phase = "error"
                status = "ERROR: No delivery route"

        elif phase == "to_analyzer":
            phase = "done"
            status = "Transport complete! Press R to restart"

        return

    target_x, target_y = route[path_index]
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)
    step = 130 * dt

    if distance <= step:
        robot_x, robot_y = target_x, target_y
        path_index += 1
    elif distance > 0:
        robot_x += dx / distance * step
        robot_y += dy / distance * step


reset_task()

running = True

while running:
    dt = min(clock.tick(60) / 1000.0, 0.05)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False
            elif event.key == pygame.K_r:
                reset_task()

    move_robot(dt)

    # Background and grid
    screen.fill(BG)

    for col in range(COLS + 1):
        x = ORIGIN_X + col * TILE
        pygame.draw.line(
            screen, GRID, (x, ORIGIN_Y),
            (x, ORIGIN_Y + ROWS * TILE)
        )

    for row in range(ROWS + 1):
        y = ORIGIN_Y + row * TILE
        pygame.draw.line(
            screen, GRID, (ORIGIN_X, y),
            (ORIGIN_X + COLS * TILE, y)
        )

    # Planned route
    if phase not in ("done", "error") and route:
        route_color = GREEN if carrying else BLUE
        points = [(int(robot_x), int(robot_y))]
        points.extend(route[path_index:])

        if len(points) > 1:
            pygame.draw.lines(
                screen, route_color, False, points, 4
            )

        for point in points[1:]:
            pygame.draw.circle(screen, route_color, point, 4)

    # Sample shelf
    pygame.draw.rect(screen, WHITE, shelf, border_radius=10)
    pygame.draw.rect(screen, BLUE, shelf, 3, border_radius=10)
    draw_text("SAMPLE SHELF", 72, 130, DARK, small_font)

    for i in range(3):
        x = 85 + i * 48
        pygame.draw.rect(
            screen, GRAY, (x, 165, 25, 42), border_radius=5
        )
        pygame.draw.rect(
            screen, ORANGE, (x + 4, 180, 17, 22),
            border_radius=3
        )

    # Analyzer
    pygame.draw.rect(screen, WHITE, analyzer, border_radius=10)
    pygame.draw.rect(screen, GREEN, analyzer, 3, border_radius=10)
    draw_text("ANALYZER", 690, 135, DARK, small_font)

    pygame.draw.rect(
        screen, (225, 240, 232),
        (680, 175, 130, 65), border_radius=6
    )
    pygame.draw.circle(screen, GREEN, (745, 207), 17)

    # Obstacle
    pygame.draw.rect(
        screen, BROWN, workbench, border_radius=8
    )
    draw_text("WORKBENCH", 360, 255, DARK, small_font)

    # Pickup and delivery markers
    pygame.draw.circle(screen, ORANGE, cell_to_pos(PICKUP), 7)
    pygame.draw.circle(screen, GREEN, cell_to_pos(DELIVERY), 7)

    # Robot
    robot_pos = (int(robot_x), int(robot_y))
    pygame.draw.circle(screen, BLUE, robot_pos, ROBOT_RADIUS)
    pygame.draw.circle(screen, WHITE, robot_pos, ROBOT_RADIUS, 3)
    pygame.draw.circle(
        screen, WHITE, (robot_pos[0] + 6, robot_pos[1] - 5), 4
    )

    if carrying:
        pygame.draw.circle(
            screen, ORANGE,
            (robot_pos[0] + 17, robot_pos[1] - 14), 7
        )

    draw_text(
        "ROBOT", robot_pos[0] - 27, robot_pos[1] + 23,
        DARK, small_font
    )

    # Header
    pygame.draw.rect(screen, DARK, (0, 0, WIDTH, 50))
    draw_text(
        "PAI | AI-Powered Sample Transport", 20, 12, WHITE
    )

    # Status panel
    pygame.draw.rect(
        screen, WHITE, (25, 525, 850, 75), border_radius=8
    )

    if phase == "done":
        status_color = GREEN
    elif phase == "error":
        status_color = ORANGE
    else:
        status_color = BLUE

    draw_text("Status: " + status, 42, 535, status_color, small_font)
    draw_text(
        "AI: A* path planning  |  R: Restart  |  ESC: Quit",
        42, 561, DARK, small_font
    )

    pygame.display.flip()

pygame.quit()