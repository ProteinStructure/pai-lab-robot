#!/usr/bin/env python3
"""ROS 2 + Pygame multi-robot laboratory workflow demo.

Run three processes in the same ROS 2 environment:
  python3 lab_system.py view
  python3 lab_system.py B
  python3 lab_system.py A

Robot A performs five simulated lab stages. Robot B receives a ROS 2
assistance request and relocates the movable cart blocking the corridor.
The Pygame window visualizes live state from ROS 2 topics.
"""

import heapq
import json
import sys

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Shared grid map. Coordinates are grid cells, not physical measurements.
COLS, ROWS = 10, 7
STATIC_WALLS = {(4, y) for y in range(ROWS) if y != 3}
CART_START = (4, 3)
CART_PARKING = (3, 2)
ROBOT_A_START = (0, 3)
ROBOT_B_START = (1, 5)
ROBOT_B_APPROACH = (3, 3)
ROBOT_B_SAFE_BAY = (2, 5)

STAGES = [
    ("Sample collection", (1, 3), 5.0),
    ("Weighing", (6, 1), 7.0),
    ("Reagent addition", (8, 1), 7.0),
    ("Mixing", (8, 5), 7.0),
    ("Analysis", (6, 5), 7.0),
]


def astar(start, goal, blocked):
    """Find a four-connected shortest path using A* and Manhattan distance."""
    if start in blocked or goal in blocked:
        return None
    frontier = [(0, start)]
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
        x, y = current
        for nxt in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            nx, ny = nxt
            if not (0 <= nx < COLS and 0 <= ny < ROWS) or nxt in blocked:
                continue
            new_cost = cost_so_far[current] + 1
            if new_cost < cost_so_far.get(nxt, float("inf")):
                cost_so_far[nxt] = new_cost
                came_from[nxt] = current
                heuristic = abs(nx - goal[0]) + abs(ny - goal[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
    return None


def publish_json(publisher, payload):
    msg = String()
    msg.data = json.dumps(payload)
    publisher.publish(msg)


class RobotA(Node):
    """Experiment robot: requests support if the blocked corridor is impassable."""

    def __init__(self):
        super().__init__("robot_a")
        self.state_pub = self.create_publisher(String, "/lab/robot_a/state", 10)
        self.request_pub = self.create_publisher(
            String, "/lab/robot_a/assistance_request", 10
        )
        self.create_subscription(
            String, "/lab/robot_b/status", self.on_robot_b_status, 10
        )
        self.pos = ROBOT_A_START
        self.cart_pos = CART_START
        self.stage_index = 0
        self.phase = "moving"
        self.status = "Moving to sample shelf"
        self.path = []
        self.path_index = 1
        self.move_elapsed = 0.0
        self.process_remaining = 0.0
        self.assistance_requested = False
        self.timer = self.create_timer(0.2, self.tick)
        self.plan_current_stage()
        self.publish_state()
        self.get_logger().info(
            "Robot A started: sample collection -> weighing -> reagent addition -> mixing -> analysis."
        )

    def blocked_cells(self):
        blocked = set(STATIC_WALLS)
        if self.cart_pos is not None:
            blocked.add(self.cart_pos)
        return blocked

    def plan_current_stage(self):
        if self.stage_index >= len(STAGES):
            self.phase = "done"
            self.status = "Laboratory workflow completed successfully"
            self.path = []
            self.publish_state()
            return
        stage_name, target, _ = STAGES[self.stage_index]
        new_path = astar(self.pos, target, self.blocked_cells())
        if new_path is None:
            if self.cart_pos == CART_START and not self.assistance_requested:
                self.phase = "waiting_support"
                self.status = "Passage blocked - requesting Robot B"
                self.assistance_requested = True
                request = String()
                request.data = "MOVE_CART: corridor_1"
                self.request_pub.publish(request)
                self.get_logger().warning(
                    "No route to next workstation. Assistance request sent."
                )
            else:
                self.phase = "aborted"
                self.status = "No safe route available - workflow aborted"
                self.get_logger().error(self.status)
            self.publish_state()
            return
        self.path = new_path
        self.path_index = 1
        self.move_elapsed = 0.0
        self.phase = "moving"
        self.status = f"Moving to {stage_name}"
        self.get_logger().info(f"New A* route to {stage_name}: {self.path}")
        self.publish_state()
        if len(self.path) <= 1:
            self.begin_processing()

    def begin_processing(self):
        stage_name, _, duration = STAGES[self.stage_index]
        self.phase = "processing"
        self.process_remaining = duration
        self.status = f"Processing: {stage_name}"
        self.get_logger().info(self.status)
        self.publish_state()

    def on_robot_b_status(self, msg):
        if msg.data == "CART_RELOCATED" and self.phase == "waiting_support":
            self.cart_pos = CART_PARKING
            self.get_logger().info(
                "Robot B moved the cart. Replanning with the updated map."
            )
            self.plan_current_stage()
        elif msg.data == "FAILED_NO_ROUTE" and self.phase == "waiting_support":
            self.phase = "aborted"
            self.status = "Robot B cannot reach the cart - safe stop"
            self.get_logger().error(self.status)
            self.publish_state()

    def tick(self):
        dt = 0.2
        if self.phase == "moving":
            self.move_elapsed += dt
            if self.move_elapsed >= 0.65:
                self.move_elapsed = 0.0
                if self.path_index < len(self.path):
                    self.pos = self.path[self.path_index]
                    self.path_index += 1
                if self.path_index >= len(self.path):
                    self.begin_processing()
        elif self.phase == "processing":
            self.process_remaining -= dt
            if self.process_remaining <= 0:
                stage_name = STAGES[self.stage_index][0]
                self.get_logger().info(f"Completed: {stage_name}")
                self.stage_index += 1
                if self.stage_index >= len(STAGES):
                    self.phase = "done"
                    self.status = "LAB WORKFLOW COMPLETED SUCCESSFULLY"
                    self.get_logger().info(self.status)
                    self.publish_state()
                else:
                    self.plan_current_stage()
        self.publish_state()

    def publish_state(self):
        stage_name = (
            STAGES[self.stage_index][0]
            if self.stage_index < len(STAGES)
            else "Completed"
        )
        publish_json(self.state_pub, {
            "x": self.pos[0], "y": self.pos[1], "phase": self.phase,
            "status": self.status, "stage_index": self.stage_index,
            "stage_name": stage_name,
            "carrying": self.stage_index >= 1 and self.phase != "aborted",
            "path": self.path,
        })


class RobotB(Node):
    """Support robot: navigates to and relocates the movable cart."""

    def __init__(self):
        super().__init__("robot_b")
        self.state_pub = self.create_publisher(String, "/lab/robot_b/state", 10)
        self.status_pub = self.create_publisher(String, "/lab/robot_b/status", 10)
        self.environment_pub = self.create_publisher(
            String, "/lab/environment/state", 10
        )
        self.create_subscription(
            String, "/lab/robot_a/assistance_request",
            self.on_assistance_request, 10
        )
        self.pos = ROBOT_B_START
        self.cart_pos = CART_START
        self.phase = "idle"
        self.status = "Waiting for assistance request"
        self.path = []
        self.path_index = 1
        self.move_elapsed = 0.0
        self.relocate_remaining = 0.0
        self.request_handled = False
        self.timer = self.create_timer(0.2, self.tick)
        self.publish_environment()
        self.publish_state()
        self.get_logger().info("Robot B started: waiting for support requests.")

    def publish_environment(self):
        publish_json(self.environment_pub, {
            "cart": list(self.cart_pos) if self.cart_pos else None,
            "cart_status": "blocking" if self.cart_pos == CART_START else "parked",
        })

    def publish_state(self):
        publish_json(self.state_pub, {
            "x": self.pos[0], "y": self.pos[1], "phase": self.phase,
            "status": self.status, "path": self.path,
        })

    def publish_status(self, value):
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def on_assistance_request(self, msg):
        if self.request_handled or not msg.data.startswith("MOVE_CART"):
            return
        self.request_handled = True
        self.get_logger().warning(f"Assistance request received: {msg.data}")
        blocked = set(STATIC_WALLS) | {CART_START}
        route = astar(self.pos, ROBOT_B_APPROACH, blocked)
        if route is None:
            self.phase = "failed"
            self.status = "Cannot reach movable cart"
            self.publish_status("FAILED_NO_ROUTE")
            self.publish_state()
            self.get_logger().error(self.status)
            return
        self.path = route
        self.path_index = 1
        self.phase = "moving"
        self.status = "Moving to blocked passage"
        self.publish_status("ACCEPTED")
        self.get_logger().info(f"Robot B route: {route}")
        self.publish_state()
        if len(self.path) <= 1:
            self.start_relocation()

    def start_relocation(self):
        self.phase = "relocating"
        self.status = "Relocating movable cart"
        self.relocate_remaining = 3.0
        self.get_logger().info(self.status)
        self.publish_state()

    def tick(self):
        dt = 0.2
        if self.phase in ("moving", "retreating"):
            self.move_elapsed += dt
            if self.move_elapsed >= 0.65:
                self.move_elapsed = 0.0
                if self.path_index < len(self.path):
                    self.pos = self.path[self.path_index]
                    self.path_index += 1
                if self.path_index >= len(self.path):
                    if self.phase == "moving":
                        self.start_relocation()
                    else:
                        self.phase = "done"
                        self.status = "Cart relocated; Robot B returned to safe bay"
                        self.get_logger().info(self.status)
        elif self.phase == "relocating":
            self.relocate_remaining -= dt
            if self.relocate_remaining <= 0:
                self.cart_pos = CART_PARKING
                self.publish_environment()
                # Notify Robot A, then clear the corridor entrance so that
                # the two simulated robots do not occupy the same grid cell.
                self.publish_status("CART_RELOCATED")
                self.path = astar(
                    self.pos,
                    ROBOT_B_SAFE_BAY,
                    set(STATIC_WALLS) | {CART_PARKING},
                ) or []
                self.path_index = 1
                self.move_elapsed = 0.0
                self.phase = "retreating" if self.path else "done"
                self.status = "Cart relocated - retreating to safe bay"
                self.get_logger().info(self.status)
        self.publish_state()


class LabVisualizer(Node):
    """Pygame display subscribed to live ROS 2 robot/environment state."""

    def __init__(self):
        super().__init__("lab_visualizer")
        self.robot_a = {
            "x": ROBOT_A_START[0], "y": ROBOT_A_START[1],
            "phase": "waiting", "status": "Waiting for Robot A",
            "stage_index": 0, "stage_name": STAGES[0][0],
            "carrying": False, "path": [],
        }
        self.robot_b = {
            "x": ROBOT_B_START[0], "y": ROBOT_B_START[1],
            "phase": "idle", "status": "Waiting for Robot B", "path": [],
        }
        self.environment = {"cart": list(CART_START), "cart_status": "blocking"}
        self.create_subscription(String, "/lab/robot_a/state", self.on_robot_a, 10)
        self.create_subscription(String, "/lab/robot_b/state", self.on_robot_b, 10)
        self.create_subscription(
            String, "/lab/environment/state", self.on_environment, 10
        )

    def on_robot_a(self, msg):
        try:
            self.robot_a = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("Invalid Robot A state message")

    def on_robot_b(self, msg):
        try:
            self.robot_b = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("Invalid Robot B state message")

    def on_environment(self, msg):
        try:
            self.environment = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warning("Invalid environment state message")


def run_visualizer():
    rclpy.init()
    node = LabVisualizer()
    pygame.init()
    screen = pygame.display.set_mode((1120, 700))
    pygame.display.set_caption("ROS 2 Multi-Robot Laboratory Automation")
    clock = pygame.time.Clock()

    title_font = pygame.font.SysFont("DejaVu Sans", 25, bold=True)
    font = pygame.font.SysFont("DejaVu Sans", 18)
    small = pygame.font.SysFont("DejaVu Sans", 14)

    BG = (244, 247, 250)
    GRID = (218, 225, 232)
    DARK = (43, 55, 70)
    BLUE = (55, 125, 220)
    GREEN = (45, 165, 110)
    ORANGE = (232, 145, 55)
    PURPLE = (135, 90, 190)
    WHITE = (255, 255, 255)
    WALL = (145, 155, 166)

    TILE = 68
    OX, OY = 48, 105
    workflow_names = [stage[0] for stage in STAGES]

    def center(cell):
        x, y = cell
        return (OX + x * TILE + TILE // 2, OY + y * TILE + TILE // 2)

    def draw_text(surface, text, pos, color=DARK, used_font=font):
        surface.blit(used_font.render(str(text), True, color), pos)

    def draw_station(surface, cell, label, color):
        px, py = center(cell)
        rect = pygame.Rect(px - 28, py - 20, 56, 40)
        pygame.draw.rect(surface, color, rect, border_radius=7)
        pygame.draw.rect(surface, WHITE, rect, 2, border_radius=7)
        label_img = small.render(label, True, WHITE)
        surface.blit(label_img, label_img.get_rect(center=(px, py)))

    def draw_robot(surface, state, color, label):
        px, py = center((int(state.get("x", 0)), int(state.get("y", 0))))
        path = state.get("path", [])
        if isinstance(path, list) and len(path) > 1:
            points = [center(tuple(point)) for point in path]
            pygame.draw.lines(surface, color, False, points, 3)
        pygame.draw.circle(surface, color, (px, py), 21)
        pygame.draw.circle(surface, WHITE, (px, py), 21, 3)
        mark = title_font.render(label, True, WHITE)
        surface.blit(mark, mark.get_rect(center=(px, py - 1)))

    running = True
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            screen.fill(BG)
            pygame.draw.rect(screen, DARK, (0, 0, 1120, 62))
            draw_text(screen, "ROS 2 | MULTI-ROBOT LABORATORY AUTOMATION",
                      (25, 18), WHITE, title_font)

            # Grid and fixed wall, leaving the single corridor at row 3.
            for x in range(COLS + 1):
                px = OX + x * TILE
                pygame.draw.line(screen, GRID, (px, OY),
                                 (px, OY + ROWS * TILE), 1)
            for y in range(ROWS + 1):
                py = OY + y * TILE
                pygame.draw.line(screen, GRID, (OX, py),
                                 (OX + COLS * TILE, py), 1)
            for cell in STATIC_WALLS:
                px, py = center(cell)
                pygame.draw.rect(screen, WALL,
                                 pygame.Rect(px - 28, py - 28, 56, 56),
                                 border_radius=4)

            # Laboratory stations.
            draw_station(screen, (1, 3), "SAMPLE", BLUE)
            draw_station(screen, (6, 1), "WEIGH", PURPLE)
            draw_station(screen, (8, 1), "REAGENT", ORANGE)
            draw_station(screen, (8, 5), "MIX", GREEN)
            draw_station(screen, (6, 5), "ANALYZE", BLUE)

            # Movable cart, updated by Robot B's environment topic.
            cart = node.environment.get("cart")
            if cart is not None:
                cx, cy = center(tuple(cart))
                cart_rect = pygame.Rect(cx - 23, cy - 23, 46, 46)
                pygame.draw.rect(screen, ORANGE, cart_rect, border_radius=5)
                pygame.draw.rect(screen, DARK, cart_rect, 2, border_radius=5)
                draw_text(screen, "C", (cx - 6, cy - 11), WHITE, font)

            draw_robot(screen, node.robot_a, BLUE, "A")
            draw_robot(screen, node.robot_b, GREEN, "B")

            # Information panel.
            panel = pygame.Rect(770, 88, 325, 520)
            pygame.draw.rect(screen, WHITE, panel, border_radius=12)
            pygame.draw.rect(screen, GRID, panel, 2, border_radius=12)

            draw_text(screen, "ROBOT A | EXPERIMENT", (790, 108), BLUE, font)
            draw_text(screen, node.robot_a.get("phase", "unknown"),
                      (790, 140), DARK, small)
            a_status = str(node.robot_a.get("status", "Waiting"))
            for i in range(0, min(len(a_status), 90), 34):
                draw_text(screen, a_status[i:i + 34],
                          (790, 165 + (i // 34) * 19), DARK, small)

            draw_text(screen, "ROBOT B | SUPPORT", (790, 235), GREEN, font)
            draw_text(screen, node.robot_b.get("phase", "unknown"),
                      (790, 266), DARK, small)
            b_status = str(node.robot_b.get("status", "Waiting"))
            for i in range(0, min(len(b_status), 90), 34):
                draw_text(screen, b_status[i:i + 34],
                          (790, 291 + (i // 34) * 19), DARK, small)

            draw_text(screen, "EXPERIMENT WORKFLOW", (790, 365), DARK, font)
            current_index = int(node.robot_a.get("stage_index", 0))
            a_phase = node.robot_a.get("phase", "")
            for i, name in enumerate(workflow_names):
                y = 401 + i * 34
                if i < current_index or a_phase == "done":
                    color, prefix = GREEN, "[DONE]"
                elif i == current_index and a_phase != "aborted":
                    color, prefix = BLUE, "[NOW]"
                else:
                    color, prefix = WALL, "[WAIT]"
                draw_text(screen, f"{prefix} {name}", (790, y), color, small)

            cart_status = node.environment.get("cart_status", "blocking")
            draw_text(screen, f"Movable cart: {cart_status}",
                      (790, 585), ORANGE, small)
            draw_text(screen, "ESC: close visualizer", (48, 620), DARK, small)

            pygame.display.flip()
            clock.tick(30)
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()


def main():
    if len(sys.argv) != 2 or sys.argv[1].lower() not in ("a", "b", "view"):
        print("Usage: python3 lab_system.py A | B | view")
        return 2
    role = sys.argv[1].lower()
    if role == "view":
        run_visualizer()
        return 0

    rclpy.init()
    node = RobotA() if role == "a" else RobotB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
