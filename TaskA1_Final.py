"""Group 5 Task A1: visit both supply checkpoints before reaching base.

Extends Task 1's map, eight-direction motion and destination-cell penalties.
Compare both checkpoint orders and choose the minimum total weighted time.
Run: python TaskA1_Checkpoints.py
Use --seed N for another reproducible example, or --no-show to save only.
"""

# REVIEW NOTE: Week 9b p.3 requires one checkpoint in each cost region,
# both visited before base, with checkpoints and route plotted together.
# REVIEW NOTE: Repository command: python "Task A1_Final" --no-show.
# Keep the original module docstring/argparse help text unchanged for compatibility.
import argparse
import math
from pathlib import Path
import random

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y):
        
        self.resolution = resolution
        self.rr = rr
        self.min_x = self.min_y = 0
        self.max_x = self.max_y = 0
        self.obstacle_map = None
        self.x_width = self.y_width = 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y

        # REVIEW CHANGE 1: Cache static coordinate membership; preserve list attributes.
        # Rebuild these caches if region coordinates are modified after construction.
        self._fc_x = frozenset(fc_x)
        self._fc_y = frozenset(fc_y)
        self._tc_x = frozenset(tc_x)
        self._tc_y = frozenset(tc_y)

        self.Delta_TC = 0.30   # +30 % time-consuming area (pink)
        self.Delta_FC = 0.15   # +15 % fuel-consuming area (yellow)
        self.costPerGrid = 1.0

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy, avoid_goal=None):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node  = self.Node(self.calc_xy_index(gx, self.min_x),
                               self.calc_xy_index(gy, self.min_y), 0.0, -1)
        

        if not self.verify_node(start_node) or not self.verify_node(goal_node):
            return [], [], None

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if not open_set:
                return [], [], None

            c_id = min(open_set,
                       key=lambda o: open_set[o].cost +
                       self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * self.costPerGrid,
                                 c_id)

                px = self.calc_grid_position(node.x, self.min_x)
                py = self.calc_grid_position(node.y, self.min_y)
                if avoid_goal is not None and (px, py) == avoid_goal:
                    continue

                # Time-consuming area (pink) +30%
                if px in self._tc_x and py in self._tc_y:
                    node.cost += self.Delta_TC * self.motion[i][2]

                # Fuel-consuming area (yellow) +15%
                if px in self._fc_x and py in self._fc_y:
                    node.cost += self.Delta_FC * self.motion[i][2]

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry, goal_node.cost

    def calc_final_path(self, goal_node, closed_set):
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def calc_heuristic(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y) * self.costPerGrid

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print(f"Map range: x[{self.min_x}, {self.max_x}], y[{self.min_y}, {self.max_y}]")

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        self.obstacle_map = [[False] * self.y_width for _ in range(self.x_width)]
        # REVIEW CHANGE 2: Reuse obstacle pairs; retain order and exact collision math.
        obstacle_points = tuple(zip(ox, oy))
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in obstacle_points:
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    # REVIEW TODO (not applied): Lecture p.5 uses diagonal=2, legacy code sqrt(2).
    # Changing it would change paths/costs; preserve the existing model in this revision.
    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]


def plan_route(planner, points):
    rx, ry = [], []
    total_time = 0.0

    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]
        # REVIEW NOTE: Avoid the base on earlier legs; the last leg may reach it.
        avoid = points[-1] if i < len(points) - 2 else None

        xs, ys, trip_time = planner.planning(
            *start, *end, avoid_goal=avoid
        )

        if trip_time is None:
            return [], [], None

        # REVIEW NOTE: Planner returns goal-to-start; join legs start-to-goal.
        xs.reverse()
        ys.reverse()

        if rx:
            xs = xs[1:]
            ys = ys[1:]

        rx.extend(xs)
        ry.extend(ys)
        total_time += trip_time

    return rx, ry, total_time


def create_environment():
    # Start & Goal (Group 5)
    sx, sy = 5.0, 40.0
    gx, gy = 20.0, 30.0
    grid_size = 1.0
    robot_radius = 0.5

    # ---------- Obstacles ----------
    ox, oy = [], []

    # Outer borders
    for i in range(-10, 61):
        ox.append(i)
        oy.append(-10.0)
        ox.append(60.0)
        oy.append(i)
        ox.append(i)
        oy.append(60.0)
        ox.append(-10.0)
        oy.append(i)

    # Horizontal barrier at y=45 (x=10 to 50)
    for i in range(10, 51):
        ox.append(float(i))
        oy.append(45.0)

    # Vertical barrier at x=10 (y=10 to 45)
    for y in range(10, 46):
        ox.append(10.0)
        oy.append(float(y))

    # Diagonal from (30,45) to (40,0)
    for k in range(0, 101):
        x = 30.0 + 0.1 * k
        y = 45.0 - 4.5 * 0.1 * k
        ox.append(x)
        oy.append(y)

    # ---------- Fuel-consuming area (yellow) ----------
    fc_x, fc_y = [], []
    for i in range(-10, 31):
        for j in range(0, 11):
            fc_x.append(i)
            fc_y.append(j)

    # ---------- Time-consuming area (pink) ----------
    tc_x, tc_y = [], []
    for i in range(50, 61):
        for j in range(0, 11):
            tc_x.append(i)
            tc_y.append(j)

    planner = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    return planner, (sx, sy), (gx, gy), (ox, oy)


def generate_checkpoint(planner, xs, ys, rng):
    """Sample strictly inside a cost area and reject occupied grid cells."""
    # REVIEW CHANGE 3: Bounds are constant within this call; preserve random draw order.
    # REVIEW NOTE: Strictly interior bounds satisfy the checkpoint-area requirement.
    x_min, x_max = min(xs) + 1, max(xs) - 1
    y_min, y_max = min(ys) + 1, max(ys) - 1
    for _ in range(10000):
        point = (rng.randint(x_min, x_max),
                 rng.randint(y_min, y_max))
        node = planner.Node(planner.calc_xy_index(point[0], planner.min_x),
                            planner.calc_xy_index(point[1], planner.min_y), 0, -1)
        if planner.verify_node(node):
            return point
    raise ValueError("No free checkpoint found inside the cost area")


def compare_orders(planner, start, goal, fuel_checkpoint, time_checkpoint):
    """Keep only reachable orders; return both candidates and the winner."""
    orders = [
        ("Fuel -> Time", [start, fuel_checkpoint, time_checkpoint, goal]),
        ("Time -> Fuel", [start, time_checkpoint, fuel_checkpoint, goal]),
    ]
    candidates = []
    for name, points in orders:
        rx, ry, total = plan_route(planner, points)
        candidates.append({"order": name, "points": points,
                           "rx": rx, "ry": ry, "time": total})
    reachable = [route for route in candidates if route["time"] is not None]
    if not reachable:
        raise ValueError("Neither checkpoint order has a valid route")
    winner = min(reachable, key=lambda route: route["time"])
    return candidates, winner


def plot_route(planner, start, goal, obstacles, winner, output, show):
    """Display only the chosen route, with leg colours and direction arrows."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, ax = plt.subplots(figsize=(11, 8))
    ax.add_patch(Rectangle((min(planner.fc_x), min(planner.fc_y)),
                          max(planner.fc_x) - min(planner.fc_x),
                          max(planner.fc_y) - min(planner.fc_y),
                          facecolor="#f9df70", alpha=0.45,
                          label="Fuel area (+15% time)"))
    ax.add_patch(Rectangle((min(planner.tc_x), min(planner.tc_y)),
                          max(planner.tc_x) - min(planner.tc_x),
                          max(planner.tc_y) - min(planner.tc_y),
                          facecolor="#f5a0a8", alpha=0.5,
                          label="Time area (+30% time)"))
    ax.plot(*obstacles, ".", color="#303846", markersize=2.4, label="Obstacles")

    path = list(zip(winner["rx"], winner["ry"]))
    cursor = 0
    colours = ["#1276ad", "#c56a00", "#7a459e"]
    for leg, target in enumerate(winner["points"][1:]):
        end = path.index(target, cursor)
        segment = path[cursor:end + 1]
        xs, ys = zip(*segment)
        colour = colours[leg]
        ax.plot(xs, ys, color=colour, linewidth=2.5, label=f"Leg {leg + 1}")
        if len(segment) > 1:
            for j in sorted(set([max(1, len(segment) // 3),
                                  max(1, 2 * len(segment) // 3)])):
                ax.annotate("", xy=segment[j], xytext=segment[j - 1],
                            arrowprops={"arrowstyle": "->", "color": colour,
                                        "lw": 2.4}, zorder=6)
        cursor = end

    ax.plot(*start, "*", color="#1264a3", markersize=17, zorder=8, label="Start")
    ax.plot(*goal, "*", color="#288545", markersize=17, zorder=8, label="Base / Goal")
    checkpoint_names = ["Fuel", "Time"] if winner["order"] == "Fuel -> Time" else ["Time", "Fuel"]
    for number, (point, name) in enumerate(zip(winner["points"][1:3], checkpoint_names), 1):
        colour = "#1264a3" if name == "Fuel" else "#ac268b"
        ax.plot(*point, "D", color=colour, markersize=10, zorder=8,
                label=f"Drop {number}: {name} checkpoint")
        offset = (9, 9) if name == "Fuel" else (-100, 12)
        ax.annotate(f"{number}. {name} {point}", point, xytext=offset,
                    textcoords="offset points", fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "none"},
                    zorder=9)
    ax.set(xlim=(-12, 72), ylim=(-12, 62), xlabel="X", ylabel="Y",
           title=("Group 5 - Task A1: supply drop-off route\n"
                  f"{winner['order']} | Total weighted time: {winner['time']:.4f} min"))
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.96)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=5,
                        help="Random seed for reproducible checkpoints (default: 5)")
    parser.add_argument("--no-show", action="store_true", help="Save results without opening a plot window")
    parser.add_argument("--output", type=Path,
                        default=Path(__file__).resolve().parent / "A1_results" / "A1_route.png")
    args = parser.parse_args()

    planner, start, goal, obstacles = create_environment()
    # REVIEW NOTE: Default seed=5; do not alter sampling order or selected checkpoints.
    rng = random.Random(args.seed)
    fuel = generate_checkpoint(planner, planner.fc_x, planner.fc_y, rng)
    time = generate_checkpoint(planner, planner.tc_x, planner.tc_y, rng)
    candidates, winner = compare_orders(planner, start, goal, fuel, time)

    path = list(zip(winner["rx"], winner["ry"]))
    assert path[0] == start and path[-1] == goal
    assert fuel in path and time in path
    assert goal not in path[:-1], "Base reached before completing both drop-offs"

    lines = ["AAE1001 - Group 5 - Task A1", f"Seed: {args.seed}",
             f"Start: {start}; Base: {goal}",
             f"Fuel checkpoint: {fuel}", f"Time checkpoint: {time}", "",
             "Compare both checkpoint orders:"]
    for route in candidates:
        value = "unreachable" if route["time"] is None else f"{route['time']:.4f} minutes"
        lines.append(f"  {route['order']}: {value}")
    lines.extend(["", f"Selected order: {winner['order']}",
                  f"Total weighted travel time: {winner['time']:.4f} minutes",
                  f"Grid moves: {len(path) - 1}",
                  "Both checkpoints visited before first arrival at base: PASS",
                  "Model: original Task 1 map and 8-direction movement;",
                  "entering the yellow/pink area adds 15%/30% to step time.",
                  "Each order uses three A* searches; choose the smaller total."])
    summary = "\n".join(lines) + "\n"
    print(summary)
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".txt").write_text(summary)
    print(f"Figure: {output}")
    print(f"Summary: {output.with_suffix('.txt')}")
    plot_route(planner, start, goal, obstacles, winner, output, not args.no_show)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram stopped.")

