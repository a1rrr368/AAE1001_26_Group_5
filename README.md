import math
import heapq
import matplotlib.pyplot as plt


show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution
        self.rr = rr
        self.min_x = -10
        self.max_x = 60
        self.min_y = -10
        self.max_y = 60

        self.obstacles = set(zip(ox, oy))

        self.motion = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2))
        ]

    def planning(self, sx, sy, gx, gy):

        start = (round(sx), round(sy))
        goal = (round(gx), round(gy))

        open_set = []
        heapq.heappush(
            open_set,
            (0.0, start)
        )

        g_cost = {start: 0.0}
        parent = {}
        closed_set = set()

        while open_set:

            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            if current == goal:
                break

            closed_set.add(current)

            x, y = current

            for dx, dy, move_cost in self.motion:

                nx = x + dx
                ny = y + dy

                neighbor = (nx, ny)

                if not (
                    self.min_x < nx < self.max_x
                    and self.min_y < ny < self.max_y
                ):
                    continue

                if neighbor in self.obstacles:
                    continue

                if neighbor in closed_set:
                    continue

                # Prevent diagonal movement through blocked corners
                if dx != 0 and dy != 0:
                    if (
                        (x + dx, y) in self.obstacles
                        or (x, y + dy) in self.obstacles
                    ):
                        continue

                new_cost = g_cost[current] + move_cost

                if new_cost < g_cost.get(
                    neighbor, float("inf")
                ):

                    g_cost[neighbor] = new_cost
                    parent[neighbor] = current

                    heuristic = math.hypot(
                        gx - nx,
                        gy - ny
                    )

                    f_cost = new_cost + heuristic

                    heapq.heappush(
                        open_set,
                        (f_cost, neighbor)
                    )

        if goal not in g_cost:
            print("No path found.")
            return [], []

        path = [goal]
        current = goal

        while current != start:
            current = parent[current]
            path.append(current)

        path.reverse()

        rx = [point[0] for point in path]
        ry = [point[1] for point in path]

        return rx, ry


def main():

    # Start and goal coordinates
    sx, sy = 5, 40
    gx, gy = 20, 30

    # A* parameters
    grid_size = 1.0
    robot_radius = 0.0

    # Set obstacle positions
    ox, oy = [], []

    # Boundary obstacles
    for i in range(-10, 61):
        ox.append(i)
        oy.append(-10.0)

    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)

    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)

    for i in range(-10, 61):
        ox.append(60.0)
        oy.append(i)

    # Vertical obstacle: (10,10) to (10,45)
    for i in range(10, 46):
        ox.append(10.0)
        oy.append(float(i))

    # Horizontal obstacle: (10,45) to (50,45)
    for i in range(10, 51):
        ox.append(float(i))
        oy.append(45.0)

    # Diagonal obstacle: (30,45) to (40,0)
    for i in range(101):
        t = i / 100.0

        x = 30.0 + 10.0 * t
        y = 45.0 - 45.0 * t

        ox.append(x)
        oy.append(y)

    # Create the A* planner
    a_star = AStarPlanner(
        ox, oy, grid_size, robot_radius
    )

    # Calculate the path
    rx, ry = a_star.planning(
        sx, sy, gx, gy
    )

    if not rx:
        return

    # Calculate the travel distance and time
    total_distance = 0.0

    for i in range(1, len(rx)):
        dx = rx[i] - rx[i - 1]
        dy = ry[i] - ry[i - 1]

        total_distance += math.hypot(dx, dy)

    print("Start node:", (sx, sy))
    print("Goal node:", (gx, gy))
    print("Number of path nodes:", len(rx))
    print("Total distance:", round(total_distance, 2))
    print("Travel time without area penalties:",
          round(total_distance, 2), "minutes")

    # Plot the obstacles and path
    if show_animation:

        plt.figure(figsize=(10, 8))

        # Fuel-consuming area
        plt.fill(
            [0, 30, 30, 0],
            [0, 0, 10, 10],
            color="yellow",
            alpha=0.6,
            label="Fuel-consuming area"
        )

        # Time-consuming area
        plt.fill(
            [50, 60, 60, 50],
            [-10, -10, 0, 0],
            color="red",
            alpha=0.5,
            label="Time-consuming area"
        )

        # Obstacles
        plt.plot(ox, oy, ".k", markersize=2)

        # Start and goal
        plt.plot(
            sx, sy, "*b",
            markersize=14,
            label="Start"
        )

        plt.plot(
            gx, gy, "*g",
            markersize=14,
            label="Goal"
        )

        # A* path
        plt.plot(
            rx, ry, "-r",
            linewidth=2,
            label="A* path"
        )

        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Group 5 - A* Pathfinding")

        plt.xlim(-10, 60)
        plt.ylim(-10, 60)

        plt.xticks(range(-10, 61, 10))
        plt.yticks(range(-10, 61, 10))

        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.show()


if __name__ == "__main__":
    main()


----------------

"""
AAE1001 - Group 5 - Task 2
Full fixed version (Matplotlib 3.10+ compatible)
"""

import math
import matplotlib.pyplot as plt
import warnings
from matplotlib import MatplotlibDeprecationWarning

# Suppress the deprecation warnings
warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

show_animation = False   # set to False if you only want numbers
# Task 2: horizontal jet stream across the map
JET_STREAM_Y_MIN = 5.0
JET_STREAM_HEIGHT = 5.0
JET_STREAM_FACTOR = 0.95

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

        self.Delta_TC = 0.30   # +30 % time-consuming area (pink)
        self.Delta_FC = 0.15   # +15 % fuel-consuming area (yellow)
        self.costPerGrid = 1.0

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node  = self.Node(self.calc_xy_index(gx, self.min_x),
                               self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if not open_set:
                print("Open set is empty – no path found")
                return [], [], None

            c_id = min(open_set,
                       key=lambda o: open_set[o].cost +
                       self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            # Fixed plotting (no float deprecation warning)
            if show_animation:
                x_pos = int(round(self.calc_grid_position(current.x, self.min_x)))
                y_pos = int(round(self.calc_grid_position(current.y, self.min_y)))
                plt.plot(x_pos, y_pos, "xc")
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set) % 20 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print(f"\n>>> Total Trip time required (T_best) = {current.cost:.4f} minutes")
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

                # Time-consuming area (pink) +30%
                if px in self.tc_x and py in self.tc_y:
                    node.cost += self.Delta_TC * self.motion[i][2]

                # Fuel-consuming area (yellow) +15%
                if px in self.fc_x and py in self.fc_y:
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
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]


def main():
    print("=" * 60)
    print("AAE1001 – Group 5 – Task 1")
    print("=" * 60)

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

    # ---------- Plot setup ----------
    if show_animation:
        plt.figure(figsize=(10, 8))
        plt.plot(ox, oy, ".k", markersize=2, label="Obstacles")
        plt.plot(fc_x, fc_y, "oy", markersize=3, alpha=0.4, label="Fuel-consuming (+15%)")
        plt.plot(tc_x, tc_y, "or", markersize=3, alpha=0.4, label="Time-consuming (+30%)")
        plt.plot(sx, sy, "b*", markersize=15, label="Start")
        plt.plot(gx, gy, "g*", markersize=15, label="Goal")
        plt.grid(True)
        plt.axis("equal")
        plt.title("Group 5 – Path Planning")
        plt.legend(loc="upper right")

    # ---------- Run A* ----------
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rx, ry, T_best = a_star.planning(sx, sy, gx, gy)

    if T_best is None:
        print("No path found!")
        return

    if show_animation:
        plt.plot(rx, ry, "-r", linewidth=2, label="Optimal path")
        plt.legend(loc="upper right")
        plt.show()

    # ============================================================
    # Cost calculation for the three scenarios
    # ============================================================
    print("\n" + "=" * 60)
    print(f"T_best used for all scenarios = {T_best:.4f} minutes")
    print("=" * 60)

    aircraft = {
        "A321neo":      {"rate": 54, "cap": 200, "low": 10, "med": 15, "high": 20, "Cc": 1800},
        "A330-900neo":  {"rate": 84, "cap": 300, "low": 15, "med": 21, "high": 27, "Cc": 2000},
        "A350-900":     {"rate": 90, "cap": 350, "low": 20, "med": 27, "high": 34, "Cc": 2500},
    }

    scenarios = [
        {"name": "Scenario 1", "pax": 3300, "max_f": 13, "time": "med",  "Cf": 0.85},
        {"name": "Scenario 2", "pax": 1500, "max_f":  7, "time": "high", "Cf": 0.96},
        {"name": "Scenario 3", "pax": 2250, "max_f": 25, "time": "low",  "Cf": 0.78},
    ]

    for sc in scenarios:
        print(f"\n----- {sc['name']} -----")
        print(f"Passengers: {sc['pax']} | Max flights: {sc['max_f']} | "
              f"Time cost: {sc['time']} | Fuel price: {sc['Cf']} $/kg")

        best_total = float("inf")
        best_name = None
        best_n = None

        for name, ac in aircraft.items():
            n_needed = math.ceil(sc["pax"] / ac["cap"])
            if n_needed > sc["max_f"]:
                print(f"  {name:12s}: needs {n_needed} flights > max {sc['max_f']} → INFEASIBLE")
                continue

            Ct = ac[sc["time"]]
            trip = sc["Cf"] * ac["rate"] * T_best + Ct * T_best + ac["Cc"]
            total = trip * n_needed

            print(f"  {name:12s}: {n_needed:2d} flights × ${trip:8.2f} = ${total:10.2f}")

            if total < best_total:
                best_total = total
                best_name = name
                best_n = n_needed

        print(f"  >>> BEST: {best_n} flights of {best_name}  →  total cost ${best_total:.2f}")


if __name__ == "__main__":
    main()
