"""
AAE1001 - Group 5 - Task 1
Full fixed version (Matplotlib 3.10+ compatible)
"""

import math
import random
import matplotlib.pyplot as plt
import warnings
from matplotlib import MatplotlibDeprecationWarning

# Suppress the deprecation warnings
warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

show_animation = True   # set to False if you only want numbers

class RRTStarPlanner:

    def __init__(self, ox, oy, resolution, rr, fc_x, fc_y, tc_x, tc_y,
                 step_size=3.0, max_iter=5000, goal_sample_rate=0.1, seed=0):
        self.resolution = resolution
        self.rr = rr
        self.min_x = self.min_y = 0
        self.max_x = self.max_y = 0
        self.obstacle_map = None
        self.x_width = self.y_width = 0
        self.calc_obstacle_map(ox, oy)

        self.fc_x = fc_x
        self.fc_y = fc_y
        self.tc_x = tc_x
        self.tc_y = tc_y
        self.step_size = step_size / resolution
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        self.rng = random.Random(seed)

        self.Delta_TC = 0.30   # +30 % time-consuming area (pink)
        self.Delta_FC = 0.15   # +15 % fuel-consuming area (yellow)
        self.costPerGrid = 1.0

    class Node:
        def __init__(self, x, y, cost=0.0, parent_index=-1):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):
        start = self.Node(self.calc_xy_index(sx, self.min_x),
                          self.calc_xy_index(sy, self.min_y))
        goal = self.Node(self.calc_xy_index(gx, self.min_x),
                         self.calc_xy_index(gy, self.min_y))
        if not self.is_free(start.x, start.y) or not self.is_free(goal.x, goal.y):
            print("Start or goal is occupied – no path found")
            return [], [], None

        nodes = [start]
        children = [[]]
        goal_index = None
        for iteration in range(self.max_iter):
            if self.rng.random() < self.goal_sample_rate:
                sample = goal
            else:
                sample = self.Node(
                    self.rng.uniform(0, self.x_width - 1),
                    self.rng.uniform(0, self.y_width - 1),
                )

            nearest_index = min(
                range(len(nodes)),
                key=lambda i: self.distance(nodes[i], sample),
            )
            nearest = nodes[nearest_index]
            distance = self.distance(nearest, sample)
            if distance == 0:
                continue

            scale = min(self.step_size, distance) / distance
            new_node = self.Node(
                nearest.x + (sample.x - nearest.x) * scale,
                nearest.y + (sample.y - nearest.y) * scale,
            )
            if not self.is_collision_free(nearest, new_node):
                continue

            radius = min(
                self.step_size,
                2.0 * math.sqrt(self.x_width * self.y_width / math.pi)
                * math.sqrt(math.log(len(nodes) + 1) / (len(nodes) + 1)),
            )
            near_indices = [
                i for i, node in enumerate(nodes)
                if self.distance(node, new_node) <= radius
            ]
            if nearest_index not in near_indices:
                near_indices.append(nearest_index)

            parent_index = nearest_index
            new_node.cost = nearest.cost + self.segment_cost(nearest, new_node)
            for candidate_index in near_indices:
                candidate = nodes[candidate_index]
                if not self.is_collision_free(candidate, new_node):
                    continue
                candidate_cost = candidate.cost + self.segment_cost(candidate, new_node)
                if candidate_cost < new_node.cost:
                    parent_index = candidate_index
                    new_node.cost = candidate_cost

            new_node.parent_index = parent_index
            nodes.append(new_node)
            children.append([])
            new_index = len(nodes) - 1
            children[parent_index].append(new_index)

            for neighbor_index in near_indices:
                neighbor = nodes[neighbor_index]
                if neighbor_index == parent_index or self.is_ancestor(
                        nodes, neighbor_index, new_index):
                    continue
                if not self.is_collision_free(new_node, neighbor):
                    continue
                rewired_cost = new_node.cost + self.segment_cost(new_node, neighbor)
                if rewired_cost < neighbor.cost:
                    children[neighbor.parent_index].remove(neighbor_index)
                    neighbor.parent_index = new_index
                    neighbor.cost = rewired_cost
                    children[new_index].append(neighbor_index)
                    self.update_descendant_costs(nodes, children, neighbor_index)

            if goal_index is None and self.distance(new_node, goal) <= self.step_size \
                    and self.is_collision_free(new_node, goal):
                goal.parent_index = new_index
                goal.cost = new_node.cost + self.segment_cost(new_node, goal)
                nodes.append(goal)
                children.append([])
                goal_index = len(nodes) - 1
                children[new_index].append(goal_index)

            if show_animation:
                plt.plot(
                    [self.calc_grid_position(nearest.x, self.min_x),
                     self.calc_grid_position(new_node.x, self.min_x)],
                    [self.calc_grid_position(nearest.y, self.min_y),
                     self.calc_grid_position(new_node.y, self.min_y)],
                    "-c", linewidth=0.5,
                )
                if iteration % 20 == 0:
                    plt.pause(0.001)

        if goal_index is None:
            print(f"RRT* reached its iteration limit ({self.max_iter}) – no path found")
            return [], [], None

        goal_node = nodes[goal_index]
        print(f"\n>>> RRT* best trip time after {self.max_iter} iterations "
              f"(T_best) = {goal_node.cost:.4f} minutes")
        rx, ry = self.calc_final_path(nodes, goal_index)
        return rx, ry, goal_node.cost

    @staticmethod
    def is_ancestor(nodes, ancestor_index, node_index):
        while node_index != -1:
            if node_index == ancestor_index:
                return True
            node_index = nodes[node_index].parent_index
        return False

    def update_descendant_costs(self, nodes, children, root_index):
        pending = [root_index]
        while pending:
            parent_index = pending.pop()
            parent = nodes[parent_index]
            for child_index in children[parent_index]:
                child = nodes[child_index]
                child.cost = parent.cost + self.segment_cost(parent, child)
                pending.append(child_index)

    def calc_final_path(self, nodes, goal_index):
        rx, ry = [], []
        current_index = goal_index
        while current_index != -1:
            node = nodes[current_index]
            rx.append(self.calc_grid_position(node.x, self.min_x))
            ry.append(self.calc_grid_position(node.y, self.min_y))
            current_index = node.parent_index
        return rx, ry

    @staticmethod
    def distance(n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def is_free(self, x, y):
        ix, iy = round(x), round(y)
        if ix < 0 or iy < 0 or ix >= self.x_width or iy >= self.y_width:
            return False
        return not self.obstacle_map[ix][iy]

    def is_collision_free(self, start, end):
        distance = self.distance(start, end)
        steps = max(1, math.ceil(distance / 0.5))
        for i in range(steps + 1):
            ratio = i / steps
            x = start.x + (end.x - start.x) * ratio
            y = start.y + (end.y - start.y) * ratio
            if not self.is_free(x, y):
                return False
        return True

    def segment_cost(self, start, end):
        distance = self.distance(start, end)
        steps = max(1, math.ceil(distance / 0.5))
        step_distance = distance / steps
        total_cost = 0.0
        for i in range(steps):
            ratio = (i + 0.5) / steps
            x = self.calc_grid_position(
                start.x + (end.x - start.x) * ratio, self.min_x
            )
            y = self.calc_grid_position(
                start.y + (end.y - start.y) * ratio, self.min_y
            )
            penalty = self.costPerGrid
            if round(x) in self.tc_x and round(y) in self.tc_y:
                penalty += self.Delta_TC
            if round(x) in self.fc_x and round(y) in self.fc_y:
                penalty += self.Delta_FC
            total_cost += step_distance * penalty
        return total_cost

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

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
        plt.title("Group 5 – RRT* Path Planning")
        plt.legend(loc="upper right")

    # ---------- Run RRT* ----------
    planner = RRTStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rx, ry, T_best = planner.planning(sx, sy, gx, gy)

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