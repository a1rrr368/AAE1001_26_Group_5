"""
AAE1001 - Group 5 - Task A3.2: RRT*
Output-preserving maintenance (Matplotlib 3.10+ compatible).
Review notes: REVIEW CHANGES.md. Printed headings are retained for compatibility.
"""

import math
import random
import importlib.util
import sys
import time
from pathlib import Path
from matplotlib.collections import LineCollection
import numpy as np
import matplotlib.pyplot as plt
import warnings
from matplotlib import MatplotlibDeprecationWarning

import Task2_Final as astar_module

# REVIEW NOTES: Suppress only Matplotlib deprecation warnings, not runtime errors.
warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

show_animation = True   # False skips plotting; route and cost calculations still run.

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

        # REVIEW 1: Cache membership lookups; keep original lists for compatibility.
        # Independent x/y membership preserves the original rectangular-area rule.
        # Regions are static here; rebuild these caches if their lists are changed.
        self._fc_x = frozenset(fc_x)
        self._fc_y = frozenset(fc_y)
        self._tc_x = frozenset(tc_x)
        self._tc_y = frozenset(tc_y)
        self.step_size = step_size / resolution
        self.max_iter = max_iter
        self.goal_sample_rate = goal_sample_rate
        # REVIEW 3: Keep the local seed and sampling order unchanged for reproducibility.
        # A new planner repeats the seed; repeated planning() calls advance this RNG.
        self.rng = random.Random(seed)

        # REVIEW NOTES: Both regions add to weighted travel time in this model.
        # These are not separate fuel-only charges; overlapping penalties add.
        self.Delta_TC = 0.30   # Pink region: +30% of base movement cost.
        self.Delta_FC = 0.15   # Yellow region: +15% of base movement cost.
        self.costPerGrid = 1.0

    class Node:
        def __init__(self, x, y, cost=0.0, parent_index=-1):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy, progress_callback=None):
        start = self.Node(self.calc_xy_index(sx, self.min_x),
                          self.calc_xy_index(sy, self.min_y))
        goal = self.Node(self.calc_xy_index(gx, self.min_x),
                         self.calc_xy_index(gy, self.min_y))
        if not self.is_free(start.x, start.y) or not self.is_free(goal.x, goal.y):
            print("Start or goal is occupied – no path found")
            return [], [], None

        # REVIEW 2: Reuse the map-size factor; preserve operation order and rounding.
        radius_factor = 2.0 * math.sqrt(self.x_width * self.y_width / math.pi)
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
                radius_factor * math.sqrt(math.log(len(nodes) + 1) / (len(nodes) + 1)),
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
            if progress_callback is not None:
                parent = nodes[new_node.parent_index]
                progress_callback(
                    self.calc_grid_position(parent.x, self.min_x),
                    self.calc_grid_position(parent.y, self.min_y),
                    self.calc_grid_position(new_node.x, self.min_x),
                    self.calc_grid_position(new_node.y, self.min_y),
                )

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

    # REVIEW NOTES: Return goal-to-start coordinates; the existing plot accepts this order.
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

    # REVIEW NOTES: Collision checking samples at spacing <=0.5 grid units and
    # rounds each sample to an occupancy cell; it is not exact continuous geometry.
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

    # REVIEW NOTES: Accumulate midpoint-sampled Euclidean segment costs.
    # This differs from Dijkstra destination-cell penalties; keep native costs distinct.
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
            if round(x) in self._tc_x and round(y) in self._tc_y:
                penalty += self.Delta_TC
            if round(x) in self._fc_x and round(y) in self._fc_y:
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

def load_dijkstra_module():
    module_path = Path(__file__).with_name("TaskA3.1_Final.py")
    spec = importlib.util.spec_from_file_location("task_a3_1_dijkstra", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load Dijkstra planner from {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def score_route(rx, ry, fc_x, fc_y, tc_x, tc_y, resolution=1.0):
    """Score any returned route with the same midpoint-sampled surcharge model."""
    if len(rx) < 2:
        return None

    fuel_x, fuel_y = frozenset(fc_x), frozenset(fc_y)
    time_x, time_y = frozenset(tc_x), frozenset(tc_y)
    total = 0.0
    route = list(zip(rx, ry))
    for (x0, y0), (x1, y1) in zip(route, route[1:]):
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, math.ceil(distance / (resolution * 0.5)))
        step_distance = distance / steps
        for index in range(steps):
            ratio = (index + 0.5) / steps
            x = x0 + (x1 - x0) * ratio
            y = y0 + (y1 - y0) * ratio
            penalty = 1.0
            if round(x) in time_x and round(y) in time_y:
                penalty += 0.30
            if round(x) in fuel_x and round(y) in fuel_y:
                penalty += 0.15
            total += step_distance * penalty
    return total


def calculate_scenario1_cost(route_cost):
    """Return the least Scenario 1 operating cost for a shared route score."""
    aircraft = (
        {"name": "A321neo", "rate": 54, "capacity": 200, "time": 15, "fixed": 1800},
        {"name": "A330-900neo", "rate": 84, "capacity": 300, "time": 21, "fixed": 2000},
        {"name": "A350-900", "rate": 90, "capacity": 350, "time": 27, "fixed": 2500},
    )
    candidates = []
    for plane in aircraft:
        flights = math.ceil(3300 / plane["capacity"])
        if flights <= 13:
            per_flight = (
                0.85 * plane["rate"] * route_cost
                + plane["time"] * route_cost
                + plane["fixed"]
            )
            candidates.append((per_flight * flights, plane["name"], flights))
    return min(candidates)


def main():
    global show_animation

    print("=" * 72)
    print("A* vs Dijkstra vs RRT* — shared-map path planning comparison")
    print("A* jet-stream discount is disabled; all routes use the same map and regions.")
    print("=" * 72)

    sx, sy = 5.0, 40.0
    gx, gy = 20.0, 30.0
    grid_size = 1.0
    robot_radius = 0.5

    ox, oy = [], []
    for i in range(-10, 61):
        ox.extend((i, 60.0, i, -10.0))
        oy.extend((-10.0, i, 60.0, i))
    for i in range(10, 51):
        ox.append(float(i))
        oy.append(45.0)
    for y in range(10, 46):
        ox.append(10.0)
        oy.append(float(y))
    for k in range(101):
        ox.append(30.0 + 0.1 * k)
        oy.append(45.0 - 0.45 * k)

    fc_x, fc_y = [], []
    for x in range(-10, 31):
        for y in range(11):
            fc_x.append(x)
            fc_y.append(y)

    tc_x, tc_y = [], []
    for x in range(50, 61):
        for y in range(11):
            tc_x.append(x)
            tc_y.append(y)

    names = ("A*", "Dijkstra", "RRT*")
    colors = ("#1f77b4", "#ff7f0e", "#2ca02c")
    cmaps = ("Blues", "Oranges", "Greens")
    results = {
        name: {
            "elapsed": 0.0,
            "route_cost": None,
            "operating_cost": None,
            "aircraft": None,
            "flights": None,
            "explored": 0,
            "status": "Pending",
        }
        for name in names
    }

    fig = plt.figure(figsize=(18, 13))
    grid = fig.add_gridspec(2, 2)
    map_axes = [
        fig.add_subplot(grid[0, 0]),
        fig.add_subplot(grid[0, 1]),
        fig.add_subplot(grid[1, 0]),
    ]
    metric_grid = grid[1, 1].subgridspec(3, 1, hspace=0.6)
    metric_axes = [fig.add_subplot(metric_grid[i, 0]) for i in range(3)]
    fig.suptitle("Path planner comparison — light to dark shows exploration order", fontsize=16)
    fig.subplots_adjust(left=0.06, right=0.97, top=0.92, bottom=0.07,
                        wspace=0.30, hspace=0.28)

    exploration_artists = []
    for index, (axis, name) in enumerate(zip(map_axes, names)):
        axis.plot(ox, oy, ".k", markersize=2, label="Obstacles", zorder=1)
        axis.scatter(fc_x, fc_y, s=8, c="gold", alpha=0.22, label="Fuel area (+15%)")
        axis.scatter(tc_x, tc_y, s=8, c="magenta", alpha=0.18, label="Time area (+30%)")
        axis.plot(sx, sy, "b*", markersize=12, label="Start", zorder=5)
        axis.plot(gx, gy, "r*", markersize=12, label="Goal", zorder=5)
        axis.set_xlim(-12, 62)
        axis.set_ylim(-12, 62)
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True, alpha=0.2)
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_title(f"{name}: waiting")

        if name == "RRT*":
            artist = LineCollection(
                [],
                cmap=cmaps[index],
                linewidths=0.8,
                alpha=0.8,
                zorder=3,
            )
            artist.set_array(np.array([]))
            axis.add_collection(artist)
        else:
            artist = axis.scatter(
                [],
                [],
                c=[],
                cmap=cmaps[index],
                vmin=0,
                vmax=1,
                s=12,
                alpha=0.75,
                zorder=3,
            )
        fig.colorbar(artist, ax=axis, fraction=0.045, pad=0.025,
                     label="Exploration order")
        exploration_artists.append(artist)
        axis.legend(loc="lower left", fontsize=7)

    metric_specs = (
        ("elapsed", "Elapsed time (seconds)", "{:.2f} s"),
        ("route_cost", "Shared route score (minutes)", "{:.2f}"),
        ("operating_cost", "Scenario 1 minimum total cost ($)", "${:,.0f}"),
    )
    metric_bars = []
    metric_labels = []
    for axis, (key, title, fmt) in zip(metric_axes, metric_specs):
        bars = axis.barh(names, [0.0] * len(names), color=colors)
        axis.set_title(title, fontsize=10)
        axis.grid(axis="x", alpha=0.25)
        axis.set_axisbelow(True)
        axis.set_xlim(0, 1)
        labels = [
            axis.text(0.02, bar.get_y() + bar.get_height() / 2, "Pending",
                      va="center", ha="left", fontsize=8)
            for bar in bars
        ]
        metric_bars.append((key, fmt, bars))
        metric_labels.append(labels)

    progress_data = [
        {"points": [], "segments": [], "last_refresh": 0, "last_drawn": 0}
        for _ in names
    ]
    refresh_every = 100

    def refresh_dashboard():
        for chart_index, (key, fmt, bars) in enumerate(metric_bars):
            values = [results[name][key] or 0.0 for name in names]
            max_value = max(values, default=0.0)
            metric_axes[chart_index].set_xlim(0, max(1.0, max_value * 1.2))
            for name, bar, label in zip(
                    names, bars, metric_labels[chart_index]):
                value = results[name][key]
                bar.set_width(value or 0.0)
                if value is None:
                    label.set_text(results[name]["status"])
                    label.set_x(0.02 * metric_axes[chart_index].get_xlim()[1])
                else:
                    label.set_text(fmt.format(value))
                    label.set_x(value + max_value * 0.015)
        fig.canvas.draw_idle()

    def update_map(index, force=False):
        data = progress_data[index]
        if not force and data["last_refresh"] - data["last_drawn"] < refresh_every:
            return
        artist = exploration_artists[index]
        if index == 2:
            artist.set_segments(data["segments"])
            artist.set_array(np.linspace(0.05, 1.0, len(data["segments"])))
        else:
            artist.set_offsets(np.asarray(data["points"], dtype=float).reshape(-1, 2))
            artist.set_array(np.linspace(0.05, 1.0, len(data["points"])))
        data["last_drawn"] = data["last_refresh"]
        axis = map_axes[index]
        status = results[names[index]]["status"]
        axis.set_title(f"{names[index]}: {status} | expanded {results[names[index]]['explored']}")

    def progress_callback(index, start_time):
        def update(*coordinates):
            data = progress_data[index]
            if index == 2:
                data["segments"].append(
                    [(coordinates[0], coordinates[1]),
                     (coordinates[2], coordinates[3])]
                )
                results[names[index]]["explored"] = len(data["segments"])
            else:
                data["points"].append((coordinates[0], coordinates[1]))
                results[names[index]]["explored"] = len(data["points"])
            data["last_refresh"] += 1
            results[names[index]]["elapsed"] = time.perf_counter() - start_time
            if data["last_refresh"] - data["last_drawn"] >= refresh_every:
                update_map(index, force=True)
                refresh_dashboard()
                plt.pause(0.001)
        return update

    plt.show(block=False)
    plt.pause(0.1)

    dijkstra_module = load_dijkstra_module()
    astar_module.JET_STREAM_FACTOR = 1.0
    astar_module.JET_STREAM_Y_MIN = 5.0
    astar_module.show_animation = False
    dijkstra_module.show_animation = False
    show_animation = False

    planners = (
        ("A*", 0, lambda: astar_module.AStarPlanner(
            ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y
        )),
        ("Dijkstra", 1, lambda: dijkstra_module.DijkstraPlanner(
            ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y
        )),
        ("RRT*", 2, lambda: RRTStarPlanner(
            ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y,
            seed=0
        )),
    )

    for name, index, planner_factory in planners:
        print(f"\nRunning {name}...")
        results[name]["status"] = "Running"
        map_axes[index].set_title(f"{name}: preparing")
        refresh_dashboard()
        plt.pause(0.01)

        planner = planner_factory()
        started = time.perf_counter()
        if name == "A*":
            rx, ry, native_cost = planner.planning(
                sx, sy, gx, gy,
                animate=False,
                print_result=False,
                progress_callback=progress_callback(index, started),
            )
        else:
            rx, ry, native_cost = planner.planning(
                sx, sy, gx, gy,
                progress_callback=progress_callback(index, started),
            )
        elapsed = time.perf_counter() - started

        results[name]["elapsed"] = elapsed
        if native_cost is None:
            results[name]["status"] = "No path found"
            print(f"{name}: no path found.")
            update_map(index, force=True)
            refresh_dashboard()
            continue

        shared_cost = score_route(
            rx, ry, fc_x, fc_y, tc_x, tc_y, grid_size
        )
        operating_cost, aircraft_name, flight_count = calculate_scenario1_cost(shared_cost)
        path_length = sum(
            math.hypot(rx[i + 1] - rx[i], ry[i + 1] - ry[i])
            for i in range(len(rx) - 1)
        )
        results[name].update({
            "route_cost": shared_cost,
            "operating_cost": operating_cost,
            "aircraft": aircraft_name,
            "flights": flight_count,
            "status": "Complete",
        })

        update_map(index, force=True)
        map_axes[index].plot(
            rx, ry, color=colors[index], linewidth=2.5,
            label=f"Final path ({path_length:.1f} units)", zorder=4
        )
        map_axes[index].legend(loc="lower left", fontsize=7)
        map_axes[index].set_title(
            f"{name}: {elapsed:.2f}s | shared score {shared_cost:.2f}"
        )
        refresh_dashboard()
        plt.pause(0.05)

        print(
            f"{name}: runtime={elapsed:.3f}s, native route cost={native_cost:.4f}, "
            f"shared route score={shared_cost:.4f}, path length={path_length:.2f}, "
            f"Scenario 1=${operating_cost:.2f} "
            f"({flight_count} x {aircraft_name}), "
            f"explored={results[name]['explored']}"
        )

    print("\nComparison complete. Close the figure window to exit.")
    refresh_dashboard()
    plt.show()


if __name__ == "__main__":
    main()
