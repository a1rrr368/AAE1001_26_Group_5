"""Compare A*, Dijkstra, and RRT* in a 20x20x20 voxel scene."""

import heapq
import math
import random
import time
from io import BytesIO
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from PIL import Image


SIZE = 20
START = (1.0, 1.0, 1.0)
GOAL = (18.0, 18.0, 18.0)
FUEL_REGIONS = (
    (7, 10, 8, 12, 4, 9),
    (13, 16, 8, 11, 12, 15),
)
FUEL_FACTOR = 1.15
RRT_ITERATIONS = 10_000
COLORS = ("#1f77b4", "#ff7f0e", "#2ca02c")
CMAPS = ("Blues", "Oranges", "Greens")
OUTPUT_DIR = Path(__file__).resolve().parent


def build_scene():
    """Create fixed walls, a central block, an XY plate, and fuel cuboids."""
    obstacles = set()
    for y in range(SIZE):
        for z in range(SIZE):
            if not (2 <= y <= 6 and 2 <= z <= 6):
                obstacles.add((6, y, z))
            if not (13 <= y <= 17 and 13 <= z <= 17):
                obstacles.add((13, y, z))

    for x in range(8, 11):
        for y in range(8, 11):
            for z in range(8, 11):
                obstacles.add((x, y, z))
    obstacles.update(
        (x, y, 9)
        for x in range(7, 12)
        for y in range(7, 12)
    )

    fuel = set()
    for x0, x1, y0, y1, z0, z1 in FUEL_REGIONS:
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                for z in range(z0, z1 + 1):
                    fuel.add((x, y, z))
    return obstacles, fuel


def distance(first, second):
    return math.dist(first, second)


def grid_neighbors(point, obstacles):
    x, y, z = point
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                if dx == dy == dz == 0:
                    continue
                neighbor = (x + dx, y + dy, z + dz)
                if not all(0 <= value < SIZE for value in neighbor):
                    continue
                changed_axes = [
                    axis for axis, delta in enumerate((dx, dy, dz)) if delta
                ]
                blocked_corner = False
                for mask in range(1, (1 << len(changed_axes)) - 1):
                    intermediate = list(point)
                    for bit, axis in enumerate(changed_axes):
                        if mask & (1 << bit):
                            intermediate[axis] += (dx, dy, dz)[axis]
                    if tuple(intermediate) in obstacles:
                        blocked_corner = True
                        break
                if not blocked_corner and neighbor not in obstacles:
                    yield neighbor


def grid_edge_cost(first, second, fuel):
    midpoint = tuple((a + b) / 2 for a, b in zip(first, second))
    cell = tuple(math.floor(value + 0.5) for value in midpoint)
    multiplier = FUEL_FACTOR if cell in fuel else 1.0
    return distance(first, second) * multiplier


def grid_search(obstacles, fuel, heuristic):
    start = tuple(int(value) for value in START)
    goal = tuple(int(value) for value in GOAL)
    costs = {start: 0.0}
    parents = {start: None}
    queue = [(heuristic(start, goal), 0.0, start)]
    explored = []
    while queue:
        _, current_cost, current = heapq.heappop(queue)
        if current_cost != costs.get(current):
            continue
        explored.append(current)
        if current == goal:
            path = []
            while current is not None:
                path.append(tuple(float(v) for v in current))
                current = parents[current]
            path.reverse()
            return path, explored, current_cost

        for neighbor in grid_neighbors(current, obstacles):
            candidate_cost = current_cost + grid_edge_cost(current, neighbor, fuel)
            if candidate_cost < costs.get(neighbor, math.inf):
                costs[neighbor] = candidate_cost
                parents[neighbor] = current
                priority = candidate_cost + heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, candidate_cost, neighbor))
    raise RuntimeError("Grid planner could not find a route.")


def line_is_free(first, second, obstacles):
    segments = max(1, math.ceil(distance(first, second) * 4))
    for index in range(segments + 1):
        ratio = index / segments
        cell = tuple(
            min(SIZE - 1, max(0, math.floor(a + (b - a) * ratio + 0.5)))
            for a, b in zip(first, second)
        )
        if cell in obstacles:
            return False
    return True


def continuous_edge_cost(first, second, fuel):
    length = distance(first, second)
    segments = max(1, math.ceil(length * 2))
    cost = 0.0
    for index in range(segments):
        ratio = (index + 0.5) / segments
        cell = tuple(
            min(SIZE - 1, max(0, math.floor(a + (b - a) * ratio + 0.5)))
            for a, b in zip(first, second)
        )
        cost += length / segments * (FUEL_FACTOR if cell in fuel else 1.0)
    return cost


def update_descendant_costs(index, points, parents, costs, children, fuel):
    stack = list(children[index])
    while stack:
        child = stack.pop()
        costs[child] = (
            costs[parents[child]]
            + continuous_edge_cost(points[parents[child]], points[child], fuel)
        )
        stack.extend(children[child])


def rrt_star(obstacles, fuel, iterations=RRT_ITERATIONS, seed=0):
    rng = random.Random(seed)
    points = [START]
    parents = [-1]
    costs = [0.0]
    children = [set()]
    explored = []
    step_size = 2.6

    for _ in range(iterations):
        sample = (
            GOAL
            if rng.random() < 0.14
            else tuple(rng.uniform(0, SIZE - 1) for _ in range(3))
        )
        nearest = min(range(len(points)), key=lambda i: distance(points[i], sample))
        near_point = points[nearest]
        separation = distance(near_point, sample)
        if separation == 0:
            continue
        ratio = min(step_size, separation) / separation
        new_point = tuple(
            a + (b - a) * ratio for a, b in zip(near_point, sample)
        )
        if not line_is_free(near_point, new_point, obstacles):
            continue

        radius = min(
            4.0,
            max(1.6, 8.0 * (math.log(len(points) + 1) / (len(points) + 1)) ** (1 / 3)),
        )
        near_indices = [
            index for index, point in enumerate(points)
            if distance(point, new_point) <= radius
        ]
        if nearest not in near_indices:
            near_indices.append(nearest)

        parent = nearest
        new_cost = costs[parent] + continuous_edge_cost(
            points[parent], new_point, fuel
        )
        for candidate in near_indices:
            candidate_cost = costs[candidate] + continuous_edge_cost(
                points[candidate], new_point, fuel
            )
            if candidate_cost < new_cost and line_is_free(
                points[candidate], new_point, obstacles
            ):
                parent = candidate
                new_cost = candidate_cost

        new_index = len(points)
        points.append(new_point)
        parents.append(parent)
        costs.append(new_cost)
        children.append(set())
        children[parent].add(new_index)
        explored.append((near_point, new_point))

        for candidate in near_indices:
            if candidate == parent or candidate == 0:
                continue
            rewired_cost = new_cost + continuous_edge_cost(
                new_point, points[candidate], fuel
            )
            if rewired_cost < costs[candidate] and line_is_free(
                new_point, points[candidate], obstacles
            ):
                old_parent = parents[candidate]
                children[old_parent].remove(candidate)
                children[new_index].add(candidate)
                parents[candidate] = new_index
                costs[candidate] = rewired_cost
                update_descendant_costs(
                    candidate, points, parents, costs, children, fuel
                )

    goal_candidates = [
        (costs[index] + continuous_edge_cost(point, GOAL, fuel), index)
        for index, point in enumerate(points)
        if distance(point, GOAL) <= step_size
        and line_is_free(point, GOAL, obstacles)
    ]
    if not goal_candidates:
        raise RuntimeError(
            f"RRT* did not connect to the goal after {iterations} iterations."
        )
    route_cost, current = min(goal_candidates)
    path = [GOAL]
    while current != -1:
        path.append(points[current])
        current = parents[current]
    path.reverse()
    return path, explored, route_cost


def smooth_path(path, obstacles):
    smoothed = [path[0]]
    current = 0
    while current < len(path) - 1:
        candidate = len(path) - 1
        while candidate > current + 1 and not line_is_free(
            path[current], path[candidate], obstacles
        ):
            candidate -= 1
        smoothed.append(path[candidate])
        current = candidate
    return smoothed


def route_length(path):
    return sum(distance(a, b) for a, b in zip(path, path[1:]))


def run_planners(obstacles, fuel):
    euclidean = lambda point, goal: distance(point, goal)
    zero = lambda _point, _goal: 0.0

    def run_rrt():
        path, explored, _ = rrt_star(
            obstacles, fuel, iterations=RRT_ITERATIONS
        )
        path = smooth_path(path, obstacles)
        cost = sum(
            continuous_edge_cost(first, second, fuel)
            for first, second in zip(path, path[1:])
        )
        return path, explored, cost

    specs = (
        ("A*", lambda: grid_search(obstacles, fuel, euclidean)),
        ("Dijkstra", lambda: grid_search(obstacles, fuel, zero)),
        ("RRT*", run_rrt),
    )
    results = []
    for (name, run), color, cmap in zip(specs, COLORS, CMAPS):
        started = time.perf_counter()
        path, explored, cost = run()
        elapsed = time.perf_counter() - started
        results.append({
            "name": name,
            "path": path,
            "explored": explored,
            "cost": cost,
            "length": route_length(path),
            "elapsed": elapsed,
            "color": color,
            "cmap": cmap,
        })
        print(
            f"{name}: weighted cost={cost:.3f}, "
            f"distance={route_length(path):.3f}, "
            f"search={elapsed:.3f}s, explored={len(explored)}"
        )
    if results[2]["length"] >= min(
        results[0]["length"], results[1]["length"]
    ):
        raise RuntimeError(
            "RRT* did not produce the shortest route in this comparison scene."
        )
    print(
        f"RRT* smoothed route is shortest "
        f"({results[2]['length']:.3f} units; "
        f"{RRT_ITERATIONS} iterations)."
    )
    return results


def add_scene(axis, obstacles, start, goal, result):
    obstacle_points = np.asarray(list(obstacles))
    axis.scatter(
        obstacle_points[:, 0], obstacle_points[:, 1], obstacle_points[:, 2],
        c="#333333", marker="s", s=13, alpha=0.55, depthshade=False,
    )
    for x0, x1, y0, y1, z0, z1 in FUEL_REGIONS:
        axis.bar3d(
            x0 - 0.48, y0 - 0.48, z0 - 0.48,
            x1 - x0 + 0.96, y1 - y0 + 0.96, z1 - z0 + 0.96,
            color="gold", alpha=0.16, shade=False, edgecolor="#d6aa00",
            linewidth=0.35,
        )
    axis.scatter(*start, c="blue", marker="*", s=105, depthshade=False)
    axis.scatter(*goal, c="red", marker="*", s=105, depthshade=False)
    axis.set(
        xlim=(-0.5, SIZE - 0.5), ylim=(-0.5, SIZE - 0.5),
        zlim=(-0.5, SIZE - 0.5), xlabel="X", ylabel="Y", zlabel="Z",
    )
    axis.set_xticks(range(0, SIZE, 4))
    axis.set_yticks(range(0, SIZE, 4))
    axis.set_zticks(range(0, SIZE, 4))
    axis.set_box_aspect((1, 1, 1))
    axis.view_init(elev=25, azim=-55)
    axis.grid(True, alpha=0.18)
    axis.set_title(
        f"{result['name']} | cost {result['cost']:.2f} | "
        f"distance {result['length']:.2f}"
    )


def add_summary(figure, axis, results):
    axis.set_axis_off()
    metrics = (
        ("elapsed", "Search time (s)", "{:.3f}"),
        ("cost", "Weighted path cost", "{:.2f}"),
        ("length", "Path length", "{:.2f}"),
    )
    subgrid = axis.get_subplotspec().subgridspec(3, 1, hspace=0.8)
    for row, (key, title, value_format) in enumerate(metrics):
        chart = figure.add_subplot(subgrid[row, 0])
        values = [result[key] for result in results]
        bars = chart.barh(
            [result["name"] for result in results],
            values,
            color=COLORS,
        )
        chart.set_title(title, fontsize=10)
        chart.grid(axis="x", alpha=0.25)
        chart.set_axisbelow(True)
        chart.spines[["top", "right", "left"]].set_visible(False)
        chart.tick_params(axis="y", length=0)
        chart.set_xlim(0, max(values) * 1.32 if max(values) else 1)
        for bar, value in zip(bars, values):
            chart.text(
                value + max(values) * 0.025,
                bar.get_y() + bar.get_height() / 2,
                value_format.format(value),
                va="center",
                fontsize=8,
            )


def make_figure(obstacles, results):
    figure = plt.figure(figsize=(14, 10))
    grid = figure.add_gridspec(2, 2, hspace=0.3, wspace=0.25)
    map_axes = (
        figure.add_subplot(grid[0, 0], projection="3d"),
        figure.add_subplot(grid[0, 1], projection="3d"),
        figure.add_subplot(grid[1, 0], projection="3d"),
    )
    summary_axis = figure.add_subplot(grid[1, 1])
    figure.suptitle("A* vs Dijkstra vs RRT* — 20×20×20 3D map", fontsize=16)

    exploration_artists = []
    route_artists = []
    for axis, result in zip(map_axes, results):
        add_scene(axis, obstacles, START, GOAL, result)
        artist = axis.scatter(
            [], [], [], c=[], cmap=result["cmap"], vmin=0, vmax=1,
            s=12, alpha=0.7, depthshade=False,
        )
        route, = axis.plot(
            [], [], [], color=result["color"], linewidth=2.8, zorder=5
        )
        exploration_artists.append(artist)
        route_artists.append(route)

    add_summary(figure, summary_axis, results)
    figure.legend(
        handles=(
            Line2D([], [], marker="s", linestyle="none", color="#333333",
                   label="Fixed obstacle"),
            Patch(facecolor="gold", alpha=0.35, label="Fuel area (+15%)"),
            Line2D([], [], marker="*", linestyle="none", color="blue",
                   label="Start"),
            Line2D([], [], marker="*", linestyle="none", color="red",
                   label="Goal"),
        ),
        loc="upper center", ncol=4, bbox_to_anchor=(0.5, 0.94), frameon=False,
    )
    figure.subplots_adjust(top=0.88, left=0.04, right=0.98, bottom=0.05)
    return figure, map_axes, exploration_artists, route_artists


def save_outputs(figure, results, map_axes, exploration_artists, route_artists):
    gif_path = OUTPUT_DIR / "TaskA3_3D_comparison.gif"
    histogram_path = OUTPUT_DIR / "TaskA3_3D_histogram.png"
    exploration_frames = 12
    route_frames = 8
    images = []

    for frame in range(exploration_frames + route_frames):
        for index, (axis, result) in enumerate(zip(map_axes, results)):
            if frame < exploration_frames:
                end = int((frame + 1) * len(result["explored"]) / exploration_frames)
                explored = result["explored"][:end]
                if result["name"] == "RRT*":
                    coordinates = [point for _, point in explored]
                else:
                    coordinates = explored
                if coordinates:
                    points = np.asarray(coordinates, dtype=float)
                    artist = exploration_artists[index]
                    artist._offsets3d = (
                        points[:, 0], points[:, 1], points[:, 2]
                    )
                    artist.set_array(np.linspace(0.05, 1.0, len(points)))
                axis.set_title(
                    f"{result['name']} | exploring {end}/{len(result['explored'])}"
                )
            else:
                progress = (frame - exploration_frames + 1) / route_frames
                end = max(2, math.ceil(progress * len(result["path"])))
                route = result["path"][:end]
                coordinates = np.asarray(route)
                route_artists[index].set_data_3d(
                    coordinates[:, 0], coordinates[:, 1], coordinates[:, 2]
                )
                axis.set_title(
                    f"{result['name']} | cost {result['cost']:.2f} | "
                    f"distance {result['length']:.2f}"
                )
        figure.canvas.draw()
        buffer = BytesIO()
        figure.savefig(buffer, format="png", dpi=75)
        buffer.seek(0)
        with Image.open(buffer) as image:
            images.append(image.convert("RGB").copy())

    images[0].save(
        gif_path, save_all=True, append_images=images[1:],
        duration=100, loop=0, optimize=True,
    )
    figure.savefig(histogram_path, dpi=160, bbox_inches="tight")
    plt.close(figure)
    print(f"Saved animation: {gif_path}")
    print(f"Saved comparison histogram: {histogram_path}")


def main():
    obstacles, fuel = build_scene()
    results = run_planners(obstacles, fuel)
    figure, axes, exploration_artists, route_artists = make_figure(
        obstacles, results
    )
    save_outputs(figure, results, axes, exploration_artists, route_artists)


if __name__ == "__main__":
    main()
