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
