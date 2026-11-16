"""
AAE1001 - Group 5 - Task 1 & Task 2
Full fixed version (Matplotlib 3.10+ compatible) with Jet Stream integration
"""

import math
import matplotlib.pyplot as plt
import warnings
from matplotlib import MatplotlibDeprecationWarning

# REVIEW NOTE: Suppress Matplotlib deprecation warnings only; runtime errors remain visible.
warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

show_animation = True   # False skips plotting, but still computes the route and all costs.

# ============================================================
# Task 2: Jet Stream Parameters (You can change JET_STREAM_Y_MIN here)
# ============================================================

# The main routine sweeps candidate lower y-coordinates and selects the
# placement with the lowest Scenario 1 total cost.
JET_STREAM_Y_MIN = 5.0
JET_STREAM_HEIGHT = 5.0
JET_STREAM_FACTOR = 0.95  # 5% off base movement cost inside the band, not total trip cost.

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

        # REVIEW CHANGE 1: Cache static region membership to avoid repeated list scans.
        # Preserve independent x/y checks and the original lists used by callers.
        # Rebuild these caches if the region coordinate lists are changed later.
        self._fc_x = frozenset(fc_x)
        self._fc_y = frozenset(fc_y)
        self._tc_x = frozenset(tc_x)
        self._tc_y = frozenset(tc_y)

        # REVIEW NOTE: Lecture p.5 treats both regions as extra flight time.
        # Keep the existing destination-cell charging rule for these regions.
        # In an overlap the penalties add; these are not separate fuel-only charges.
        self.Delta_TC = 0.30   # Pink: +30% of the base step cost.
        self.Delta_FC = 0.15   # Yellow: +15% of the base step cost.
        self.costPerGrid = 1.0

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy, animate=None, print_result=True,
                 progress_callback=None):
        if animate is None:
            animate = show_animation

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node  = self.Node(self.calc_xy_index(gx, self.min_x),
                               self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        # REVIEW CHANGE 2: Register Escape once per search instead of per visited node.
        if animate:
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

        while True:
            if not open_set:
                print("Open set is empty – no path found")
                return [], [], None

            # REVIEW NOTE: Keep insertion-order tie breaking to preserve the route.
            c_id = min(open_set,
                       key=lambda o: open_set[o].cost +
                       self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]
            if progress_callback is not None:
                progress_callback(
                    self.calc_grid_position(current.x, self.min_x),
                    self.calc_grid_position(current.y, self.min_y),
                )

            if animate:
                x_pos = int(round(self.calc_grid_position(current.x, self.min_x)))
                y_pos = int(round(self.calc_grid_position(current.y, self.min_y)))
                plt.plot(x_pos, y_pos, "xc")
                if len(closed_set) % 20 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                if print_result:
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

                # Pink: add 30% of the base step on entering the destination cell.
                if px in self._tc_x and py in self._tc_y:
                    node.cost += self.Delta_TC * self.motion[i][2]

                # Yellow: add 15% to weighted step time under the existing model.
                if px in self._fc_x and py in self._fc_y:
                    node.cost += self.Delta_FC * self.motion[i][2]

                # REVIEW NOTE: Inclusive band boundaries; subtract 5% of the base step.
                # Regional surcharges above remain additive and are not discounted.
                # REVIEW TODO (not applied): Lecture p.23 says cost falls by 5%;
                # document/confirm this base-step interpretation before claiming compliance.
                if JET_STREAM_Y_MIN <= py <= (JET_STREAM_Y_MIN + JET_STREAM_HEIGHT):
                    node.cost -= (1.0 - JET_STREAM_FACTOR) * self.motion[i][2] * self.costPerGrid

                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue
                if n_id in closed_set:
                    continue
                if n_id not in open_set or open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry, goal_node.cost

    # REVIEW NOTE: Coordinates are returned goal-to-start, as in the original plot.
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

    # REVIEW TODO (not applied): Discounted edges can cost less than this heuristic.
    # Consider a 0.95-scaled bound/reopening nodes before claiming global optimality.
    # Keep the current heuristic/search policy here because changing it may alter outputs.
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

        # REVIEW CHANGE 3: Reuse paired obstacle coordinates in the grid scan.
        # Keep pair order and the exact hypot <= radius collision calculation.
        obstacle_points = tuple(zip(ox, oy))
        self.obstacle_map = [[False] * self.y_width for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in obstacle_points:
                    if math.hypot(iox - x, ioy - y) <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    # REVIEW TODO (not applied): Lecture p.5 specifies diagonal=2 minutes.
    # The legacy sqrt(2) cost is retained to keep the existing outputs unchanged.
    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]


def s1_cost(t):
    """Scenario 1 total for 11 A330-900neo flights."""
    return 11 * (0.85 * 84 * t + 21 * t + 2000)


def sweep_jet_stream(planner, sx, sy, gx, gy, y_values=range(-10, 56)):
    """Evaluate the Scenario 1 cost for each candidate jet-stream placement."""
    global JET_STREAM_Y_MIN

    results = []
    for y0 in y_values:
        JET_STREAM_Y_MIN = float(y0)
        _, _, trip_time = planner.planning(
            sx, sy, gx, gy, animate=False, print_result=False)
        cost = s1_cost(trip_time) if trip_time is not None else float("inf")
        results.append({
            "y_lo": y0,
            "t_best": trip_time,
            "s1_cost": cost,
        })
    return results


def main():
    global JET_STREAM_Y_MIN

    print("=" * 60)
    print("AAE1001 – Group 5 – Task 1 & Task 2 (Jet Stream)")
    print("=" * 60)

    # REVIEW NOTE: Lecture p.13 Group 5: S(5,40), E(20,30).
    # Preserve the supplied barriers and yellow/pink region bounds.
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

    # ---------- Sweep jet-stream placement and choose minimum Scenario 1 cost ----------
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, fc_x, fc_y, tc_x, tc_y)
    rows = sweep_jet_stream(a_star, sx, sy, gx, gy)
    best = min(rows, key=lambda row: row["s1_cost"])
    best_cost = best["s1_cost"]
    best_rows = [row for row in rows if row["s1_cost"] == best_cost]

    print(f"\n{'y_lo':>5} {'band':>13} {'T_best':>10} {'S1 cost':>12}")
    for row in rows:
        if row["t_best"] is None:
            print(f"{row['y_lo']:>5} [{row['y_lo']:>3},{row['y_lo'] + 5:>3}] "
                  f"{'NO PATH':>10} {'N/A':>12}")
            continue
        mark = "  <-- best" if row["s1_cost"] == best_cost else ""
        print(f"{row['y_lo']:>5} [{row['y_lo']:>3},{row['y_lo'] + 5:>3}] "
              f"{row['t_best']:>10.4f} {row['s1_cost']:>12.2f}{mark}")

    if best["t_best"] is None:
        print("No feasible route found for any jet-stream placement.")
        return

    JET_STREAM_Y_MIN = float(best["y_lo"])
    rx, ry, T_best = a_star.planning(
        sx, sy, gx, gy, animate=False, print_result=False)
    if T_best is None:
        print("No path found at the selected jet-stream placement.")
        return

    print(f"\nBest jet-stream lower y-coordinate: {best['y_lo']}")
    if len(best_rows) > 1:
        tied_y_values = ", ".join(str(row["y_lo"]) for row in best_rows)
        print(f"Other placements with the same minimum cost: {tied_y_values}")
    print(f"Best band: [{best['y_lo']}, {best['y_lo'] + JET_STREAM_HEIGHT}]")
    print(f"Best Scenario 1 total cost: ${best['s1_cost']:.2f}")

    # ---------- Plot the selected placement and its route ----------
    if show_animation:
        valid_rows = [row for row in rows if row["t_best"] is not None]
        plt.figure(figsize=(10, 6))
        plt.plot(
            [row["y_lo"] for row in valid_rows],
            [row["s1_cost"] for row in valid_rows],
            "-o",
            markersize=4,
            label="Minimum Scenario 1 cost",
        )
        plt.scatter(
            [row["y_lo"] for row in best_rows],
            [row["s1_cost"] for row in best_rows],
            color="red",
            zorder=3,
            label="Minimum-cost placement",
        )
        plt.xlabel("Jet-stream lower boundary y-coordinate")
        plt.ylabel("Minimum Scenario 1 total cost ($)")
        plt.title("Scenario 1 Cost by Jet-Stream Placement")
        plt.grid(True)
        plt.legend()

        js_x, js_y = [], []
        for i in range(-10, 61):
            for j in range(int(JET_STREAM_Y_MIN),
                           int(JET_STREAM_Y_MIN + JET_STREAM_HEIGHT) + 1):
                js_x.append(i)
                js_y.append(float(j))

        plt.figure(figsize=(10, 8))
        plt.plot(ox, oy, ".k", markersize=2, label="Obstacles")
        plt.plot(fc_x, fc_y, "oy", markersize=3, alpha=0.4, label="Fuel-consuming (+15%)")
        plt.plot(tc_x, tc_y, "or", markersize=3, alpha=0.4, label="Time-consuming (+30%)")
        plt.plot(js_x, js_y, "sb", markersize=3, alpha=0.2,
                 label=f"Jet Stream y:[{int(JET_STREAM_Y_MIN)}, "
                       f"{int(JET_STREAM_Y_MIN + JET_STREAM_HEIGHT)}] (-5%)")
        plt.plot(sx, sy, "b*", markersize=15, label="Start")
        plt.plot(gx, gy, "g*", markersize=15, label="Goal")
        plt.plot(rx, ry, "-r", linewidth=2, label="Optimal path")
        plt.grid(True)
        plt.axis("equal")
        plt.title("Group 5 – Path Planning with Jet Stream (Task 2)")
        plt.legend(loc="upper right")
        plt.show()

    # ============================================================
    # Cost calculation for the three scenarios (retain existing output order).
    # REVIEW NOTE: Per flight = Cf * rate * T_best + Ct * T_best + Cc.
    # rate is kg/min; rate * T_best is fuel mass. Do not multiply by time twice.
    # Use unrounded costs and ceil(passengers / capacity) before displaying totals.
    # This model uses the same weighted T_best for fuel and time, even with vacancies.
    # ============================================================
    print("\n" + "=" * 60)
    print(f"T_best used for all scenarios = {T_best:.4f} minutes")
    print("=" * 60)

    # REVIEW NOTE: Lecture p.5 aircraft rates, capacities and fixed/time charges.
    # Only cruise operations are included; no taxi, takeoff or landing costs.
    aircraft = {
        "A321neo":      {"rate": 54, "cap": 200, "low": 10, "med": 15, "high": 20, "Cc": 1800},
        "A330-900neo":  {"rate": 84, "cap": 300, "low": 15, "med": 21, "high": 27, "Cc": 2000},
        "A350-900":     {"rate": 90, "cap": 350, "low": 20, "med": 27, "high": 34, "Cc": 2500},
    }

    # REVIEW NOTE: Lecture p.8 supplies these passenger counts, limits and prices.
    # Scenario 2 says monthly demand but a weekly flight cap. The existing model
    # compares one batch against max_f=7; it does not allocate flights by week.
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
            # REVIEW NOTE: Include the partly occupied final flight in the count.
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
