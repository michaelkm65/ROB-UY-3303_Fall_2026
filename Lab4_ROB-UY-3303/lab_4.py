"""
ROB-UY 3303 - Lab 04: Tree Search Motion Planning (A*)
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_4.py
The GUI (gui_4.py) imports this file and uses your planner and trackers on
the robot. map_4.py (same folder) holds the maps.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

Units: meters, seconds, radians.
Grid: 200 x 200 cells of 1 cm (see map_4.py). A cell is a list [i, j] of
two ints: i = column (along x), j = row (along y). OCCUPANCY_GRID[i][j] is
1 if the CENTRE of the robot cannot be in cell [i, j].
"""
import math
import time

import numpy as np

import map_4

# =====================================================================
# PART 0 - STUDENT TODO: COPY YOUR LAB 3 CODE (lab_3.py)
# =====================================================================
# Copy your robot constants and these three functions from lab_3.py
# (same names, inputs and outputs). Copy any helper function they use too.
WHEEL_RADIUS_M = 0.03375     # wheel radius r (m)
WHEELBASE_M = 0.17968        # distance between the wheels = 2L (m)


def get_wheel_velocities(v, w):
    L = WHEELBASE_M / 2.0
    phi_dot_l = (v - L * w) / WHEEL_RADIUS_M
    phi_dot_r = (v + L * w) / WHEEL_RADIUS_M
    return phi_dot_l, phi_dot_r


K_RHO = 1 # previously 0.8
K_ALPHA = 3 # previously 1.8
K_BETA = -0.5 # previously -0.5
MAX_V = 0.25 # previously 0.35
MAX_W = 1.5 # previously 2.5

def point_tracking_controller(X_des, X):
    """
    Inputs:
      X_des = desired robot pose [x_des, y_des, theta_des]
      X     = current robot pose [x, y, theta]
    Returns:
      (phi_dot_l, phi_dot_r) in rad/s
    """
    dx = X_des[0] - X[0]
    dy = X_des[1] - X[1]
    theta = X[2]
    theta_des = X_des[2]

    # Coordinate transformation: rho (slide 29/35)
    rho = math.hypot(dx, dy)

    # When close to the target position, purely turn in-place to align theta
    if rho < 0.015:
        v = 0.0
        w = 1.5 * wrap_angle(theta_des - theta)
        return get_wheel_velocities(v, w)

    gamma = math.atan2(dy, dx)
    alpha = wrap_angle(gamma - theta)
    beta = wrap_angle(theta_des - theta - alpha)

    # Turn in place if heading error exceeds 20 degrees
    if abs(alpha) > math.radians(30):
        v = 0.0
        w = K_ALPHA * alpha
    else:
        v = K_RHO * rho
        w = K_ALPHA * alpha + K_BETA * beta

    # Velocity saturation
    v = max(-MAX_V, min(MAX_V, v))
    w = max(-MAX_W, min(MAX_W, w))

    return get_wheel_velocities(v, w)
    # TODO: copy from lab_3.py


LOOKAHEAD_M = 0.15     # Delta (m): main tuning knob        # previously 0.05
PATH_POS_TOL = 0.02    # m   (assignment hint)
PATH_ANG_TOL = 0.3     # rad (assignment hint)
PATH_SPACING = 0.02    # m between densified path points


def _densify_path(P, ds=PATH_SPACING, dth=0.2):
    """Insert poses between waypoints so 'closest pose' is meaningful.
    Interpolates position linearly and heading along the shortest angle.
    A zero-length segment (turn in place) is split by heading change."""
    out = [list(P[0])]
    for a, b in zip(P[:-1], P[1:]):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        dtheta = wrap_angle(b[2] - a[2])
        n = max(1, math.ceil(dist / ds), math.ceil(abs(dtheta) / dth))
        for k in range(1, n + 1):
            f = k / n
            out.append([a[0] + f * (b[0] - a[0]),
                        a[1] + f * (b[1] - a[1]),
                        wrap_angle(a[2] + f * dtheta)])
    return out


def _closest_index(P, X):
    """Index of the path pose closest to the robot position."""
    best_i, best_d = 0, float("inf")
    for i, p in enumerate(P):
        d = math.hypot(p[0] - X[0], p[1] - X[1])
        if d < best_d:
            best_i, best_d = i, d
    return best_i


def _lookahead_pose(P, i_c, delta):
    """Walk forward from i_c until the arc length along the path reaches delta."""
    s, i = 0.0, i_c
    while i < len(P) - 1 and s < delta:
        s += math.hypot(P[i + 1][0] - P[i][0], P[i + 1][1] - P[i][1])
        i += 1
    return P[i]    # clamps to the final pose near the end of the path


def path_tracking_controller(P, X):
    """
    Inputs:
      P = path, list of poses [[x, y, theta], ...]
      X = current robot pose [x, y, theta]
    Returns:
      (phi_dot_l, phi_dot_r) in rad/s
    """
    P = _densify_path(P)                             # so closest pose is meaningful
    i_c = _closest_index(P, X)                       # 1. closest pose p_c,t
    X_des = _lookahead_pose(P, i_c, LOOKAHEAD_M)     # 2. pose Delta further along P
    return point_tracking_controller(X_des, X)       # 3. point tracking  # TODO: copy from lab_3.py


# =====================================================================
# PROVIDED - ROBOT SIMULATOR (same as Lab 3, do not change)
# =====================================================================
DT = 0.1                    # s, time step (same as the GUI control loop)
POS_TOL = 0.03              # m, goal reached if closer than this
ANG_TOL = 0.1               # rad, and heading error smaller than this


def wrap_angle(a):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def get_robot_velocities(phi_dot_l, phi_dot_r):
    """Wheel speeds (rad/s) -> robot (v, w)."""
    L = WHEELBASE_M / 2.0
    v = WHEEL_RADIUS_M * (phi_dot_r + phi_dot_l) / 2.0
    w = WHEEL_RADIUS_M * (phi_dot_r - phi_dot_l) / (2.0 * L)
    return v, w


def robot_simulator(X_tm1, phi_dot_l, phi_dot_r, delta_T, n_substeps=10):
    """Moves the robot for delta_T seconds with constant wheel speeds.
    Returns X_t (new pose [x, y, theta]) and the list of poses visited."""
    x, y, th = X_tm1
    h = delta_T / n_substeps
    poses = []
    for _ in range(n_substeps):
        v, w = get_robot_velocities(phi_dot_l, phi_dot_r)
        x += v * h * math.cos(th + w * h / 2.0)
        y += v * h * math.sin(th + w * h / 2.0)
        th = wrap_angle(th + w * h)
        poses.append([x, y, th])
    return [x, y, th], poses


def is_at_goal(X_des, X):
    """True if X is within POS_TOL and ANG_TOL of X_des."""
    return (math.hypot(X_des[0] - X[0], X_des[1] - X[1]) < POS_TOL and
            abs(wrap_angle(X_des[2] - X[2])) < ANG_TOL)


# =====================================================================
# PROVIDED - THE GRID MAP (do not change)
# =====================================================================
GRID_N = map_4.GRID_N                   # 200 cells along x and along y
CELL_SIZE_M = map_4.CELL_SIZE_M         # 0.01 m
ROBOT_SPEED_M_S = 0.12                  # m/s, robot speed on normal floor (k = 1)

OCCUPANCY_GRID = None       # 200 x 200, 1 = robot centre cannot be in the cell
TRAVERSABILITY_GRID = None  # 200 x 200, k in (0, 1]: speed allowed = k * ROBOT_SPEED_M_S
COLLISION_GRID = None       # 200 x 200, 1 = the robot body touches an obstacle or a wall


def load_map(name):
    """Loads a map of map_4.MAPS: the obstacles grown by the robot radius
    and a safety margin (configuration space, Lecture 08A), and the
    traversability."""
    global COLLISION_GRID
    load_grid(map_4.build_c_space(name), map_4.build_traversability_grid(name))
    COLLISION_GRID = map_4.grow_obstacles(map_4.build_obstacle_grid(name), map_4.ROBOT_RADIUS_M)


def load_grid(occupancy, traversability=None):
    """Loads any 200 x 200 occupancy grid (1 = occupied) directly, e.g. for
    the unit tests. traversability = None means k = 1 everywhere."""
    global OCCUPANCY_GRID, TRAVERSABILITY_GRID, COLLISION_GRID
    OCCUPANCY_GRID = np.asarray(occupancy, dtype=np.uint8)
    COLLISION_GRID = OCCUPANCY_GRID
    if traversability is None:
        traversability = np.ones((GRID_N, GRID_N))
    TRAVERSABILITY_GRID = np.asarray(traversability, dtype=float)


def empty_grid():
    """200 x 200 grid of zeros, to build test maps: grid[i][j] = 1 ..."""
    return np.zeros((GRID_N, GRID_N), dtype=np.uint8)


def in_bounds(X):
    """True if cell X = [i, j] is on the grid."""
    return 0 <= X[0] < GRID_N and 0 <= X[1] < GRID_N


def is_occupied(X):
    """True if cell X = [i, j] is in collision."""
    return OCCUPANCY_GRID[X[0]][X[1]] == 1


def traversability(X):
    """Traversability k of cell X = [i, j]."""
    return float(TRAVERSABILITY_GRID[X[0]][X[1]])


def new_cost_array():
    """200 x 200 array full of infinity (one value per cell)."""
    return np.full((GRID_N, GRID_N), math.inf)


def meters_to_cell(x, y):
    """Position (m) -> cell [i, j] that contains it."""
    return [int(math.floor((x - map_4.GRID_ORIGIN_M[0]) / CELL_SIZE_M)),
            int(math.floor((y - map_4.GRID_ORIGIN_M[1]) / CELL_SIZE_M))]


def cell_to_meters(X):
    """Cell [i, j] -> position (m) [x, y] of its centre."""
    return [map_4.GRID_ORIGIN_M[0] + (X[0] + 0.5) * CELL_SIZE_M,
            map_4.GRID_ORIGIN_M[1] + (X[1] + 0.5) * CELL_SIZE_M]


def robot_in_collision(X):
    """True if the robot at pose X = [x, y, theta] (m) touches an obstacle
    or a wall of the map loaded with load_map (no safety margin)."""
    i, j = meters_to_cell(X[0], X[1])
    return not in_bounds([i, j]) or COLLISION_GRID[i][j] == 1


def check_path(P, X_0, X_d):
    """Checks a planned path of cells. Returns (True, "ok") or (False, reason)."""
    if not P:
        return False, "no path returned"
    if list(P[0]) != list(X_0) or list(P[-1]) != list(X_d):
        return False, "path must start at X_0 and end at X_d"
    for k, X in enumerate(P):
        if not in_bounds(X):
            return False, f"cell {k} {X} is off the grid"
        if is_occupied(X) and k > 0:
            return False, f"cell {k} {X} is in collision"
        if k > 0 and abs(X[0] - P[k - 1][0]) + abs(X[1] - P[k - 1][1]) != 1:
            return False, f"cells {k - 1} and {k} are not neighbors"
    return True, "ok"


def path_time_s(P):
    """Time (s) to drive along the path of cells P at the speed allowed in
    each cell entered (k * ROBOT_SPEED_M_S)."""
    return sum(CELL_SIZE_M / (ROBOT_SPEED_M_S * traversability(X)) for X in P[1:])


# =====================================================================
# PART 2 - THE A* PLANNER (Lecture 09A and 09B)
# =====================================================================
# DO NOT USE ANY GEN AI TOOLS OR EXAMPLE A* CODE FOR THIS PART (Step 6 only).
#
# Two cost functions are used in this lab (cost_mode):
#   "distance": shortest path. The cost of a step to a neighbor cell is the
#               distance travelled (m). Part 2.
#   "time":     minimum time. The cost of a step is the time (s) needed to
#               drive into the neighbor cell at the speed allowed there,
#               k * ROBOT_SPEED_M_S. Part 3.
# For now, write everything for "distance". You add "time" in Part 3.

# STUDENT TODO - Step 1: the Node class.
# Member variables:
#   X        - [i, j], the cell of the node
#   parent   - the parent Node (None for the root)
#   g_value  - g(n): path cost from the root to this node
#   h_value  - h(n): estimated path cost from this node to the goal
#   f_value  - f(n) = g(n) + h(n)
class Node:
    def __init__(self, X, parent, g_value, h_value):
        self.X = X
        self.parent = parent
        self.g_value = g_value
        self.h_value = h_value
        self.f_value = g_value + h_value


# STUDENT TODO - Step 1: the cost of one step and the heuristic.
# get_step_cost(X_from, X_to, cost_mode) -> cost of moving from cell X_from
#   to the neighbor cell X_to (m for "distance", s for "time").
# get_heuristic(X, X_d, cost_mode) -> h(n) for a node at cell X and the goal
#   cell X_d. It must be ADMISSIBLE (never overestimates the cost to the goal,
#   Lecture 09B): use the straight line distance from X to X_d (m).
def get_step_cost(X_from, X_to, cost_mode="distance"):
    distance = math.hypot(X_to[0] - X_from[0], X_to[1] - X_from[1]) * CELL_SIZE_M
    if cost_mode == "time":
        return distance / (ROBOT_SPEED_M_S * traversability(X_to))
    return distance


def get_heuristic(X, X_d, cost_mode="distance"):
    distance = math.hypot(X_d[0] - X[0], X_d[1] - X[1]) * CELL_SIZE_M
    if cost_mode == "time":
        # k <= 1, so the maximum speed gives a lower bound on travel time.
        return distance / ROBOT_SPEED_M_S
    return distance


# STUDENT TODO - Step 2: get_neighbors(n, X_d, best_g, cost_mode)
# Returns a list of up to 4 NEW nodes, children of node n: the cells one
# step in +x, -x, +y and -y from n.X. Set the parent, g, h and f of each.
# Do not create a neighbor if it is:
#   - out of bounds:  off the grid (use in_bounds)
#   - in collision:   occupied (use is_occupied)
#   - already added:  best_g is a 200 x 200 array (new_cost_array()) with
#                     the lowest g found so far for each cell. Only create
#                     the neighbor if its g is LOWER than best_g at its
#                     cell, and then store its g in best_g. This stops the
#                     search from going around cycles, and keeps A* optimal
#                     when the cost of a step is not always the same (Part 3).
def get_neighbors(n, X_d, best_g, cost_mode="distance"):
    neighbors = []
    for di, dj in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        X = [n.X[0] + di, n.X[1] + dj]
        if not in_bounds(X) or is_occupied(X):
            continue

        g = n.g_value + get_step_cost(n.X, X, cost_mode)
        if g < best_g[X[0]][X[1]]:
            best_g[X[0]][X[1]] = g
            h = get_heuristic(X, X_d, cost_mode)
            neighbors.append(Node(X, n, g, h))
    return neighbors


# STUDENT TODO - Step 3: is_goal(n, X_d) -> True if node n is at cell X_d.
def is_goal(n, X_d):
    return n.X[0] == X_d[0] and n.X[1] == X_d[1]


# STUDENT TODO - Step 4: A_star_planner(X_0, X_d, cost_mode)
#   X_0 = start cell [i0, j0], X_d = goal cell [id, jd].
#   Returns the path P = [X_0, X_1, ..., X_d], a list of cells [i, j], or
#   None if there is no path (the fringe becomes empty).
# Follow GENERAL-SEARCH (Lecture 09A) with the A* strategy (Lecture 09B):
# create the root node, keep a list of nodes on the fringe, pick the node
# with the lowest f(n) from the fringe, check if it is the goal, otherwise
# add its children to the fringe. Build P by following the parents from
# the goal node back to the root. Add any helper functions you want.
def A_star_planner(X_0, X_d, cost_mode="distance"):
    root = Node(X_0, None, 0.0, get_heuristic(X_0, X_d, cost_mode))
    best_g = new_cost_array()
    best_g[X_0[0]][X_0[1]] = 0.0
    fringe = [root]

    while fringe:
        n = min(fringe, key=lambda node: node.f_value)
        fringe.remove(n)

        # A cheaper route to this cell may have been found meanwhile.
        if n.g_value > best_g[n.X[0]][n.X[1]]:
            continue

        if is_goal(n, X_d):
            P = []
            while n is not None:
                P.append(n.X)
                n = n.parent
            P.reverse()
            return P

        fringe.extend(get_neighbors(n, X_d, best_g, cost_mode))

    return None




# PROVIDED - example unit test (empty room, straight line).
def a_star_unit_test_1():
    load_grid(empty_grid())
    X_0, X_d = [25, 25], [125, 25]
    P = A_star_planner(X_0, X_d)
    ok, msg = check_path(P, X_0, X_d)
    optimal_cells = 101             # 100 steps of 1 cell + the start cell
    return ok and len(P) == optimal_cells, P


# STUDENT TODO - Step 5: 2 more unit tests, each returns (passed, P).
# Build a map with empty_grid(), set some cells to 1, call load_grid(...),
# then plan and compare with the optimal path that you computed by hand
# (use check_path and len(P)). Suggested cases:
#   2: a wall of obstacles between X_0 and X_d, with a gap the path must go through
#   3: X_d fully enclosed by obstacles -> A_star_planner must return None
def a_star_unit_test_2():
    grid = empty_grid()
    grid[75, :] = 1
    grid[75, 50] = 0  # The only opening through the wall.
    load_grid(grid)
    X_0, X_d = [25, 25], [125, 25]
    P = A_star_planner(X_0, X_d)
    ok, msg = check_path(P, X_0, X_d)
    # 100 horizontal steps + 25 up + 25 down, plus the start cell.
    optimal_cells = 151
    return ok and len(P) == optimal_cells and [75, 50] in P, P


def a_star_unit_test_3():
    grid = empty_grid()
    # A small room keeps the exhaustive search for an unreachable goal short.
    grid[:, :] = 1
    grid[20:100, 20:100] = 0
    X_0, X_d = [25, 25], [35, 25]
    for di, dj in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        grid[X_d[0] + di, X_d[1] + dj] = 1 # obstacles around the start node
    load_grid(grid)
    P = A_star_planner(X_0, X_d)
    return P is None, P


# STUDENT TODO: run the 3 tests, print how many pass out of 3, plot the
# map and the 3 paths (e.g. plt.imshow(grid.T, origin="lower")).
def run_a_star_unit_tests(plot=True):
    tests = [
        ("Empty room", a_star_unit_test_1, [25, 25], [125, 25]),
        ("Wall with a gap", a_star_unit_test_2, [25, 25], [125, 25]),
        ("Enclosed goal", a_star_unit_test_3, [25, 25], [35, 25]),
    ]
    results = []
    for name, test, start, goal in tests:
        passed, P = test()
        results.append((name, passed, P, OCCUPANCY_GRID.copy(), start, goal))
        print(f"  {name}: {'PASSED' if passed else 'FAILED'}")
    print(f"A* unit tests passed: {sum(result[1] for result in results)}/3")

    if plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        for ax, (name, passed, P, grid, start, goal) in zip(axes, results):
            ax.imshow(grid.T, origin="lower", cmap="Greys", vmin=0, vmax=1)
            if P:
                ax.plot([X[0] for X in P], [X[1] for X in P], "b-", label="Path")
            else:
                ax.text(0.5, 0.95, "No path returned", transform=ax.transAxes,
                        ha="center", va="top")
            ax.plot(start[0], start[1], "go", label="Start")
            ax.plot(goal[0], goal[1], "rx", label="Goal")
            ax.set_title(f"{name}: {'PASSED' if passed else 'FAILED'}")
            ax.set_xlabel("i (column)")
            ax.set_ylabel("j (row)")
            ax.legend()
        fig.tight_layout()
        plt.show()


# =====================================================================
# PART 3 - MINIMUM TIME PLANNING WITH A TRAVERSABILITY MAP
# =====================================================================
# In map_4.py, "slow_zones" give each cell a traversability k in (0, 1]:
# the robot may drive at k * ROBOT_SPEED_M_S in that cell.
#
# STUDENT TODO - Step 1: add cost_mode "time" to get_step_cost and get_heuristic.
#   step cost = time (s) to travel one cell into X_to at the speed allowed in X_to
#   heuristic = time (s) to drive the straight line to the goal at the
#               highest possible speed. Explain in your report why it is admissible.
#
# STUDENT TODO - Step 2: 2 unit tests, each returns (passed, P_distance, P_time).
# Use path_time_s(P) to compare the time of two paths.
#   1: a slow zone (k = 0.2) across the straight line from X_0 to X_d, that the
#      robot can go around. "time" must go around it, "distance" goes through.
#   2: the same with k = 0.9: going around is now slower, "time" goes through.
def _run_time_cost_case(k, avoid_slow_zone):
    terrain = np.ones((GRID_N, GRID_N))
    terrain[65:85, 15:36] = k
    load_grid(empty_grid(), terrain)
    X_0, X_d = [25, 25], [125, 25]
    P_distance = A_star_planner(X_0, X_d, "distance")
    P_time = A_star_planner(X_0, X_d, "time")
    distance_ok, _ = check_path(P_distance, X_0, X_d)
    time_ok, _ = check_path(P_time, X_0, X_d)
    if not distance_ok or not time_ok:
        return False, P_distance, P_time

    distance_enters_slow = any(traversability(X) < 1 for X in P_distance)
    time_enters_slow = any(traversability(X) < 1 for X in P_time)
    # Straight route: 100 steps, including 20 slow cells.
    straight_time = (80 + 20 / k) * CELL_SIZE_M / ROBOT_SPEED_M_S
    # Going below the zone requires 11 steps down and 11 back up.
    detour_time = 122 * CELL_SIZE_M / ROBOT_SPEED_M_S
    passed = (len(P_distance) == 101 and distance_enters_slow and
              math.isclose(path_time_s(P_distance), straight_time))
    if avoid_slow_zone:
        passed = (passed and not time_enters_slow and len(P_time) == 123 and
                  math.isclose(path_time_s(P_time), detour_time) and
                  path_time_s(P_time) < path_time_s(P_distance))
    else:
        passed = (passed and time_enters_slow and len(P_time) == 101 and
                  math.isclose(path_time_s(P_time), straight_time) and
                  straight_time < detour_time)
    return passed, P_distance, P_time


def time_cost_unit_test_1():
    return _run_time_cost_case(0.2, avoid_slow_zone=True)


def time_cost_unit_test_2():
    return _run_time_cost_case(0.9, avoid_slow_zone=False)


# STUDENT TODO: run the 2 tests, print how many pass out of 2, plot the paths.
def run_time_cost_unit_tests(plot=True):
    results = []
    for k, test in [(0.2, time_cost_unit_test_1), (0.9, time_cost_unit_test_2)]:
        passed, P_distance, P_time = test()
        results.append((k, passed, P_distance, P_time, TRAVERSABILITY_GRID.copy()))
        print(f"  Slow zone k={k}: {'PASSED' if passed else 'FAILED'}")
        if P_distance and P_time:
            print(f"    Distance planner: {path_time_s(P_distance):.3f} s; "
                  f"time planner: {path_time_s(P_time):.3f} s")
    print(f"Time cost unit tests passed: {sum(result[1] for result in results)}/2")

    if plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        for ax, (k, passed, P_distance, P_time, terrain) in zip(axes, results):
            ax.imshow(terrain.T, origin="lower", cmap="YlGn", vmin=0, vmax=1)
            for P, style, label in [(P_distance, "b-", "Distance planner"),
                                    (P_time, "r--", "Time planner")]:
                if P:
                    ax.plot([X[0] for X in P], [X[1] for X in P], style, label=label)
            ax.plot(25, 25, "go", label="Start")
            ax.plot(125, 25, "kx", label="Goal")
            ax.set_xlim(15, 135)
            ax.set_ylim(5, 45)
            ax.set_xlabel("i (column)")
            ax.set_ylabel("j (row)")
            ax.set_title(f"k={k}: {'PASSED' if passed else 'FAILED'}")
            ax.legend()
        fig.tight_layout()
        plt.show()


# =====================================================================
# PART 4 - FROM PATH TO TRAJECTORY, AND TRAJECTORY TRACKING (Lecture 06A)
# =====================================================================
# A path P only says WHERE the robot goes. A trajectory T also says WHEN:
#   T = [p_0, p_1, ..., p_M],   p_j = [t, x, y, theta]_j   (s, m, m, rad)
# With a trajectory, the robot drives slowly in slow zones, because the
# time stamps there are further apart.
TRAJ_LOOKAHEAD_S = 1.0          # s, the Delta t of Lecture 06A (you may tune it)
HEADING_LOOKAHEAD_CELLS = 10    # cells, see path_to_trajectory


# STUDENT TODO - Step 1: path_to_trajectory(P, cost_mode) -> T
#   P = path of cells from A_star_planner.
#   For each cell, p_j = [t_j, x_j, y_j, theta_j]:
#   - x_j, y_j:  centre of the cell in meters (cell_to_meters)
#   - t_j:       t_0 = 0, then t_j = t_(j-1) + (distance from p_(j-1) to p_j) / speed,
#                with speed = ROBOT_SPEED_M_S for "distance", and
#                k * ROBOT_SPEED_M_S of cell j for "time".
#   - theta_j:   direction of travel. On a grid path the direction of a
#                single step is always 0, 90, 180 or -90 deg, so use the
#                direction from cell j to cell j + HEADING_LOOKAHEAD_CELLS
#                (or to the last cell near the end). The last pose keeps the
#                heading of the pose before it.
def path_to_trajectory(P, cost_mode="distance"):
    if not P:
        return []
    positions = [cell_to_meters(cell) for cell in P]
    T = []
    t = 0.0
    for j, (x, y) in enumerate(positions):
        if j > 0:
            distance = math.hypot(x - positions[j - 1][0], y - positions[j - 1][1])
            speed = ROBOT_SPEED_M_S
            if cost_mode == "time":
                speed *= traversability(P[j])
            t += distance / speed
        if j < len(P) - 1:
            ahead = positions[min(j + HEADING_LOOKAHEAD_CELLS, len(P) - 1)]
            theta = math.atan2(ahead[1] - y, ahead[0] - x)
        else:
            theta = T[-1][3] if T else 0.0
        T.append([t, x, y, theta])
    return T


def _trajectory_pose(T, t):
    """Interpolate a pose at time t, clamping to the trajectory endpoints."""
    from bisect import bisect_right

    if t <= T[0][0]:
        return T[0][1:]
    if t >= T[-1][0]:
        return T[-1][1:]
    j = bisect_right(T, t, key=lambda pose: pose[0])
    a, b = T[j - 1], T[j]
    f = (t - a[0]) / (b[0] - a[0])
    return [a[1] + f * (b[1] - a[1]),
            a[2] + f * (b[2] - a[2]),
            wrap_angle(a[3] + f * wrap_angle(b[3] - a[3]))]


# STUDENT TODO - Step 2: trajectory_tracking_controller(T, X, t) -> (phi_dot_l, phi_dot_r)
#   T = trajectory, X = current pose [x, y, theta], t = time (s) since the
#   robot started the trajectory. Use "Trajectory Tracking Control via
#   Points" of Lecture 06A. At each time step t:
#     1. find the pose p_t on T that has the same time stamp as t
#     2. find the pose p_des,t on T that is a time TRAJ_LOOKAHEAD_S further
#        along T (the last pose of T if t + TRAJ_LOOKAHEAD_S is after the end)
#     3. return point_tracking_controller([x, y, theta] of p_des,t, X)
def trajectory_tracking_controller(T, X, t):
    if not T:
        return 0.0, 0.0
    X_des = _trajectory_pose(T, t + TRAJ_LOOKAHEAD_S)
    return point_tracking_controller(X_des, X)


# STUDENT TODO - Step 3: 2 unit tests, each returns (passed, T, visited_poses).
# Plan on a map of map_4.py with load_map(name) and A_star_planner, make the
# trajectory, then loop: trajectory_tracking_controller -> robot_simulator
# (t increases by DT each loop) until is_at_goal(T[-1][1:], X), with a time
# limit of T[-1][0] + 20 s. A test passes if the robot reaches the goal and
# robot_in_collision is False for every pose visited. Also record the largest
# distance between the robot and p_t (the tracking error) and print it.
#   1: map "box_sim", X = [0, 0, 0], goal (1.5, 0.0) m, cost_mode "distance"
#   2: map "terrain_sim", X = [0, 0, 0], goal (1.5, 0.0) m, cost_mode "time"
def _run_trajectory_test(map_name, cost_mode):
    load_map(map_name)
    X = [0.0, 0.0, 0.0]
    P = A_star_planner(meters_to_cell(*X[:2]), meters_to_cell(1.5, 0.0), cost_mode)
    T = path_to_trajectory(P, cost_mode)
    visited = [X[:]]
    if not T:
        print(f"  {map_name}: no path returned")
        return False, T, visited
    t, max_error = 0.0, 0.0
    collision = robot_in_collision(X)
    while True:
        reference = _trajectory_pose(T, t)
        max_error = max(max_error, math.hypot(X[0] - reference[0], X[1] - reference[1]))
        if is_at_goal(T[-1][1:], X) or t >= T[-1][0] + 20.0:
            break
        step = min(DT, T[-1][0] + 20.0 - t)
        wheels = trajectory_tracking_controller(T, X, t)
        X, subposes = robot_simulator(X, *wheels, step)
        for j, pose in enumerate(subposes):
            reference = _trajectory_pose(T, t + step * (j + 1) / len(subposes))
            max_error = max(max_error, math.hypot(pose[0] - reference[0], pose[1] - reference[1]))
        visited.extend(subposes)
        collision = collision or any(robot_in_collision(pose) for pose in subposes)
        t += step
    passed = is_at_goal(T[-1][1:], X) and not collision
    print(f"  {map_name}: {'PASSED' if passed else 'FAILED'}, "
          f"max tracking error = {max_error:.3f} m, collision = {collision}")
    return passed, T, visited


def traj_tracker_unit_test_1():
    return _run_trajectory_test("box_sim", "distance")


def traj_tracker_unit_test_2():
    return _run_trajectory_test("terrain_sim", "time")


# STUDENT TODO: run the 2 tests, print how many pass out of 2, plot each
# planned path and the poses visited (as subfigures of one figure).
def run_traj_tracker_unit_tests(plot=True):
    results = []
    for name, test in [("box_sim", traj_tracker_unit_test_1),
                       ("terrain_sim", traj_tracker_unit_test_2)]:
        passed, T, visited = test()
        results.append((name, passed, T, visited, COLLISION_GRID.copy()))
    print(f"Trajectory tracker unit tests passed: {sum(r[1] for r in results)}/2")
    if plot:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        extent = [map_4.GRID_ORIGIN_M[0], map_4.GRID_ORIGIN_M[0] + GRID_N * CELL_SIZE_M,
                  map_4.GRID_ORIGIN_M[1], map_4.GRID_ORIGIN_M[1] + GRID_N * CELL_SIZE_M]
        for ax, (name, passed, T, visited, grid) in zip(axes, results):
            ax.imshow(grid.T, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1)
            for x_min, y_min, x_max, y_max, k in map_4.MAPS[name]["slow_zones"]:
                if k >= 1.0:
                    continue
                ax.add_patch(Rectangle(
                    (x_min, y_min), x_max - x_min, y_max - y_min,
                    facecolor="orange", edgecolor="darkorange", alpha=0.3,
                    label=f"Slow zone (k={k:g})"))
            if T:
                ax.plot([p[1] for p in T], [p[2] for p in T], "b-", label="Planned")
            ax.plot([p[0] for p in visited], [p[1] for p in visited], "r--", label="Visited")
            ax.set_title(f"{name}: {'PASSED' if passed else 'FAILED'}")
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.legend()
        fig.tight_layout()
        plt.show()


# =====================================================================
# PART 5 - A* ANALYSIS: PATH LENGTH AND PLANNING TIME VS MAP DENSITY
# =====================================================================
# PROVIDED - a random free cell, for random start and goal cells.
def random_free_cell(rng):
    while True:
        X = [int(rng.integers(0, GRID_N)), int(rng.integers(0, GRID_N))]
        if not is_occupied(X):
            return X


# STUDENT TODO: run_density_analysis(plot=True)
#   For 5 densities (e.g. 0.0, 0.05, 0.10, 0.15, 0.20):
#     load_grid(map_4.random_map(density, seed))
#     100 times: X_0 = random_free_cell(rng), X_d = random_free_cell(rng),
#       time the call A_star_planner(X_0, X_d) with time.perf_counter(),
#       record the path length (m) and the planning time (s).
#       Count the plans that return None (no path) separately.
#   Plot the mean path length vs density, and the mean planning time vs density.
# Tip: with a plain list as the fringe, searching it for the lowest f at every
# step makes this analysis slow (~10 min). Python's heapq module can give you
# the lowest f much faster. Mention your choice in the report.
def run_density_analysis(plot=True):
    rng = np.random.default_rng(0)
    results = []
    for seed, density in enumerate([0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.90, 0.95, 0.99]):
        load_grid(map_4.random_map(density, seed=42))
        lengths, durations = [], []
        failures = 0
        for _ in range(100):
            X_0, X_d = random_free_cell(rng), random_free_cell(rng)
            started = time.perf_counter()
            P = A_star_planner(X_0, X_d)
            durations.append(time.perf_counter() - started)
            if P is None:
                failures += 1
            else:
                lengths.append((len(P) - 1) * CELL_SIZE_M)
        result = {"density": density, "mean_path_length_m": float(np.mean(lengths)) if lengths else math.nan,
                  "mean_planning_time_s": float(np.mean(durations)), "no_path_count": failures}
        results.append(result)
        print(f"  Density {density:.2f}: mean path {result['mean_path_length_m']:.3f} m "
              f"(successful plans), mean time {result['mean_planning_time_s']:.4f} s "
              f"(all plans), no path {failures}/100", flush=True)
    if plot:
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        densities = [r["density"] for r in results]
        axes[0].plot(densities, [r["mean_path_length_m"] for r in results], "o-")
        axes[0].set_ylabel("Mean path length of successful plans (m)")
        axes[1].plot(densities, [r["mean_planning_time_s"] for r in results], "o-")
        axes[1].set_ylabel("Mean planning time of all plans (s)")
        for ax in axes:
            ax.set_xlabel("Obstacle density")
            ax.grid(True)
        fig.tight_layout()
        plt.show()
    return results


if __name__ == "__main__":
    # print("=== Part 2: A* (shortest path) ===")
    # run_a_star_unit_tests()
    # print("\n=== Part 3: A* (minimum time) ===")
    # run_time_cost_unit_tests()
    # print("\n=== Part 4: trajectory tracking ===")
    # run_traj_tracker_unit_tests()
    print("\n=== Part 5: density analysis ===")   # slow: run it once for your report
    run_density_analysis()
