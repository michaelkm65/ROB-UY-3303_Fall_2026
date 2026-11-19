"""
ROB-UY 3303 - Lab 03: Path Following
STARTER CODE - complete every function marked STUDENT TODO.
Run the unit tests with:   python lab_3.py
The GUI (gui_03.py) imports this file and uses your controllers on the robot.

Write your code only where it says STUDENT TODO. Sections marked
PROVIDED are given to you: do not change them.

Units: meters, seconds, radians.  v = forward speed (m/s), w = rotational
speed (rad/s, + = counter-clockwise), phi_dot_l / phi_dot_r = wheel speeds (rad/s).
"""
import math
import matplotlib.pyplot as plt
# =====================================================================
# STUDENT TODO - ROBOT CONSTANTS (your Lab 2 values, same as in gui_03.py)
# =====================================================================
WHEEL_RADIUS_M = 0.03375     # wheel radius r (m)
WHEELBASE_M = 0.17968        # distance between the wheels = 2L (m)


# =====================================================================
# PART 1 - WHEEL VELOCITIES
# =====================================================================
# STUDENT TODO: get_wheel_velocities(v, w) -> (phi_dot_l, phi_dot_r)
# Use the equations you derived in Step 1.
def get_wheel_velocities(v, w):
    L = WHEELBASE_M / 2.0
    phi_dot_l = (v - L * w) / WHEEL_RADIUS_M
    phi_dot_r = (v + L * w) / WHEEL_RADIUS_M
    return phi_dot_l, phi_dot_r


# STUDENT TODO: 5 unit tests, each returns True if it passes.
#   1: v > 0, w = 0     2: v < 0, w = 0     3: v = 0, w > 0
#   4: v = 0, w < 0     5: v = 0, w = 0
TOL = 1e-9

def _close(a, b):
    return abs(a - b) < TOL


def wheel_velocity_unit_test_1():
    # 1: v > 0, w = 0 -> Straight forward: both wheels equal and positive
    v, w = 0.25, 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    expected = v / WHEEL_RADIUS_M
    return _close(phi_l, expected) and _close(phi_r, expected)


def wheel_velocity_unit_test_2():
    # 2: v < 0, w = 0 -> Straight reverse: both wheels equal and negative
    v, w = -0.25, 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    expected = v / WHEEL_RADIUS_M
    return _close(phi_l, expected) and _close(phi_r, expected)


def wheel_velocity_unit_test_3():
    # 3: v = 0, w > 0 -> Turn CCW in place: left backward (-), right forward (+)
    v, w = 0.0, 1.5
    L = WHEELBASE_M / 2.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    expected_r = (L * w) / WHEEL_RADIUS_M
    expected_l = -expected_r
    return _close(phi_l, expected_l) and _close(phi_r, expected_r)


def wheel_velocity_unit_test_4():
    # 4: v = 0, w < 0 -> Turn CW in place: left forward (+), right backward (-)
    v, w = 0.0, -1.5
    L = WHEELBASE_M / 2.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    expected_r = (L * w) / WHEEL_RADIUS_M
    expected_l = -expected_r
    return _close(phi_l, expected_l) and _close(phi_r, expected_r)


def wheel_velocity_unit_test_5():
    # 5: v = 0, w = 0 -> Stopped: both wheels 0
    v, w = 0.0, 0.0
    phi_l, phi_r = get_wheel_velocities(v, w)
    return _close(phi_l, 0.0) and _close(phi_r, 0.0)


# STUDENT TODO: run the 5 tests and print how many pass out of 5.
def run_wheel_velocity_unit_tests():
    tests = [
        ("Test 1 (v > 0, w = 0)", wheel_velocity_unit_test_1),
        ("Test 2 (v < 0, w = 0)", wheel_velocity_unit_test_2),
        ("Test 3 (v = 0, w > 0)", wheel_velocity_unit_test_3),
        ("Test 4 (v = 0, w < 0)", wheel_velocity_unit_test_4),
        ("Test 5 (v = 0, w = 0)", wheel_velocity_unit_test_5),
    ]

    passed_count = 0
    for name, test_func in tests:
        passed = test_func()
        if passed:
            passed_count += 1
        print(f"  {name}: {'PASSED' if passed else 'FAILED'}")

    print(f"Wheel velocity unit tests passed: {passed_count}/{len(tests)}")

# =====================================================================
# PROVIDED - ROBOT SIMULATOR (do not change)
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
# PART 3 - POINT TRACKING
# =====================================================================
# STUDENT TODO: point_tracking_controller(X_des, X) -> (phi_dot_l, phi_dot_r)
#   X_des = desired pose [x, y, theta], X = current pose [x, y, theta].
#   Use the controller of Lecture 05A (rho, alpha, beta, gains, backwards
#   method), then get_wheel_velocities(v, w). Add any helper functions you need.


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


    # # Coordinate transformation: alpha and beta (slide 29/35)
    # gamma = math.atan2(dy, dx)
    # alpha = wrap_angle(gamma - theta)
    # beta = wrap_angle(theta_des - theta - alpha)
    #
    # # Backwards method when |alpha| > pi / 2 (slides 40-43 / 46-49)
    # if abs(alpha) > (math.pi / 2.0):
    #     alpha = wrap_angle(math.atan2(-dy, -dx) - theta)
    #     beta = wrap_angle(theta_des - theta - alpha)
    #     v = -K_RHO * rho
    # else:
    #     v = K_RHO * rho
    #
    # # Angular velocity control law (slide 36/42)
    # w = K_ALPHA * alpha + K_BETA * beta
    #
    # # Velocity saturation limits to protect motor commands
    # v = max(-MAX_V, min(MAX_V, v))
    # w = max(-MAX_W, min(MAX_W, w))
    #
    # # Convert linear/angular velocity to wheel speeds (Section 1)
    # return get_wheel_velocities(v, w)

    # Coordinate transformation
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


# STUDENT TODO: 6 unit tests (cases in the assignment), each returns
# (passed, visited_poses). Loop: point_tracking_controller -> robot_simulator
# until is_at_goal(X_des, X). Add a time limit so a test can fail.
pass  # TODO: replace this with your implementation


def _run_test_simulation(X_init, X_des, max_time_s=60.0):
    """Runs simulator until goal is reached or timeout."""
    X = list(X_init)
    visited_poses = [list(X)]
    dt = DT
    max_steps = int(max_time_s / dt)
    
    for _ in range(max_steps):
        if is_at_goal(X_des, X):
            return True, visited_poses
        phi_l, phi_r = point_tracking_controller(X_des, X)
        X, subposes = robot_simulator(X, phi_l, phi_r, dt)
        visited_poses.extend(subposes)
        
    return is_at_goal(X_des, X), visited_poses

def point_tracker_unit_test_1():
    return _run_test_simulation([0.0, 0.0, 0.0], [1.5, 1.5, 0.0])

# 2. X=[0,0,0], x_des>1, y_des<-1, theta_des=0
def point_tracker_unit_test_2():
    return _run_test_simulation([0.0, 0.0, 0.0], [1.5, -1.5, 0.0])

# 3. X=[0,0,0], x_des<-1, y_des>1, theta_des=0
def point_tracker_unit_test_3():
    return _run_test_simulation([0.0, 0.0, 0.0], [-1.5, 1.5, 0.0])

# 4. X=[0,0,0], x_des<-1, y_des<-1, theta_des=0
def point_tracker_unit_test_4():
    return _run_test_simulation([0.0, 0.0, 0.0], [-1.5, -1.5, 0.0])

# 5. X=[0,0,0], x_des>1, y_des>1, theta_des=pi
def point_tracker_unit_test_5():
    return _run_test_simulation([0.0, 0.0, 0.0], [1.5, 1.5, math.pi])

# 6. X=[1,1,0], x_des>2, y_des>2, theta_des=pi
def point_tracker_unit_test_6():
    return _run_test_simulation([1.0, 1.0, 0.0], [2.5, 2.5, math.pi])


def run_point_tracker_unit_tests(plot=True):
    tests = [
        ("Test 1", point_tracker_unit_test_1),
        ("Test 2", point_tracker_unit_test_2),
        ("Test 3", point_tracker_unit_test_3),
        ("Test 4", point_tracker_unit_test_4),
        ("Test 5", point_tracker_unit_test_5),
        ("Test 6", point_tracker_unit_test_6),
    ]
    
    passed_count = 0
    all_trajectories = []
    
    for name, test_fn in tests:
        passed, poses = test_fn()
        if passed:
            passed_count += 1
        all_trajectories.append((name, passed, poses))
        print(f"  {name}: {'PASSED' if passed else 'FAILED'}")
        
    print(f"Point tracker unit tests passed: {passed_count}/{len(tests)}")
    
    # Step 5: Plot all paths on a single figure
    if plot:
        plt.figure(figsize=(9, 7))
        for name, passed, poses in all_trajectories:
            xs = [p[0] for p in poses]
            ys = [p[1] for p in poses]
            line, = plt.plot(xs, ys, label=f"{name} ({'Pass' if passed else 'Fail'})")
            # Mark start and end
            plt.plot(xs[0], ys[0], 'o', color=line.get_color())
            plt.arrow(xs[-1], ys[-1], 0.1 * math.cos(poses[-1][2]), 0.1 * math.sin(poses[-1][2]),
                      head_width=0.04, color=line.get_color())
                      
        plt.title("Point Tracking Controller Unit Tests (Paths)")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.grid(True)
        plt.axis("equal")
        plt.legend()
        plt.savefig("point_tracker_unit_tests.png", dpi=300)
        plt.show()

pass  # TODO: replace this with your implementation


# =====================================================================
# PART 4 - PATH TRACKING
# =====================================================================
# STUDENT TODO: path_tracking_controller(P, X) -> (phi_dot_l, phi_dot_r)
#   P = path, list of poses [[x, y, theta], ...], X = current pose.
#   Use the method of Lecture 06A (closest point, look-ahead distance Delta,
#   point tracking to that point). Add any helper functions you need.
pass  # TODO: replace this with your implementation

# =====================================================================
# PART 4 - PATH TRACKING
# =====================================================================
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
    return point_tracking_controller(X_des, X)       # 3. point tracking


# ---- simulation loop ---------------------------------------------------
def _path_goal_reached(P, X):
    return (math.hypot(P[-1][0] - X[0], P[-1][1] - X[1]) < PATH_POS_TOL and
            abs(wrap_angle(P[-1][2] - X[2])) < PATH_ANG_TOL)


def _run_path_simulation(P, X_init, max_time_s=120.0):
    """Runs controller + simulator until the final path pose is reached or timeout.
    Returns (passed, visited_poses)."""
    X = list(X_init)
    visited_poses = [list(X)]
    for _ in range(int(max_time_s / DT)):
        if _path_goal_reached(P, X):
            return True, visited_poses
        phi_l, phi_r = path_tracking_controller(P, X)
        X, subposes = robot_simulator(X, phi_l, phi_r, DT)
        visited_poses.extend(subposes)
    return _path_goal_reached(P, X), visited_poses


# ---- the three assignment cases: (path, initial pose) ------------------
def _case_1():
    return [[0, 0.1, 0], [2, 0.1, 0]], [0.0, 0.0, 0.0]


def _case_2():
    return ([[0, 0, 0], [1.0, 0, 0], [1.0, 0, math.pi / 2], [1.0, 1.0, math.pi / 2]],
            [0.0, 0.0, 0.0])


def _case_3():
    return [[0, 0, math.pi / 4], [2.0, 2.0, math.pi / 4]], [0.0, 0.0, 0.0]


# ---- unit tests: each returns (passed, visited_poses) ------------------
def path_tracker_unit_test_1():
    P, X0 = _case_1()
    return _run_path_simulation(P, X0)


def path_tracker_unit_test_2():
    P, X0 = _case_2()
    return _run_path_simulation(P, X0)


def path_tracker_unit_test_3():
    P, X0 = _case_3()
    return _run_path_simulation(P, X0)


# ---- runner: print pass count, plot each path as a subfigure -----------
def run_path_tracker_unit_tests(plot=True):
    tests = [
        ("Test 1", path_tracker_unit_test_1, _case_1),
        ("Test 2", path_tracker_unit_test_2, _case_2),
        ("Test 3", path_tracker_unit_test_3, _case_3),
    ]

    passed_count = 0
    results = []
    for name, test_fn, case_fn in tests:
        passed, poses = test_fn()
        P, _ = case_fn()
        passed_count += int(passed)
        results.append((name, passed, poses, P))
        print(f"  {name}: {'PASSED' if passed else 'FAILED'}")
    print(f"Path tracker unit tests passed: {passed_count}/{len(tests)}")

    if plot:
        fig, axs = plt.subplots(1, 3, figsize=(15, 5))
        for ax, (name, passed, poses, P) in zip(axs, results):
            Pd = _densify_path(P)
            ax.plot([p[0] for p in Pd], [p[1] for p in Pd], "k--", label="desired path")
            ax.plot([p[0] for p in P], [p[1] for p in P], "ks", label="waypoints")
            ax.plot([p[0] for p in poses], [p[1] for p in poses], label="robot")
            ax.plot(poses[0][0], poses[0][1], "go", label="start")
            ax.plot(poses[-1][0], poses[-1][1], "rx", label="end")
            ax.set_title(f"{name} ({'Pass' if passed else 'Fail'})")
            ax.set_xlabel("X (m)")
            ax.set_ylabel("Y (m)")
            ax.axis("equal")
            ax.grid(True)
            ax.legend()
        plt.tight_layout()
        plt.savefig("path_tracker_unit_tests.png", dpi=300)
        plt.show()

if __name__ == "__main__":
    print("=== Part 1: wheel velocities ===")
    run_wheel_velocity_unit_tests()
    print("\n=== Part 3: point tracking ===")
    run_point_tracker_unit_tests()
    try:
        run_extra_point_tests()
    except NameError:
        pass
    print("\n=== Part 4: path tracking ===")
    run_path_tracker_unit_tests()
