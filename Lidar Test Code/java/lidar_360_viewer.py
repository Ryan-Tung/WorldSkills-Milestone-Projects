import collections
import logging
import threading

from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Arc, Circle
import matplotlib.pyplot as plt
from networktables import NetworkTables
import numpy as np

# ============================================================
# MATPLOTLIB CONFIGURATION & KEYBIND UNBINDING
# ============================================================
for keymap in list(plt.rcParams.keys()):
    if keymap.startswith("keymap."):
        plt.rcParams[keymap] = []

logging.basicConfig(level=logging.WARNING)

# ============================================================
# CONFIGURATION
# ============================================================
ROBOT_IP = "10.12.34.2"

# LiDAR
MAX_DISTANCE_METERS = 5.0
MIN_DISTANCE_METERS = 0.05
CONNECT_DISTANCE = 0.30
MAX_SCAN_POINTS = 360

# MAP
MAP_SIZE_METERS = 20.0
GRID_RESOLUTION = 0.05  # 5 cm per cell
MAP_CELLS = int(MAP_SIZE_METERS / GRID_RESOLUTION)

FREE_UPDATE = -0.25
OCCUPIED_UPDATE = 0.85
LOG_ODDS_MIN = -5.0
LOG_ODDS_MAX = 5.0

# ROBOT POSE & LOG KEYS
POSE_TABLE_NAME = "Drive"
POSE_X_KEY = "PoseX"
POSE_Y_KEY = "PoseY"
POSE_HEADING_KEY = "PoseHeading"

LOG_KEYS = ["DriveLog", "RobotContainer", "LidarLog", "IRLog"]

# DRIVE CONTROL CONFIGURATION (Differential/Unicycle Teleop)
CONTROL_TABLE_NAME = "DriveControls"
MAX_LINEAR_SPEED = 1.0  # Linear speed factor
MAX_ANGULAR_SPEED = 1.0  # Angular speed factor

# ============================================================
# THREAD SAFETY & GLOBAL DATA
# ============================================================
data_lock = threading.Lock()

angles_1, distances_1 = np.array([]), np.array([])
angles_2, distances_2 = np.array([]), np.array([])

# Raw values from NetworkTables
raw_robot_x = 0.0
raw_robot_y = 0.0
raw_robot_heading = 0.0

# Reference offsets for global frame reset (R key)
ref_x = 0.0
ref_y = 0.0
ref_heading = 0.0

# Transformed relative pose
robot_x = 0.0
robot_y = 0.0
robot_heading = 0.0  # Theta in degrees relative to X_G axis

scan_update_counter = 0
last_mapped_scan = -1

occupancy_grid = np.zeros((MAP_CELLS, MAP_CELLS), dtype=np.float32)

robot_path_x, robot_path_y = [], []
log_history = collections.deque(maxlen=10)

pressed_keys = set()
control_table = None


# ============================================================
# KEYBOARD CONTROL CALLBACKS (WASD & R)
# ============================================================


def compute_relative_pose(rx, ry, rh):
    """Transforms raw NetworkTables pose into the reset global frame."""
    dx = rx - ref_x
    dy = ry - ref_y

    # Rotate vector so current facing direction maps to 0 degrees (+Y axis)
    alpha = np.radians(0.0 - ref_heading)
    rel_x = dx * np.cos(alpha) - dy * np.sin(alpha)
    rel_y = dx * np.sin(alpha) + dy * np.cos(alpha)
    rel_heading = (rh - ref_heading + 0.0) % 360.0

    return rel_x, rel_y, rel_heading


def reset_global_frame():
    """Resets global frame origin (0,0), sets heading to 90 deg, and triggers NavX reset on NT."""
    global ref_x, ref_y, ref_heading
    global robot_x, robot_y, robot_heading
    global occupancy_grid

    with data_lock:
        ref_x = raw_robot_x
        ref_y = raw_robot_y
        ref_heading = raw_robot_heading

        robot_x, robot_y, robot_heading = compute_relative_pose(
            raw_robot_x, raw_robot_y, raw_robot_heading
        )

        # Reset map and trajectory history for the new frame
        occupancy_grid.fill(0)
        robot_path_x.clear()
        robot_path_y.clear()

        # Publish NavX reset command to NetworkTables
        if control_table is not None:
            control_table.putBoolean("ResetNavX", True)
            control_table.putNumber("TargetHeading", 0.0)

        log_history.append(
            "[SYSTEM] Frame & NavX Reset: Pose set to (0.0, 0.0, θ=0.0°)"
        )


def on_key_press(event):
    if event.key is not None:
        key = event.key.lower()
        if key == " ":
            pressed_keys.clear()
        elif key == "r":
            reset_global_frame()
        else:
            pressed_keys.add(key)


def on_key_release(event):
    if event.key is not None:
        key = event.key.lower()
        pressed_keys.discard(key)


def publish_drive_commands():
    """Publishes Forward/Backward and Turning commands via WASD."""
    if control_table is None:
        return 0.0, 0.0

    forward = 0.0
    turn = 0.0

    # Linear Movement (W = Forward, S = Reverse)
    if "w" in pressed_keys:
        forward += MAX_LINEAR_SPEED
    if "s" in pressed_keys:
        forward -= MAX_LINEAR_SPEED

    # Angular Movement (A = Turn Left / CCW, D = Turn Right / CW)
    if "d" in pressed_keys:
        turn += MAX_ANGULAR_SPEED
    if "a" in pressed_keys:
        turn -= MAX_ANGULAR_SPEED

    control_table.putNumber("CmdForward", forward)
    control_table.putNumber("CmdTurn", turn)

    # Clear pulse after sending
    if "r" not in pressed_keys:
        control_table.putBoolean("ResetNavX", False)

    return forward, turn


# ============================================================
# NETWORKTABLE CALLBACKS
# ============================================================


def value_changed_callback(table, key, value, isNew):
    global angles_1, distances_1, angles_2, distances_2
    global raw_robot_x, raw_robot_y, raw_robot_heading
    global robot_x, robot_y, robot_heading
    global scan_update_counter

    with data_lock:
        if key == "ScanAngles_Part1":
            angles_1 = np.asarray(value, dtype=float)
        elif key == "ScanDistances_Part1":
            distances_1 = np.asarray(value, dtype=float)
        elif key == "ScanAngles_Part2":
            angles_2 = np.asarray(value, dtype=float)
        elif key == "ScanDistances_Part2":
            distances_2 = np.asarray(value, dtype=float)
            scan_update_counter += 1

        elif key == POSE_X_KEY:
            raw_robot_x = float(value)
            robot_x, robot_y, robot_heading = compute_relative_pose(
                raw_robot_x, raw_robot_y, raw_robot_heading
            )
        elif key == POSE_Y_KEY:
            raw_robot_y = float(value)
            robot_x, robot_y, robot_heading = compute_relative_pose(
                raw_robot_x, raw_robot_y, raw_robot_heading
            )
        elif key == POSE_HEADING_KEY:
            raw_robot_heading = float(value)
            robot_x, robot_y, robot_heading = compute_relative_pose(
                raw_robot_x, raw_robot_y, raw_robot_heading
            )


def log_callback(table, key, value, isNew):
    log_line = f"[{key}] ROBOT: {value}"
    with data_lock:
        log_history.append(log_line)


# ============================================================
# UTILITY & MATH FUNCTIONS
# ============================================================


def get_combined_scan():
    with data_lock:
        a1, d1 = angles_1.copy(), distances_1.copy()
        a2, d2 = angles_2.copy(), distances_2.copy()

    n1, n2 = min(len(a1), len(d1)), min(len(a2), len(d2))
    if n1 == 0 and n2 == 0:
        return np.array([]), np.array([])

    angles = np.concatenate([a1[:n1], a2[:n2]])
    distances = np.concatenate([d1[:n1], d2[:n2]])

    valid = np.isfinite(angles) & np.isfinite(distances)
    angles, distances = angles[valid], distances[valid]

    order = np.argsort(angles)
    return angles[order], distances[order]


def lidar_to_local(angles_deg, distances_mm):
    distances_m = distances_mm / 1000.0
    angles_rad = np.radians(angles_deg)
    return distances_m * np.sin(angles_rad), distances_m * np.cos(angles_rad)


def local_to_global(local_x, local_y, x_robot, y_robot, heading_deg):
    heading = np.radians(heading_deg)
    gx = x_robot + local_x * np.cos(heading) + local_y * np.sin(heading)
    gy = y_robot - local_x * np.sin(heading) + local_y * np.cos(heading)
    return gx, gy


def world_to_grid(x, y):
    map_min = -MAP_SIZE_METERS / 2
    grid_x = ((x - map_min) / GRID_RESOLUTION).astype(int)
    grid_y = ((y - map_min) / GRID_RESOLUTION).astype(int)
    return grid_x, grid_y


def valid_grid_cell(x, y):
    return 0 <= x < MAP_CELLS and 0 <= y < MAP_CELLS


def bresenham(x0, y0, x1, y1):
    points = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return points


def update_occupancy_map(local_x, local_y, x_robot, y_robot, heading):
    global occupancy_grid
    world_x, world_y = local_to_global(
        local_x, local_y, x_robot, y_robot, heading
    )

    robot_grid_x, robot_grid_y = world_to_grid(
        np.array([x_robot]), np.array([y_robot])
    )
    rx, ry = int(robot_grid_x[0]), int(robot_grid_y[0])

    if not valid_grid_cell(rx, ry):
        return

    obstacle_grid_x, obstacle_grid_y = world_to_grid(world_x, world_y)

    for gx, gy in zip(obstacle_grid_x, obstacle_grid_y):
        if not valid_grid_cell(gx, gy):
            continue
        ray = bresenham(rx, ry, int(gx), int(gy))
        if len(ray) < 2:
            continue

        for cell_x, cell_y in ray[:-1]:
            if valid_grid_cell(cell_x, cell_y):
                occupancy_grid[cell_y, cell_x] += FREE_UPDATE

        occupancy_grid[gy, gx] += OCCUPIED_UPDATE

    np.clip(
        occupancy_grid, LOG_ODDS_MIN, LOG_ODDS_MAX, out=occupancy_grid
    )


def build_segments(angles_deg, distances_mm):
    if len(angles_deg) < 2:
        return []

    distances_m = distances_mm / 1000.0
    valid = (
        np.isfinite(distances_m)
        & (distances_m > MIN_DISTANCE_METERS)
        & (distances_m <= MAX_DISTANCE_METERS)
    )

    angles_deg, distances_m = angles_deg[valid], distances_m[valid]
    if len(angles_deg) < 2:
        return []

    if len(angles_deg) > MAX_SCAN_POINTS:
        indices = np.linspace(
            0, len(angles_deg) - 1, MAX_SCAN_POINTS
        ).astype(int)
        angles_deg, distances_m = angles_deg[indices], distances_m[indices]

    lx, ly = lidar_to_local(angles_deg, distances_m * 1000)
    points = np.column_stack([lx, ly])
    segments = []

    for i in range(len(points) - 1):
        p1, p2 = points[i], points[i + 1]
        if np.linalg.norm(p2 - p1) <= CONNECT_DISTANCE:
            segments.append([p1, p2])

    return segments


def update_map_display():
    probability = 1 - 1 / (1 + np.exp(occupancy_grid))
    map_image.set_data(probability)


# ============================================================
# MAIN PLOT UPDATE
# ============================================================


def update_plot(frame):
    global last_mapped_scan

    fwd, trn = publish_drive_commands()

    with data_lock:
        cx, cy, ch = robot_x, robot_y, robot_heading
        current_scan_counter = scan_update_counter
        logs_text = "\n".join(log_history)

    log_display.set_text(
        logs_text if logs_text else "Waiting for system logs..."
    )

    angles, distances = get_combined_scan()
    if len(angles) == 0:
        return

    distances_m = distances / 1000.0
    valid = (
        np.isfinite(angles)
        & np.isfinite(distances_m)
        & (distances_m > MIN_DISTANCE_METERS)
        & (distances_m <= MAX_DISTANCE_METERS)
    )

    valid_angles, valid_distances = angles[valid], distances_m[valid]
    local_x, local_y = lidar_to_local(valid_angles, valid_distances * 1000)

    # Update Local LiDAR Subplot
    current_points = np.column_stack([local_x, local_y])
    local_scan_points.set_offsets(current_points)
    segments = build_segments(valid_angles, valid_distances * 1000)
    local_environment.set_segments(segments)
    local_environment_glow.set_segments(segments)

    # Path Tracing (Global Frame)
    robot_path_x.append(cx)
    robot_path_y.append(cy)
    if len(robot_path_x) > 5000:
        del robot_path_x[:-5000]
        del robot_path_y[:-5000]

    robot_path.set_data(robot_path_x, robot_path_y)

    # Occupancy Map Update
    if current_scan_counter != last_mapped_scan:
        last_mapped_scan = current_scan_counter
        if len(local_x) > MAX_SCAN_POINTS:
            indices = np.linspace(
                0, len(local_x) - 1, MAX_SCAN_POINTS
            ).astype(int)
            mx, my = local_x[indices], local_y[indices]
        else:
            mx, my = local_x, local_y

        update_occupancy_map(mx, my, cx, cy, ch)
        update_map_display()

    # Update Global Robot Marker, Arrow Vector, & Theta Text
    global_robot_body.set_center((cx, cy))

    rad = np.radians(ch)
    arrow_len = 1.2
    end_x = cx + arrow_len * np.cos(rad)
    end_y = cy + arrow_len * np.sin(rad)

    global_heading_line.set_data([cx, end_x], [cy, end_y])
    global_reference_x_line.set_data([cx, cx + 1.0], [cy, cy])

    theta_label.set_position((cx + 0.3, cy + 0.3))
    theta_label.set_text(f"θ = {ch:.1f}°")

    # Local Subplot Orientation Indicators
    robot_marker.set_data([0], [0])
    robot_direction.set_data(
        [0, 0.5 * np.sin(rad)], [0, 0.5 * np.cos(rad)]
    )

    # Status Bar
    status_text.set_text(
        f"● POSE: X_G={cx:.2f}m | Y_G={cy:.2f}m | θ={ch:.1f}°   "
        f"| TELEOP (WASD): Fwd={fwd:+.1f} Turn={trn:+.1f} | [R] Reset Frame & NavX"
    )


# ============================================================
# MAIN APPLICATION SETUP
# ============================================================


def main():
    global control_table
    global local_environment, local_environment_glow, local_scan_points
    global robot_marker, robot_direction, robot_path
    global global_robot_body, global_heading_line, global_reference_x_line, theta_label
    global map_image, status_text, log_display

    print(f"Connecting to NetworkTables at {ROBOT_IP}...")
    NetworkTables.initialize(server=ROBOT_IP)
    NetworkTables.setUpdateRate(0.010)

    lidar_table = NetworkTables.getTable("Lidar")
    pose_table = NetworkTables.getTable(POSE_TABLE_NAME)
    sd_table = NetworkTables.getTable("SmartDashboard")
    control_table = NetworkTables.getTable(CONTROL_TABLE_NAME)

    lidar_table.addEntryListener(value_changed_callback)
    pose_table.addEntryListener(value_changed_callback)

    for key_name in LOG_KEYS:
        sd_table.addEntryListener(log_callback, key=key_name)

    # Styling UI window
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(16, 9.5), facecolor="#080b0e")

    fig.canvas.mpl_connect("key_press_event", on_key_press)
    fig.canvas.mpl_connect("key_release_event", on_key_release)

    gs = fig.add_gridspec(2, 2, height_ratios=[3.2, 1.0], hspace=0.25, wspace=0.18)

    # ------------------------------------------------------------
    # 1. GLOBAL FRAME SUBPLOT (X_G, Y_G, Robot Circle, Theta Arrow)
    # ------------------------------------------------------------
    ax_map = fig.add_subplot(gs[0, 0])
    ax_map.set_title(
        "GLOBAL FRAME (X_G , Y_G)", color="#00E5FF", fontsize=12, fontweight="bold", pad=10
    )
    ax_map.set_facecolor("#04070a")
    ax_map.set_aspect("equal")
    ax_map.set_xlim(-MAP_SIZE_METERS / 2, MAP_SIZE_METERS / 2)
    ax_map.set_ylim(-MAP_SIZE_METERS / 2, MAP_SIZE_METERS / 2)

    ax_map.set_xlabel("X_G Axis (meters)", color="#80A0C0", fontweight="bold")
    ax_map.set_ylabel("Y_G Axis (meters)", color="#80A0C0", fontweight="bold")
    ax_map.tick_params(colors="#507090")
    ax_map.grid(color="#122535", linestyle="--", alpha=0.6)

    ax_map.axhline(0, color="#1E3A52", linewidth=1.2, linestyle=":")
    ax_map.axvline(0, color="#1E3A52", linewidth=1.2, linestyle=":")

    cyan_cmap = LinearSegmentedColormap.from_list(
        "dark_cyan", ["#04070a", "#00E5FF"]
    )

    map_image = ax_map.imshow(
        np.zeros_like(occupancy_grid),
        origin="lower",
        extent=[
            -MAP_SIZE_METERS / 2,
            MAP_SIZE_METERS / 2,
            -MAP_SIZE_METERS / 2,
            MAP_SIZE_METERS / 2,
        ],
        cmap=cyan_cmap,
        vmin=0,
        vmax=1,
        alpha=0.65,
        interpolation="nearest",
    )

    (robot_path,) = ax_map.plot(
        [], [], color="#00E5FF", linewidth=1.5, alpha=0.75, label="Global Path"
    )

    global_robot_body = Circle(
        (0, 0), 0.45, facecolor="#00E5FF", edgecolor="#FFFFFF", alpha=0.85, zorder=25
    )
    ax_map.add_patch(global_robot_body)

    (global_heading_line,) = ax_map.plot(
        [], [], color="#FFEA00", linewidth=2.5, zorder=30, label="Heading (θ)"
    )
    (global_reference_x_line,) = ax_map.plot(
        [], [], color="#FFFFFF", linewidth=1.0, linestyle="--", zorder=26
    )

    theta_label = ax_map.text(
        0, 0, "θ = 0.0°", color="#FFEA00", fontsize=10, fontweight="bold", zorder=35
    )

    # ------------------------------------------------------------
    # 2. LOCAL LiDAR SUBPLOT
    # ------------------------------------------------------------
    ax_local = fig.add_subplot(gs[0, 1])
    ax_local.set_title(
        "LOCAL LiDAR FRAME", color="#00E5FF", fontsize=12, fontweight="bold", pad=10
    )
    ax_local.set_facecolor("#04070a")
    ax_local.set_aspect("equal")
    ax_local.set_xlim(-MAX_DISTANCE_METERS, MAX_DISTANCE_METERS)
    ax_local.set_ylim(-MAX_DISTANCE_METERS, MAX_DISTANCE_METERS)
    ax_local.set_xlabel("+Y (Right) / -Y (Left) [m]", color="#80A0C0")
    ax_local.set_ylabel("+X (Forward) / -X (Back) [m]", color="#80A0C0")
    ax_local.tick_params(colors="#507090")
    ax_local.grid(color="#122535", linestyle="--", alpha=0.6)

    local_environment_glow = LineCollection(
        [], linewidths=6, color="#00E5FF", alpha=0.08, capstyle="round"
    )
    ax_local.add_collection(local_environment_glow)

    local_environment = LineCollection(
        [], linewidths=1.8, color="#00E5FF", alpha=0.9, capstyle="round"
    )
    ax_local.add_collection(local_environment)

    local_scan_points = ax_local.scatter(
        [], [], s=5, color="#A5F3FC", alpha=0.5, edgecolors="none"
    )

    (robot_marker,) = ax_local.plot(
        [0], [0], marker="o", markersize=8, color="#00E5FF", markeredgecolor="white"
    )
    (robot_direction,) = ax_local.plot(
        [0, 0], [0, 0.5], color="#FFEA00", linewidth=2
    )

    for r in range(1, 6):
        ax_local.add_patch(
            Circle(
                (0, 0), r, fill=False, edgecolor="#122535", linewidth=0.8, linestyle="--"
            )
        )
        ax_local.text(0.05, r, f"{r}m", color="#3A5D7C", fontsize=7)

    # ------------------------------------------------------------
    # 3. CONSOLE LOG WINDOW
    # ------------------------------------------------------------
    ax_log = fig.add_subplot(gs[1, :])
    ax_log.set_title(
        "SYSTEM CONSOLE LOGS", color="#00E5FF", fontsize=10, fontweight="bold", loc="left"
    )
    ax_log.set_facecolor("#020305")
    ax_log.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    log_display = ax_log.text(
        0.01,
        0.85,
        "Initializing NetworkTables listener...",
        color="#00E5FF",
        fontsize=8.5,
        family="monospace",
        verticalalignment="top",
    )

    # Status Bar
    status_text = fig.text(
        0.02, 0.015, "● LIVE", color="#00E5FF", fontsize=9.5, fontweight="bold"
    )

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=50,
        blit=False,
        cache_frame_data=False,
    )

    plt.tight_layout(rect=[0, 0.03, 1, 0.98])
    plt.show()


if __name__ == "__main__":
    main()