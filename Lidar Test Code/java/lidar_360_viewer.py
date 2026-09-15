import collections
import logging
import threading

from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle
import matplotlib.pyplot as plt
from networktables import NetworkTables
import numpy as np


# ============================================================
# MATPLOTLIB CONFIGURATION
# ============================================================

for keymap in list(plt.rcParams.keys()):
    if keymap.startswith("keymap."):
        plt.rcParams[keymap] = []

logging.basicConfig(level=logging.WARNING)


# ============================================================
# CONFIGURATION
# ============================================================

ROBOT_IP = "10.12.34.2"

# ------------------------------------------------------------
# LiDAR
# ------------------------------------------------------------

MAX_DISTANCE_METERS = 5.0
MIN_DISTANCE_METERS = 0.05

CONNECT_DISTANCE = 0.30
MAX_SCAN_POINTS = 360


# ------------------------------------------------------------
# MAP
# ------------------------------------------------------------

MAP_SIZE_METERS = 20.0
GRID_RESOLUTION = 0.05
MAP_CELLS = int(MAP_SIZE_METERS / GRID_RESOLUTION)

# Reduced update strength to prevent ghosting
FREE_UPDATE = -0.35
OCCUPIED_UPDATE = 0.55

LOG_ODDS_MIN = -3.0
LOG_ODDS_MAX = 3.0

# Slowly forget old measurements
MAP_DECAY = 0.995


# ------------------------------------------------------------
# POSE
# ------------------------------------------------------------

POSE_TABLE_NAME = "Drive"

POSE_X_KEY = "PoseX"
POSE_Y_KEY = "PoseY"
POSE_HEADING_KEY = "PoseHeading"


# ------------------------------------------------------------
# LOGGING
# ------------------------------------------------------------

LOG_KEYS = [
    "DriveLog",
    "RobotContainer",
    "LidarLog",
    "IRLog"
]


# ------------------------------------------------------------
# DRIVE CONTROL
# ------------------------------------------------------------

CONTROL_TABLE_NAME = "DriveControls"

MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 0.5


# ============================================================
# THREAD SAFETY & GLOBAL DATA
# ============================================================

data_lock = threading.Lock()

angles_1 = np.array([])
distances_1 = np.array([])

angles_2 = np.array([])
distances_2 = np.array([])


# Raw NetworkTables pose
raw_robot_x = 0.0
raw_robot_y = 0.0
raw_robot_heading = 0.0


# Reference frame after R reset
ref_x = 0.0
ref_y = 0.0
ref_heading = 0.0


# Relative robot pose
robot_x = 0.0
robot_y = 0.0
robot_heading = 0.0


# Scan tracking
scan_update_counter = 0
last_mapped_scan = -1


# Occupancy grid
occupancy_grid = np.zeros(
    (MAP_CELLS, MAP_CELLS),
    dtype=np.float32
)


# Robot path
robot_path_x = []
robot_path_y = []

# Only add a path point after moving this distance
PATH_POINT_DISTANCE = 0.05

last_path_x = None
last_path_y = None


# Logs
log_history = collections.deque(maxlen=10)


# Keyboard
pressed_keys = set()

control_table = None


# ============================================================
# POSE TRANSFORMATION
# ============================================================

def compute_relative_pose(rx, ry, rh):
    """
    Convert raw robot pose into the user-reset global frame.

    After pressing R:
        robot position = (0, 0)
        robot heading = 0 degrees

    The translation is rotated into the new global frame.
    """

    dx = rx - ref_x
    dy = ry - ref_y

    # Rotate the displacement by the inverse reference heading
    alpha = np.radians(ref_heading)

    rel_x = (
        dx * np.cos(alpha)
        - dy * np.sin(alpha)
    )

    rel_y = (
        dx * np.sin(alpha)
        + dy * np.cos(alpha)
    )

    rel_heading = (
        rh - ref_heading
    ) % 360.0

    return rel_x, rel_y, rel_heading


# ============================================================
# RESET GLOBAL FRAME
# ============================================================

def reset_global_frame():

    global ref_x
    global ref_y
    global ref_heading

    global robot_x
    global robot_y
    global robot_heading

    global occupancy_grid

    global last_path_x
    global last_path_y

    with data_lock:

        # Capture current raw pose
        ref_x = raw_robot_x
        ref_y = raw_robot_y
        ref_heading = raw_robot_heading

        # Immediately transform current pose
        robot_x, robot_y, robot_heading = compute_relative_pose(
            raw_robot_x,
            raw_robot_y,
            raw_robot_heading
        )

        # ----------------------------------------------------
        # Clear map
        # ----------------------------------------------------

        occupancy_grid.fill(0)

        # ----------------------------------------------------
        # Clear path
        # ----------------------------------------------------

        robot_path_x.clear()
        robot_path_y.clear()

        last_path_x = 0.0
        last_path_y = 0.0

        # ----------------------------------------------------
        # Tell robot to reset NavX
        # ----------------------------------------------------

        if control_table is not None:

            control_table.putBoolean(
                "ResetNavX",
                True
            )

            control_table.putNumber(
                "TargetHeading",
                0.0
            )

        log_history.append(
            "[SYSTEM] Global frame reset: "
            "Pose = (0.00, 0.00), Heading = 0.0°"
        )


# ============================================================
# KEYBOARD
# ============================================================

def on_key_press(event):

    if event.key is None:
        return

    key = event.key.lower()

    if key == " ":

        pressed_keys.clear()

    elif key == "r":

        reset_global_frame()

    else:

        pressed_keys.add(key)


def on_key_release(event):

    if event.key is None:
        return

    key = event.key.lower()

    pressed_keys.discard(key)


# ============================================================
# DRIVE COMMANDS
# ============================================================

def publish_drive_commands():

    if control_table is None:
        return 0.0, 0.0

    forward = 0.0
    turn = 0.0

    # Forward / reverse
    if "w" in pressed_keys:
        forward += MAX_LINEAR_SPEED

    if "s" in pressed_keys:
        forward -= MAX_LINEAR_SPEED

    # Turning
    if "d" in pressed_keys:
        turn += MAX_ANGULAR_SPEED

    if "a" in pressed_keys:
        turn -= MAX_ANGULAR_SPEED

    control_table.putNumber(
        "CmdForward",
        forward
    )

    control_table.putNumber(
        "CmdTurn",
        turn
    )

    # Reset pulse
    if "r" not in pressed_keys:

        control_table.putBoolean(
            "ResetNavX",
            False
        )

    return forward, turn


# ============================================================
# NETWORKTABLE CALLBACK
# ============================================================

def value_changed_callback(
    table,
    key,
    value,
    isNew
):

    global angles_1
    global distances_1

    global angles_2
    global distances_2

    global raw_robot_x
    global raw_robot_y
    global raw_robot_heading

    global robot_x
    global robot_y
    global robot_heading

    global scan_update_counter

    with data_lock:

        # ----------------------------------------------------
        # LiDAR
        # ----------------------------------------------------

        if key == "ScanAngles_Part1":

            angles_1 = np.asarray(
                value,
                dtype=float
            )

        elif key == "ScanDistances_Part1":

            distances_1 = np.asarray(
                value,
                dtype=float
            )

        elif key == "ScanAngles_Part2":

            angles_2 = np.asarray(
                value,
                dtype=float
            )

        elif key == "ScanDistances_Part2":

            distances_2 = np.asarray(
                value,
                dtype=float
            )

            scan_update_counter += 1

        # ----------------------------------------------------
        # Pose
        # ----------------------------------------------------

        elif key == POSE_X_KEY:

            raw_robot_x = float(value)

            robot_x, robot_y, robot_heading = (
                compute_relative_pose(
                    raw_robot_x,
                    raw_robot_y,
                    raw_robot_heading
                )
            )

        elif key == POSE_Y_KEY:

            raw_robot_y = float(value)

            robot_x, robot_y, robot_heading = (
                compute_relative_pose(
                    raw_robot_x,
                    raw_robot_y,
                    raw_robot_heading
                )
            )

        elif key == POSE_HEADING_KEY:

            raw_robot_heading = float(value)

            robot_x, robot_y, robot_heading = (
                compute_relative_pose(
                    raw_robot_x,
                    raw_robot_y,
                    raw_robot_heading
                )
            )


# ============================================================
# LOG CALLBACK
# ============================================================

def log_callback(
    table,
    key,
    value,
    isNew
):

    log_line = f"[{key}] ROBOT: {value}"

    with data_lock:
        log_history.append(log_line)


# ============================================================
# COMBINE LiDAR SCANS
# ============================================================

def get_combined_scan():

    with data_lock:

        a1 = angles_1.copy()
        d1 = distances_1.copy()

        a2 = angles_2.copy()
        d2 = distances_2.copy()

    n1 = min(
        len(a1),
        len(d1)
    )

    n2 = min(
        len(a2),
        len(d2)
    )

    if n1 == 0 and n2 == 0:

        return (
            np.array([]),
            np.array([])
        )

    angles = np.concatenate(
        [
            a1[:n1],
            a2[:n2]
        ]
    )

    distances = np.concatenate(
        [
            d1[:n1],
            d2[:n2]
        ]
    )

    valid = (
        np.isfinite(angles)
        & np.isfinite(distances)
    )

    angles = angles[valid]
    distances = distances[valid]

    order = np.argsort(angles)

    return (
        angles[order],
        distances[order]
    )


# ============================================================
# LiDAR COORDINATES
# ============================================================

def lidar_to_local(
    angles_deg,
    distances_mm
):

    distances_m = distances_mm / 1000.0

    angles_rad = np.radians(
        angles_deg
    )

    # IMPORTANT:
    #
    # Local frame:
    #   X = Forward
    #   Y = Right
    #
    local_x = (
        distances_m
        * np.cos(angles_rad)
    )

    local_y = (
        distances_m
        * np.sin(angles_rad)
    )

    return local_x, local_y


# ============================================================
# LOCAL -> GLOBAL
# ============================================================

def local_to_global(
    local_x,
    local_y,
    x_robot,
    y_robot,
    heading_deg
):

    heading = np.radians(
        heading_deg
    )

    # Rotate local robot coordinates
    # into the global frame.

    global_x = (
        x_robot
        + local_x * np.cos(heading)
        + local_y * np.sin(heading)
    )
 
    global_y = (
        y_robot
        + local_x * np.sin(heading)
        - local_y * np.cos(heading)
    )

    return global_x, global_y


# ============================================================
# WORLD -> GRID
# ============================================================

def world_to_grid(
    x,
    y
):

    map_min = -MAP_SIZE_METERS / 2

    grid_x = (
        (x - map_min)
        / GRID_RESOLUTION
    ).astype(int)

    grid_y = (
        (y - map_min)
        / GRID_RESOLUTION
    ).astype(int)

    return grid_x, grid_y


def valid_grid_cell(
    x,
    y
):

    return (
        0 <= x < MAP_CELLS
        and
        0 <= y < MAP_CELLS
    )


# ============================================================
# BRESENHAM
# ============================================================

def bresenham(
    x0,
    y0,
    x1,
    y1
):

    points = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)

    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1

    err = dx - dy

    while True:

        points.append(
            (x0, y0)
        )

        if (
            x0 == x1
            and
            y0 == y1
        ):
            break

        e2 = 2 * err

        if e2 > -dy:

            err -= dy
            x0 += sx

        if e2 < dx:

            err += dx
            y0 += sy

    return points


# ============================================================
# OCCUPANCY MAP
# ============================================================

def update_occupancy_map(
    local_x,
    local_y,
    x_robot,
    y_robot,
    heading
):

    global occupancy_grid

    # --------------------------------------------------------
    # Convert LiDAR points into global coordinates
    # --------------------------------------------------------

    world_x, world_y = local_to_global(
        local_x,
        local_y,
        x_robot,
        y_robot,
        heading
    )

    # --------------------------------------------------------
    # Robot grid position
    # --------------------------------------------------------

    robot_grid_x, robot_grid_y = world_to_grid(
        np.array([x_robot]),
        np.array([y_robot])
    )

    rx = int(robot_grid_x[0])
    ry = int(robot_grid_y[0])

    if not valid_grid_cell(rx, ry):
        return

    # --------------------------------------------------------
    # Apply slow decay to old evidence
    # --------------------------------------------------------

    occupancy_grid *= MAP_DECAY

    # --------------------------------------------------------
    # Convert obstacles to grid coordinates
    # --------------------------------------------------------

    obstacle_grid_x, obstacle_grid_y = world_to_grid(
        world_x,
        world_y
    )

    # --------------------------------------------------------
    # Ray tracing
    # --------------------------------------------------------

    for gx, gy in zip(
        obstacle_grid_x,
        obstacle_grid_y
    ):

        if not valid_grid_cell(
            gx,
            gy
        ):
            continue

        ray = bresenham(
            rx,
            ry,
            int(gx),
            int(gy)
        )

        if len(ray) < 2:
            continue

        # Free space
        for cell_x, cell_y in ray[:-1]:

            if valid_grid_cell(
                cell_x,
                cell_y
            ):

                occupancy_grid[
                    cell_y,
                    cell_x
                ] += FREE_UPDATE

        # Occupied endpoint
        occupancy_grid[
            gy,
            gx
        ] += OCCUPIED_UPDATE

    np.clip(
        occupancy_grid,
        LOG_ODDS_MIN,
        LOG_ODDS_MAX,
        out=occupancy_grid
    )


# ============================================================
# LiDAR SEGMENTS
# ============================================================

def build_segments(
    angles_deg,
    distances_mm
):

    if len(angles_deg) < 2:
        return []

    distances_m = (
        distances_mm / 1000.0
    )

    valid = (
        np.isfinite(distances_m)
        &
        (
            distances_m
            > MIN_DISTANCE_METERS
        )
        &
        (
            distances_m
            <= MAX_DISTANCE_METERS
        )
    )

    angles_deg = angles_deg[valid]
    distances_m = distances_m[valid]

    if len(angles_deg) < 2:
        return []

    if len(angles_deg) > MAX_SCAN_POINTS:

        indices = np.linspace(
            0,
            len(angles_deg) - 1,
            MAX_SCAN_POINTS
        ).astype(int)

        angles_deg = angles_deg[
            indices
        ]

        distances_m = distances_m[
            indices
        ]

    lx, ly = lidar_to_local(
        angles_deg,
        distances_m * 1000
    )

    points = np.column_stack(
        [lx, ly]
    )

    segments = []

    for i in range(
        len(points) - 1
    ):

        p1 = points[i]
        p2 = points[i + 1]

        if np.linalg.norm(
            p2 - p1
        ) <= CONNECT_DISTANCE:

            segments.append(
                [p1, p2]
            )

    return segments


# ============================================================
# MAP DISPLAY
# ============================================================

def update_map_display():

    probability = (
        1.0
        -
        1.0
        /
        (
            1.0
            +
            np.exp(occupancy_grid)
        )
    )

    map_image.set_data(
        probability
    )


# ============================================================
# PATH UPDATE
# ============================================================

def update_robot_path(
    x,
    y
):

    global last_path_x
    global last_path_y

    # First point
    if last_path_x is None:

        robot_path_x.append(x)
        robot_path_y.append(y)

        last_path_x = x
        last_path_y = y

        return

    distance = np.hypot(
        x - last_path_x,
        y - last_path_y
    )

    # Only add a point after actually moving
    if distance >= PATH_POINT_DISTANCE:

        robot_path_x.append(x)
        robot_path_y.append(y)

        last_path_x = x
        last_path_y = y

    # Limit path history
    if len(robot_path_x) > 5000:

        del robot_path_x[:-5000]
        del robot_path_y[:-5000]


# ============================================================
# MAIN PLOT UPDATE
# ============================================================

def update_plot(frame):

    global last_mapped_scan

    # --------------------------------------------------------
    # Drive
    # --------------------------------------------------------

    fwd, trn = publish_drive_commands()

    # --------------------------------------------------------
    # Copy pose safely
    # --------------------------------------------------------

    with data_lock:

        cx = robot_x
        cy = robot_y
        ch = robot_heading

        current_scan_counter = (
            scan_update_counter
        )

        logs_text = "\n".join(
            log_history
        )

    # --------------------------------------------------------
    # Logs
    # --------------------------------------------------------

    log_display.set_text(
        logs_text
        if logs_text
        else
        "Waiting for system logs..."
    )

    # --------------------------------------------------------
    # Get LiDAR
    # --------------------------------------------------------

    angles, distances = (
        get_combined_scan()
    )

    if len(angles) == 0:
        return

    distances_m = (
        distances / 1000.0
    )

    valid = (
        np.isfinite(angles)
        &
        np.isfinite(distances_m)
        &
        (
            distances_m
            > MIN_DISTANCE_METERS
        )
        &
        (
            distances_m
            <= MAX_DISTANCE_METERS
        )
    )

    valid_angles = angles[valid]
    valid_distances = distances_m[valid]

    # ========================================================
    # LOCAL LiDAR
    # ========================================================

    local_x, local_y = lidar_to_local(
        valid_angles,
        valid_distances * 1000
    )

    current_points = np.column_stack(
        [
            local_y,
            local_x
        ]
    )

    local_scan_points.set_offsets(
        current_points
    )

    segments_xy = build_segments(
        valid_angles,
        valid_distances * 1000
    )

    # build_segments returns:
    # [local_x, local_y]
    #
    # But the plot has:
    # horizontal = Y
    # vertical   = X
    #
    # Therefore swap them.

    segments_for_plot = [
        [
            [p1[1], p1[0]],
            [p2[1], p2[0]]
        ]
        for p1, p2 in segments_xy
    ]

    local_environment.set_segments(
        segments_for_plot
    )

    local_environment_glow.set_segments(
        segments_for_plot
    )

    # ========================================================
    # GLOBAL PATH
    # ========================================================

    update_robot_path(
        cx,
        cy
    )

    robot_path.set_data(
        robot_path_x,
        robot_path_y
    )

    # ========================================================
    # GLOBAL OCCUPANCY MAP
    # ========================================================

    if (
        current_scan_counter
        != last_mapped_scan
    ):

        last_mapped_scan = (
            current_scan_counter
        )

        if len(local_x) > MAX_SCAN_POINTS:

            indices = np.linspace(
                0,
                len(local_x) - 1,
                MAX_SCAN_POINTS
            ).astype(int)

            mx = local_x[indices]
            my = local_y[indices]

        else:

            mx = local_x
            my = local_y

        update_occupancy_map(
            mx,
            my,
            cx,
            cy,
            ch
        )

        update_map_display()

    # ========================================================
    # GLOBAL ROBOT
    # ========================================================

    global_robot_body.set_center(
        (cx, cy)
    )

    # Global heading arrow
    #
    # This SHOULD rotate because this is the
    # global frame.

    rad = np.radians(ch)

    arrow_len = 1.2

    end_x = (
        cx
        +
        arrow_len * np.cos(rad)
    )

    end_y = (
        cy
        +
        arrow_len * np.sin(rad)
    )

    global_heading_line.set_data(
        [cx, end_x],
        [cy, end_y]
    )

    # Global X reference
    global_reference_x_line.set_data(
        [cx, cx + 1.0],
        [cy, cy]
    )

    theta_label.set_position(
        (
            cx + 0.3,
            cy + 0.3
        )
    )

    theta_label.set_text(
        f"θ = {ch:.1f}°"
    )

    # ========================================================
    # LOCAL ROBOT
    # ========================================================

    robot_marker.set_data(
        [0],
        [0]
    )

    # IMPORTANT:
    #
    # LOCAL FRAME DOES NOT USE GLOBAL HEADING.
    #
    # Robot is ALWAYS facing +X locally.
    #
    # Since the plot has:
    #   horizontal = local Y
    #   vertical   = local X
    #
    # forward is simply:
    #
    #   horizontal = 0
    #   vertical   = +0.5

    robot_direction.set_data(
        [0, 0],
        [0, 0.5]
    )

    # ========================================================
    # STATUS BAR
    # ========================================================

    status_text.set_text(
        f"● POSE: "
        f"X_G={cx:.2f}m | "
        f"Y_G={cy:.2f}m | "
        f"θ={ch:.1f}°   "
        f"| TELEOP: "
        f"Fwd={fwd:+.1f} "
        f"Turn={trn:+.1f} "
        f"| [R] Reset Frame & NavX"
    )


# ============================================================
# MAIN
# ============================================================

def main():

    global control_table

    global local_environment
    global local_environment_glow
    global local_scan_points

    global robot_marker
    global robot_direction
    global robot_path

    global global_robot_body
    global global_heading_line
    global global_reference_x_line
    global theta_label

    global map_image
    global status_text
    global log_display

    # --------------------------------------------------------
    # NetworkTables
    # --------------------------------------------------------

    print(
        f"Connecting to NetworkTables at {ROBOT_IP}..."
    )

    NetworkTables.initialize(
        server=ROBOT_IP
    )

    NetworkTables.setUpdateRate(
        0.010
    )

    lidar_table = NetworkTables.getTable(
        "Lidar"
    )

    pose_table = NetworkTables.getTable(
        POSE_TABLE_NAME
    )

    sd_table = NetworkTables.getTable(
        "SmartDashboard"
    )

    control_table = NetworkTables.getTable(
        CONTROL_TABLE_NAME
    )

    lidar_table.addEntryListener(
        value_changed_callback
    )

    pose_table.addEntryListener(
        value_changed_callback
    )

    for key_name in LOG_KEYS:

        sd_table.addEntryListener(
            log_callback,
            key=key_name
        )

    # --------------------------------------------------------
    # Figure
    # --------------------------------------------------------

    plt.style.use(
        "dark_background"
    )

    fig = plt.figure(
        figsize=(16, 9.5),
        facecolor="#080b0e"
    )

    fig.canvas.mpl_connect(
        "key_press_event",
        on_key_press
    )

    fig.canvas.mpl_connect(
        "key_release_event",
        on_key_release
    )

    gs = fig.add_gridspec(
        2,
        2,
        height_ratios=[3.2, 1.0],
        hspace=0.25,
        wspace=0.18
    )

    # ========================================================
    # GLOBAL MAP
    # ========================================================

    ax_map = fig.add_subplot(
        gs[0, 0]
    )

    ax_map.set_title(
        "GLOBAL FRAME (X_G , Y_G)",
        color="#00E5FF",
        fontsize=12,
        fontweight="bold",
        pad=10
    )

    ax_map.set_facecolor(
        "#04070a"
    )

    ax_map.set_aspect(
        "equal"
    )

    ax_map.set_xlim(
        -MAP_SIZE_METERS / 2,
        MAP_SIZE_METERS / 2
    )

    ax_map.set_ylim(
        -MAP_SIZE_METERS / 2,
        MAP_SIZE_METERS / 2
    )

    ax_map.set_xlabel(
        "X_G Axis (meters)",
        color="#80A0C0",
        fontweight="bold"
    )

    ax_map.set_ylabel(
        "Y_G Axis (meters)",
        color="#80A0C0",
        fontweight="bold"
    )

    ax_map.tick_params(
        colors="#507090"
    )

    ax_map.grid(
        color="#122535",
        linestyle="--",
        alpha=0.6
    )

    ax_map.axhline(
        0,
        color="#1E3A52",
        linewidth=1.2,
        linestyle=":"
    )

    ax_map.axvline(
        0,
        color="#1E3A52",
        linewidth=1.2,
        linestyle=":"
    )

    cyan_cmap = (
        LinearSegmentedColormap.from_list(
            "dark_cyan",
            [
                "#04070a",
                "#00E5FF"
            ]
        )
    )

    map_image = ax_map.imshow(
        np.zeros_like(
            occupancy_grid
        ),
        origin="lower",
        extent=[
            -MAP_SIZE_METERS / 2,
            MAP_SIZE_METERS / 2,
            -MAP_SIZE_METERS / 2,
            MAP_SIZE_METERS / 2
        ],
        cmap=cyan_cmap,
        vmin=0,
        vmax=1,
        alpha=0.65,
        interpolation="nearest"
    )

    # Path
    (
        robot_path,
    ) = ax_map.plot(
        [],
        [],
        color="#00E5FF",
        linewidth=1.5,
        alpha=0.75,
        label="Global Path"
    )

    # Robot
    global_robot_body = Circle(
        (0, 0),
        0.45,
        facecolor="#00E5FF",
        edgecolor="#FFFFFF",
        alpha=0.85,
        zorder=25
    )

    ax_map.add_patch(
        global_robot_body
    )

    # Heading
    (
        global_heading_line,
    ) = ax_map.plot(
        [],
        [],
        color="#FFEA00",
        linewidth=2.5,
        zorder=30,
        label="Heading (θ)"
    )

    # X-axis reference
    (
        global_reference_x_line,
    ) = ax_map.plot(
        [],
        [],
        color="#FFFFFF",
        linewidth=1.0,
        linestyle="--",
        zorder=26
    )

    theta_label = ax_map.text(
        0,
        0,
        "θ = 0.0°",
        color="#FFEA00",
        fontsize=10,
        fontweight="bold",
        zorder=35
    )

    # ========================================================
    # LOCAL LiDAR
    # ========================================================

    ax_local = fig.add_subplot(
        gs[0, 1]
    )

    ax_local.set_title(
        "LOCAL LiDAR FRAME",
        color="#00E5FF",
        fontsize=12,
        fontweight="bold",
        pad=10
    )

    ax_local.set_facecolor(
        "#04070a"
    )

    ax_local.set_aspect(
        "equal"
    )

    ax_local.set_xlim(
        -MAX_DISTANCE_METERS,
        MAX_DISTANCE_METERS
    )

    ax_local.set_ylim(
        -MAX_DISTANCE_METERS,
        MAX_DISTANCE_METERS
    )

    # IMPORTANT:
    #
    # Horizontal = local Y
    # Vertical   = local X

    ax_local.set_xlabel(
        "+Y (Right) / -Y (Left) [m]",
        color="#80A0C0"
    )

    ax_local.set_ylabel(
        "+X (Forward) / -X (Back) [m]",
        color="#80A0C0"
    )

    ax_local.tick_params(
        colors="#507090"
    )

    ax_local.grid(
        color="#122535",
        linestyle="--",
        alpha=0.6
    )

    local_environment_glow = (
        LineCollection(
            [],
            linewidths=6,
            color="#00E5FF",
            alpha=0.08,
            capstyle="round"
        )
    )

    ax_local.add_collection(
        local_environment_glow
    )

    local_environment = (
        LineCollection(
            [],
            linewidths=1.8,
            color="#00E5FF",
            alpha=0.9,
            capstyle="round"
        )
    )

    ax_local.add_collection(
        local_environment
    )

    local_scan_points = ax_local.scatter(
        [],
        [],
        s=5,
        color="#A5F3FC",
        alpha=0.5,
        edgecolors="none"
    )

    # Robot
    (
        robot_marker,
    ) = ax_local.plot(
        [0],
        [0],
        marker="o",
        markersize=8,
        color="#00E5FF",
        markeredgecolor="white"
    )

    # Local forward direction
    (
        robot_direction,
    ) = ax_local.plot(
        [0, 0],
        [0, 0.5],
        color="#FFEA00",
        linewidth=2
    )

    # Range rings
    for r in range(1, 6):

        ax_local.add_patch(
            Circle(
                (0, 0),
                r,
                fill=False,
                edgecolor="#122535",
                linewidth=0.8,
                linestyle="--"
            )
        )

        ax_local.text(
            0.05,
            r,
            f"{r}m",
            color="#3A5D7C",
            fontsize=7
        )

    # ========================================================
    # CONSOLE
    # ========================================================

    ax_log = fig.add_subplot(
        gs[1, :]
    )

    ax_log.set_title(
        "SYSTEM CONSOLE LOGS",
        color="#00E5FF",
        fontsize=10,
        fontweight="bold",
        loc="left"
    )

    ax_log.set_facecolor(
        "#020305"
    )

    ax_log.tick_params(
        left=False,
        bottom=False,
        labelleft=False,
        labelbottom=False
    )

    log_display = ax_log.text(
        0.01,
        0.85,
        "Initializing NetworkTables listener...",
        color="#00E5FF",
        fontsize=8.5,
        family="monospace",
        verticalalignment="top"
    )

    # ========================================================
    # STATUS BAR
    # ========================================================

    status_text = fig.text(
        0.02,
        0.015,
        "● LIVE",
        color="#00E5FF",
        fontsize=9.5,
        fontweight="bold"
    )

    # ========================================================
    # ANIMATION
    # ========================================================

    ani = FuncAnimation(
        fig,
        update_plot,
        interval=50,
        blit=False,
        cache_frame_data=False
    )

    plt.tight_layout(
        rect=[
            0,
            0.03,
            1,
            0.98
        ]
    )

    plt.show()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    main()