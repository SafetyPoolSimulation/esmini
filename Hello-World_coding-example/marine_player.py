"""
Marine ASV Simulation - Option 2: LOS Guidance
Sparse waypoints + Fossen/Breivik Line-of-Sight cross-track error controller.
Fossen 3-DOF horizontal-plane physics (surge + yaw inertia) injected into
esmini via ExternalController and SE_ReportObjectPos.

Run from this directory:
    python marine_player.py
"""

import ctypes
import sys
import os
import math
import csv
import time
import xml.etree.ElementTree as ET

# ---------------------------------------------------------------------------
# 1. Load esminiLib
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_bin_dir    = os.path.join(_script_dir, "..", "bin")

if sys.platform == "win32":
    _lib_path = os.path.join(_bin_dir, "esminiLib.dll")
elif sys.platform == "darwin":
    _lib_path = os.path.join(_bin_dir, "libesminiLib.dylib")
else:
    _lib_path = os.path.join(_bin_dir, "libesminiLib.so")

if not os.path.exists(_lib_path):
    sys.exit(f"ERROR: esminiLib not found at: {_lib_path}")

se = ctypes.CDLL(_lib_path)

# ---------------------------------------------------------------------------
# 2. SE_ScenarioObjectState struct  (must match esminiLib.hpp exactly)
# ---------------------------------------------------------------------------
class SE_ScenarioObjectState(ctypes.Structure):
    _fields_ = [
        ("id",            ctypes.c_int),
        ("model_id",      ctypes.c_int),
        ("ctrl_type",     ctypes.c_int),
        ("timestamp",     ctypes.c_float),
        ("x",             ctypes.c_float),
        ("y",             ctypes.c_float),
        ("z",             ctypes.c_float),
        ("h",             ctypes.c_float),
        ("p",             ctypes.c_float),
        ("r",             ctypes.c_float),
        ("roadId",        ctypes.c_uint32),
        ("junctionId",    ctypes.c_uint32),
        ("t",             ctypes.c_float),
        ("laneId",        ctypes.c_int),
        ("laneOffset",    ctypes.c_float),
        ("s",             ctypes.c_float),
        ("speed",         ctypes.c_float),
        ("centerOffsetX", ctypes.c_float),
        ("centerOffsetY", ctypes.c_float),
        ("centerOffsetZ", ctypes.c_float),
        ("width",         ctypes.c_float),
        ("length",        ctypes.c_float),
        ("height",        ctypes.c_float),
        ("objectType",    ctypes.c_int),
        ("objectCategory",ctypes.c_int),
        ("wheel_angle",   ctypes.c_float),
        ("wheel_rot",     ctypes.c_float),
        ("visibilityMask",ctypes.c_int),
    ]

# ---------------------------------------------------------------------------
# 3. Declare argument / return types for every API function used
# ---------------------------------------------------------------------------
se.SE_InitWithArgs.argtypes  = [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
se.SE_InitWithArgs.restype   = ctypes.c_int

se.SE_StepDT.argtypes        = [ctypes.c_double]
se.SE_StepDT.restype         = ctypes.c_int

se.SE_GetQuitFlag.argtypes   = []
se.SE_GetQuitFlag.restype    = ctypes.c_int

se.SE_GetObjectState.argtypes = [ctypes.c_int, ctypes.POINTER(SE_ScenarioObjectState)]
se.SE_GetObjectState.restype  = ctypes.c_int

# SE_ReportObjectPos(object_id, timestamp, x, y, z, h, p, r)
se.SE_ReportObjectPos.argtypes = [
    ctypes.c_int,
    ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
]
se.SE_ReportObjectPos.restype  = ctypes.c_int

# --- Bounding-box structs (must match esminiLib.hpp) for scaled markers ---
class SE_Center(ctypes.Structure):
    _fields_ = [("x_", ctypes.c_float), ("y_", ctypes.c_float), ("z_", ctypes.c_float)]

class SE_Dimensions(ctypes.Structure):
    _fields_ = [("width_", ctypes.c_float), ("length_", ctypes.c_float), ("height_", ctypes.c_float)]

class SE_OSCBoundingBox(ctypes.Structure):
    _fields_ = [("center_", SE_Center), ("dimensions_", SE_Dimensions)]

# SE_AddObjectWithBoundingBox(name, type, category, role, model_id, model_3d, bb, scale_mode)
se.SE_AddObjectWithBoundingBox.argtypes = [
    ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
    ctypes.c_char_p, SE_OSCBoundingBox, ctypes.c_int,
]
se.SE_AddObjectWithBoundingBox.restype = ctypes.c_int

se.SE_Close.argtypes  = []
se.SE_Close.restype   = None

# ---------------------------------------------------------------------------
# 4. Fossen 3-DOF horizontal-plane physics (surge, sway, yaw -- coupled)
# ---------------------------------------------------------------------------
class MarinePhysics:
    # --- Rigid-body + added mass (15 m ASV, ~5 000 kg) ---
    MASS            = 5_000.0    # rigid-body hull mass         [kg]
    ADDED_SURGE     = 1_000.0    # surge added mass  (-Xu_dot)   [kg]
    ADDED_SWAY      = 2_000.0    # sway added mass   (-Yv_dot)   [kg]  (larger than surge for slender hulls)
    IZ              = 80_000.0   # rigid-body yaw inertia        [kg*m²]
    ADDED_YAW       = 20_000.0   # yaw added inertia (-Nr_dot)   [kg*m²]

    # --- Damping: linear + quadratic (non-linear drag dominates at speed) ---
    DAMP_SURGE_LIN  = 200.0      # [N*s/m]
    DAMP_SURGE_QUAD = 40.0       # [N*s²/m²]
    # Sway damping is much stiffer than surge: a hull's broadside profile area
    # (resisting sideways motion) is far larger than its bow-on profile area,
    # so lateral drag must be strong enough to counteract the Coriolis/
    # centripetal coupling force (m_u*u*r) induced during turns, or sway
    # velocity blows up and the vessel spirals instead of converging.
    DAMP_SWAY_LIN   = 12_000.0   # [N*s/m]
    DAMP_SWAY_QUAD  = 3_000.0    # [N*s²/m²]
    DAMP_YAW_LIN    = 5_000.0    # [N*m*s/rad]
    DAMP_YAW_QUAD   = 2_000.0    # [N*m*s²/rad²]

    K_THRUST        = 5_000.0    # P-gain: speed error -> surge force
    K_RUDDER        = 50_000.0   # P-gain: yaw-rate error -> yaw torque

    # --- Environmental disturbance: constant earth-fixed ocean current ---
    CURRENT_SPEED   = 0.3        # [m/s]
    CURRENT_DIR     = math.radians(200.0)  # earth-fixed bearing current flows toward

    def __init__(self):
        self.global_x = 0.0
        self.global_y = 0.0
        self.global_h = 0.0
        self.u = 0.0   # surge  [m/s] (relative to water)
        self.v = 0.0   # sway   [m/s] (relative to water)
        self.r = 0.0   # yaw rate [rad/s]

    def update(self, target_speed: float, r_cmd: float, dt: float):
        m_u = self.MASS + self.ADDED_SURGE
        m_v = self.MASS + self.ADDED_SWAY
        m_r = self.IZ + self.ADDED_YAW

        tau_u = (target_speed - self.u) * self.K_THRUST   # surge thrust (speed P-control)
        tau_r = (r_cmd - self.r) * self.K_RUDDER           # yaw torque (rudder P-control)
        # no lateral thruster -- sway only arises from Coriolis/centripetal coupling below

        d_u = self.DAMP_SURGE_LIN * self.u + self.DAMP_SURGE_QUAD * abs(self.u) * self.u
        d_v = self.DAMP_SWAY_LIN  * self.v + self.DAMP_SWAY_QUAD  * abs(self.v) * self.v
        d_r = self.DAMP_YAW_LIN   * self.r + self.DAMP_YAW_QUAD   * abs(self.r) * self.r

        # -- Coupled 3-DOF kinetics: M*nu_dot + C(nu)*nu + D(nu)*nu = tau --
        u_dot = (tau_u - d_u + m_v * self.v * self.r) / m_u
        v_dot = (     0 - d_v - m_u * self.u * self.r) / m_v
        r_dot = (tau_r - d_r + (m_u - m_v) * self.u * self.v) / m_r

        self.u += u_dot * dt
        self.v += v_dot * dt
        self.r += r_dot * dt

        # -- Kinematics: body -> earth  J(psi)*nu, plus earth-fixed current drift --
        current_x = self.CURRENT_SPEED * math.cos(self.CURRENT_DIR)
        current_y = self.CURRENT_SPEED * math.sin(self.CURRENT_DIR)
        self.global_x += (self.u * math.cos(self.global_h)
                         - self.v * math.sin(self.global_h) + current_x) * dt
        self.global_y += (self.u * math.sin(self.global_h)
                         + self.v * math.cos(self.global_h) + current_y) * dt
        self.global_h += self.r * dt


# ---------------------------------------------------------------------------
# 5. Line-of-Sight (LOS) Guidance  --  Fossen/Breivik (2005)
#
#   Path frame between consecutive waypoints k -> k+1:
#     alpha_k  = atan2(Deltay, Deltax)          path tangent angle
#     e    = cross-track error (signed, positive = port of path)
#     psi_d  = alpha_k − atan(e / Delta)      desired heading  (Delta = lookahead)
#
#   Waypoint switch: advance when vessel is within ACCEPTANCE_RADIUS of
#   the current target waypoint.
# ---------------------------------------------------------------------------
class LOSGuidance:
    LOOKAHEAD_DIST   = 20.0   # Delta  [m]  -- typically 1–5 vessel lengths
    ACCEPTANCE_RADIUS = 8.0   # switch radius [m]

    def __init__(self, waypoints: list):
        """waypoints: list of (x, y) tuples in earth-fixed frame."""
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints.")
        self.waypoints   = waypoints
        self.target_idx  = 1          # index of the current target wp
        self.reached_end = False
        self.last_xte    = 0.0        # most recent signed cross-track error [m]

    def compute_desired_heading(self, x: float, y: float) -> float | None:
        """Return psi_d [rad], or None when all waypoints are reached."""
        if self.reached_end:
            return None

        # -- Waypoint switch --
        xw, yw = self.waypoints[self.target_idx]
        if math.hypot(xw - x, yw - y) < self.ACCEPTANCE_RADIUS:
            self.target_idx += 1
            if self.target_idx >= len(self.waypoints):
                self.reached_end = True
                return None

        # -- Active segment: k -> k+1 --
        x_k,  y_k  = self.waypoints[self.target_idx - 1]
        x_k1, y_k1 = self.waypoints[self.target_idx]

        alpha = math.atan2(y_k1 - y_k, x_k1 - x_k)   # path tangent

        # Cross-track error (positive = vessel is to port of path)
        dx = x - x_k
        dy = y - y_k
        e  = -dx * math.sin(alpha) + dy * math.cos(alpha)
        self.last_xte = e   # stashed for telemetry/logging

        # LOS desired heading
        psi_d = alpha - math.atan2(e, self.LOOKAHEAD_DIST)
        return psi_d

    @staticmethod
    def wrap_angle(angle: float) -> float:
        """Wrap angle to (−π, π]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def read_commanded_speed(osc_path: str, default: float = 5.0) -> float:
    """Read the Ego's AbsoluteTargetSpeed from the OSC file so the scenario's
    logical SpeedAction intent actually drives the physics, instead of a
    hardcoded duplicate value."""
    try:
        root = ET.parse(osc_path).getroot()
        elem = root.find(".//AbsoluteTargetSpeed")
        if elem is not None and "value" in elem.attrib:
            return float(elem.attrib["value"])
    except (ET.ParseError, OSError, ValueError):
        pass
    return default


def build_srpt_reference(waypoints: list, speed: float, duration: float, dt: float) -> list:
    """Naive 'Streaming Rigid Position Trajectory' baseline: constant-speed,
    open-loop interpolation along straight segments between waypoints, with
    no dynamics and no feedback -- the first-draft approach being compared
    against."""
    seg_lengths = [
        math.hypot(waypoints[i + 1][0] - waypoints[i][0], waypoints[i + 1][1] - waypoints[i][1])
        for i in range(len(waypoints) - 1)
    ]
    total_dist = sum(seg_lengths)
    n_steps    = int(duration / dt)
    ref_path   = []
    traveled   = 0.0
    seg_idx    = 0
    for i in range(n_steps):
        t = i * dt
        if traveled >= total_dist:
            ref_path.append(waypoints[-1])
            continue
        while seg_idx < len(seg_lengths) - 1 and traveled > sum(seg_lengths[: seg_idx + 1]):
            seg_idx += 1
        seg_start_dist = sum(seg_lengths[:seg_idx])
        x_k,  y_k  = waypoints[seg_idx]
        x_k1, y_k1 = waypoints[seg_idx + 1]
        frac = 0.0 if seg_lengths[seg_idx] == 0 else (traveled - seg_start_dist) / seg_lengths[seg_idx]
        frac = max(0.0, min(1.0, frac))
        ref_path.append((x_k + frac * (x_k1 - x_k), y_k + frac * (y_k1 - y_k)))
        traveled += speed * dt
    return ref_path


def export_results(log_rows: list, waypoints: list, commanded_speed: float, fixed_dt: float, out_dir: str):
    """Write telemetry.csv and a trajectory_comparison.png contrasting the
    emergent Fossen/LOS path against the naive rigid streamed-trajectory baseline."""
    csv_path = os.path.join(out_dir, "telemetry.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "x", "y", "heading_deg", "u_mps", "v_mps", "r_dps", "cross_track_error_m"])
        writer.writerows(log_rows)
    print(f"Telemetry written to {csv_path}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping comparison plot.")
        return

    duration = log_rows[-1][0] if log_rows else 0.0
    srpt_path = build_srpt_reference(waypoints, commanded_speed, duration, fixed_dt)

    fig, (ax_path, ax_xte) = plt.subplots(1, 2, figsize=(12, 5))

    wp_x, wp_y = zip(*waypoints)
    ax_path.plot(wp_x, wp_y, "ko--", label="Waypoints", markersize=5)
    ax_path.plot(*zip(*srpt_path), label="SRPT (rigid, open-loop)", linestyle=":", color="gray")
    emergent_x = [row[1] for row in log_rows]
    emergent_y = [row[2] for row in log_rows]
    ax_path.plot(emergent_x, emergent_y, label="Fossen + LOS (emergent, closed-loop)", color="tab:blue")
    ax_path.set_xlabel("X [m]")
    ax_path.set_ylabel("Y [m]")
    ax_path.set_title("Path comparison")
    ax_path.axis("equal")
    ax_path.legend()
    ax_path.grid(True)

    times = [row[0] for row in log_rows]
    xtes  = [row[7] for row in log_rows]
    ax_xte.plot(times, xtes, color="tab:red")
    ax_xte.axhline(0.0, color="black", linewidth=0.8)
    ax_xte.set_xlabel("Time [s]")
    ax_xte.set_ylabel("Cross-track error [m]")
    ax_xte.set_title("LOS cross-track error over time")
    ax_xte.grid(True)

    fig.tight_layout()
    png_path = os.path.join(out_dir, "trajectory_comparison.png")
    fig.savefig(png_path, dpi=150)
    print(f"Comparison plot written to {png_path}")


def _make_bb(width: float, length: float, height: float) -> SE_OSCBoundingBox:
    """Bounding box centred vertically so the scaled marker sits on the water."""
    return SE_OSCBoundingBox(SE_Center(0.0, 0.0, height / 2.0),
                             SE_Dimensions(width, length, height))


def add_path_markers(waypoints: list, dot_spacing: float = 12.0) -> int:
    """Draw the *intended* path in the 3D viewer so it can be compared against
    esmini's live trail of the *actual* GNC path:
      - a tall cone at every waypoint (the points the boat must hit)
      - small cones evenly spaced along the straight segments (the intended lane)
    Markers are static MiscObjects, which only need to be reported once. Cones
    are scaled up via a bounding box to stay visible across the ~300 m field."""
    wp_bb  = _make_bb(6.0, 6.0, 12.0)
    dot_bb = _make_bb(2.5, 2.5, 4.0)
    count  = 0

    # Tall cone at each waypoint
    for i, (x, y) in enumerate(waypoints):
        oid = se.SE_AddObjectWithBoundingBox(
            f"waypoint_{i}".encode("ascii"), 3, 0, 0, -1, b"cone-100.osgb", wp_bb, 2)
        if oid >= 0:
            se.SE_ReportObjectPos(oid, 0.0, float(x), float(y), 0.0, 0.0, 0.0, 0.0)
            count += 1

    # Small cones along the straight segment between consecutive waypoints
    for (x0, y0), (x1, y1) in zip(waypoints[:-1], waypoints[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        n   = max(1, int(round(seg / dot_spacing)))
        for k in range(1, n):  # skip endpoints (already marked by waypoint cones)
            frac = k / n
            xd   = x0 + frac * (x1 - x0)
            yd   = y0 + frac * (y1 - y0)
            oid  = se.SE_AddObjectWithBoundingBox(
                f"lane_dot_{count}".encode("ascii"), 3, 0, 0, -1, b"cone-45.osgb", dot_bb, 2)
            if oid >= 0:
                se.SE_ReportObjectPos(oid, 0.0, float(xd), float(yd), 0.0, 0.0, 0.0, 0.0)
                count += 1

    return count

# ---------------------------------------------------------------------------
# 6. Main simulation
# ---------------------------------------------------------------------------
def main():
    # --- Run mode ---
    #   default          : headless, runs as fast as possible, exports CSV + plot
    #   --live / --demo   : opens the esmini 3D viewer and paces to wall-clock
    #                       time so the GNC run is watchable in real time
    live_view = any(flag in sys.argv for flag in ("--live", "--demo"))

    # Build argv for SE_InitWithArgs
    scenario_path = os.path.join(_script_dir, "scenario.xosc")
    argv_strs  = [b"marine_sim", b"--osc", scenario_path.encode("ascii")]
    if live_view:
        # open a 3D viewer window (x, y, width, height) so the boat is visible,
        # and enable entity trails (mode 3 = dots + line) to trace the actual path
        argv_strs += [b"--window", b"60", b"60", b"1200", b"800", b"--trail_mode", b"3"]
    argc       = len(argv_strs)
    argv_type  = ctypes.c_char_p * argc
    argv       = argv_type(*argv_strs)

    if se.SE_InitWithArgs(argc, argv) != 0:
        sys.exit("ERROR: Failed to initialize esmini.")

    if live_view:
        print("esmini initialized -- LIVE demo mode (real-time 3D viewer).")
    else:
        print("esmini initialized -- headless analysis mode (pass --live for the 3D viewer).")

    # --- Simulation constants ---
    EGO_ID          = 0
    FIXED_DT        = 0.016        # 60 Hz deterministic step
    SIM_DURATION    = 90.0         # enough for the full waypoint path
    TOTAL_STEPS     = int(SIM_DURATION / FIXED_DT)
    COMMANDED_SPEED = read_commanded_speed(scenario_path)  # [m/s] -- from scenario SpeedAction

    # Heading PD-controller: heading error [rad] -> desired yaw rate [rad/s]
    KP_HEADING   = 1.2             # tuned for 15 m ASV at 5 m/s
    KD_HEADING   = 0.4             # damps overshoot on turns
    MAX_YAW_RATE = 0.3             # [rad/s]  ≈ 17 deg/s

    # --- Sparse waypoints (earth-fixed, metres) ---
    # Path: depart east -> broad starboard turn -> run east -> broad port turn
    WAYPOINTS = [
        (  0.0,   0.0),
        ( 80.0,   0.0),
        (130.0,  50.0),
        (190.0,  50.0),
        (240.0,   0.0),
        (300.0,   0.0),
    ]

    vessel = MarinePhysics()
    los    = LOSGuidance(WAYPOINTS)
    prev_heading_error = 0.0

    # Seed physics position from scenario init
    state = SE_ScenarioObjectState()
    if se.SE_GetObjectState(EGO_ID, ctypes.byref(state)) == 0:
        vessel.global_x = state.x
        vessel.global_y = state.y
        vessel.global_h = state.h

    # In live mode, draw the intended path so it can be compared visually with
    # esmini's live trail of the actual GNC-driven path.
    if live_view:
        n_markers = add_path_markers(WAYPOINTS)
        print(f"Placed {n_markers} intended-path markers "
              f"(tall cones = waypoints, small cones = intended lane). "
              f"Boat trail = actual GNC path.")

    print(f"Waypoints: {WAYPOINTS}")
    print("-" * 70)

    log_rows = []  # (time, x, y, heading_deg, u, v, r_dps, cross_track_error)

    wall_start = time.perf_counter()  # wall-clock anchor for real-time pacing

    # --- Deterministic execution loop ---
    for i in range(TOTAL_STEPS):
        # Step A: Query esmini state (position confirmation)
        if se.SE_GetObjectState(EGO_ID, ctypes.byref(state)) != 0:
            break

        # Step B: LOS guidance -> desired heading psi_d
        psi_d = los.compute_desired_heading(vessel.global_x, vessel.global_y)
        if psi_d is None:
            print("All waypoints reached -- stopping.")
            break

        # Step C: Heading PD-controller -> desired yaw rate r_cmd
        heading_error = LOSGuidance.wrap_angle(psi_d - vessel.global_h)
        error_rate = LOSGuidance.wrap_angle(heading_error - prev_heading_error) / FIXED_DT
        prev_heading_error = heading_error
        r_cmd = KP_HEADING * heading_error + KD_HEADING * error_rate
        r_cmd = max(-MAX_YAW_RATE, min(MAX_YAW_RATE, r_cmd))  # rate saturation

        # Step D: Fossen 3-DOF physics update
        vessel.update(COMMANDED_SPEED, r_cmd, FIXED_DT)

        # Step E: Inject physics-driven state back into esmini
        se.SE_ReportObjectPos(
            EGO_ID, i * FIXED_DT,
            vessel.global_x, vessel.global_y, 0.0,
            vessel.global_h, 0.0, 0.0
        )

        # Step F: Advance scenario engine by fixed timestep
        se.SE_StepDT(FIXED_DT)

        if se.SE_GetQuitFlag():
            break

        # Step G: In live mode, pace the loop to wall-clock time so the viewer
        # plays at real speed instead of finishing in a fraction of a second.
        if live_view:
            target_wall = wall_start + (i + 1) * FIXED_DT
            lag = target_wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)

        log_rows.append((
            i * FIXED_DT, vessel.global_x, vessel.global_y,
            math.degrees(vessel.global_h), vessel.u, vessel.v,
            math.degrees(vessel.r), los.last_xte,
        ))

        # Telemetry -- print once per simulated second
        if i % 60 == 0:
            sim_time   = i * FIXED_DT
            wp_idx     = min(los.target_idx, len(WAYPOINTS) - 1)
            xw, yw     = WAYPOINTS[wp_idx]
            xte        = math.hypot(vessel.global_x - xw, vessel.global_y - yw)
            print(
                f"t={sim_time:5.1f}s | "
                f"u={vessel.u:4.2f}m/s v={vessel.v:+4.2f}m/s r={math.degrees(vessel.r):+5.1f}dps | "
                f"X={vessel.global_x:6.1f} Y={vessel.global_y:6.1f} "
                f"hdg={math.degrees(vessel.global_h):+6.1f}deg | "
                f"->WP{wp_idx}({xw:.0f},{yw:.0f}) dist={xte:.1f}m"
            )

    se.SE_Close()
    print("Simulation completed.")

    export_results(log_rows, WAYPOINTS, COMMANDED_SPEED, FIXED_DT, _script_dir)


if __name__ == "__main__":
    main()
