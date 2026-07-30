"""
Highway Agent Driver Model - Kinematic bicycle + pure-pursuit lane following.
A GNC-style external driver model applied to the *agent* vehicles (A1-A5) of
resources/xosc/highway_merge.xosc. Each agent follows its lane centreline via
esmini's road API (SE_GetRoadInfoAtDistance) with pure-pursuit lateral control
and a P longitudinal controller, plus small seeded noise/bias so the motion
looks natural and imperfect instead of esmini's exact ideal path.

State per agent is integrated by a kinematic bicycle model and injected back
into esmini every step via SE_ReportObjectPos. The Ego is left untouched -- it
stays under storyboard control (scripted lane change).

Run from this directory:
    python vehicle_agent_player.py          # headless, fast, exports CSV + plot
    python vehicle_agent_player.py --live    # 3D viewer, real-time pacing
"""

import ctypes
import sys
import os
import math
import csv
import random
import time

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
# 2. Structs  (must match esminiLib.hpp exactly)
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

class SE_RoadInfo(ctypes.Structure):
    _fields_ = [
        ("global_pos_x",     ctypes.c_float),
        ("global_pos_y",     ctypes.c_float),
        ("global_pos_z",     ctypes.c_float),
        ("local_pos_x",      ctypes.c_float),
        ("local_pos_y",      ctypes.c_float),
        ("local_pos_z",      ctypes.c_float),
        ("angle",            ctypes.c_float),   # heading to target, in vehicle frame
        ("road_heading",     ctypes.c_float),
        ("road_pitch",       ctypes.c_float),
        ("road_roll",        ctypes.c_float),
        ("trail_heading",    ctypes.c_float),
        ("curvature",        ctypes.c_float),
        ("speed_limit",      ctypes.c_float),
        ("roadId",           ctypes.c_uint32),
        ("junctionId",       ctypes.c_uint32),
        ("laneId",           ctypes.c_int),
        ("laneOffset",       ctypes.c_float),
        ("s",                ctypes.c_float),
        ("t",                ctypes.c_float),
        ("road_type",        ctypes.c_int),
        ("road_rule",        ctypes.c_int),
        ("lane_type",        ctypes.c_int),
        ("trail_wheel_angle",ctypes.c_float),
    ]

# ---------------------------------------------------------------------------
# 3. Declare argument / return types for every API function used
# ---------------------------------------------------------------------------
se.SE_InitWithArgs.argtypes  = [ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
se.SE_InitWithArgs.restype   = ctypes.c_int

se.SE_StepDT.argtypes        = [ctypes.c_float]   # C signature is float, not double
se.SE_StepDT.restype         = ctypes.c_int

se.SE_GetQuitFlag.argtypes   = []
se.SE_GetQuitFlag.restype    = ctypes.c_int

se.SE_GetIdByName.argtypes   = [ctypes.c_char_p]
se.SE_GetIdByName.restype    = ctypes.c_int

se.SE_GetObjectState.argtypes = [ctypes.c_int, ctypes.POINTER(SE_ScenarioObjectState)]
se.SE_GetObjectState.restype  = ctypes.c_int

# SE_GetRoadInfoAtDistance(object_id, lookahead, *data, lookAheadMode, inRoadDrivingDirection)
se.SE_GetRoadInfoAtDistance.argtypes = [
    ctypes.c_int, ctypes.c_float, ctypes.POINTER(SE_RoadInfo), ctypes.c_int, ctypes.c_bool,
]
se.SE_GetRoadInfoAtDistance.restype  = ctypes.c_int

# SE_ReportObjectPos(object_id, timestamp, x, y, z, h, p, r)
se.SE_ReportObjectPos.argtypes = [
    ctypes.c_int,
    ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
    ctypes.c_float, ctypes.c_float, ctypes.c_float,
]
se.SE_ReportObjectPos.restype  = ctypes.c_int

# SE_ReportObjectSpeed(object_id, speed) -- keeps the viewer's info text realistic
se.SE_ReportObjectSpeed.argtypes = [ctypes.c_int, ctypes.c_float]
se.SE_ReportObjectSpeed.restype  = ctypes.c_int

se.SE_Close.argtypes  = []
se.SE_Close.restype   = None

# ---------------------------------------------------------------------------
# 4. Tuning constants  (grouped up top so the model is easy to re-tune)
# ---------------------------------------------------------------------------
# --- Which entities to drive -------------------------------------------------
EGO_NAME    = "Ego"                          # left under storyboard control
AGENT_NAMES = ["A1", "A2", "A3", "A4", "A5"] # driven by this external model

# --- Kinematic bicycle model (generic passenger car) -------------------------
WHEELBASE   = 2.7                     # front-to-rear axle distance [m]
MAX_STEER   = math.radians(35.0)      # steering angle saturation   [rad]
MAX_ACCEL   = 3.0                     # throttle limit              [m/s²]
MAX_DECEL   = 6.0                     # braking limit               [m/s²]
MIN_SPEED   = 0.0                     # [m/s]
MAX_SPEED   = 60.0                    # [m/s]

# --- Pure-pursuit lateral controller -----------------------------------------
#   A longer look-ahead damps the geometric corner-cutting oscillation that
#   makes short-look-ahead pure pursuit weave; this keeps tracking smooth.
LOOKAHEAD_BASE = 8.0                  # base look-ahead distance    [m]
LOOKAHEAD_K    = 0.8                   # speed-scaled look-ahead gain [s]
LANE_CENTER_MODE = 0                  # SE_GetRoadInfoAtDistance: 0 = lane centre

# --- Longitudinal P controller -----------------------------------------------
KP_SPEED    = 0.6                     # speed error [m/s] -> accel [m/s²] (gentle)

# --- Natural imperfection: small, BOUNDED in-lane variation (stays legal) -----
#   Imperfection is applied as a gentle lateral-offset *target* inside the lane
#   (drivers never sit dead-centre), NOT as steering-angle noise -- a sustained
#   steering offset double-integrates (steer -> heading -> position) into large
#   lane-crossing sway. Pure pursuit tracks the offset target, so lateral
#   position stays clamped well inside the lane and the car never crosses a line.
RANDOM_SEED            = 20260730     # master seed -> per-agent sub-streams
LANE_OFFSET_BIAS_STD   = 0.25         # constant per-agent off-centre bias   [m]
LANE_OFFSET_WANDER_STD = 0.30         # slow in-lane drift amplitude         [m]
LANE_OFFSET_WANDER_TAU = 2.5          # in-lane drift correlation time       [s]
LANE_OFFSET_MAX        = 0.70         # hard clamp -> stays inside the lane  [m]
SPEED_BIAS_STD         = 1.0          # constant per-agent speed offset    [m/s]
SPEED_WANDER_STD       = 0.9          # slow target-speed wander           [m/s]
SPEED_WANDER_TAU       = 4.0          # speed-wander correlation time        [s]
STEER_SMOOTH_TAU       = 0.30         # steering actuator lag (low-pass)     [s]
STEER_RATE_LIMIT       = math.radians(40.0)  # max steering rate       [rad/s]

# --- Simulation timing -------------------------------------------------------
FIXED_DT     = 0.02                   # 50 Hz deterministic step [s]
SIM_DURATION = 20.0                   # [s]


# ---------------------------------------------------------------------------
# 5. Kinematic bicycle model  (rear-axle reference)
#
#   x_dot     = v * cos(psi)
#   y_dot     = v * sin(psi)
#   psi_dot   = v / L * tan(delta)          delta = front steering angle
#   v_dot     = a                            a = longitudinal acceleration
# ---------------------------------------------------------------------------
class BicycleModel:
    def __init__(self, x: float, y: float, yaw: float, speed: float):
        self.x     = x
        self.y     = y
        self.yaw   = yaw
        self.speed = speed

    def update(self, steer: float, accel: float, dt: float):
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        accel = max(-MAX_DECEL, min(MAX_ACCEL, accel))

        self.x   += self.speed * math.cos(self.yaw) * dt
        self.y   += self.speed * math.sin(self.yaw) * dt
        self.yaw += self.speed / WHEELBASE * math.tan(steer) * dt
        self.speed = max(MIN_SPEED, min(MAX_SPEED, self.speed + accel * dt))


# ---------------------------------------------------------------------------
# 6. Ornstein-Uhlenbeck process -- smooth, mean-reverting coloured noise
#
#   dx = -(x / tau) dt + sigma * sqrt(2/tau) dW
#   Stationary std = sigma, correlation time = tau. Unlike white noise it
#   varies slowly, so it reads as gentle drift rather than high-frequency shake.
# ---------------------------------------------------------------------------
class OrnsteinUhlenbeck:
    def __init__(self, tau: float, sigma: float, rng: random.Random):
        self.tau   = tau
        self.sigma = sigma
        self.rng   = rng
        self.value = 0.0

    def sample(self, dt: float) -> float:
        self.value += (-self.value / self.tau * dt
                       + self.sigma * math.sqrt(2.0 * dt / self.tau) * self.rng.gauss(0.0, 1.0))
        return self.value


# ---------------------------------------------------------------------------
# 7. Per-agent driver: pure-pursuit + P-speed + smooth seeded imperfection
# ---------------------------------------------------------------------------
class AgentDriver:
    def __init__(self, name: str, obj_id: int, model: BicycleModel,
                 target_speed: float, rng: random.Random):
        self.name         = name
        self.obj_id       = obj_id
        self.model        = model
        # Constant per-agent biases -> each car has its own steady "personality"
        self.lane_bias    = rng.gauss(0.0, LANE_OFFSET_BIAS_STD)   # off-centre preference
        self.target_speed = max(0.0, target_speed + rng.gauss(0.0, SPEED_BIAS_STD))
        # Slow, correlated wander sources (smooth, not per-step white noise)
        self.lane_wander  = OrnsteinUhlenbeck(LANE_OFFSET_WANDER_TAU, LANE_OFFSET_WANDER_STD, rng)
        self.speed_wander = OrnsteinUhlenbeck(SPEED_WANDER_TAU, SPEED_WANDER_STD, rng)
        self.steer        = 0.0   # filtered steering state (actuator output)
        self.last_steer   = 0.0

    def step(self, dt: float, timestamp: float):
        """Compute controls from the road look-ahead, integrate, and report."""
        ld = LOOKAHEAD_BASE + LOOKAHEAD_K * self.model.speed

        # --- Lateral: pure pursuit toward a slightly-offset in-lane target ---
        info = SE_RoadInfo()
        rc = se.SE_GetRoadInfoAtDistance(
            self.obj_id, ctypes.c_float(ld), ctypes.byref(info),
            LANE_CENTER_MODE, True)
        if rc >= 0:
            # Bounded lateral offset within the lane (never crosses the line)
            off = self.lane_bias + self.lane_wander.sample(dt)
            off = max(-LANE_OFFSET_MAX, min(LANE_OFFSET_MAX, off))
            # Shift the lane-centre look-ahead point sideways by `off`, then aim
            perp = info.road_heading + math.pi / 2.0
            tx = info.global_pos_x + off * math.cos(perp)
            ty = info.global_pos_y + off * math.sin(perp)
            alpha = math.atan2(ty - self.model.y, tx - self.model.x) - self.model.yaw
            alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi   # wrap to (-pi, pi]
            steer_cmd = math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)
        else:
            # Off road / end of road: hold current wheel angle
            steer_cmd = self.steer

        # --- Steering actuator: first-order lag + rate limit -> smooth wheel --
        lp    = dt / (STEER_SMOOTH_TAU + dt)
        steer_target = self.steer + lp * (steer_cmd - self.steer)
        max_delta    = STEER_RATE_LIMIT * dt
        steer_target = max(self.steer - max_delta, min(self.steer + max_delta, steer_target))
        self.steer   = steer_target

        # --- Longitudinal: P-controller holding the slowly-wandering target ---
        target_speed = self.target_speed + self.speed_wander.sample(dt)
        accel = KP_SPEED * (target_speed - self.model.speed)

        self.model.update(self.steer, accel, dt)
        self.last_steer = self.steer

        se.SE_ReportObjectPos(
            self.obj_id, ctypes.c_float(timestamp),
            self.model.x, self.model.y, 0.0,
            self.model.yaw, 0.0, 0.0)
        se.SE_ReportObjectSpeed(self.obj_id, self.model.speed)


# ---------------------------------------------------------------------------
# 8. Telemetry export: per-agent CSV + trajectory plot
# ---------------------------------------------------------------------------
def export_results(log_rows: list, ego_path: list, out_dir: str):
    """Write vehicle_telemetry.csv (one row per agent per step) and a
    vehicle_trajectories.png overlaying every agent's driven path."""
    csv_path = os.path.join(out_dir, "vehicle_telemetry.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s", "agent", "x", "y", "heading_deg",
                         "speed_mps", "steer_deg", "target_speed_mps"])
        writer.writerows(log_rows)
    print(f"Telemetry written to {csv_path}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available -- skipping trajectory plot.")
        return

    # Group rows by agent for per-track plotting
    tracks: dict[str, tuple[list, list]] = {}
    for t_s, agent, x, y, *_ in log_rows:
        xs, ys = tracks.setdefault(agent, ([], []))
        xs.append(x)
        ys.append(y)

    fig, ax = plt.subplots(figsize=(11, 7))
    for agent, (xs, ys) in sorted(tracks.items()):
        ax.plot(xs, ys, label=agent, linewidth=1.6)
        ax.plot(xs[0], ys[0], "o", markersize=5)   # start marker
    if ego_path:
        ex, ey = zip(*ego_path)
        ax.plot(ex, ey, "k--", linewidth=1.2, label="Ego (storyboard)")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Agent driver model -- lane-following trajectories")
    ax.axis("equal")
    ax.legend()
    ax.grid(True)
    fig.tight_layout()
    png_path = os.path.join(out_dir, "vehicle_trajectories.png")
    fig.savefig(png_path, dpi=150)
    print(f"Trajectory plot written to {png_path}")


# ---------------------------------------------------------------------------
# 9. Main simulation
# ---------------------------------------------------------------------------
def main():
    # --- Run mode ---
    #   default          : headless, runs as fast as possible, exports CSV + plot
    #   --live / --demo   : opens the esmini 3D viewer and paces to wall-clock
    #                       time so the driver model is watchable in real time
    live_view = any(flag in sys.argv for flag in ("--live", "--demo"))

    scenario_path = os.path.join(_script_dir, "..", "resources", "xosc", "highway_merge.xosc")
    # --fixed_timestep makes SE_StepDT deterministic; without it esmini runs in
    # realtime and the first step jumps sim-time to the wall-clock elapsed since
    # init, instantly firing the scenario's "sim time > 20 s" stop trigger.
    argv_strs  = [b"vehicle_agent_sim", b"--osc", scenario_path.encode("ascii"),
                  b"--fixed_timestep", f"{FIXED_DT}".encode("ascii")]
    if live_view:
        argv_strs += [b"--window", b"60", b"60", b"1200", b"800", b"--trail_mode", b"3"]
    else:
        argv_strs += [b"--headless"]
    argc       = len(argv_strs)
    argv_type  = ctypes.c_char_p * argc
    argv       = argv_type(*argv_strs)

    if se.SE_InitWithArgs(argc, argv) != 0:
        sys.exit("ERROR: Failed to initialize esmini.")

    if live_view:
        print("esmini initialized -- LIVE demo mode (real-time 3D viewer).")
    else:
        print("esmini initialized -- headless analysis mode (pass --live for the 3D viewer).")

    ego_id = se.SE_GetIdByName(EGO_NAME.encode("ascii"))

    # --- Build one driver per agent, seeding pose/speed from esmini init ---
    #   No .xosc edit and no esminiController property are needed: an empirical
    #   probe confirmed SE_ReportObjectPos fully overrides these agents' motion
    #   every step (their storyboard SpeedActions are step-instant and esmini
    #   does not re-integrate over a reported pose).
    drivers = []
    state = SE_ScenarioObjectState()
    for idx, name in enumerate(AGENT_NAMES):
        oid = se.SE_GetIdByName(name.encode("ascii"))
        if oid < 0 or oid == ego_id:
            print(f"  skipping {name} (id={oid})")
            continue
        if se.SE_GetObjectState(oid, ctypes.byref(state)) != 0:
            print(f"  could not read init state for {name} -- skipping")
            continue
        model = BicycleModel(state.x, state.y, state.h, state.speed)
        rng   = random.Random(RANDOM_SEED + idx)   # independent, reproducible
        drv   = AgentDriver(name, oid, model, state.speed, rng)
        drivers.append(drv)
        print(f"  {name}: id={oid} start=({state.x:7.1f},{state.y:7.1f}) "
              f"hdg={math.degrees(state.h):+6.1f}deg v0={state.speed:4.1f} "
              f"target={drv.target_speed:4.1f} lane_bias={drv.lane_bias:+.2f}m")

    if not drivers:
        se.SE_Close()
        sys.exit("ERROR: No agents to drive.")

    total_steps = int(SIM_DURATION / FIXED_DT)
    print(f"Driving {len(drivers)} agents for {SIM_DURATION:.0f}s "
          f"({total_steps} steps @ {1.0 / FIXED_DT:.0f} Hz). Ego stays on storyboard.")
    print("-" * 70)

    log_rows = []   # (time, agent, x, y, heading_deg, speed, steer_deg, target_speed)
    ego_path = []   # (x, y) reference track of the storyboard-driven Ego

    wall_start = time.perf_counter()

    for i in range(total_steps):
        t_s = i * FIXED_DT

        # Step A: drive each agent (guidance -> control -> integrate -> report)
        for drv in drivers:
            drv.step(FIXED_DT, t_s)
            log_rows.append((
                t_s, drv.name, drv.model.x, drv.model.y,
                math.degrees(drv.model.yaw), drv.model.speed,
                math.degrees(drv.last_steer), drv.target_speed,
            ))

        # Step B: advance the scenario engine (also moves the storyboard Ego)
        se.SE_StepDT(FIXED_DT)

        # Step C: record Ego reference position (read-only, never reported)
        if ego_id >= 0 and se.SE_GetObjectState(ego_id, ctypes.byref(state)) == 0:
            ego_path.append((state.x, state.y))

        if se.SE_GetQuitFlag():
            break

        # Step D: in live mode, pace to wall-clock time for a real-time view
        if live_view:
            target_wall = wall_start + (i + 1) * FIXED_DT
            lag = target_wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)

        # Telemetry -- print once per simulated second
        if i % int(1.0 / FIXED_DT) == 0:
            summary = " | ".join(
                f"{d.name} v={d.model.speed:4.1f} "
                f"str={math.degrees(d.last_steer):+4.1f}deg"
                for d in drivers)
            print(f"t={t_s:5.1f}s | {summary}")

    se.SE_Close()
    print("Simulation completed.")

    export_results(log_rows, ego_path, _script_dir)


if __name__ == "__main__":
    main()
