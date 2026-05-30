#!/usr/bin/env python3
"""
simulate.py — Meltybrain estimator simulation

Runs synthetic sensor data through Python ports of AngleEstimator and
IRArenaTracker, comparing estimated outputs against known ground truth.

Scenarios
---------
  spinup     : spin up to full speed, hold, spin down — tests ω estimation
  impact     : steady spin, sudden ω drop (simulates getting hit), recovery
  translate  : steady spin + slow arena translation — tests position tracking
  full       : spin-up + translate + impact + spin-down (default)

Usage
-----
  python3 tools/simulate.py                       # full scenario, all sensors
  python3 tools/simulate.py --scenario impact
  python3 tools/simulate.py --no-mag              # disable magnetometer fusion
  python3 tools/simulate.py --no-arena            # disable arena tracker
  python3 tools/simulate.py --noise 0.1           # higher accelerometer noise

Dependencies: numpy, matplotlib  (pip install numpy matplotlib)
"""

import argparse
import math
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as patches

# ─── Configuration ────────────────────────────────────────────────────────────

DT           = 1.0 / 1000.0   # simulation timestep: 1 kHz (matches Teensy loop)
ARENA_SIZE   = 2.44            # metres — 8 ft square arena
MAX_OMEGA    = 200.0           # rad/s ≈ 1910 RPM

# Robot geometry — sensor separation distances from geometric centre
Y1 = 0.033   # metres to accelerometer 1 (above centre)
Y2 = 0.033   # metres to accelerometer 2 (below centre)

# Sensor noise (1-sigma)
ACCEL_SIGMA   = 0.05   # g    — LIS331 noise + vibration
MAG_SIGMA     = 0.04   # rad  — magnetometer heading noise
BEARING_SIGMA = 0.03   # rad  — IR wall bearing noise
WIDTH_SIGMA   = 0.008  # rad  — IR angular-width noise

# AngleEstimator.cpp constants (must match)
OMEGA_IIR  = 0.2     # weight of new sample in ω IIR  (0.2 new + 0.8 old)
MAG_GAIN   = 0.005   # magnetometer soft-correction gain

# IRArena.h constants (must match)
WALL_MATCH_RAD  = 0.26   # rad — max bearing gap for slot match
WALL_MAX_DRIFT  = 0.05   # rad — max per-rev drift before persistence resets
WALL_MIN_PERSIST = 5     # revolutions before slot → confirmed wall
WALL_DECAY_REVS = 3      # missed revolutions before slot removed
EMA_ALPHA       = 0.3    # smoothing on wall bearing + width
MIN_OMEGA_ARENA = 15.0   # rad/s — minimum spin for useful IR measurements


# ─── Ground truth trajectory ─────────────────────────────────────────────────

def _omega_at(t: float, scenario: str) -> float:
    """Ground truth ω(t) in rad/s for each scenario."""
    if scenario == 'spinup':
        if   t < 2.0:  return MAX_OMEGA * (t / 2.0)
        elif t < 8.0:  return MAX_OMEGA
        elif t < 10.0: return MAX_OMEGA * (1.0 - (t - 8.0) / 2.0)
        return 0.0

    elif scenario == 'impact':
        if   t < 1.0:  return MAX_OMEGA * t             # quick spin-up
        elif t < 5.0:  return MAX_OMEGA                  # hold
        elif t < 5.1:  return MAX_OMEGA * (1.0 - 0.25 * (t - 5.0) / 0.1)  # sudden impact
        elif t < 6.5:  return MAX_OMEGA * 0.75           # damaged/disrupted
        elif t < 7.5:  return MAX_OMEGA * 0.75 + MAX_OMEGA * 0.25 * (t - 6.5)  # recovery
        elif t < 11.0: return MAX_OMEGA
        elif t < 13.0: return MAX_OMEGA * (1.0 - (t - 11.0) / 2.0)
        return 0.0

    else:  # 'translate' or 'full'
        if   t < 2.0:  return MAX_OMEGA * (t / 2.0)
        elif scenario == 'full' and 5.0 <= t < 5.1:
            return MAX_OMEGA * (1.0 - 0.25 * (t - 5.0) / 0.1)  # impact in full
        elif scenario == 'full' and 5.1 <= t < 6.5:
            return MAX_OMEGA * 0.75
        elif scenario == 'full' and 6.5 <= t < 7.5:
            return MAX_OMEGA * 0.75 + MAX_OMEGA * 0.25 * (t - 6.5)
        elif t < 10.0: return MAX_OMEGA
        elif t < 12.0: return MAX_OMEGA * (1.0 - (t - 10.0) / 2.0)
        return 0.0


def _velocity_at(t: float, scenario: str):
    """Ground truth translation velocity (vx, vy) in m/s."""
    if scenario in ('translate', 'full') and 3.0 <= t <= 9.0:
        return (0.06 * math.sin(t * 0.4),
                0.05 * math.cos(t * 0.35))
    return 0.0, 0.0


def make_trajectory(scenario: str, dt: float = DT):
    """
    Build ground truth arrays for a given scenario.
    Returns (ts, angles, omegas, xs, ys) — all numpy arrays.
    angles is unwrapped (continuously increasing) for clean error plots.
    """
    durations = {'spinup': 11.0, 'impact': 14.0, 'translate': 13.0, 'full': 13.0}
    duration  = durations.get(scenario, 13.0)

    ts = np.arange(0.0, duration, dt)
    angles = np.zeros(len(ts))
    omegas = np.zeros(len(ts))
    xs     = np.zeros(len(ts))
    ys     = np.zeros(len(ts))

    angle = 0.0
    x, y  = 0.0, 0.0
    half  = ARENA_SIZE / 2.0 - 0.05   # keep 5 cm from walls

    for i, t in enumerate(ts):
        omega   = _omega_at(t, scenario)
        vx, vy  = _velocity_at(t, scenario)

        angle  += omega * dt
        x       = max(-half, min(half, x + vx * dt))
        y       = max(-half, min(half, y + vy * dt))

        omegas[i] = omega
        angles[i] = angle   # unwrapped
        xs[i]     = x
        ys[i]     = y

    return ts, angles, omegas, xs, ys


# ─── Sensor simulation ────────────────────────────────────────────────────────

def sim_accel(omega: float, y1: float = Y1, y2: float = Y2, cy: float = 0.0,
              sigma: float = ACCEL_SIGMA):
    """
    Simulate LIS331 dual-accelerometer y-axis readings (in g) with Gaussian noise.

    Physics: centripetal acceleration along the separation axis:
        ay1 = ω² × (cy − y1)   ← negative at typical cy=0 (pulls toward centre)
        ay2 = ω² × (cy + y2)   ← positive

    From these the estimator recovers ω²  =  (ay2 − ay1) / (y1 + y2).
    """
    w2       = omega ** 2
    ay1_true = w2 * (cy - y1)
    ay2_true = w2 * (cy + y2)
    return (ay1_true + random.gauss(0, sigma),
            ay2_true + random.gauss(0, sigma))


def sim_mag(angle_true: float) -> float:
    """
    Simulate magnetometer heading: ground truth + noise, wrapped to [0, 2π].
    """
    a = angle_true + random.gauss(0, MAG_SIGMA)
    return math.fmod(a, 2.0 * math.pi) % (2.0 * math.pi)


def sim_ir_hits(robot_x: float, robot_y: float,
                arena_size: float = ARENA_SIZE) -> list:
    """
    Generate one revolution's IR wall hits from position (robot_x, robot_y).

    For each arena wall, compute the true center bearing and angular width as
    seen from the robot, then add Gaussian noise.

    Returns list of (bearing_rad, angular_width_rad) for each of the 4 walls.
    """
    half  = arena_size / 2.0
    # Each wall defined by two corner points (start, end)
    walls = [
        ((+half, -half), (+half, +half)),   # right
        ((+half, +half), (-half, +half)),   # top
        ((-half, +half), (-half, -half)),   # left
        ((-half, -half), (+half, -half)),   # bottom
    ]

    hits = []
    for (x1, y1_w), (x2, y2_w) in walls:
        a1 = math.atan2(y1_w - robot_y, x1 - robot_x)
        a2 = math.atan2(y2_w - robot_y, x2 - robot_x)

        # Signed angular span from a1 to a2, wrapped to (−π, π)
        diff = math.fmod(a2 - a1 + 3.0 * math.pi, 2.0 * math.pi) - math.pi

        # Centre bearing = midpoint of the arc
        bearing = math.fmod(a1 + diff / 2.0, 2.0 * math.pi)
        if bearing < 0.0:
            bearing += 2.0 * math.pi

        width = abs(diff)

        # Add noise
        bearing = math.fmod(bearing + random.gauss(0, BEARING_SIGMA), 2.0 * math.pi)
        if bearing < 0.0:
            bearing += 2.0 * math.pi
        width = max(0.01, width + random.gauss(0, WIDTH_SIGMA))

        hits.append((bearing, width))

    return hits


# ─── AngleEstimator (Python port of AngleEstimator.cpp) ──────────────────────

class AngleEstimatorPy:
    """
    Python port of AngleEstimator.cpp, dual-sensor path.

    Differences from the C++ version:
      - angle is stored UNWRAPPED (keeps accumulating) for cleaner error plots.
        Use angle % (2π) wherever the wrapped value is needed.
      - No Eigen dependency, no Vector3d — just scalar floats.
    """

    def __init__(self, y1: float = Y1, y2: float = Y2,
                 omega_iir: float = OMEGA_IIR, mag_gain: float = MAG_GAIN):
        self.y1         = y1
        self.y2         = y2
        self.omega_iir  = omega_iir
        self.mag_gain   = mag_gain

        self.angle      = 0.0   # radians, unwrapped
        self.omega      = 0.0   # rad/s
        self._last_t    = 0.0
        self._first     = True

    def update(self, t: float, ay1: float, ay2: float,
               mag_angle: float = None):
        """
        t         : current time (s)
        ay1, ay2  : y-axis accelerometer readings (g), sensor 1 and 2
        mag_angle : magnetometer angle (rad, wrapped [0, 2π]), or None
        Returns   : (angle_unwrapped, omega)
        """
        if self._first:
            self._last_t = t
            self._first  = False
            return self.angle, self.omega

        dt = t - self._last_t
        self._last_t = t
        if dt <= 0.0:
            return self.angle, self.omega

        # ── ω from differential formula ───────────────────────────────────────
        omega_sq = (ay2 - ay1) / (self.y1 + self.y2)
        if omega_sq > 0.0:
            omega_raw  = math.sqrt(omega_sq)
            self.omega = (1.0 - self.omega_iir) * self.omega + self.omega_iir * omega_raw

        # ── Integrate angle ───────────────────────────────────────────────────
        self.angle += self.omega * dt

        # ── Magnetometer soft correction ──────────────────────────────────────
        if mag_angle is not None:
            angle_w = self.angle % (2.0 * math.pi)
            error   = math.fmod(mag_angle - angle_w + 3.0 * math.pi, 2.0 * math.pi) - math.pi
            self.angle += self.mag_gain * error

        return self.angle, self.omega

    def correct_angle(self, delta: float):
        """Apply an external correction (e.g. from IR arena heading fix)."""
        self.angle += delta


# ─── IRArenaTracker (Python port of IRArena.cpp) ─────────────────────────────

class _Wall:
    __slots__ = ('bearing', 'width', 'persistence', 'miss_count', 'is_wall')

    def __init__(self, bearing: float, width: float):
        self.bearing     = bearing
        self.width       = width
        self.persistence = 1
        self.miss_count  = 0
        self.is_wall     = False


class IRArenaTrackerPy:
    """
    Python port of IRArenaTracker.cpp.

    Call process_revolution() once per completed revolution with a list of
    (bearing, angular_width) hits from sim_ir_hits().
    """

    def __init__(self, arena_size: float = ARENA_SIZE):
        self.arena_size  = arena_size
        self._slots: list[_Wall] = []

        self.pos_x        = 0.0
        self.pos_y        = 0.0
        self.has_position = False

        self._arena_rot   = 0.0
        self._rot_calibrated = False
        self.heading_error   = 0.0
        self.has_heading_fix = False

    # ── Public ────────────────────────────────────────────────────────────────

    def process_revolution(self, hits: list):
        """
        hits: list of (bearing_rad, angular_width_rad).
        Updates wall slots, position estimate, and heading error.
        """
        if not hits:
            return

        hit_matched   = [False] * len(hits)
        n_slots_start = len(self._slots)           # snapshot before adding new ones
        slot_seen     = [False] * n_slots_start

        # ── Match hits → existing slots ───────────────────────────────────────
        for hi, (bearing, width) in enumerate(hits):
            best_d, best_si = WALL_MATCH_RAD, -1
            for si, slot in enumerate(self._slots):
                d = self._adist(bearing, slot.bearing)
                if d < best_d:
                    best_d, best_si = d, si

            if best_si >= 0 and not slot_seen[best_si]:
                slot  = self._slots[best_si]
                delta = math.fmod(bearing - slot.bearing + 3*math.pi, 2*math.pi) - math.pi

                if abs(delta) > WALL_MAX_DRIFT:
                    # Object moved too much — reset, probably not a wall
                    slot.persistence = 0
                    slot.is_wall     = False

                slot.bearing  = (slot.bearing + EMA_ALPHA * delta) % (2*math.pi)
                slot.width   += EMA_ALPHA * (width - slot.width)
                slot.persistence += 1
                slot.miss_count   = 0
                if slot.persistence >= WALL_MIN_PERSIST:
                    slot.is_wall = True

                slot_seen[best_si] = True
                hit_matched[hi]    = True

        # ── New candidate slots for unmatched hits ────────────────────────────
        for hi, (bearing, width) in enumerate(hits):
            if not hit_matched[hi] and len(self._slots) < 8:
                self._slots.append(_Wall(bearing, width))

        # ── Decay / remove stale slots ────────────────────────────────────────
        # Only decay slots that existed at the start of this revolution —
        # newly added slots are implicitly "seen" (they came from this rev's hits).
        for si in range(n_slots_start - 1, -1, -1):
            if not slot_seen[si]:
                self._slots[si].miss_count += 1
                if self._slots[si].miss_count >= WALL_DECAY_REVS:
                    self._slots.pop(si)

        self._estimate_heading()
        self._estimate_position()

    def wall_count(self) -> int:
        return sum(1 for s in self._slots if s.is_wall)

    # ── Private ───────────────────────────────────────────────────────────────

    def _estimate_heading(self):
        walls = [s for s in self._slots if s.is_wall]
        if len(walls) < 2:
            return
        # Circular mean of bearing×4 maps 90° periodicity to 2π.
        sn = sum(math.sin(w.bearing * 4) for w in walls)
        cs = sum(math.cos(w.bearing * 4) for w in walls)
        rot = math.atan2(sn, cs) / 4.0
        if rot < 0:
            rot += math.pi / 2.0

        if not self._rot_calibrated:
            self._arena_rot      = rot
            self._rot_calibrated = True
        else:
            err = rot - self._arena_rot
            while err >  math.pi / 4: err -= math.pi / 2
            while err < -math.pi / 4: err += math.pi / 2
            self.heading_error   = err
            self.has_heading_fix = abs(err) > 0.005

    def _estimate_position(self):
        walls  = [s for s in self._slots if s.is_wall]
        paired = [False] * len(walls)
        sx, sy, n_pairs = 0.0, 0.0, 0

        for i in range(len(walls)):
            if paired[i]: continue
            for j in range(i + 1, len(walls)):
                if paired[j]: continue
                if abs(self._adist(walls[i].bearing, walls[j].bearing) - math.pi) < 0.52:
                    wA, wB = walls[i].width, walls[j].width
                    if wA + wB < 1e-6: continue
                    # Position along the axis defined by wall A's bearing.
                    # Closer wall → wider return.
                    pos = (self.arena_size / 2.0) * (wA - wB) / (wA + wB)
                    sx += pos * math.cos(walls[i].bearing)
                    sy += pos * math.sin(walls[i].bearing)
                    paired[i] = paired[j] = True
                    n_pairs += 1
                    if n_pairs >= 2: break
            if n_pairs >= 2: break

        if n_pairs > 0:
            self.pos_x        = sx
            self.pos_y        = sy
            self.has_position = True

    @staticmethod
    def _adist(a: float, b: float) -> float:
        """Shortest angular distance [0, π]."""
        d = abs(a - b)
        return d if d <= math.pi else 2.0 * math.pi - d


# ─── Simulation runner ────────────────────────────────────────────────────────

def run_simulation(scenario: str = 'full',
                   use_mag: bool   = True,
                   use_arena: bool = True,
                   accel_sigma: float = ACCEL_SIGMA,
                   seed: int = 42) -> dict:
    """
    Run the full simulation.  Returns a dict of recorded time-series arrays.
    """
    random.seed(seed)

    ts, angles_truth, omegas_truth, xs_truth, ys_truth = make_trajectory(scenario)

    estimator = AngleEstimatorPy()
    tracker   = IRArenaTrackerPy() if use_arena else None

    angle_est_arr  = np.zeros(len(ts))
    omega_est_arr  = np.zeros(len(ts))

    pos_times, xs_est, ys_est = [], [], []
    heading_fix_times, heading_fix_errors = [], []
    wall_count_times, wall_counts = [], []

    last_rev_angle = 0.0   # tracks next revolution boundary

    for i, t in enumerate(ts):
        omega_true = omegas_truth[i]
        angle_true = angles_truth[i]   # unwrapped

        # ── Sensor readings ───────────────────────────────────────────────────
        ay1, ay2 = sim_accel(omega_true, sigma=accel_sigma)

        # Magnetometer: available when spinning (same every loop — gain is low)
        mag_a = None
        if use_mag and omega_true > 5.0:
            mag_a = sim_mag(angle_true % (2.0 * math.pi))

        # ── Angle estimator ───────────────────────────────────────────────────
        angle_est, omega_est = estimator.update(t, ay1, ay2, mag_a)
        angle_est_arr[i] = angle_est
        omega_est_arr[i] = omega_est

        # ── IR arena tracker (once per revolution) ────────────────────────────
        if tracker is not None and omega_true >= MIN_OMEGA_ARENA:
            if angle_true >= last_rev_angle + 2.0 * math.pi:
                last_rev_angle += 2.0 * math.pi

                hits = sim_ir_hits(xs_truth[i], ys_truth[i])
                tracker.process_revolution(hits)

                wc = tracker.wall_count()
                wall_count_times.append(t)
                wall_counts.append(wc)

                if tracker.has_position:
                    pos_times.append(t)
                    xs_est.append(tracker.pos_x)
                    ys_est.append(tracker.pos_y)

                if tracker.has_heading_fix:
                    heading_fix_times.append(t)
                    heading_fix_errors.append(math.degrees(tracker.heading_error))
                    estimator.correct_angle(tracker.heading_error * 0.3)

    return {
        't':               ts,
        'omega_truth':     omegas_truth,
        'angle_truth':     angles_truth,
        'x_truth':         xs_truth,
        'y_truth':         ys_truth,
        'omega_est':       omega_est_arr,
        'angle_est':       angle_est_arr,
        'pos_times':       np.array(pos_times),
        'xs_est':          np.array(xs_est),
        'ys_est':          np.array(ys_est),
        'fix_times':       np.array(heading_fix_times),
        'fix_errors':      np.array(heading_fix_errors),
        'wall_count_times': np.array(wall_count_times),
        'wall_counts':     np.array(wall_counts),
    }


# ─── Plotting ─────────────────────────────────────────────────────────────────

def _angle_error_deg(est_arr, truth_arr):
    """
    Per-sample heading error (degrees), wrap-corrected to (−180°, 180°).
    Both inputs are unwrapped arrays.
    """
    errs = []
    for e, t in zip(est_arr, truth_arr):
        ew = e % (2.0 * math.pi)
        tw = t % (2.0 * math.pi)
        err = math.fmod(ew - tw + 3.0 * math.pi, 2.0 * math.pi) - math.pi
        errs.append(math.degrees(err))
    return np.array(errs)


def plot_results(rec: dict, scenario: str):
    fig = plt.figure(figsize=(15, 9))
    fig.suptitle(f"Meltybrain estimator simulation — {scenario}", fontsize=13, y=0.98)
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.42, wspace=0.35)

    ts = rec['t']

    # ── 1. ω over time ────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(ts, rec['omega_truth'], color='0.4', lw=1.5, label='Truth')
    ax1.plot(ts, rec['omega_est'],   'b-',        lw=0.9, label='Estimated', alpha=0.8)
    ax1.set_title('Angular velocity')
    ax1.set_ylabel('ω (rad/s)')
    ax1.set_xlabel('Time (s)')
    ax1.legend(fontsize=8)
    ax1.grid(True, alpha=0.3)

    # ── 2. ω error ────────────────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 0])
    omega_err = rec['omega_est'] - rec['omega_truth']
    ax2.plot(ts, omega_err, color='steelblue', lw=0.8)
    ax2.axhline(0, color='k', lw=0.5)
    ax2.fill_between(ts, omega_err, alpha=0.2, color='steelblue')
    ax2.set_title('ω error')
    ax2.set_ylabel('ω error (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)

    # ── 3. Heading error over time ────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[0, 1])
    angle_err = _angle_error_deg(rec['angle_est'], rec['angle_truth'])
    ax3.plot(ts, angle_err, color='seagreen', lw=0.8)
    ax3.axhline(0, color='k', lw=0.5)
    ax3.fill_between(ts, angle_err, alpha=0.2, color='seagreen')
    # Mark heading fix events
    for ft in rec['fix_times']:
        ax3.axvline(ft, color='orange', alpha=0.5, lw=0.6)
    ax3.set_title('Heading error  (▏= IR fix)')
    ax3.set_ylabel('Error (°)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)

    # ── 4. Arena map — truth path vs estimated position ───────────────────────
    ax4 = fig.add_subplot(gs[0, 2])
    half = ARENA_SIZE / 2.0
    ax4.add_patch(patches.Rectangle((-half, -half), ARENA_SIZE, ARENA_SIZE,
                                     lw=2, edgecolor='0.5', facecolor='#f8f8f8'))
    ax4.plot(rec['x_truth'], rec['y_truth'], 'k-', lw=1.2, label='Truth path', alpha=0.6, zorder=2)

    if len(rec['xs_est']) > 0:
        sc = ax4.scatter(rec['xs_est'], rec['ys_est'],
                         s=20, c=rec['pos_times'], cmap='plasma',
                         zorder=4, label='Arena est.', alpha=0.85)
        plt.colorbar(sc, ax=ax4, label='Time (s)', shrink=0.8)
        # Error lines from estimate to nearest truth point
        for xe, ye, te in zip(rec['xs_est'], rec['ys_est'], rec['pos_times']):
            idx = int(np.searchsorted(ts, te))
            if idx < len(ts):
                ax4.plot([xe, rec['x_truth'][idx]], [ye, rec['y_truth'][idx]],
                         'r-', alpha=0.25, lw=0.6, zorder=3)
        ax4.legend(fontsize=8, loc='lower right')
    else:
        ax4.text(0.5, 0.5, 'Arena tracker\nnot running', transform=ax4.transAxes,
                 ha='center', va='center', color='gray', fontsize=9)

    ax4.set_xlim(-half * 1.12, half * 1.12)
    ax4.set_ylim(-half * 1.12, half * 1.12)
    ax4.set_aspect('equal')
    ax4.set_title('Arena position')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.grid(True, alpha=0.2)

    # ── 5. Position error over time ───────────────────────────────────────────
    ax5 = fig.add_subplot(gs[1, 1])
    if len(rec['xs_est']) > 0:
        pos_errs_cm = []
        for xe, ye, te in zip(rec['xs_est'], rec['ys_est'], rec['pos_times']):
            idx = int(np.searchsorted(ts, te))
            if idx < len(ts):
                ex = xe - rec['x_truth'][idx]
                ey = ye - rec['y_truth'][idx]
                pos_errs_cm.append(math.sqrt(ex**2 + ey**2) * 100)
        ax5.plot(rec['pos_times'][:len(pos_errs_cm)], pos_errs_cm,
                 'o-', ms=3, lw=0.8, color='tomato')
        ax5.axhline(0, color='k', lw=0.5)
        ax5.set_title('Position error')
        ax5.set_ylabel('Error (cm)')
        ax5.set_xlabel('Time (s)')
        ax5.grid(True, alpha=0.3)
    else:
        ax5.text(0.5, 0.5, 'No position data', transform=ax5.transAxes,
                 ha='center', va='center', color='gray')
        ax5.set_title('Position error')

    # ── 6. Wall tracker convergence ───────────────────────────────────────────
    ax6 = fig.add_subplot(gs[1, 2])
    if len(rec['wall_counts']) > 0:
        ax6.step(rec['wall_count_times'], rec['wall_counts'],
                 where='post', lw=1.5, color='darkorchid')
        ax6.axhline(4, color='g', lw=1, linestyle='--', label='4 walls confirmed')
        ax6.set_ylim(0, 7)
        ax6.legend(fontsize=8)
    else:
        ax6.text(0.5, 0.5, 'Arena tracker\nnot enabled', transform=ax6.transAxes,
                 ha='center', va='center', color='gray', fontsize=9)
    ax6.set_title('Arena tracker: wall convergence')
    ax6.set_ylabel('Confirmed walls')
    ax6.set_xlabel('Time (s)')
    ax6.grid(True, alpha=0.3)

    plt.savefig('simulation_results.png', dpi=150, bbox_inches='tight')
    print("[sim] Saved simulation_results.png")
    plt.show()


# ─── Summary stats ─────────────────────────────────────────────────────────────

def print_summary(rec: dict):
    ts = rec['t']

    spinning = rec['omega_truth'] > 10.0
    if spinning.any():
        errs = np.abs(_angle_error_deg(rec['angle_est'][spinning],
                                       rec['angle_truth'][spinning]))
        print(f"  Heading error (spinning):  "
              f"mean={np.mean(errs):.2f}°  max={np.max(errs):.2f}°  "
              f"RMS={np.sqrt(np.mean(errs**2)):.2f}°")

    omega_err = np.abs(rec['omega_est'] - rec['omega_truth'])
    print(f"  ω error (all time):        "
          f"mean={np.mean(omega_err):.1f} rad/s  max={np.max(omega_err):.1f} rad/s")

    if len(rec['xs_est']) > 0:
        pos_errs = []
        for xe, ye, te in zip(rec['xs_est'], rec['ys_est'], rec['pos_times']):
            idx = int(np.searchsorted(ts, te))
            if idx < len(ts):
                pos_errs.append(math.sqrt((xe - rec['x_truth'][idx])**2 +
                                          (ye - rec['y_truth'][idx])**2) * 100)
        if pos_errs:
            pos_errs = np.array(pos_errs)
            print(f"  Position error:            "
                  f"mean={np.mean(pos_errs):.1f} cm  max={np.max(pos_errs):.1f} cm  "
                  f"(first valid at t={rec['pos_times'][0]:.1f}s)")

    if len(rec['fix_times']) > 0:
        print(f"  IR heading fixes applied:  {len(rec['fix_times'])}")


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Meltybrain estimator simulation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split('Dependencies')[0].strip()
    )
    parser.add_argument('--scenario', default='full',
                        choices=['spinup', 'impact', 'translate', 'full'],
                        help='Trajectory to simulate (default: full)')
    parser.add_argument('--no-mag',   action='store_true',
                        help='Disable magnetometer fusion')
    parser.add_argument('--no-arena', action='store_true',
                        help='Disable IR arena tracker')
    parser.add_argument('--noise',    type=float, default=ACCEL_SIGMA,
                        metavar='G',
                        help=f'Accelerometer noise sigma in g (default: {ACCEL_SIGMA})')
    parser.add_argument('--seed',     type=int, default=42,
                        help='Random seed for reproducibility (default: 42)')
    args = parser.parse_args()

    print(f"[sim] Scenario '{args.scenario}'  "
          f"mag={'on' if not args.no_mag else 'OFF'}  "
          f"arena={'on' if not args.no_arena else 'OFF'}  "
          f"noise={args.noise}g  seed={args.seed}")

    rec = run_simulation(
        scenario    = args.scenario,
        use_mag     = not args.no_mag,
        use_arena   = not args.no_arena,
        accel_sigma = args.noise,
        seed        = args.seed,
    )

    print("[sim] Results:")
    print_summary(rec)
    plot_results(rec, args.scenario)


if __name__ == '__main__':
    main()
