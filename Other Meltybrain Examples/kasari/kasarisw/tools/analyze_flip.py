import argparse
import json
import numpy as np
from scipy.signal import medfilt
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description="Analyze accelerometer data for simplified flip detection")
    parser.add_argument('logfile', help="Path to the JSON event log file")
    parser.add_argument('--ema_alpha', type=float, default=0.01, help="EMA smoothing alpha (default: 0.01)")
    parser.add_argument('--median_kernel', type=int, default=5, help="Median filter kernel size (odd number, 0 to disable, default: 5)")
    parser.add_argument('--calib_count', type=int, default=50, help="Calibration sample count (default: 50)")
    parser.add_argument('--calib_ay_var_thresh', type=float, default=0.1, help="Max AY variance for valid calibration (default: 0.1)")
    parser.add_argument('--calib_min_g', type=float, default=-4.0, help="Calibration min G (default: -4.0)")
    parser.add_argument('--calib_max_g', type=float, default=4.0, help="Calibration max G (default: 4.0)")
    parser.add_argument('--calib_offset_add', type=float, default=1.0, help="Calibration offset addition (default: 1.0)")
    parser.add_argument('--reg_window', type=int, default=50, help="Window size for AY-AZ regression (default: 50)")
    parser.add_argument('--reg_min_ay', type=float, default=2.0, help="Min |AY| to add to regression window (default: 2.0)")
    parser.add_argument('--reg_alpha', type=float, default=0.1, help="Smoothing alpha for gravity_score (default: 0.1)")
    parser.add_argument('--reg_corr_thresh', type=float, default=0.1, help="Min |slope| to apply correction (default: 0.1)")
    parser.add_argument('--flip_low_thresh', type=float, default=-2.0, help="Low threshold for initial flip correction (default: -2.0)")
    parser.add_argument('--flip_hyst_high', type=float, default=0.2, help="Hysteresis high threshold (default: 0.2)")
    parser.add_argument('--flip_thresh', type=float, default=0.55, help="Flip detection threshold (default: 0.55)")
    parser.add_argument('--verbose', action='store_true', help="Enable verbose debug prints")
    args = parser.parse_args()

    if args.verbose:
        print(f"Loading log file: {args.logfile}")

    # Load data
    ts_list, ay_list, az_list = [], [], []
    with open(args.logfile) as f:
        for line in f:
            event = json.loads(line)
            if event[0] == 'Accelerometer':
                ts, ay, az = event[1] / 1e6, event[2], event[3]
                ts_list.append(ts)
                ay_list.append(ay)
                az_list.append(az)

    if not ts_list:
        print("No Accelerometer events found in log.")
        return

    print(f"Loaded {len(ts_list)} Accelerometer events.")

    ts = np.array(ts_list)
    ay_raw = np.array(ay_list)
    az_raw = np.array(az_list)

    if args.verbose:
        print("Applying filters...")

    # Pre-filter: Median on raw AZ/AY if enabled
    if args.median_kernel > 0 and args.median_kernel % 2 == 1:
        az_filt = medfilt(az_raw, kernel_size=args.median_kernel)
        ay_filt = medfilt(ay_raw, kernel_size=args.median_kernel)
    else:
        az_filt = az_raw.copy()
        ay_filt = ay_raw.copy()

    # EMA smoothing
    smoothed_az = np.zeros_like(az_filt)
    smoothed_ay = np.zeros_like(ay_filt)
    smoothed_az[0] = az_filt[0]
    smoothed_ay[0] = ay_filt[0]
    for i in range(1, len(az_filt)):
        smoothed_az[i] = args.ema_alpha * az_filt[i] + (1 - args.ema_alpha) * smoothed_az[i-1]
        smoothed_ay[i] = args.ema_alpha * ay_filt[i] + (1 - args.ema_alpha) * smoothed_ay[i-1]

    # Calibration simulation
    calib_samples_az = []
    calib_samples_ay = []  # For variance check
    calib_done = False
    calib_complete_idx = -1
    offset = 0.0

    # Runtime vars
    corrected_az = np.full_like(smoothed_az, np.nan)
    gravity_score = np.full_like(smoothed_az, np.nan)
    gravity_score[0] = smoothed_az[0]  # Initial
    flipped = np.zeros_like(smoothed_az, dtype=bool)  # Use flipped_raw directly
    reg_window = []  # List of (ay, az) tuples

    prev_flipped = False
    state_changes = []

    for i in range(len(smoothed_az)):
        if args.verbose and i % 1000 == 0:
            print(f"Processing sample {i}/{len(smoothed_az)} (ts={ts[i]:.3f}s)")

        az_sm = smoothed_az[i]
        ay_sm = smoothed_ay[i]

        # Calibration (only if not done)
        if not calib_done:
            calib_samples_az.append(az_sm)
            calib_samples_ay.append(ay_sm)
            if len(calib_samples_az) >= args.calib_count:
                # Check AY variance
                ay_var = np.var(calib_samples_ay)
                if args.verbose:
                    print(f"Calibration attempt at i={i}: AY var={ay_var:.3f} (thresh={args.calib_ay_var_thresh})")
                if ay_var <= args.calib_ay_var_thresh:
                    # Use median for robustness
                    sorted_az = np.sort(calib_samples_az)
                    median_az = sorted_az[len(sorted_az) // 2]
                    offset = median_az + args.calib_offset_add
                    calib_done = True
                    if calib_complete_idx == -1:
                        calib_complete_idx = i
                    print(f"Calibration completed at ts={ts[i]:.3f}s: offset={offset:.2f}")
                    # Backfill corrected_az and gravity_score
                    for j in range(calib_complete_idx + 1):
                        corrected_az[j] = smoothed_az[j] - offset
                        gravity_score[j] = corrected_az[j]
                else:
                    # Shift window: remove oldest
                    calib_samples_az = calib_samples_az[1:]
                    calib_samples_ay = calib_samples_ay[1:]
                    if args.verbose:
                        print("Calibration delayed: High AY variance")

        # Post-calib processing
        if calib_done:
            corrected_az[i] = az_sm - offset

            # Regression for correlation if spinning (for gravity_score, not flip detection)
            if abs(ay_sm) > args.reg_min_ay:
                reg_window.append((ay_sm, az_sm))
                if len(reg_window) > args.reg_window:
                    reg_window.pop(0)

                if len(reg_window) == args.reg_window:
                    ay_vals, az_vals = zip(*reg_window)
                    ay_vals = np.array(ay_vals)
                    az_vals = np.array(az_vals)
                    n = len(ay_vals)
                    sum_ay = np.sum(ay_vals)
                    sum_az = np.sum(az_vals)
                    sum_ay2 = np.sum(ay_vals**2)
                    sum_ay_az = np.sum(ay_vals * az_vals)
                    denom = n * sum_ay2 - sum_ay**2
                    if denom != 0:
                        a = (n * sum_ay_az - sum_ay * sum_az) / denom
                        b = (sum_az - a * sum_ay) / n
                        if abs(a) > args.reg_corr_thresh:
                            if i == calib_complete_idx:
                                gravity_score[i] = b
                            else:
                                gravity_score[i] = args.reg_alpha * b + (1 - args.reg_alpha) * gravity_score[i-1]
                            if args.verbose and i % 1000 == 0:
                                print(f"Regression update at ts={ts[i]:.3f}s: slope={a:.3f}, intercept={b:.3f}, gravity_score={gravity_score[i]:.3f}")
                        else:
                            gravity_score[i] = args.reg_alpha * corrected_az[i] + (1 - args.reg_alpha) * gravity_score[i-1] if i > 0 else corrected_az[i]
                    else:
                        gravity_score[i] = args.reg_alpha * corrected_az[i] + (1 - args.reg_alpha) * gravity_score[i-1] if i > 0 else corrected_az[i]
                else:
                    gravity_score[i] = args.reg_alpha * corrected_az[i] + (1 - args.reg_alpha) * gravity_score[i-1] if i > 0 else corrected_az[i]
            else:
                gravity_score[i] = args.reg_alpha * corrected_az[i] + (1 - args.reg_alpha) * gravity_score[i-1] if i > 0 else corrected_az[i]

            # Flip detection using corrected_az
            score = corrected_az[i]
            if score < args.flip_low_thresh:
                offset -= 2.0  # Correct offset
                flipped[i] = False
            elif score > args.flip_hyst_high and prev_flipped:
                flipped[i] = True
            elif score > args.flip_thresh:
                flipped[i] = True
            else:
                flipped[i] = False

            # Print state change if different from previous
            if i > 0 and flipped[i] != flipped[i-1]:
                change_type = "Flip" if flipped[i] else "Unflip"
                print(f"State change at ts={ts[i]:.3f}s: {change_type} (corrected_az={score:.2f})")
                state_changes.append((ts[i], change_type, score))

            prev_flipped = flipped[i]

    # Compute stats for upright and flipped periods
    upright_idx = np.where(~flipped)[0]
    flipped_idx = np.where(flipped)[0]
    upright_corrected_az = corrected_az[upright_idx]
    flipped_corrected_az = corrected_az[flipped_idx]

    print("\nSummary Statistics:")
    print(f"Total samples: {len(flipped)}")
    print(f"Flipped samples: {len(flipped_idx)} ({(len(flipped_idx) / len(flipped) * 100):.2f}%)")
    print(f"Number of state changes: {len(state_changes)}")
    if state_changes:
        print("State changes:")
        for t, change_type, score in state_changes:
            print(f" - ts={t:.3f}s: {change_type} (corrected_az={score:.2f})")
    else:
        print("No state changes detected.")
    
    print("\nCorrected AZ Statistics:")
    print(f"Upright periods (n={len(upright_corrected_az)}):")
    print(f"  Mean: {np.mean(upright_corrected_az):.3f}, Min: {np.min(upright_corrected_az):.3f}, Max: {np.max(upright_corrected_az):.3f}")
    print(f"Flipped periods (n={len(flipped_corrected_az)}):")
    print(f"  Mean: {np.mean(flipped_corrected_az) if len(flipped_corrected_az) > 0 else np.nan:.3f}, "
          f"Min: {np.min(flipped_corrected_az) if len(flipped_corrected_az) > 0 else np.nan:.3f}, "
          f"Max: {np.max(flipped_corrected_az) if len(flipped_corrected_az) > 0 else np.nan:.3f}")

    # Plots
    fig, axs = plt.subplots(5, 1, sharex=True, figsize=(12, 12))

    axs[0].plot(ts, az_raw, label='raw_az', alpha=0.5)
    axs[0].plot(ts, smoothed_az, label='smoothed_az', lw=2)
    axs[0].legend()
    axs[0].set_ylabel('G')
    axs[0].set_title('Raw and Smoothed Z Acceleration')

    axs[1].plot(ts, ay_raw, label='raw_ay', alpha=0.5)
    axs[1].plot(ts, smoothed_ay, label='smoothed_ay', lw=2)
    axs[1].legend()
    axs[1].set_ylabel('G')
    axs[1].set_title('Raw and Smoothed Y Acceleration (for reference)')

    axs[2].plot(ts, corrected_az, label='corrected_az (used for flip)', color='purple', alpha=0.5)
    axs[2].plot(ts, gravity_score, label='gravity_score (reference only)', color='magenta', lw=2)
    axs[2].legend()
    axs[2].set_ylabel('G')
    axs[2].set_title('Corrected AZ and Gravity Score')

    axs[3].plot(ts, flipped, label='flipped', color='black', drawstyle='steps-post', lw=2)
    axs[3].plot(ts, corrected_az, label='corrected_az (used for flip)', color='purple', alpha=0.5)
    axs[3].axhline(args.flip_thresh, color='b', ls='--', label='flip_thresh')
    axs[3].axhline(args.flip_hyst_high, color='g', ls='--', label='hyst_high')
    axs[3].axhline(args.flip_low_thresh, color='r', ls='--', label='flip_low_thresh')
    axs[3].legend()
    axs[3].set_ylabel('Flipped (1/0)')
    axs[3].set_title('Detected Flipped Status')
    axs[3].set_ylim(-1.0, 2.0)

    # Correlation slope over time (for debugging)
    slopes = np.full_like(ts, np.nan)
    for i in range(args.reg_window - 1, len(ts)):
        window_start = i - args.reg_window + 1
        ay_win = smoothed_ay[window_start:i+1]
        az_win = smoothed_az[window_start:i+1]
        if len(ay_win) > 0:
            cov = np.cov(ay_win, az_win)
            if cov[0,0] != 0:
                slopes[i] = cov[0,1] / cov[0,0]

    axs[4].plot(ts, slopes, label='regression_slope', color='orange')
    axs[4].axhline(args.reg_corr_thresh, color='r', ls='--', label='corr_thresh')
    axs[4].legend()
    axs[4].set_ylabel('Slope')
    axs[4].set_title('AY-AZ Regression Slope Over Time')
    axs[4].set_xlabel('Time (s)')

    # Calibration completion line
    if calib_complete_idx != -1:
        calib_ts = ts[calib_complete_idx]
        for ax in axs:
            ax.axvline(calib_ts, color='gray', ls='-', alpha=0.5, label='Calib Done')
        axs[0].legend()  # Update one legend

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
