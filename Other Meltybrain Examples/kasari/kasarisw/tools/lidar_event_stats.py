import json
import sys
import numpy as np

def main():
    # Read input
    if len(sys.argv) > 1:
        with open(sys.argv[1], 'r') as f:
            lines = f.readlines()
    else:
        lines = sys.stdin.readlines()

    # Parse LIDAR events
    events = []
    for line in lines:
        try:
            event = json.loads(line.strip())
            if event[0] == "Lidar":
                events.append(event)
        except json.JSONDecodeError:
            continue

    if not events:
        print("No LIDAR events found.")
        return

    # Extract timestamps
    ts = np.array([e[1] for e in events], dtype=np.int64)
    intervals = np.diff(ts)

    # Filter disconnection intervals (>1,000,000 µs)
    disconnection_threshold = 1_000_000
    valid_intervals = intervals[intervals <= disconnection_threshold]
    disconnection_intervals = intervals[intervals > disconnection_threshold]

    # Timing statistics for valid intervals
    print("LIDAR Timing Statistics (in µs, excluding disconnections):")
    print(f"  Total Intervals: {len(intervals)}")
    print(f"  Disconnection Intervals (>1,000,000 µs): {len(disconnection_intervals)} ({len(disconnection_intervals)/len(intervals)*100:.2f}%)")
    if len(valid_intervals) > 0:
        print(f"  Valid Intervals: {len(valid_intervals)}")
        print(f"  Mean: {np.mean(valid_intervals):.2f}")
        print(f"  Median: {np.median(valid_intervals):.2f}")
        print(f"  Std Dev: {np.std(valid_intervals):.2f}")
        print(f"  Min: {np.min(valid_intervals):.2f}")
        print(f"  Max: {np.max(valid_intervals):.2f}")

        # Intervals outside 1500–2500 µs
        target_min, target_max = 1500, 2500
        outside = (valid_intervals < target_min) | (valid_intervals > target_max)
        outside_count = np.sum(outside)
        print(f"\nIntervals Outside {target_min}–{target_max} µs:")
        print(f"  Count: {outside_count} ({outside_count/len(valid_intervals)*100:.2f}%)")
        print(f"  Below {target_min} µs: {np.sum(valid_intervals < target_min)} ({np.sum(valid_intervals < target_min)/len(valid_intervals)*100:.2f}%)")
        print(f"  Above {target_max} µs: {np.sum(valid_intervals > target_max)} ({np.sum(valid_intervals > target_max)/len(valid_intervals)*100:.2f}%)")

        # Distribution bins
        bins = [0, 1000, 1500, 2000, 2500, 3000, 10_000, 1_000_000]
        bin_labels = ["<1000", "1000–1500", "1500–2000", "1900–2500", "2500–3000", "3000-10,000", "3000–1,000,000"]
        hist, _ = np.histogram(valid_intervals, bins=bins)
        print("\nInterval Distribution:")
        for label, count in zip(bin_labels, hist):
            print(f"  {label} µs: {count} ({count/len(valid_intervals)*100:.2f}%)")
    else:
        print("No valid intervals after filtering disconnections.")

if __name__ == "__main__":
    main()
