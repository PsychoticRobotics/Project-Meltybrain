import os
import json
from typing import Dict

def count_high_accel_events(directory: str) -> Dict[str, int]:
    """
    Scan all .log files in the given directory and count Accelerometer events
    with |accel_y| >= 10G for each file.
    
    Parameters:
    directory (str): Path to the directory containing log files.
    
    Returns:
    Dict[str, int]: Dictionary with filename as key and count as value.
    """
    results = {}
    
    # Get all .log files in the directory
    log_files = [f for f in os.listdir(directory) if f.endswith('.log')]
    
    for filename in sorted(log_files):
        filepath = os.path.join(directory, filename)
        count = 0
        
        try:
            with open(filepath, 'r') as file:
                for line in file:
                    if line.strip():
                        try:
                            event = json.loads(line)
                            if event[0] == "Accelerometer":
                                accel_y = event[2]  # Assuming format: ["Accelerometer", ts, accel_y, accel_z]
                                if abs(accel_y) >= 10:
                                    count += 1
                        except (json.JSONDecodeError, IndexError):
                            # Skip invalid lines or malformed events
                            continue
            results[filename] = count
        except Exception as e:
            print(f"Error processing {filename}: {e}")
            results[filename] = -1  # Indicate error
    
    return results

def print_results(results: Dict[str, int]):
    """
    Print the results in a readable format.
    """
    print("High Acceleration Events (>=10G on Y axis):")
    for filename, count in results.items():
        if count >= 0:
            print(f"{filename}: {count}")
        else:
            print(f"{filename}: Error")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python accel_counter.py <directory_path>")
        sys.exit(1)
    
    directory = sys.argv[1]
    if not os.path.isdir(directory):
        print(f"Error: {directory} is not a valid directory.")
        sys.exit(1)
    
    results = count_high_accel_events(directory)
    print_results(results)
