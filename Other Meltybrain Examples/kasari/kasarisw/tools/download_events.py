import socket
import struct
import time
import json
import os
from datetime import datetime
import sys
from parse_events import parse_event

# Function to process events from buffer
def process_events(event_buffer, current_log, last_ts, n, timestamp_str, total_events):
    while len(event_buffer) > 0:
        old_len = len(event_buffer)
        event, event_buffer = parse_event(event_buffer)
        if event:
            total_events += 1
            ts = event[1]
            if last_ts is not None and ts < last_ts - 100000:
                if current_log:
                    current_log.close()
                n += 1
                current_log = None

            if current_log is None:
                log_path = f"log/kasarisw_{timestamp_str}_{n}.log"
                current_log = open(log_path, 'w')
                print(f"Starting new log file: {log_path}")

            json.dump(event, current_log)
            current_log.write('\n')
            current_log.flush()
            last_ts = ts
        else:
            if len(event_buffer) == old_len:
                break
    return event_buffer, current_log, last_ts, n, total_events

# Main code for download_events.py
BATCH_FLUSH_SIZE = 4096
BATCH_DATA_SIZE = BATCH_FLUSH_SIZE - 4

def main():
    # Command line argument for IP:port
    if len(sys.argv) > 1:
        host_port = sys.argv[1]
    else:
        host_port = "192.168.1.248:8081"
    host, port_str = host_port.split(':')
    port = int(port_str)

    # Connect with retries for 60 seconds
    sock = None
    for attempt in range(60):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, port))
            sock.settimeout(None)
            print(f"Connected to {host}:{port}")
            break
        except Exception as e:
            print(f"Connection attempt {attempt+1} failed: {e}. Retrying in 1 second...")
            time.sleep(1)
    else:
        print("Failed to connect after 60 attempts.")
        sys.exit(1)

    # Prepare for streaming
    batch_buffer = b''
    event_buffer = b''
    total_bytes = 0
    total_events = 0
    last_progress = time.time()

    os.makedirs('log', exist_ok=True)
    timestamp_str = datetime.now().strftime('%Y-%m-%d_%H%M%S')
    n = 1
    current_log = None
    last_ts = None

    while True:
        chunk = sock.recv(4096)
        if not chunk:
            break
        batch_buffer += chunk
        total_bytes += len(chunk)

        now = time.time()
        if now > last_progress + 1:
            print(f"Downloaded {total_bytes} bytes, parsed {total_events} events")
            last_progress = now

        while len(batch_buffer) >= BATCH_FLUSH_SIZE:
            batch = batch_buffer[:BATCH_FLUSH_SIZE]
            seq_bytes = batch[0:4]
            seq = struct.unpack('<I', seq_bytes)[0]
            if seq == 0xffffffff:
                msg = batch[4:].rstrip(b'\x00').decode('utf-8', errors='ignore')
                print("---");
                print("PANIC:", msg)
                print("---");
                batch_buffer = batch_buffer[BATCH_FLUSH_SIZE:]
                continue
            data = batch[4:]  # Ignore seq
            event_buffer += data
            batch_buffer = batch_buffer[BATCH_FLUSH_SIZE:]

            # Process events
            event_buffer, current_log, last_ts, n, total_events = process_events(
                event_buffer, current_log, last_ts, n, timestamp_str, total_events
            )

    # After reception, process any remaining event_buffer
    event_buffer, current_log, last_ts, n, total_events = process_events(
        event_buffer, current_log, last_ts, n, timestamp_str, total_events
    )

    if len(event_buffer) > 0:
        print(f"Incomplete data at end: {len(event_buffer)} bytes remaining")

    if current_log:
        current_log.close()

    sock.close()
    print(f"Download complete. Total bytes: {total_bytes}, total events: {total_events}")

if __name__ == "__main__":
    main()
