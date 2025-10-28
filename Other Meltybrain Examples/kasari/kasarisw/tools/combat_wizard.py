import socket
import struct
import tkinter as tk
from tkinter import ttk
import threading
import time
from collections import defaultdict
import queue
import json
import os
from datetime import datetime
import argparse
import sys

# Graph constants
GRAPH_WIDTH = 800
GRAPH_HEIGHT = 300
GRAPH_MARGIN_LEFT = 120
GRAPH_MARGIN_BOTTOM = 40
GRAPH_MARGIN_TOP = 20
GRAPH_MARGIN_RIGHT = 10
GRAPH_TICK_LENGTH = 5
GRAPH_LABEL_OFFSET = 10

# Shared queues for events and status (thread-safe)
event_queue = queue.Queue()
status_queue = queue.Queue()

# Event tags
EVENT_TAGS = {
    'Lidar': 0,
    'Accelerometer': 1,
    'Receiver': 2,
    'Vbat': 3,
    'WifiControl': 4,
    'Planner': 5,
    'Stats': 6,
}

def parse_event(buffer):
    """Parse a single event from the buffer, returning the event and remaining buffer."""
    if len(buffer) < 2:  # Min for tag
        return None, buffer
    encoded_tag = struct.unpack('<H', buffer[0:2])[0]
    tag = encoded_tag ^ 0x5555
    
    if tag == 0:  # Lidar: 2 (tag) + 8 (ts) + 16 (4*f32) = 26 bytes
        if len(buffer) < 26:
            return None, buffer
        timestamp, d1, d2, d3, d4 = struct.unpack('<Qffff', buffer[2:26])
        return ('Lidar', timestamp, d1, d2, d3, d4), buffer[26:]
    
    elif tag == 1:  # Accelerometer: 2 + 8 + 8 = 18 bytes
        if len(buffer) < 18:
            return None, buffer
        timestamp, acceleration_y, acceleration_z = struct.unpack('<Qff', buffer[2:18])
        return ('Accelerometer', timestamp, acceleration_y, acceleration_z), buffer[18:]
    
    elif tag == 2:  # Receiver: Min 2 + 8 + 1 + 1 = 12 bytes; +4 if flag==1
        if len(buffer) < 12:
            return None, buffer
        timestamp, channel, flag = struct.unpack('<QBB', buffer[2:12])
        if flag == 0:
            return ('Receiver', timestamp, channel, None), buffer[12:]
        elif flag == 1:
            if len(buffer) < 16:
                return None, buffer
            pulse_length = struct.unpack('<f', buffer[12:16])[0]
            return ('Receiver', timestamp, channel, pulse_length), buffer[16:]
        else:
            # Invalid flag: skip 1 byte to slide
            print(f"Invalid Receiver flag: {flag:02x}", file=sys.stderr)
            return None, buffer[1:]
    
    elif tag == 3:  # Vbat: 2 + 8 + 4 = 14 bytes
        if len(buffer) < 14:
            return None, buffer
        timestamp, voltage = struct.unpack('<Qf', buffer[2:14])
        return ('Vbat', timestamp, voltage), buffer[14:]
    
    elif tag == 4:  # WifiControl: 2 + 8 + 1 + 12 (3*f32) = 23 bytes
        if len(buffer) < 23:
            return None, buffer
        timestamp, mode, r, m, t = struct.unpack('<QBfff', buffer[2:23])
        return ('WifiControl', timestamp, mode, r, m, t), buffer[23:]

    elif tag == 5:  # Planner: 2 + 8 + 12 (3*f32 for plan) + 24 (6*f32 for vectors) + 4 (theta) + 4 (rpm) = 54 bytes
        if len(buffer) < 54:
            return None, buffer
        timestamp, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm = struct.unpack('<Qfffffffffff', buffer[2:54])
        return ('Planner', timestamp, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm), buffer[54:]

    elif tag == 6:  # Stats: 2 + 8 + 8 + 8 + 8 = 34 bytes
        if len(buffer) < 34:
            return None, buffer
        timestamp, min_dur, max_dur, avg_dur = struct.unpack('<QQQQ', buffer[2:34])
        return ('Stats', timestamp, min_dur, max_dur, avg_dur), buffer[34:]
    
    else:
        # Invalid tag: skip 1 byte to slide/resync
        print(f"Skipped byte (invalid tag peek): {buffer[0]:02x}", file=sys.stderr)
        return None, buffer[1:]

def process_events(event_buffer, current_log, last_ts, n, timestamp_str, total_events, log_dir):
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
                log_path = f"{log_dir}/kasarisw_{timestamp_str}_{n}.log"
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

BATCH_FLUSH_SIZE = 4096
BATCH_DATA_SIZE = BATCH_FLUSH_SIZE - 4

def download_logs(host, timestamp_str, log_dir):
    if host == "dummy":
        return [], []
    port = 8081
    sock = None
    for attempt in range(60):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10)
            sock.connect((host, port))
            sock.settimeout(None)
            print(f"Connected to {host}:{port} for download")
            break
        except Exception as e:
            print(f"Download connection attempt {attempt+1} failed: {e}. Retrying in 1 second...")
            time.sleep(1)
    else:
        print("Failed to connect for download after 60 attempts.")
        return [], []

    batch_buffer = b''
    event_buffer = b''
    total_bytes = 0
    total_events = 0
    last_progress = time.time()

    os.makedirs(log_dir, exist_ok=True)
    n = 1
    current_log = None
    last_ts = None
    log_files = []
    panics = []

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
                print("---")
                print("PANIC:", msg)
                print("---")
                panics.append(msg)
                if current_log is None:
                    log_path = f"{log_dir}/kasarisw_{timestamp_str}_{n}.log"
                    current_log = open(log_path, 'w')
                    print(f"Starting new log file: {log_path}")
                    log_files.append(log_path)
                json.dump(['Panic', int(time.time() * 1000000), msg], current_log)
                current_log.write('\n')
                current_log.flush()
                batch_buffer = batch_buffer[BATCH_FLUSH_SIZE:]
                continue
            data = batch[4:]
            event_buffer += data
            batch_buffer = batch_buffer[BATCH_FLUSH_SIZE:]

            event_buffer, current_log, last_ts, n, total_events = process_events(
                event_buffer, current_log, last_ts, n, timestamp_str, total_events, log_dir
            )
            if current_log and current_log.name not in log_files:
                log_files.append(current_log.name)

    event_buffer, current_log, last_ts, n, total_events = process_events(
        event_buffer, current_log, last_ts, n, timestamp_str, total_events, log_dir
    )
    if current_log and current_log.name not in log_files:
        log_files.append(current_log.name)

    if len(event_buffer) > 0:
        print(f"Incomplete data at end: {len(event_buffer)} bytes remaining")

    sock.close()
    print(f"Download complete. Total bytes: {total_bytes}, total events: {total_events}")
    return log_files, panics

class CombatWizard:
    def __init__(self, root):
        self.root = root
        self.root.title("Combat Wizard")
        self.root.geometry("1100x900")
        
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        
        self.latest_data = defaultdict(lambda: "No data yet")
        
        self.host_port = None
        self.host = None
        self.running = False
        self.thread = None
        self.sock = None
        
        self.autonomous_active = False
        
        self.timestamp_str = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        self.realtime_log_path = None
        self.log_file = None
        
        self.step = 1
        
        self.events = []
        self.match_stats = {}
        self.rpm_data = []
        self.vbat_data = []
        self.events_per_sec = defaultdict(int)
        self.panics = []
        
        self.create_widgets()
        
        self.update_gui()

    def create_widgets(self):
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.main_frame.rowconfigure(0, weight=1)
        self.main_frame.rowconfigure(1, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        
        self.status_frame = ttk.LabelFrame(self.main_frame, text="Robot Status", padding="10")
        self.status_frame.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        self.status_frame.rowconfigure(0, weight=1)
        self.status_frame.columnconfigure(0, weight=1)
        
        self.create_status_widgets(self.status_frame)
        
        self.workflow_frame = ttk.LabelFrame(self.main_frame, text="Workflow", padding="10")
        self.workflow_frame.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        self.workflow_frame.rowconfigure(0, weight=1)
        self.workflow_frame.columnconfigure(0, weight=1)
        
        self.update_workflow()

    def create_status_widgets(self, frame):
        self.status_label = ttk.Label(frame, text="Status: Disconnected")
        self.status_label.grid(row=0, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Lidar:").grid(row=1, column=0, sticky=tk.W)
        self.lidar_label = ttk.Label(frame, text=self.format_lidar())
        self.lidar_label.grid(row=2, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Accelerometer:").grid(row=3, column=0, sticky=tk.W)
        self.accel_label = ttk.Label(frame, text=self.format_accel())
        self.accel_label.grid(row=4, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Receiver:").grid(row=5, column=0, sticky=tk.W)
        self.receiver_label = ttk.Label(frame, text=self.format_receiver())
        self.receiver_label.grid(row=6, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Vbat:").grid(row=7, column=0, sticky=tk.W)
        self.vbat_label = ttk.Label(frame, text=self.format_vbat())
        self.vbat_label.grid(row=8, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Planner:").grid(row=9, column=0, sticky=tk.W)
        self.planner_label = ttk.Label(frame, text=self.format_planner())
        self.planner_label.grid(row=10, column=0, columnspan=3, sticky=tk.W)
        
        ttk.Label(frame, text="Stats:").grid(row=11, column=0, sticky=tk.W)
        self.stats_label = ttk.Label(frame, text=self.format_stats())
        self.stats_label.grid(row=12, column=0, columnspan=3, sticky=tk.W)

    def update_workflow(self):
        for widget in self.workflow_frame.winfo_children():
            widget.destroy()
        
        if self.step == 1:
            self.step_choose_address()
        elif self.step == 2:
            self.step_connecting()
        elif self.step == 3:
            self.step_prepare_arena()
        elif self.step == 4:
            self.step_battle_mode()
        elif self.step == 5:
            self.step_downloading()
        elif self.step == 6:
            self.status_frame.grid_remove()
            self.workflow_frame.grid(row=0, rowspan=2, sticky=(tk.N, tk.S, tk.E, tk.W))
            self.step_final_stats()

    def step_choose_address(self):
        ttk.Label(self.workflow_frame, text="Choose robot address:").grid(row=0, column=0, sticky=tk.W)
        self.host_entry = ttk.Combobox(self.workflow_frame, values=["192.168.2.1", "192.168.1.248", "127.0.0.1", "dummy"])
        self.host_entry.set("192.168.2.1")
        self.host_entry.grid(row=0, column=1, sticky=tk.W)
        
        ttk.Button(self.workflow_frame, text="Connect", command=self.start_connect).grid(row=0, column=2, sticky=tk.W)

    def start_connect(self):
        self.host = self.host_entry.get()
        self.log_dir = 'sim_log' if self.host == '127.0.0.1' else 'log'
        os.makedirs(self.log_dir, exist_ok=True)
        self.host_port = f"{self.host}:8080"
        if self.host == "dummy":
            self.running = True
            self.thread = threading.Thread(target=self.dummy_streaming, daemon=True)
            self.thread.start()
            status_queue.put("Connected (dummy)")
            self.step = 3
            self.update_workflow()
        else:
            self.step = 2
            self.update_workflow()
            self.connect()

    def dummy_streaming(self):
        while self.running:
            ts = int(time.time() * 1000000)
            event = ('Vbat', ts, 12.0)
            event_queue.put(event)
            time.sleep(1)

    def step_connecting(self):
        base_text = "Power on the robot and connect to its WiFi network.\nWaiting for connection..."
        guidance = ""

        if self.host == "192.168.2.1":
            guidance = "\n\nTo connect to the robot's access point (AP mode):\n- SSID: kasarisw\n- No password\n- Set a static IP on your device: 192.168.2.2\n- Subnet mask: 255.255.255.0\n- Gateway: 192.168.2.1\n- DNS: 8.8.8.8 (optional)"
        elif self.host == "127.0.0.1":
            guidance = "\n\nThis is localhost (127.0.0.1). Ensure the simulator is running with the --listen flag enabled.\nStart the simulator in a terminal: ./target/debug/simulator --sim --listen"
        else:
            guidance = "\n\nThis is a custom IP (" + self.host + ")\n"

        ttk.Label(self.workflow_frame, text=base_text + guidance).grid(row=0, column=0, sticky=tk.W)

    def step_prepare_arena(self):
        ttk.Label(self.workflow_frame, text="Remove the robot's safety covers and close the arena door.").grid(row=0, column=0, sticky=tk.W)
        ttk.Button(self.workflow_frame, text="Confirm", command=self.confirm_prepare).grid(row=1, column=0, sticky=tk.W)

    def confirm_prepare(self):
        self.step = 4
        self.update_workflow()
        self.start_logging()

    def step_battle_mode(self):
        style = ttk.Style()
        style.configure('Large.TButton', font=('Helvetica', 20), padding=10)

        self.limited_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.workflow_frame, text="Limited Streaming", variable=self.limited_var).grid(row=0, column=0, sticky=tk.W)
        
        self.autonomous_button = ttk.Button(self.workflow_frame, text="Autonomous Mode: OFF", command=self.toggle_autonomous, style='Large.TButton')
        self.autonomous_button.grid(row=1, column=0, sticky=tk.NSEW, pady=10)

        ttk.Button(self.workflow_frame, text="End Match", command=self.end_match).grid(row=2, column=0, sticky=tk.W)

    def toggle_autonomous(self):
        self.autonomous_active = not self.autonomous_active
        if self.autonomous_active:
            self.autonomous_button.config(text="Autonomous Mode: ON")
        else:
            self.autonomous_button.config(text="Autonomous Mode: OFF")

    def end_match(self):
        self.autonomous_active = False
        self.autonomous_button.config(text="Autonomous Mode: OFF")
        if self.log_file:
            self.realtime_log_path = self.log_file.name
            self.log_file.close()
            self.log_file = None
        self.disconnect()
        self.compute_stats()
        self.step = 5
        self.update_workflow()
        threading.Thread(target=self.perform_download, daemon=True).start()

    def compute_stats(self):
        self.rpm_data = []
        self.vbat_data = []
        self.events_per_sec = defaultdict(int)
        if not self.events:
            self.match_stats = {
                'duration': 0,
                'avg_rpm': 0,
                'min_vbat': 0,
                'max_vbat': 0,
                'num_lidar': 0,
            }
            return
        
        first_ts = self.events[0][1]
        last_ts = self.events[-1][1]
        duration = (last_ts - first_ts) / 1_000_000.0
        
        rpms = []
        vbats = []
        num_lidar = 0
        
        for event in self.events:
            rel_ts = (event[1] - first_ts) / 1_000_000.0
            bin_sec = int(rel_ts)
            self.events_per_sec[bin_sec] += 1
            
            if event[0] == 'Planner':
                rpm = event[12]
                rpms.append(rpm)
                self.rpm_data.append((rel_ts, rpm))
            elif event[0] == 'Vbat':
                voltage = event[2]
                vbats.append(voltage)
                self.vbat_data.append((rel_ts, voltage))
            elif event[0] == 'Lidar':
                num_lidar += 1
        
        avg_rpm = sum(rpms) / len(rpms) if rpms else 0
        min_vbat = min(vbats) if vbats else 0
        max_vbat = max(vbats) if vbats else 0
        
        self.match_stats = {
            'duration': duration,
            'avg_rpm': avg_rpm,
            'min_vbat': min_vbat,
            'max_vbat': max_vbat,
            'num_lidar': num_lidar,
        }

    def perform_download(self):
        log_files, panics = download_logs(self.host, self.timestamp_str, self.log_dir)
        self.log_files = [self.realtime_log_path] + log_files if self.realtime_log_path else log_files
        self.panics = panics
        self.step = 6
        self.root.after(0, self.update_workflow)

    def step_downloading(self):
        ttk.Label(self.workflow_frame, text="Downloading logs...").grid(row=0, column=0, sticky=tk.W)

    def step_final_stats(self):
        canvas = tk.Canvas(self.workflow_frame)
        scrollbar = ttk.Scrollbar(self.workflow_frame, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        canvas.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.E, tk.W))
        self.workflow_frame.rowconfigure(0, weight=1)
        self.workflow_frame.columnconfigure(0, weight=1)
        
        inner_frame = ttk.Frame(canvas)
        canvas.create_window((0,0), window=inner_frame, anchor="nw")
        
        def _on_frame_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
        inner_frame.bind("<Configure>", _on_frame_configure)
        
        row = 0
        ttk.Label(inner_frame, text="Match Complete. Log files:").grid(row=row, column=0, sticky=tk.W)
        row += 1
        for file in self.log_files or []:
            ttk.Label(inner_frame, text=file).grid(row=row, column=0, sticky=tk.W)
            row += 1
        
        ttk.Label(inner_frame, text="Statistics:").grid(row=row, column=0, sticky=tk.W)
        row += 1
        ttk.Label(inner_frame, text=f"Match Duration: {self.match_stats['duration']:.2f} seconds").grid(row=row, column=0, sticky=tk.W)
        row += 1
        ttk.Label(inner_frame, text=f"Average RPM: {self.match_stats['avg_rpm']:.2f}").grid(row=row, column=0, sticky=tk.W)
        row += 1
        ttk.Label(inner_frame, text=f"Min Battery Voltage: {self.match_stats['min_vbat']:.2f} V").grid(row=row, column=0, sticky=tk.W)
        row += 1
        ttk.Label(inner_frame, text=f"Max Battery Voltage: {self.match_stats['max_vbat']:.2f} V").grid(row=row, column=0, sticky=tk.W)
        row += 1
        ttk.Label(inner_frame, text=f"Number of LIDAR Events: {self.match_stats['num_lidar']}").grid(row=row, column=0, sticky=tk.W)
        row += 1
        
        if self.panics:
            ttk.Label(inner_frame, text="Panics:", foreground="red").grid(row=row, column=0, sticky=tk.W)
            row += 1
            for panic in self.panics:
                ttk.Label(inner_frame, text=panic, foreground="red").grid(row=row, column=0, sticky=tk.W)
                row += 1
        
        ttk.Label(inner_frame, text="RPM over time:").grid(row=row, column=0, sticky=tk.W)
        row += 1
        rpm_canvas = tk.Canvas(inner_frame, width=GRAPH_WIDTH, height=GRAPH_HEIGHT, bg="white")
        rpm_canvas.grid(row=row, column=0, sticky=tk.W)
        self.plot_line(rpm_canvas, self.rpm_data, "blue")
        row += 1
        
        ttk.Label(inner_frame, text="Vbat over time:").grid(row=row, column=0, sticky=tk.W)
        row += 1
        vbat_canvas = tk.Canvas(inner_frame, width=GRAPH_WIDTH, height=GRAPH_HEIGHT, bg="white")
        vbat_canvas.grid(row=row, column=0, sticky=tk.W)
        self.plot_line(vbat_canvas, self.vbat_data, "red")
        row += 1
        
        ttk.Label(inner_frame, text="Events per second:").grid(row=row, column=0, sticky=tk.W)
        row += 1
        events_canvas = tk.Canvas(inner_frame, width=GRAPH_WIDTH, height=GRAPH_HEIGHT, bg="white")
        events_canvas.grid(row=row, column=0, sticky=tk.W)
        self.plot_events(events_canvas)
        row += 1

    def plot_line(self, canvas, data, color):
        if not data:
            canvas.create_text(GRAPH_WIDTH / 2, GRAPH_HEIGHT / 2, text=f"No data")
            return
        
        data = sorted(data)
        times = [t for t, v in data]
        values = [v for t, v in data]
        min_t = 0
        max_t = self.match_stats['duration']
        min_v = min(values) if values else 0
        max_v = max(values) if values else 0
        if min_v == max_v:
            min_v -= 1
            max_v += 1
        
        height = GRAPH_HEIGHT
        width = GRAPH_WIDTH
        margin_left = GRAPH_MARGIN_LEFT
        margin_bottom = GRAPH_MARGIN_BOTTOM
        margin_top = GRAPH_MARGIN_TOP
        margin_right = GRAPH_MARGIN_RIGHT
        plot_w = width - margin_left - margin_right
        plot_h = height - margin_bottom - margin_top
        
        # Axes
        canvas.create_line(margin_left, height - margin_bottom, width - margin_right, height - margin_bottom)
        canvas.create_line(margin_left, height - margin_bottom, margin_left, margin_top)
        
        # X labels
        for i in range(5):
            x = margin_left + i * plot_w / 4
            t = min_t + i * (max_t - min_t) / 4
            canvas.create_text(x, height - margin_bottom + GRAPH_LABEL_OFFSET, text=f"{t:.0f}s")
            canvas.create_line(x, height - margin_bottom, x, height - margin_bottom + GRAPH_TICK_LENGTH)
        
        # Y labels
        for i in range(5):
            y = height - margin_bottom - i * plot_h / 4
            v = min_v + i * (max_v - min_v) / 4
            canvas.create_text(margin_left - GRAPH_LABEL_OFFSET, y, text=f"{v:.1f}", anchor=tk.E)
            canvas.create_line(margin_left - GRAPH_TICK_LENGTH, y, margin_left, y)
        
        # Data line
        points = []
        for t, v in data:
            x = margin_left + (t - min_t) / (max_t - min_t) * plot_w if max_t > min_t else margin_left
            y = height - margin_bottom - (v - min_v) / (max_v - min_v) * plot_h if max_v > min_v else height - margin_bottom
            points.extend([x, y])
        if points:
            canvas.create_line(*points, fill=color)

    def plot_events(self, canvas):
        max_time = int(self.match_stats['duration']) + 1
        counts = [self.events_per_sec.get(i, 0) for i in range(max_time)]
        if not counts:
            canvas.create_text(GRAPH_WIDTH / 2, GRAPH_HEIGHT / 2, text="No events data")
            return
        
        min_c = 0
        max_c = max(counts)
        if max_c == 0:
            max_c = 1
        
        height = GRAPH_HEIGHT
        width = GRAPH_WIDTH
        margin_left = GRAPH_MARGIN_LEFT
        margin_bottom = GRAPH_MARGIN_BOTTOM
        margin_top = GRAPH_MARGIN_TOP
        margin_right = GRAPH_MARGIN_RIGHT
        plot_w = width - margin_left - margin_right
        plot_h = height - margin_bottom - margin_top
        
        # Axes
        canvas.create_line(margin_left, height - margin_bottom, width - margin_right, height - margin_bottom)
        canvas.create_line(margin_left, height - margin_bottom, margin_left, margin_top)
        
        # X labels
        for i in range(5):
            x = margin_left + i * plot_w / 4
            t = i * (max_time) / 4
            canvas.create_text(x, height - margin_bottom + GRAPH_LABEL_OFFSET, text=f"{t:.0f}s")
            canvas.create_line(x, height - margin_bottom, x, height - margin_bottom + GRAPH_TICK_LENGTH)
        
        # Y labels
        for i in range(5):
            y = height - margin_bottom - i * plot_h / 4
            v = min_c + i * (max_c - min_c) / 4
            canvas.create_text(margin_left - GRAPH_LABEL_OFFSET, y, text=f"{v:.0f}", anchor=tk.E)
            canvas.create_line(margin_left - GRAPH_TICK_LENGTH, y, margin_left, y)
        
        # Bars
        bar_width = plot_w / max_time if max_time > 0 else 1
        for i, c in enumerate(counts):
            x1 = margin_left + i * bar_width
            y1 = height - margin_bottom
            y2 = y1 - (c - min_c) / (max_c - min_c) * plot_h if max_c > min_c else y1
            canvas.create_rectangle(x1, y1, x1 + bar_width, y2, fill="green")

    def start_logging(self):
        log_path = f"./{self.log_dir}/kasarisw_{self.timestamp_str}.log"
        self.log_file = open(log_path, 'w')

    def connect(self):
        self.running = True
        self.thread = threading.Thread(target=self.streaming, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        status_queue.put("Disconnected")

    def streaming(self):
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10)
                host, port_str = self.host_port.split(':')
                port = int(port_str)
                self.sock.connect((host, port))
                self.sock.settimeout(None)
                status_queue.put("Connected")
                if self.step == 2:
                    self.step = 3
                    self.root.after(0, self.update_workflow)
                buffer = b''
                while self.running:
                    chunk = self.sock.recv(1024)
                    if not chunk:
                        raise EOFError("Connection closed by server")
                    buffer += chunk
                    while len(buffer) >= 1:
                        old_len = len(buffer)
                        event, buffer = parse_event(buffer)
                        if event is not None:
                            event_queue.put(event)
                            if self.log_file:
                                json.dump(event, self.log_file)
                                self.log_file.write('\n')
                                self.log_file.flush()
                        else:
                            if len(buffer) >= old_len:
                                break
            except Exception as e:
                if self.running:
                    status_queue.put(f"Error: {str(e)} - Retrying in 5 seconds...")
                    time.sleep(5)
            finally:
                if self.sock:
                    try:
                        self.sock.close()
                    except:
                        pass
                    self.sock = None

    def format_lidar(self):
        if self.latest_data['Lidar'] == "No data yet":
            return "No data yet"
        ts, d1, d2, d3, d4 = self.latest_data['Lidar']
        return f"Timestamp: {ts}, d1: {d1:.2f}, d2: {d2:.2f}, d3: {d3:.2f}, d4: {d4:.2f}"

    def format_accel(self):
        if self.latest_data['Accelerometer'] == "No data yet":
            return "No data yet"
        ts, accel_y, accel_z = self.latest_data['Accelerometer']
        return f"Timestamp: {ts}, Acceleration Y: {accel_y:.2f}, Z: {accel_z:.2f}"

    def format_receiver(self):
        if self.latest_data['Receiver'] == "No data yet":
            return "No data yet"
        ts, channel, value = self.latest_data['Receiver']
        if value is None:
            return f"Timestamp: {ts}, Channel: {channel}, Flag: 0 (None)"
        else:
            return f"Timestamp: {ts}, Channel: {channel}, Pulse Length: {value:.2f}"

    def format_vbat(self):
        if self.latest_data['Vbat'] == "No data yet":
            return "No data yet"
        ts, voltage = self.latest_data['Vbat']
        return f"Timestamp: {ts}, Voltage: {voltage:.2f}"

    def format_planner(self):
        if self.latest_data['Planner'] == "No data yet":
            return "No data yet"
        ts, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm = self.latest_data['Planner']
        return (f"Timestamp: {ts}, Rotation: {rotation_speed:.2f}, Movement: ({movement_x:.2f}, {movement_y:.2f}), "
                f"Closest Wall: ({cw_x:.2f}, {cw_y:.2f}), Open Space: ({os_x:.2f}, {os_y:.2f}), "
                f"Object: ({op_x:.2f}, {op_y:.2f}), Theta: {theta:.2f}, "
                f"RPM measurement: {rpm:.2f}")

    def format_stats(self):
        if self.latest_data['Stats'] == "No data yet":
            return "No data yet"
        ts, step_min_duration_us, step_max_duration_us, step_avg_duration_us = self.latest_data['Stats']
        return f"Timestamp: {ts}, Step duration min: {step_min_duration_us}, avg: {step_avg_duration_us}, max: {step_max_duration_us}"

    def serialize_event(self, event):
        sensor_type, *data = event
        tag = EVENT_TAGS.get(sensor_type)
        if tag is None:
            raise ValueError("Unknown event type")
        ts = data[0]
        if sensor_type == 'WifiControl':
            mode, r, m, t = data[1:]
            return struct.pack('<BQBfff', tag, ts, mode, r, m, t)
        else:
            raise ValueError("Serialization only implemented for WifiControl")

    def update_gui(self):
        while not event_queue.empty():
            event = event_queue.get()
            self.events.append(event)
            sensor_type = event[0]
            if sensor_type == 'Lidar':
                self.latest_data['Lidar'] = event[1:]
            elif sensor_type == 'Accelerometer':
                self.latest_data['Accelerometer'] = event[1:]
            elif sensor_type == 'Receiver':
                self.latest_data['Receiver'] = event[1:]
            elif sensor_type == 'Vbat':
                self.latest_data['Vbat'] = event[1:]
            elif sensor_type == 'Planner':
                self.latest_data['Planner'] = event[1:]
            elif sensor_type == 'Stats':
                self.latest_data['Stats'] = event[1:]
        
        while not status_queue.empty():
            msg = status_queue.get()
            self.status_label.config(text=f"Status: {msg}")
        
        self.lidar_label.config(text=self.format_lidar())
        self.accel_label.config(text=self.format_accel())
        self.receiver_label.config(text=self.format_receiver())
        self.vbat_label.config(text=self.format_vbat())
        self.planner_label.config(text=self.format_planner())
        self.stats_label.config(text=self.format_stats())
        
        if self.sock:
            ts = int(time.time() * 1000000)
            mode = 3 if self.autonomous_active and self.limited_var.get() else 2 if self.autonomous_active else 0
            r = m = t = 0.0
            event = ('WifiControl', ts, mode, r, m, t)
            try:
                serialized = self.serialize_event(event)
                self.sock.sendall(serialized)
            except Exception as e:
                status_queue.put(f"Send error: {e}")
        
        self.root.after(100, self.update_gui)

if __name__ == "__main__":
    root = tk.Tk()
    app = CombatWizard(root)
    root.mainloop()
