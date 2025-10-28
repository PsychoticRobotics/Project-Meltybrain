import socket
import struct
import tkinter as tk
from tkinter import ttk
import threading
import time
from collections import defaultdict
import queue
from parse_events import parse_event
import json
import os
from datetime import datetime
import argparse

# Shared queues for events and status (thread-safe)
event_queue = queue.Queue()
status_queue = queue.Queue()

# Event tags (assumed based on common patterns; adjust if necessary)
EVENT_TAGS = {
    'Lidar': 0,
    'Accelerometer': 1,
    'Receiver': 2,
    'Vbat': 3,
    'WifiControl': 4,  # Assuming this tag for WifiControl
    'Planner': 5,
    'Stats': 6,
}

# GUI class
class SensorGUI:
    def __init__(self, root, enable_manual=False):
        self.root = root
        self.root.title("Sensor Data Monitor")
        self.root.geometry("1100x900")
        
        # Make the root grid expandable
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        
        # Dictionary to hold latest data
        self.latest_data = defaultdict(lambda: "No data yet")
        
        # Connection management
        self.host_port = "192.168.2.1:8080"  # Default host:port
        self.running = False
        self.thread = None
        self.sock = None
        
        # Control states
        self.enable_manual = enable_manual
        self.control_active = False
        self.autonomous_active = False
        
        if self.enable_manual:
            # Slider variables
            self.r_var = tk.DoubleVar(value=0.0)
            self.m_var = tk.DoubleVar(value=0.0)
            self.t_var = tk.DoubleVar(value=0.0)
        
        # Log file
        os.makedirs('log', exist_ok=True)
        timestamp = datetime.now().strftime('%Y-%m-%d_%H%M%S')
        log_path = f"./log/kasarisw_{timestamp}.log"
        self.log_file = open(log_path, 'w')
        
        # Create widgets
        self.create_widgets()
        
        # Bind resize event
        self.frame.bind("<Configure>", self.on_resize)
        
        # Start updating GUI
        self.update_gui()

    def create_widgets(self):
        self.frame = ttk.Frame(self.root, padding="10")
        self.frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Make frame columns expandable if needed, but primarily rely on wrap
        self.frame.columnconfigure(0, weight=1)
        self.frame.columnconfigure(1, weight=1)
        self.frame.columnconfigure(2, weight=1)
        
        # Connection controls
        ttk.Label(self.frame, text="Host:Port:").grid(row=0, column=0, sticky=tk.W)
        self.host_entry = ttk.Combobox(self.frame, values=["192.168.2.1:8080", "192.168.1.248:8080", "127.0.0.1:8080"])
        self.host_entry.set("192.168.2.1:8080")
        self.host_entry.grid(row=0, column=1, sticky=tk.W)
        
        self.connect_button = ttk.Button(self.frame, text="Connect", command=self.toggle_connect)
        self.connect_button.grid(row=0, column=2, sticky=tk.W)
        
        self.status_label = ttk.Label(self.frame, text="Status: Disconnected")
        self.status_label.grid(row=1, column=0, columnspan=3, sticky=tk.W)
        
        # Lidar section
        ttk.Label(self.frame, text="Lidar:").grid(row=2, column=0, sticky=tk.W)
        self.lidar_label = ttk.Label(self.frame, text=self.format_lidar())
        self.lidar_label.grid(row=3, column=0, columnspan=3, sticky=tk.W)
        
        # Accelerometer section
        ttk.Label(self.frame, text="Accelerometer:").grid(row=4, column=0, sticky=tk.W)
        self.accel_label = ttk.Label(self.frame, text=self.format_accel())
        self.accel_label.grid(row=5, column=0, columnspan=3, sticky=tk.W)
        
        # Receiver section (latest one, regardless of channel)
        ttk.Label(self.frame, text="Receiver:").grid(row=6, column=0, sticky=tk.W)
        self.receiver_label = ttk.Label(self.frame, text=self.format_receiver())
        self.receiver_label.grid(row=7, column=0, columnspan=3, sticky=tk.W)
        
        # Vbat section
        ttk.Label(self.frame, text="Vbat:").grid(row=8, column=0, sticky=tk.W)
        self.vbat_label = ttk.Label(self.frame, text=self.format_vbat())
        self.vbat_label.grid(row=9, column=0, columnspan=3, sticky=tk.W)
        
        # Planner section
        ttk.Label(self.frame, text="Planner:").grid(row=10, column=0, sticky=tk.W)
        self.planner_label = ttk.Label(self.frame, text=self.format_planner())
        self.planner_label.grid(row=11, column=0, columnspan=3, sticky=tk.W)
        
        # Stats section
        ttk.Label(self.frame, text="Stats:").grid(row=12, column=0, sticky=tk.W)
        self.stats_label = ttk.Label(self.frame, text=self.format_stats())
        self.stats_label.grid(row=13, column=0, columnspan=3, sticky=tk.W)
        
        # Large toggle button for control (if enabled)
        style = ttk.Style()
        style.configure('Large.TButton', font=('Helvetica', 20), padding=10)
        
        next_row = 14
        if self.enable_manual:
            self.control_button = ttk.Button(self.frame, text="Control Target: OFF", command=self.toggle_control, style='Large.TButton')
            self.control_button.grid(row=next_row, column=0, columnspan=3, sticky=tk.NSEW, pady=10)
            next_row += 1
        
        # Autonomous mode button
        self.autonomous_button = ttk.Button(self.frame, text="Autonomous Mode: OFF", command=self.toggle_autonomous, style='Large.TButton')
        self.autonomous_button.grid(row=next_row, column=0, columnspan=3, sticky=tk.NSEW, pady=10)
        next_row += 1
        
        # Sliders for r, m, t (if enabled)
        if self.enable_manual:
            ttk.Label(self.frame, text="Rotation:").grid(row=next_row, column=0, sticky=tk.W)
            self.r_scale = ttk.Scale(self.frame, from_=-2000.0, to=2000.0, orient='horizontal', variable=self.r_var)
            self.r_scale.grid(row=next_row, column=1, columnspan=2, sticky=tk.EW)
            next_row += 1
            
            ttk.Label(self.frame, text="Movement:").grid(row=next_row, column=0, sticky=tk.W)
            self.m_scale = ttk.Scale(self.frame, from_=-100.0, to=100.0, orient='horizontal', variable=self.m_var)
            self.m_scale.grid(row=next_row, column=1, columnspan=2, sticky=tk.EW)
            next_row += 1
            
            ttk.Label(self.frame, text="Turning:").grid(row=next_row, column=0, sticky=tk.W)
            self.t_scale = ttk.Scale(self.frame, from_=-100.0, to=100.0, orient='horizontal', variable=self.t_var)
            self.t_scale.grid(row=next_row, column=1, columnspan=2, sticky=tk.EW)
            
            # Initially disable sliders
            self.set_sliders_state('disabled')

    def on_resize(self, event):
        # Update wraplength based on current frame width
        width = event.width - 40  # Subtract padding
        if width > 0:
            self.status_label.config(wraplength=width)
            self.lidar_label.config(wraplength=width)
            self.accel_label.config(wraplength=width)
            self.receiver_label.config(wraplength=width)
            self.vbat_label.config(wraplength=width)
            self.planner_label.config(wraplength=width)
            self.stats_label.config(wraplength=width)

    def toggle_connect(self):
        if self.connect_button.cget("text") == "Connect":
            self.connect()
        else:
            self.disconnect()

    def connect(self):
        self.host_port = self.host_entry.get()
        self.host, port_str = self.host_port.split(':')
        self.port = int(port_str)
        self.status_label.config(text="Status: Connecting...")
        status_queue.put("Connecting...")
        self.running = True
        self.thread = threading.Thread(target=self.streaming, daemon=True)
        self.thread.start()
        self.connect_button.config(text="Disconnect")

    def disconnect(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self.status_label.config(text="Status: Disconnected")
        status_queue.put("Disconnected")
        self.connect_button.config(text="Connect")

    def streaming(self):
        while self.running:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(10)
                self.sock.connect((self.host, self.port))
                self.sock.settimeout(None)
                status_queue.put("Connected")
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

    def toggle_control(self):
        self.control_active = not self.control_active
        if self.control_active:
            self.control_button.config(text="Control Target: ON")
            self.autonomous_active = False
            self.autonomous_button.config(text="Autonomous Mode: OFF")
            self.set_sliders_state('normal')
        else:
            self.control_button.config(text="Control Target: OFF")
            self.set_sliders_state('disabled')

    def toggle_autonomous(self):
        if self.control_active:
            return  # Cannot activate autonomous if control is active
        self.autonomous_active = not self.autonomous_active
        if self.autonomous_active:
            self.autonomous_button.config(text="Autonomous Mode: ON")
        else:
            self.autonomous_button.config(text="Autonomous Mode: OFF")

    def set_sliders_state(self, state):
        self.r_scale.config(state=state)
        self.m_scale.config(state=state)
        self.t_scale.config(state=state)

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
        # Process all queued events
        while not event_queue.empty():
            event = event_queue.get()
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
        
        # Process status updates
        while not status_queue.empty():
            msg = status_queue.get()
            self.status_label.config(text=f"Status: {msg}")
        
        # Update sensor labels
        self.lidar_label.config(text=self.format_lidar())
        self.accel_label.config(text=self.format_accel())
        self.receiver_label.config(text=self.format_receiver())
        self.vbat_label.config(text=self.format_vbat())
        self.planner_label.config(text=self.format_planner())
        self.stats_label.config(text=self.format_stats())
        
        # Send WifiControl event if connected
        if self.sock:
            ts = int(time.time() * 1000000)  # Example timestamp in microseconds; adjust to match Rust ticks if needed
            if self.control_active:
                mode = 1
                r = self.r_var.get()
                m = self.m_var.get()
                t = self.t_var.get()
            elif self.autonomous_active:
                mode = 2
                r = m = t = 0.0
            else:
                mode = 0
                r = m = t = 0.0
            event = ('WifiControl', ts, mode, r, m, t)
            try:
                serialized = self.serialize_event(event)
                self.sock.sendall(serialized)
            except Exception as e:
                status_queue.put(f"Send error: {e}")
        
        # Schedule next update
        self.root.after(100, self.update_gui)

# Main entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Sensor Data Monitor")
    parser.add_argument('--enable-manual', action='store_true', help='Enable manual control button and sliders')
    args = parser.parse_args()

    root = tk.Tk()
    app = SensorGUI(root, enable_manual=args.enable_manual)
    root.mainloop()
