import sys
import matplotlib
import matplotlib.pyplot as plt
import math
import time
import statistics
import select

# Force TkAgg backend for reliable window handling
matplotlib.use('TkAgg')

class LidarData:
    def __init__(self, frame_delay=0.01, persistence_scans=1):
        self.DATA_LENGTH = 7  # angle, speed, distance 1-4, checksum
        self.MAX_DISTANCE = 1000  # in mm
        self.MIN_DISTANCE = 50   # in mm
        self.MAX_DATA_SIZE = 360  # resolution: 1 degree
        self.PACKET_SIZE = 22     # size of each packet in bytes
        self.HEAD_BYTE = 0xFA     # packet header
        self.frame_delay = frame_delay  # delay between frames in seconds (10x faster)
        self.persistence_scans = persistence_scans  # persist points for 1 scan
        self.running = True  # Flag to control main loop

        # Data storage: map angles (in radians) to (distance, scan_count)
        self.data_buffer = {}  # {angle_rad: (distance_mm, scan_count)}
        self.current_scan = 0  # Track full 360° scans
        self.last_angle = None  # Track last processed angle to detect new scan

        # Setup plot with doubled size
        self.fig = plt.figure(figsize=(12.8, 9.6))  # 2x default size (6.4x4.8 -> 12.8x9.6 inches)
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_rmax(self.MAX_DISTANCE)
        plt.ion()  # Enable interactive mode
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        print("Initialized polar plot with TkAgg backend, size=12.8x9.6 inches")

    def on_close(self, event):
        """Handle plot window closure."""
        print("Plot window closed, stopping script")
        self.running = False

    def decode_packet(self, packet):
        """Decode a 22-byte packet into angle, speed, distances, and checksum."""
        if len(packet) != self.PACKET_SIZE:
            print(f"Invalid packet length: {len(packet)} bytes (expected {self.PACKET_SIZE})")
            return None
        if packet[0] != self.HEAD_BYTE:
            print(f"Invalid header: {hex(packet[0])} (expected {hex(self.HEAD_BYTE)})")
            return None

        data = [0] * self.DATA_LENGTH
        data_idx = 0

        # Angle (byte 1)
        angle = (packet[1] - 0xA0) * 4
        if angle > 360:
            print(f"Invalid angle: {angle} degrees (exceeds 360)")
            return None
        data[data_idx] = angle
        data_idx += 1

        # Speed (bytes 2 and 3)
        speed = (packet[3] << 8) | packet[2]
        speed = speed // 64  # Convert to RPM
        data[data_idx] = speed
        data_idx += 1

        # Distances (4 sets of 4 bytes starting at indices 4, 8, 12, 16)
        for i in [4, 8, 12, 16]:
            distance = ((packet[i + 1] & 0x3F) << 8) | packet[i]
            data[data_idx] = distance
            data_idx += 1

        # Checksum (bytes 20 and 21)
        received_sum = (packet[21] << 8) | packet[20]
        checksum_valid = self.checksum(packet, received_sum, self.PACKET_SIZE - 2)
        data[data_idx] = checksum_valid

        print(f"Decoded packet: angle={angle}°, speed={speed} RPM, distances={data[2:6]}, checksum_valid={checksum_valid}")
        return data

    def checksum(self, packet, received_sum, size):
        """Calculate and verify checksum."""
        chk32 = 0
        sensor_data = packet[:size]
        data = []
        for i in range(0, size, 2):
            word = (sensor_data[i + 1] << 8) | sensor_data[i]
            data.append(word)
            chk32 = (chk32 << 1) + word

        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        is_valid = (checksum & 0x7FFF) == received_sum
        if not is_valid:
            print(f"Checksum failed: calculated={hex(checksum & 0x7FFF)}, received={hex(received_sum)}")
        return is_valid

    def plot_data(self):
        """Plot data with persistence and fading effect."""
        if not self.running:
            return

        angles, distances, alphas = [], [], []
        for angle_rad, (dist, scan_count) in self.data_buffer.items():
            if self.current_scan - scan_count < self.persistence_scans:
                angles.append(angle_rad)
                distances.append(dist)
                # Fade older points: alpha from 1.0 (newest) to 0.3 (oldest)
                alpha = 1.0 - 0.7 * (self.current_scan - scan_count) / max(1, self.persistence_scans)
                alphas.append(max(0.3, min(1.0, alpha)))

        if angles:
            self.ax.clear()
            self.ax.scatter(angles, distances, c='blue', alpha=alphas, s=10)
            self.ax.set_rmax(self.MAX_DISTANCE)
            plt.draw()
            plt.pause(0.001)
            print(f"Plotted {len(angles)} points from {self.current_scan - min(self.data_buffer.values(), default=(0,0))[1] + 1} scans")
        else:
            print("No valid points to plot")

    def process_stdin(self):
        """Read and process raw serial data from stdin with timeout."""
        buffer = bytearray()
        packet_count = 0

        while self.running:
            try:
                if select.select([sys.stdin.buffer], [], [], 0.1)[0]:
                    byte = sys.stdin.buffer.read(1)
                    if not byte:
                        print(f"End of input, processed {packet_count} packets")
                        self.running = False
                        break
                    byte = int.from_bytes(byte, 'little')

                    if byte == self.HEAD_BYTE:
                        buffer = bytearray([byte])
                        packet_bytes = sys.stdin.buffer.read(self.PACKET_SIZE - 1)
                        if len(packet_bytes) == self.PACKET_SIZE - 1:
                            buffer.extend(packet_bytes)
                            packet = list(buffer)
                            packet_count += 1
                            print(f"\nProcessing packet #{packet_count}: {''.join([f'{b:02x}' for b in packet])}")
                            data = self.decode_packet(packet)
                            if data:
                                angle = data[0]
                                speed = data[1]
                                distances = data[2:6]
                                checksum_valid = data[6]

                                if checksum_valid:
                                    # Detect new scan (full 360° rotation)
                                    if self.last_angle is not None and angle < self.last_angle:
                                        self.current_scan += 1
                                        print(f"New scan detected: scan #{self.current_scan}")
                                    self.last_angle = angle

                                    for i, dist in enumerate(distances):
                                        if self.MIN_DISTANCE <= dist <= self.MAX_DISTANCE:
                                            interpolated_angle = (angle + i) * math.pi / 180
                                            self.data_buffer[interpolated_angle] = (dist, self.current_scan)
                                            print(f"Updated point: angle={(angle + i)}°, distance={dist} mm, scan={self.current_scan}")
                                        else:
                                            print(f"Skipped distance={dist} mm (outside {self.MIN_DISTANCE}-{self.MAX_DISTANCE} mm)")

                                    self.plot_data()
                                    plt.pause(self.frame_delay)
                        else:
                            print(f"Incomplete packet: only {len(packet_bytes)} bytes after header")
                plt.pause(0.001)
            except KeyboardInterrupt:
                print(f"Interrupted by user, processed {packet_count} packets")
                self.running = False
                break
            except Exception as e:
                print(f"Error processing packet: {e}", file=sys.stderr)
                continue

        print("Exiting main loop")
        plt.close('all')

if __name__ == '__main__':
    frame_delay = 0.0013   # Delay per packet
    persistence_scans = 1  # Persist points for number of full scans

    try:
        sensor = LidarData(frame_delay, persistence_scans)
        sensor.process_stdin()
        plt.show(block=False)
        print("Script terminated")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        plt.close('all')

