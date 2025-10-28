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
    def __init__(self, frame_delay=0.1):
        self.DATA_LENGTH = 7  # angle, speed, distance 1-4, checksum
        self.MAX_DISTANCE = 3000  # in mm
        self.MIN_DISTANCE = 100   # in mm
        self.MAX_DATA_SIZE = 360  # resolution: 1 degree
        self.PACKET_SIZE = 22     # size of each packet in bytes
        self.HEAD_BYTE = 0xFA     # packet header
        self.frame_delay = frame_delay  # delay between frames in seconds
        self.running = True  # Flag to control main loop

        # Sensor data storage
        self.data = {
            'angles': [],
            'distances': [],
            'speed': [],
            'checksum': []
        }

        # Setup plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ax.set_rmax(self.MAX_DISTANCE)
        plt.ion()  # Enable interactive mode
        self.fig.canvas.mpl_connect('close_event', self.on_close)  # Handle window close
        print("Initialized polar plot with TkAgg backend")

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
            word = (sensor_data[i + 1] << 8) + sensor_data[i]
            data.append(word)
            chk32 = (chk32 << 1) + word

        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        is_valid = (checksum & 0x7FFF) == received_sum
        if not is_valid:
            print(f"Checksum failed: calculated={hex(checksum & 0x7FFF)}, received={hex(received_sum)}")
        return is_valid

    def plot_data(self):
        """Plot data on a polar plot with outlier filtering."""
        if not self.running:
            return
        angles, distances = [], []

        print(f"Attempting to plot {len(self.data['angles'])} data points")
        if len(self.data['angles']) < 7:
            print("Not enough data points for outlier filtering (<7), plotting raw data")
            angles = self.data['angles']
            distances = self.data['distances']
        else:
            for p in range(3, len(self.data['angles']) - 3):
                sample = self.data['distances'][p - 3:p + 3]
                try:
                    std = statistics.stdev(sample)
                    mean = statistics.mean(sample)
                    if abs(self.data['distances'][p] - mean) < std:
                        angles.append(self.data['angles'][p])
                        distances.append(self.data['distances'][p])
                    else:
                        print(f"Filtered out point at angle={self.data['angles'][p]*180/math.pi:.1f}°, distance={self.data['distances'][p]} mm (outside std={std:.1f})")
                except statistics.StatisticsError:
                    print("Statistics error in outlier filtering, skipping point")
                    continue

        if not angles:
            print("No points passed outlier filter, plotting raw data as fallback")
            angles = self.data['angles']
            distances = self.data['distances']

        if angles:
            self.ax.clear()
            self.ax.plot(angles, distances, ".")
            self.ax.set_rmax(self.MAX_DISTANCE)
            plt.draw()
            plt.pause(0.001)
            print(f"Plotted {len(angles)} points")
        else:
            print("No valid points to plot")

    def process_stdin(self):
        """Read and process raw serial data from stdin with timeout."""
        buffer = bytearray()
        packet_count = 0

        while self.running:
            try:
                # Use select to implement a timeout for stdin reading
                if select.select([sys.stdin.buffer], [], [], 0.1)[0]:  # 100ms timeout
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
                                    for i, dist in enumerate(distances):
                                        if self.MIN_DISTANCE <= dist <= self.MAX_DISTANCE:
                                            interpolated_angle = (angle + i) * math.pi / 180
                                            self.data['angles'].append(interpolated_angle)
                                            self.data['distances'].append(dist)
                                            self.data['speed'].append(speed)
                                            self.data['checksum'].append(checksum_valid)
                                            print(f"Added point: angle={(angle + i)}°, distance={dist} mm")
                                        else:
                                            print(f"Skipped distance={dist} mm (outside {self.MIN_DISTANCE}-{self.MAX_DISTANCE} mm)")

                                    self.plot_data()
                                    self.data['angles'].clear()
                                    self.data['distances'].clear()
                                    self.data['speed'].clear()
                                    self.data['checksum'].clear()
                                    plt.pause(self.frame_delay)
                        else:
                            print(f"Incomplete packet: only {len(packet_bytes)} bytes after header")
                # Allow other events (e.g., GUI, interrupts) to process
                plt.pause(0.001)
            except KeyboardInterrupt:
                print(f"Interrupted by user, processed {packet_count} packets")
                self.running = False
                break
            except Exception as e:
                print(f"Error processing packet: {e}", file=sys.stderr)
                continue

        print("Exiting main loop")
        plt.close('all')  # Ensure all plot windows are closed

if __name__ == '__main__':
    frame_delay = 0.1  # Delay between frames in seconds

    try:
        sensor = LidarData(frame_delay)
        sensor.process_stdin()
        plt.show(block=False)  # Non-blocking show to allow cleanup
        print("Script terminated")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        plt.close('all')
