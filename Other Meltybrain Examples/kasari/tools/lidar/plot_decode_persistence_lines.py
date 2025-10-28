import sys
import matplotlib
import matplotlib.pyplot as plt
import math
import time
import argparse
import select

# Force TkAgg backend for reliable window handling
matplotlib.use('TkAgg')

class LidarData:
    def __init__(self, debug=False, persistence_scans=1, packet_batch_size=25, frame_delay=0.0013):
        self.DATA_LENGTH = 7  # angle, speed, distance 1-4, checksum
        self.MAX_DISTANCE = 1000  # in mm
        self.MIN_DISTANCE = 40    # in mm
        self.PACKET_SIZE = 22     # size of each packet in bytes
        self.HEAD_BYTE = 0xFA     # packet header
        self.debug = debug
        self.persistence_scans = persistence_scans  # 1 scan (~0.24 seconds)
        self.packet_batch_size = packet_batch_size  # Plot every 25 packets
        self.frame_delay = frame_delay  # 1.3 ms (~769 FPS)
        self.running = True

        # Data storage: {angle_rad: (distance_mm, timestamp)}
        self.data_buffer = {}
        self.packet_count = 0
        self.total_packet_count = 0
        self.scan_time = 0.24  # Approx. scan duration at 250 RPM
        self.start_time = time.time()

        # Setup plot
        self.fig = plt.figure(figsize=(12.8, 9.6))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_rmax(self.MAX_DISTANCE)
        plt.ion()
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        print("Initialized polar plot with TkAgg backend, size=12.8x9.6 inches")

    def _print(self, message):
        if self.debug:
            print(message)

    def on_close(self, event):
        print("Plot window closed, stopping script")
        self.running = False

    def decode_packet(self, packet):
        if len(packet) != self.PACKET_SIZE or packet[0] != self.HEAD_BYTE:
            return None

        angle = (packet[1] - 0xA0) * 4
        if angle >= 360:
            return None

        speed = ((packet[3] << 8) | packet[2]) // 64
        distances = [
            ((packet[i + 1] & 0x3F) << 8) | packet[i] for i in [4, 8, 12, 16]
        ]
        received_sum = (packet[21] << 8) | packet[20]
        checksum_valid = self.checksum(packet, received_sum)

        if not checksum_valid:
            return None

        self._print(f"Decoded packet: angle={angle}°, speed={speed} RPM, distances={distances}")
        return angle, speed, distances

    def checksum(self, packet, received_sum):
        chk32 = 0
        for i in range(0, self.PACKET_SIZE - 2, 2):
            word = (packet[i + 1] << 8) + packet[i]
            chk32 = (chk32 << 1) + word
        checksum = (chk32 & 0x7FFF) + (chk32 >> 15)
        return (checksum & 0x7FFF) == received_sum

    def plot_data(self):
        if not self.running:
            return

        current_time = time.time()
        persistence_time = self.persistence_scans * self.scan_time
        self.data_buffer = {
            k: v for k, v in self.data_buffer.items()
            if current_time - v[1] < persistence_time
        }
        # Sort points by original angle (0 to 2π) for correct line connections
        points = sorted(self.data_buffer.items(), key=lambda x: x[0])
        angles, distances = [], []
        for angle_rad, (dist, _) in points:
            # Negate angle for clockwise rotation
            plot_angle = -angle_rad
            angles.append(plot_angle)
            distances.append(dist)

        if angles:
            # Append first point to close the loop
            angles.append(angles[0])
            distances.append(distances[0])
            self.ax.clear()
            # Draw lines between points
            self.ax.plot(angles, distances, 'b-', linewidth=1)  # Blue lines
            # Optionally keep scatter for visibility
            self.ax.scatter(angles[:-1], distances[:-1], c='blue', s=10)
            self.ax.set_rmax(self.MAX_DISTANCE)
            plt.draw()
            plt.pause(0.001)
            self._print(f"Plotted {len(angles)-1} points, angles from {min(angles)*180/math.pi:.1f}° to {max(angles)*180/math.pi:.1f}°")
        else:
            self._print("No valid points to plot")

    def process_stdin(self):
        buffer = bytearray()
        last_fps_time = time.time()
        last_fps_count = 0
        input_buffer = bytearray()

        while self.running:
            try:
                if select.select([sys.stdin.buffer], [], [], 0.001)[0]:
                    chunk = sys.stdin.buffer.read(4096)
                    if not chunk:
                        print(f"End of input, processed {self.total_packet_count} packets")
                        self.running = False
                        break
                    input_buffer.extend(chunk)

                while len(input_buffer) >= self.PACKET_SIZE and self.running:
                    if input_buffer[0] == self.HEAD_BYTE:
                        packet = input_buffer[:self.PACKET_SIZE]
                        input_buffer = input_buffer[self.PACKET_SIZE:]
                        self.total_packet_count += 1
                        self.packet_count += 1
                        self._print(f"Processing packet #{self.total_packet_count}: {''.join([f'{b:02x}' for b in packet])}")
                        data = self.decode_packet(packet)
                        if data:
                            angle, speed, distances = data
                            for i, dist in enumerate(distances):
                                if self.MIN_DISTANCE <= dist <= self.MAX_DISTANCE:
                                    interpolated_angle = (angle + i) * math.pi / 180
                                    self.data_buffer[interpolated_angle] = (dist, time.time())
                                    self._print(f"Updated point: angle={(angle + i)}°, distance={dist} mm")

                            if self.packet_count >= self.packet_batch_size:
                                self.plot_data()
                                self.packet_count = 0
                                time.sleep(self.frame_delay)
                    else:
                        try:
                            next_header = input_buffer.index(self.HEAD_BYTE, 1)
                            input_buffer = input_buffer[next_header:]
                        except ValueError:
                            input_buffer = bytearray()
                            break

                current_time = time.time()
                if current_time - last_fps_time >= 1.0:
                    fps = (self.total_packet_count - last_fps_count) / (current_time - last_fps_time)
                    print(f"Current FPS: {fps:.2f} (processed {self.total_packet_count} packets)")
                    last_fps_time = current_time
                    last_fps_count = self.total_packet_count

                plt.pause(0.0001)
            except KeyboardInterrupt:
                print(f"Interrupted by user, processed {self.total_packet_count} packets")
                self.running = False
                break
            except Exception as e:
                print(f"Error processing packet: {e}", file=sys.stderr)
                continue

        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            fps = self.total_packet_count / elapsed_time
            print(f"Processed {self.total_packet_count} packets in {elapsed_time:.2f} seconds ({fps:.1f} FPS)")
        print("Exiting main loop")
        plt.close('all')

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Plot LIDAR data from binary log file")
    parser.add_argument('--debug', action='store_true', help="Enable debug output")
    parser.add_argument('--batch-size', type=int, default=25, help="Number of packets per plot update")
    args = parser.parse_args()

    try:
        sensor = LidarData(debug=args.debug, persistence_scans=1, packet_batch_size=args.batch_size, frame_delay=0.0013)
        sensor.process_stdin()
        plt.show(block=False)
        print("Script terminated")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        plt.close('all')
