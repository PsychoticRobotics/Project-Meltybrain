import sys
import struct

TAG_XOR = 0x5555

def parse_event(buffer):
    """Parse a single event from the buffer, returning the event and remaining buffer."""
    if len(buffer) < 2:  # Min for tag
        return None, buffer
    encoded_tag = struct.unpack('<H', buffer[0:2])[0]
    tag = encoded_tag ^ TAG_XOR
    
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

# Main entry point
if __name__ == "__main__":
    # Initialize an empty buffer
    buffer = b''

    # Main loop to process the stream
    while True:
        # Parse as many as possible, handling skips without premature break
        while len(buffer) >= 1:
            old_len = len(buffer)
            event, buffer = parse_event(buffer)
            if event is not None:
                print(event)
            else:
                if len(buffer) >= old_len:
                    # No progress (incomplete event), wait for more
                    break
                # Else: skipped invalid, continue parsing remaining buffer

        # Read more data from stdin
        chunk = sys.stdin.buffer.read(1024)
        if not chunk:
            break
        buffer += chunk
