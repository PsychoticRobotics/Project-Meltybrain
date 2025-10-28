# Kasarisw: Spinning Robot with LIDAR Navigation

## Overview
This project features a spinning robot using a repurposed LDS02RR LIDAR (fixed
mount, fake encoder for max sampling) to enable autonomous navigation and
object following/wall avoidance. The robot spins its body (500-2000 RPM),
measures rotation via ADXL373 accelerometer, and achieves directional movement
through differential motor RPM modulation.

- **Firmware**: ESP32-based, handles sensors, motion planning, WiFi event streaming (binary over TCP).
- **Simulator**: PC GUI (egui) for log replay or virtual world simulation; visualizes LIDAR points, detections, behavior and collisions.
- **Shared Logic**: LIDAR/accel processing, binning (90 bins, 4° each), object detection (walls/objects via averages/dips).
- **Python GUI**: `event_monitor.py` for real-time monitoring, logging (JSON), controls over TCP.
- **Goals**: Compare real vs. simulated behavior; optimize non-standard LIDAR setup.

The robot maintains two rotations: physical (LIDAR orientation, integrated from
RPM) and virtual (stabilized relative to the world, limited by sensor
accuracy). In the simulator, the robot is rendered with its virtual rotation
fixed (pointing +X), keeping the world mostly static but adjusted for
measurement offsets/errors—useful for analyzing behavior. It produces event
logs over WiFi (captured via `event_monitor.py`), which can be replayed in the
simulator to compare real vs. simulated behavior, visualizing LIDAR points,
detections, and planner outputs.

## Simulator Architecture Notes
The system uses a MainLogic class shared between the embedded target and the
simulator for event processing and motion planning. In simulation mode (--sim),
SimEventSource owns the authoritative MainLogic to generate events (e.g.,
Lidar, Accelerometer, Planner) and drive physics. In replay mode (log file),
FileEventSource handles event sourcing (including synthetic injections like
--inject-autonomous) and uses a ReplayMirror (wrapping MainLogic) to process
events for consistent visualization. MyApp queries the EventSource trait for
logic data (e.g., detector, plans) uniformly, avoiding duplication. Flags like
--reverse-rotation are applied to each source's logic for RPM consistency.
--listen allows controlling the simulator using the same tools used to control
the robot over wifi.

## Project Structure
- `src/embedded.rs`: ESP32 firmware entrypoint.
- `src/sensors.rs`: Sensor tasks (LIDAR UART parsing, accel SPI, RC RMT).
- `src/shared/mod.rs`: Core logic (MainLogic, MotorModulator, event handling).
- `src/shared/algorithm.rs`: ObjectDetector (binning, RPM from accel, detection via window averages/large changes).
- `src/simulator/mod.rs`: Simulator entrypoint.
- `src/simulator/app.rs`: Egui app for visualization.
- `src/simulator/events.rs`: Event parsing from JSON logs.
- `src/simulator/physics.rs`: Virtual world raycasting, robot physics.
- `src/simulator/sources.rs`: Event sources (file replay or simulation).
- `tools/event_monitor.py`: Tkinter GUI for monitoring/controls/logging.
- `tools/download_events.py`: CLI for downloading events from robot's flash storage
- `tools/parse_events.py`: CLI binary event parser to JSON.
- `tools/combat_wizard.py`: Tkinter GUI for combat workflow. Combines event_monitor.py and download_events.py functionalities into one tool
- `rust-toolchain.toml`: Specifies ESP toolchain (channel: esp, target: xtensa-esp32-none-elf).
- `config.toml`: Cargo config with target rustflags, unstable build-std, aliases (build-pc, build-esp, run-esp).

## Key Algorithms
- **RPM from Accel**: `a_calibrated = accel_g - offset; omega = sqrt(|a| / r); rpm = (omega * 60) / (2π)` (r=0.0145m).
- **LIDAR Binning**: 90 bins; theta integrated from RPM/dt; distances binned with interpolation/fill.
- **Detection**: Window averages (5-15 bins); find min/max for wall/open; objects via large diffs/dips; fallback protrusion check.
- **Modulation**: Base RPM + amplitude * cos(theta - phase) for directional movement.
- **Autonomous**: Blend vectors: away from wall + toward open/object; clamp mag 0.5-1.0.

## Prerequisites
- Rust 1.88 (stable); project uses rust-toolchain.toml for ESP target.
- PC: No special setup.
- Embedded: Python 3 (tkinter for tools); install ESP32 toolchain:
  ```sh
  rm -rf ~/.rustup/toolchains/esp  # Clean install (optional)
  rustup override unset
  cargo install espup
  espup install
  cp $HOME/export-esp.sh .  # Copy to project root
  ```
- Hardware: ESP32, LDS02RR (RX only), ADXL373 (SPI), motors/ESCs (PWM), battery (ADC).

## Build/Run
### PC Simulator
```sh
cargo +stable build-pc  # Alias for cargo build --features pc
./target/debug/simulator [OPTIONS] [SOURCE]  # SOURCE: log file or --sim for virtual and --listen for listening to control tools
```
Options: --debug, --inject-autonomous, --lidar-distance-offset <MM>, --sim.

Profiling using flamegraph:
```sh
cargo +stable flamegraph -b simulator --features=pc -- --sim --headless --end-time 100
flamegraph --perfdata perf.data --image-width 4096
```

### Embedded Firmware
```sh
. export-esp.sh  # Source toolchain env
export SSID=... PASSWORD=...  # For STA WiFi
cargo build-esp  # Alias for cargo build --target xtensa-esp32-none-elf --features esp
cargo run-esp  # Alias for cargo run --target xtensa-esp32-none-elf --features esp --bin kasarisw (flash/run)
./monitor.sh  # Serial logs
```

## Monitoring/Control
- GUI: `python tools/event_monitor.py` (connect to 192.168.2.1:8080 AP or STA IP:8080).
  - Features: View sensors/planner, toggle modes (1=manual sliders, 2=autonomous), auto-log JSON.
- CLI Download: `python tools/download_events.py [IP:PORT]` (defaults to
  192.168.1.248:8081; downloads flash-stored events, splits into JSON logs).
- CLI: `nc IP 8080 | python tools/parse_events.py > log.json`.
- Events: Binary (tag XOR 0x5555 + payload) over TCP; JSON in logs.

## Hardware Pinout (ESP32)
- Motors: GPIO26/27 (PWM right/left).
- LIDAR: GPIO13 (encoder PWM), GPIO16 (UART RX).
- Accel: GPIO5 (CS), GPIO18 (SCLK), GPIO19 (MISO), GPIO23 (MOSI).
- RC: GPIO34 (RMT PWM in).
- Battery: GPIO39 (ADC).

## Dependencies (Cargo.toml)
- Shared: embassy-sync, ringbuffer, static_cell, arrayvec, num-traits, libm.
- Embedded: esp-hal, esp-wifi, esp-alloc, etc.
- Simulator: clap, eframe/egui_plot, serde_json.

## Event Format
- Lidar: [ts, d1..d4] (mm).
- Accel: [ts, ay, az] (G).
- Receiver: [ts, ch, pulse|null] (us).
- Vbat: [ts, voltage] (V).
- WifiControl: [ts, mode, r, m, t].
- Planner: [ts, rotation_speed, movement_x, movement_y, cw_x, cw_y, os_x, os_y, op_x, op_y, theta, rpm].
