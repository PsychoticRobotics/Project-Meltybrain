// sources.rs
use crate::events::{get_ts, parse_event};
use crate::physics::{Rect, Robot, World};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use kasarisw::shared::algorithm::{BIN_ANGLE_STEP, NUM_BINS};
use kasarisw::shared::kasari;
use kasarisw::shared::kasari::{InputEvent, MainLogic, MotorControlPlan};
use kasarisw::shared::rem_euclid_f32;
use rand::distributions::Uniform;
use rand::prelude::*;
use rand::rngs::StdRng;
use static_cell::StaticCell;
use std::collections::VecDeque;
use std::error::Error;
use std::f32::consts::PI;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Error as IoError};
use std::iter::Iterator;

pub trait EventSource {
    fn peek_next_ts(&mut self) -> Option<u64>;
    fn get_next_event(&mut self) -> Option<InputEvent>;
    fn get_robot(&self) -> Option<&Robot> {
        None
    }
    fn get_robot_mut(&mut self) -> Option<&mut Robot> {
        None
    }
    fn get_world(&self) -> Option<&World> {
        None
    }
    fn get_logic(&self) -> Option<&MainLogic> {
        None
    }
    fn get_logic_mut(&mut self) -> Option<&mut MainLogic> {
        None
    }
    fn get_robot_flipped(&self) -> bool {
        false
    }
    fn set_robot_flipped(&mut self, robot_flipped: bool) {}
    fn inject_event(&mut self, event: InputEvent) {
        self.get_logic_mut().unwrap().feed_event(event);
    }
}

struct ReplayMirror {
    logic: MainLogic,
    debug: bool,
}

impl ReplayMirror {
    pub fn new(debug: bool, reverse_rotation: bool) -> Self {
        Self {
            logic: MainLogic::new(reverse_rotation),
            debug: debug,
        }
    }
    pub fn process_event(&mut self, timestamp: u64, event: &InputEvent) {
        self.logic.feed_event(event.clone());
    }
}

pub struct FileEventSource {
    lines: Box<dyn Iterator<Item = Result<String, IoError>>>,
    next_real_event: Option<InputEvent>,
    last_read_ts: Option<u64>,
    inject_autonomous: bool,
    first_lidar_found: bool,
    next_inject_ts: Option<u64>,
    lidar_distance_offset: f32,
    mirror: ReplayMirror,
    debug: bool,
}

impl FileEventSource {
    pub fn new(
        lines: Box<dyn Iterator<Item = Result<String, IoError>>>,
        debug: bool,
        inject_autonomous: bool,
        lidar_distance_offset: f32,
        reverse_rotation: bool,
    ) -> Self {
        Self {
            lines,
            next_real_event: None,
            last_read_ts: None,
            inject_autonomous,
            first_lidar_found: false,
            next_inject_ts: None,
            lidar_distance_offset,
            mirror: ReplayMirror::new(debug, reverse_rotation),
            debug,
        }
    }

    pub fn read_next_real(&mut self) -> bool {
        loop {
            if let Some(line_res) = self.lines.next() {
                match line_res {
                    Ok(line) => {
                        if line.trim().is_empty() {
                            continue;
                        }
                        match parse_event(&line) {
                            Ok(mut event) => {
                                if self.inject_autonomous
                                    && matches!(&event, InputEvent::WifiControl(..))
                                {
                                    continue;
                                }
                                let mut adjusted_ts = get_ts(&event);
                                if matches!(&event, InputEvent::WifiControl(..)) {
                                    if let Some(last) = self.last_read_ts {
                                        if ((adjusted_ts as i128 - last as i128).abs() > 5_000_000)
                                        {
                                            adjusted_ts = last + 1000;
                                        }
                                    }
                                }
                                // Update event ts or other parameters
                                event = match event {
                                    InputEvent::Lidar(_, d1, d2, d3, d4) => {
                                        let o = self.lidar_distance_offset;
                                        InputEvent::Lidar(
                                            adjusted_ts,
                                            d1 + o,
                                            d2 + o,
                                            d3 + o,
                                            d4 + o,
                                        )
                                    }
                                    InputEvent::Accelerometer(_, ay, az) => {
                                        InputEvent::Accelerometer(adjusted_ts, ay, az)
                                    }
                                    InputEvent::Receiver(_, ch, pulse) => {
                                        InputEvent::Receiver(adjusted_ts, ch, pulse)
                                    }
                                    InputEvent::Vbat(_, voltage) => {
                                        InputEvent::Vbat(adjusted_ts, voltage)
                                    }
                                    InputEvent::WifiControl(_, mode, r, m, t) => {
                                        InputEvent::WifiControl(adjusted_ts, mode, r, m, t)
                                    }
                                    InputEvent::Planner(_, plan, cw, os, op, theta, rpm) => {
                                        InputEvent::Planner(
                                            adjusted_ts,
                                            plan,
                                            cw,
                                            os,
                                            op,
                                            theta,
                                            rpm,
                                        )
                                    }
                                    InputEvent::Stats(ts, stats) => InputEvent::Stats(ts, stats),
                                };
                                self.last_read_ts = Some(adjusted_ts);
                                if !self.first_lidar_found
                                    && matches!(&event, InputEvent::Lidar(..))
                                {
                                    self.first_lidar_found = true;
                                    if self.inject_autonomous {
                                        self.next_inject_ts = Some(adjusted_ts + 100_000);
                                    }
                                }
                                self.next_real_event = Some(event);
                                return true;
                            }
                            Err(e) => {
                                eprintln!("Skipping invalid line: {} (error: {})", line.trim(), e);
                                continue;
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Error reading line: {}", e);
                        return false;
                    }
                }
            } else {
                return false;
            }
        }
    }
}

impl EventSource for FileEventSource {
    fn peek_next_ts(&mut self) -> Option<u64> {
        if self.next_real_event.is_none() {
            if !self.read_next_real() {
                return self.next_inject_ts;
            }
        }
        match (
            self.next_real_event.as_ref().map(get_ts),
            self.next_inject_ts,
        ) {
            (None, None) => None,
            (Some(rt), None) => Some(rt),
            (None, Some(it)) => Some(it),
            (Some(rt), Some(it)) => Some(rt.min(it)),
        }
    }

    fn get_next_event(&mut self) -> Option<InputEvent> {
        if self.next_real_event.is_none() {
            self.read_next_real();
        }
        let next_real_ts = self.next_real_event.as_ref().map(get_ts);
        let next_inj_ts = self.next_inject_ts;

        let next_ts_opt = match (next_real_ts, next_inj_ts) {
            (None, None) => return None,
            (Some(rt), None) => Some(rt),
            (None, Some(it)) => Some(it),
            (Some(rt), Some(it)) => Some(rt.min(it)),
        };

        if next_ts_opt.is_some() {
            let is_inject_next =
                next_inj_ts.map_or(false, |it| next_real_ts.map_or(true, |rt| it <= rt));

            let event = if is_inject_next {
                let ts = next_inj_ts.unwrap();
                self.next_inject_ts = Some(ts + 100_000);
                InputEvent::WifiControl(ts, 2, 0.0, 0.0, 0.0)
            } else {
                self.next_real_event.take().unwrap()
            };

            self.mirror.process_event(next_ts_opt.unwrap(), &event);

            if self.debug {
                match event {
                    InputEvent::Lidar(ts, d1, d2, d3, d4) => println!(
                        "Mirrored Lidar ts={} d=({:.0},{:.0},{:.0},{:.0}) theta={:.4} rpm={:.2}",
                        ts,
                        d1,
                        d2,
                        d3,
                        d4,
                        self.mirror.logic.detector.theta,
                        self.mirror.logic.detector.rpm
                    ),
                    InputEvent::Accelerometer(ts, ay, az) => println!(
                        "Mirrored Accel ts={} ay={:.2} az={:.2} theta={:.4} rpm={:.2}",
                        ts,
                        ay,
                        az,
                        self.mirror.logic.detector.theta,
                        self.mirror.logic.detector.rpm
                    ),
                    _ => {}
                }
            }

            self.mirror
                .logic
                .step(next_ts_opt.unwrap(), None, self.debug);

            Some(event)
        } else {
            None
        }
    }

    fn get_logic(&self) -> Option<&MainLogic> {
        Some(&self.mirror.logic)
    }
    fn get_logic_mut(&mut self) -> Option<&mut MainLogic> {
        Some(&mut self.mirror.logic)
    }
}

static CHANNEL: StaticCell<PubSubChannel<CriticalSectionRawMutex, InputEvent, 32, 3, 6>> =
    StaticCell::new();

pub struct SimEventSource {
    control_logic: MainLogic,
    control_publisher:
        embassy_sync::pubsub::Publisher<'static, CriticalSectionRawMutex, InputEvent, 32, 3, 6>,
    control_subscriber:
        embassy_sync::pubsub::Subscriber<'static, CriticalSectionRawMutex, InputEvent, 32, 3, 6>,
    event_buffer: VecDeque<InputEvent>,
    modulator: kasari::MotorModulator,
    robot: Robot,
    world: World,
    last_advance_ts: u64,
    last_step_ts: u64,
    next_vbat_ts: u64,
    next_accel_ts: u64,
    next_lidar_ts: u64,
    last_lidar_ts: u64,
    next_wifi_ts: u64,
    lidar_distance_offset: f32,
    debug: bool,
    accelerometer_event_count: u64,
    rng: StdRng,
    robot_flipped: bool,
    auto_inject_wifi: bool,
}

impl SimEventSource {
    pub fn new(
        lidar_distance_offset: f32,
        debug: bool,
        arena_width: f32,
        arena_height: f32,
        no_object: bool,
        reverse_rotation: bool,
        robot_flipped: bool,
        auto_inject_wifi: bool,
    ) -> Self {
        let channel = &*CHANNEL.init(PubSubChannel::new());
        let mut source = Self {
            control_logic: MainLogic::new(reverse_rotation),
            control_publisher: channel.publisher().unwrap(),
            control_subscriber: channel.subscriber().unwrap(),
            event_buffer: VecDeque::new(),
            modulator: kasari::MotorModulator::new(),
            robot: Robot {
                pos_x: 0.0,
                pos_y: 0.0,
                vel_x: 0.0,
                vel_y: 0.0,
                theta: 0.0,
                rpm: 0.0,
            },
            world: World {
                arena: Rect {
                    min_x: -arena_width / 2.0,
                    min_y: -arena_height / 2.0,
                    max_x: arena_width / 2.0,
                    max_y: arena_height / 2.0,
                },
                objects: if !no_object {
                    vec![Rect {
                        min_x: 200.0,
                        min_y: 200.0,
                        max_x: 300.0,
                        max_y: 300.0,
                    }]
                } else {
                    vec![]
                },
            },
            last_advance_ts: 0,
            last_step_ts: 0,
            next_vbat_ts: 200_000,
            next_accel_ts: 10_000,
            next_lidar_ts: 2083,
            last_lidar_ts: 0,
            next_wifi_ts: if auto_inject_wifi { 100_000 } else { u64::MAX },
            lidar_distance_offset,
            debug,
            accelerometer_event_count: 0,
            rng: StdRng::seed_from_u64(42),
            robot_flipped,
            auto_inject_wifi,
        };

        if auto_inject_wifi {
            // Initial events at ts=0
            let wifi_event = InputEvent::WifiControl(0, 2, 0.0, 0.0, 0.0);
            source.event_buffer.push_back(wifi_event.clone());
            source.control_logic.feed_event(wifi_event);
        }

        let vbat_event = InputEvent::Vbat(0, 12.0);
        source.event_buffer.push_back(vbat_event.clone());
        source.control_logic.feed_event(vbat_event);

        source
    }

    pub fn update_robot_physics(&mut self, dt: f32) {
        self.modulator.step(self.last_advance_ts);
        const MAX_RPM_CHANGE_PER_S: f32 = 2000.0; // Limited by wheel friction
        self.robot.rpm = self
            .modulator
            .mcp
            .map_or(0.0, |p| p.rotation_speed)
            .max(self.robot.rpm - MAX_RPM_CHANGE_PER_S * dt)
            .min(self.robot.rpm + MAX_RPM_CHANGE_PER_S * dt);
        let movement_x = self
            .control_logic
            .motor_control_plan
            .map_or(0.0, |p| p.movement_x);
        let movement_y = self
            .control_logic
            .motor_control_plan
            .map_or(0.0, |p| p.movement_y);
        let angular_correction_plus_flipped_state =
            self.control_logic.angular_correction_total + if self.robot_flipped { PI } else { 0.0 };
        self.robot.update(
            dt,
            movement_x,
            movement_y,
            &self.world,
            self.control_logic.detector.theta,
            angular_correction_plus_flipped_state,
        );
    }

    pub fn ensure_buffer_has_event(&mut self) {
        while self.event_buffer.is_empty() {
            let next_ts = self.last_advance_ts + 2083; // Advance at lidar event rate
            self.advance_to_ts(next_ts);
        }
    }

    pub fn advance_to_ts(&mut self, target_ts: u64) {
        while self.last_advance_ts < target_ts {
            let next_vbat = self.next_vbat_ts;
            let next_accel = self.next_accel_ts;
            let next_lidar = self.next_lidar_ts;
            let next_wifi = if self.auto_inject_wifi {
                self.next_wifi_ts
            } else {
                u64::MAX
            };
            let next_step = self.last_step_ts + 20_000;

            let next_ts = [
                next_vbat,
                next_accel,
                next_lidar,
                next_wifi,
                next_step,
                target_ts + 1,
            ]
            .iter()
            .cloned()
            .min()
            .unwrap();

            // Advance physics
            let dt = (next_ts - self.last_advance_ts) as f32 / 1_000_000.0;
            self.update_robot_physics(dt);

            self.last_advance_ts = next_ts;

            if next_vbat <= next_ts {
                let event = InputEvent::Vbat(next_vbat, 12.0);
                self.event_buffer.push_back(event.clone());
                self.control_logic.feed_event(event);
                self.next_vbat_ts += 200_000;
            }

            if next_accel <= next_ts {
                let omega = (self.robot.rpm / 60.0 * 2.0 * PI).abs();
                let a_g = (omega * omega * 0.0145) / 9.81;
                // Allow calibration during the first 50 samples
                // Add bias similarly to what the real hardware has
                let ay = if next_ts <= 500_000 { 0.0 } else { a_g } + 3.4;
                // Simulate gravity plus noise
                let range = Uniform::from(-4.0..4.0);
                let az = 1.0 + self.rng.sample(&range);
                let event = InputEvent::Accelerometer(next_accel, ay, az);
                self.event_buffer.push_back(event.clone());
                self.control_logic.feed_event(event);
                self.next_accel_ts += 10_000;
                self.accelerometer_event_count += 1;
            }

            if next_lidar <= next_ts {
                let distance_noise = if self.rng.sample(&Uniform::from(0.0..1.0)) > 0.9 {
                    if self.rng.sample(&Uniform::from(0.0..1.0)) > 0.9 {
                        self.rng.sample(&Uniform::from(-500.0..500.0))
                    } else {
                        self.rng.sample(&Uniform::from(-50.0..50.0))
                    }
                } else {
                    0.0
                };
                let angle_noise = if self.rng.sample(&Uniform::from(0.0..1.0)) > 0.9 {
                    if self.rng.sample(&Uniform::from(0.0..1.0)) > 0.9 {
                        self.rng.sample(&Uniform::from(-0.5 * PI..0.5 * PI))
                    } else {
                        self.rng.sample(&Uniform::from(-0.05 * PI..0.05 * PI))
                    }
                } else {
                    0.0
                };
                let delta_t_sec = (next_lidar - self.last_lidar_ts) as f32 / 1_000_000.0;
                let delta_theta = self.robot.rpm / 60.0 * 2.0 * PI * delta_t_sec;
                let step_theta = delta_theta / 4.0;
                let mut distances = [0.0; 4];
                for i in 0..4 {
                    let angle =
                        rem_euclid_f32(self.robot.theta - ((3 - i) as f32 * step_theta), 2.0 * PI)
                            + angle_noise;
                    let dir_x = angle.cos();
                    let dir_y = angle.sin();
                    let true_dist =
                        self.world
                            .raycast(self.robot.pos_x, self.robot.pos_y, dir_x, dir_y);
                    distances[i] = (true_dist - self.lidar_distance_offset).max(0.0);
                    distances[i] += distance_noise;
                }
                let event = InputEvent::Lidar(
                    next_lidar,
                    distances[0],
                    distances[1],
                    distances[2],
                    distances[3],
                );
                self.event_buffer.push_back(event.clone());
                self.control_logic.feed_event(event);
                self.last_lidar_ts = next_lidar;
                self.next_lidar_ts += 2083;
            }

            if self.auto_inject_wifi && next_wifi <= next_ts {
                let event = InputEvent::WifiControl(next_wifi, 2, 0.0, 0.0, 0.0);
                self.event_buffer.push_back(event.clone());
                self.control_logic.feed_event(event);
                self.next_wifi_ts += 100_000;
            }

            if next_step <= next_ts {
                self.control_logic
                    .step(next_step, Some(&mut self.control_publisher), self.debug);
                while let Some(event) = self.control_subscriber.try_next_message_pure() {
                    self.event_buffer.push_back(event.clone());
                    if let InputEvent::Planner(ts, plan, _, _, _, _, _) = &event {
                        self.modulator.sync(
                            *ts,
                            self.control_logic.detector.theta,
                            plan.clone(),
                            self.control_logic.angular_correction_total,
                        );
                    }
                }
                self.last_step_ts = next_step;
            }
        }
    }
}

impl EventSource for SimEventSource {
    fn peek_next_ts(&mut self) -> Option<u64> {
        self.ensure_buffer_has_event();
        self.event_buffer.front().map(get_ts)
    }

    fn get_next_event(&mut self) -> Option<InputEvent> {
        self.ensure_buffer_has_event();
        self.event_buffer.pop_front()
    }

    fn get_robot(&self) -> Option<&Robot> {
        Some(&self.robot)
    }

    fn get_robot_mut(&mut self) -> Option<&mut Robot> {
        Some(&mut self.robot)
    }

    fn get_world(&self) -> Option<&World> {
        Some(&self.world)
    }

    fn get_logic(&self) -> Option<&MainLogic> {
        Some(&self.control_logic)
    }

    fn get_logic_mut(&mut self) -> Option<&mut MainLogic> {
        Some(&mut self.control_logic)
    }

    fn get_robot_flipped(&self) -> bool {
        self.robot_flipped
    }
    fn set_robot_flipped(&mut self, robot_flipped: bool) {
        self.robot_flipped = robot_flipped
    }
}
