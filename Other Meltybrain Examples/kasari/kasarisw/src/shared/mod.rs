// shared/mod.rs
#![cfg_attr(target_os = "none", no_std)]
use core::cell::RefCell;
use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use num_traits::float::FloatCore;
use num_traits::ops::euclid::Euclid;
use ringbuffer::ConstGenericRingBuffer;
use static_cell::StaticCell;

#[cfg(target_os = "none")]
use embassy_time::Instant;
#[cfg(not(target_os = "none"))]
use std::time::{SystemTime, UNIX_EPOCH};

pub mod algorithm;
use algorithm::{DetectionResult, ObjectDetector};

pub const TARGET_RPM: f32 = 1200.0;
pub const MIN_MOVE_RPM: f32 = 550.0;
pub const MIN_ATTACK_RPM: f32 = 800.0;
pub const REVERSE_ROTATION_MAX_RPM: f32 = 80.0;

pub const MOVEMENT_SPEED_CENTER: f32 = 1.3;
pub const MOVEMENT_SPEED_ATTACK: f32 = 1.3;

pub const MAX_RPM_RAMP_RATE: f32 = 1500.0; // rpm/s
pub const RPM_INITIAL_JUMP: f32 = 400.0; // rpm

pub const FAILSAFE_TIMEOUT_US: u64 = 5_000_000;

pub const LOG_LIDAR: bool = false;
pub const LOG_ALL_LIDAR: bool = false;
pub const LOG_RECEIVER: bool = false;
pub const LOG_WIFI_CONTROL: bool = false;
pub const LOG_MOTOR_CONTROL: bool = false;
pub const LOG_VBAT: bool = false;
pub const LOG_DETECTION: bool = false;

pub type EventChannel = PubSubChannel<CriticalSectionRawMutex, kasari::InputEvent, 32, 3, 6>;
pub static EVENT_CHANNEL: StaticCell<EventChannel> = StaticCell::new();

pub fn get_current_timestamp() -> u64 {
    #[cfg(target_os = "none")]
    {
        Instant::now().as_ticks()
    }
    #[cfg(not(target_os = "none"))]
    {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_micros() as u64
    }
}

// We need to define this on our own, because the num-traits one for some reason
// wants &f32 on ESP32 and f32 on PC
pub fn rem_euclid_f32(x: f32, y: f32) -> f32 {
    #[cfg(not(target_os = "none"))]
    {
        x.rem_euclid(y)
    }
    #[cfg(target_os = "none")]
    {
        x.rem_euclid(&y)
    }
}

pub mod kasari {
    use crate::shared::rem_euclid_f32;
    use crate::shared::CriticalSectionRawMutex;
    use crate::shared::ObjectDetector;
    use crate::shared::{
        get_current_timestamp, FAILSAFE_TIMEOUT_US, LOG_DETECTION, LOG_RECEIVER, LOG_VBAT,
        LOG_WIFI_CONTROL, MAX_RPM_RAMP_RATE, MIN_ATTACK_RPM, MIN_MOVE_RPM, MOVEMENT_SPEED_ATTACK,
        MOVEMENT_SPEED_CENTER, REVERSE_ROTATION_MAX_RPM, RPM_INITIAL_JUMP, TARGET_RPM,
    };
    #[cfg(target_os = "none")]
    use alloc::vec::Vec;
    use core::f32::consts::PI;
    #[cfg(target_os = "none")]
    use esp_println::println;
    use libm::{atan2f, cosf, fabsf, sqrtf};
    use num_traits::float::FloatCore;
    #[cfg(not(target_os = "none"))]
    use std::vec::Vec; // powi on esp32

    #[derive(Clone, Debug)]
    pub enum InputEvent {
        Lidar(u64, f32, f32, f32, f32), // timestamp, distance samples (mm)
        Accelerometer(u64, f32, f32),   // timestamp, acceleration Y (G), acceleration Z (G)
        Receiver(u64, u8, Option<f32>), // timestamp, channel (0=throttle), pulse length (us)
        Vbat(u64, f32),                 // timestamp, battery voltage (V)
        WifiControl(u64, u8, f32, f32, f32), // timestamp, mode, rotation speed, movement speed, turning speed
        Planner(
            u64,
            MotorControlPlan,
            (f32, f32),
            (f32, f32),
            (f32, f32),
            f32,
            f32,
        ), // timestamp, MCP, latest_closest_wall, latest_open_space, latest_object_pos, theta, rpm
        Stats(u64, Stats),                   // timestamp, statistics struct
    }

    #[derive(Clone, Copy, Debug)]
    pub struct Stats {
        pub step_min_duration_us: u64,
        pub step_max_duration_us: u64,
        pub step_avg_duration_us: u64,
    }

    #[derive(Clone, Copy, Debug)]
    pub struct MotorControlPlan {
        pub timestamp: u64,
        pub rotation_speed: f32,
        pub movement_x: f32,
        pub movement_y: f32,
    }

    /// Enum for control modes to improve readability and type safety.
    #[derive(Copy, Clone, PartialEq, Eq)]
    pub enum ControlMode {
        RcReceiver = 0,
        ManualWifi = 1,
        Autonomous = 2,
    }

    impl From<u8> for ControlMode {
        fn from(value: u8) -> Self {
            match value {
                1 => Self::ManualWifi,
                2 | 3 => Self::Autonomous,
                _ => Self::RcReceiver,
            }
        }
    }

    impl InputEvent {
        pub fn timestamp(&self) -> u64 {
            match self {
                Self::Lidar(ts, ..) => *ts,
                Self::Accelerometer(ts, ..) => *ts,
                Self::Receiver(ts, ..) => *ts,
                Self::Vbat(ts, ..) => *ts,
                Self::WifiControl(ts, ..) => *ts,
                Self::Planner(ts, ..) => *ts,
                Self::Stats(ts, ..) => *ts,
            }
        }
    }

    /// Struct for holding detection results to separate concerns.
    #[derive(Clone, Copy)]
    pub struct DetectionState {
        pub closest_wall: (f32, f32),
        pub open_space: (f32, f32),
        pub object_pos: (f32, f32),
        pub wall_distances: (f32, f32, f32, f32),
        pub position: (f32, f32),
        pub velocity: (f32, f32),
    }

    impl Default for DetectionState {
        fn default() -> Self {
            Self {
                closest_wall: (0.0, 0.0),
                open_space: (0.0, 0.0),
                object_pos: (0.0, 0.0),
                wall_distances: (0.0, 0.0, 0.0, 0.0),
                position: (0.0, 0.0),
                velocity: (0.0, 0.0),
            }
        }
    }

    /// Struct for holding control targets to make state clearer.
    #[derive(Clone, Copy)]
    pub struct ControlTargets {
        pub rotation_speed: f32,
        pub movement_x: f32,
        pub movement_y: f32,
    }

    impl Default for ControlTargets {
        fn default() -> Self {
            Self {
                rotation_speed: 0.0,
                movement_x: 0.0,
                movement_y: 0.0,
            }
        }
    }

    pub struct MainLogic {
        pub motor_control_plan: Option<MotorControlPlan>,
        pub detector: ObjectDetector,
        pub detection_state: DetectionState,
        pub vbat: f32,
        pub vbat_ok: bool,
        pub battery_present: bool,
        control_mode: ControlMode,
        control_rotation_speed: f32,
        control_movement_speed: f32,
        control_turning_speed: f32,
        last_planner_ts: u64,
        autonomous_enabled: bool,
        autonomous_start_ts: Option<u64>,
        autonomous_cycle_period_us: u64,
        autonomous_duty_cycle: f32,
        last_rpm_update_ts: Option<u64>,
        current_rotation_speed: f32,
        target: ControlTargets,
        pub reverse_rotation: bool,
        reverse_rotation_ts: u64,
        last_receiver_event_ts: u64,
        last_valid_receiver_ts: Option<u64>,
        last_receiver_pulse: f32,
        stats_step_count: u64,
        stats_step_sum_us: u64,
        stats_step_min_us: u64,
        stats_step_max_us: u64,
        pub angular_correction_flip: bool,
        angular_correction_flip_ts: u64,
        pub angular_correction_total: f32,
        pub intended_movement_comparison: f32,
        pub no_intended_movement_timestamp: u64,
        pub intended_movement_velocity_timestamp: u64,
        pub away_from_wall_timestamp: u64,
        pub close_to_wall_timestamp: u64,
        low_rpm_start_ts: Option<u64>,
        last_reset_attempt_ts: u64,
        reset_start_ts: Option<u64>,
        last_wifi_ts: u64,
        last_wifi_mode: u8,
        last_wifi_r: f32,
        last_wifi_m: f32,
        last_wifi_t: f32,
        last_publish_ts: u64,
    }

    impl MainLogic {
        pub fn new(reverse_rotation: bool) -> Self {
            Self {
                motor_control_plan: None,
                detector: ObjectDetector::new(),
                detection_state: DetectionState::default(),
                vbat: 0.0,
                vbat_ok: false,
                battery_present: false,
                control_mode: ControlMode::RcReceiver,
                control_rotation_speed: 0.0,
                control_movement_speed: 0.0,
                control_turning_speed: 0.0,
                last_planner_ts: 0,
                autonomous_enabled: false,
                autonomous_start_ts: None,
                autonomous_cycle_period_us: 8_000_000, // 8 seconds
                autonomous_duty_cycle: 0.70,           // 70% towards center, 30% towards object
                last_rpm_update_ts: None,
                current_rotation_speed: 0.0,
                target: ControlTargets::default(),
                reverse_rotation,
                reverse_rotation_ts: 0,
                last_receiver_event_ts: 0,
                last_valid_receiver_ts: None,
                last_receiver_pulse: 0.0,
                stats_step_count: 0,
                stats_step_sum_us: 0,
                stats_step_min_us: 0,
                stats_step_max_us: 0,
                angular_correction_flip: false,
                angular_correction_flip_ts: 0,
                angular_correction_total: 0.0,
                intended_movement_comparison: 0.0,
                no_intended_movement_timestamp: 0,
                intended_movement_velocity_timestamp: 0,
                away_from_wall_timestamp: 0,
                close_to_wall_timestamp: 0,
                low_rpm_start_ts: None,
                last_reset_attempt_ts: 0,
                reset_start_ts: None,
                last_wifi_ts: 0,
                last_wifi_mode: 0,
                last_wifi_r: 0.0,
                last_wifi_m: 0.0,
                last_wifi_t: 0.0,
                last_publish_ts: 0,
            }
        }

        fn reset_targets(&mut self) {
            self.target = ControlTargets::default();
        }

        /// Central method to update control targets based on current mode and inputs.
        /// This helps avoid duplicated logic in event handlers.
        fn update_targets(&mut self) {
            if !self.vbat_ok {
                self.reset_targets();
                self.motor_control_plan = None;
                return;
            }

            match self.control_mode {
                ControlMode::ManualWifi => {
                    self.target.rotation_speed = self.control_rotation_speed;
                    self.target.movement_x = 0.0;
                    self.target.movement_y = 0.0;
                }
                ControlMode::RcReceiver if !self.autonomous_enabled => {
                    self.reset_targets();
                }
                _ => {} // Autonomous or RC with autonomous: handled in autonomous_update
            }
        }

        pub fn feed_event(&mut self, event: InputEvent) {
            self.detector.update(&event);
            self.detector.rpm = self.detector.rpm.abs() * self.current_rotation_speed.signum();

            match event {
                InputEvent::Lidar(..) | InputEvent::Accelerometer(..) | InputEvent::Planner(..) => {
                }
                InputEvent::Receiver(_timestamp, _ch, pulse_length) => {
                    if _ch == 0 {
                        if LOG_RECEIVER {
                            println!("Receiver pulse_length: {:?}", pulse_length);
                        }
                        self.last_receiver_event_ts = _timestamp;
                        if let Some(pl) = pulse_length {
                            self.last_valid_receiver_ts = Some(_timestamp);
                            self.last_receiver_pulse = pl;
                        }
                    }
                }
                InputEvent::Vbat(_timestamp, vbat) => {
                    self.vbat = vbat;
                    self.battery_present = if self.battery_present {
                        vbat >= 4.0
                    } else {
                        vbat >= 4.5
                    };
                    self.vbat_ok = if self.vbat_ok {
                        vbat >= 9.0
                    } else {
                        vbat >= 10.0
                    };
                    if LOG_VBAT {
                        println!(
                            "Vbat: {} V, ok: {}, present: {}",
                            self.vbat, self.vbat_ok, self.battery_present
                        );
                    }
                    self.update_targets();
                }
                InputEvent::WifiControl(_timestamp, mode, r, m, t) => {
                    if LOG_WIFI_CONTROL {
                        println!("WifiControl({}, {}, {}, {})", mode, r, m, t);
                    }
                    self.last_wifi_ts = _timestamp;
                    self.last_wifi_mode = mode;
                    self.last_wifi_r = r;
                    self.last_wifi_m = m;
                    self.last_wifi_t = t;
                }
                InputEvent::Stats(_ts, _stats) => {}
            }
        }

        fn compute_autonomous_movement(&self, ts: u64) -> (f32, f32) {
            let wall_x = self.detection_state.closest_wall.0;
            let wall_y = self.detection_state.closest_wall.1;
            let wall_dist = sqrtf(wall_x * wall_x + wall_y * wall_y);
            let obj_x = self.detection_state.object_pos.0;
            let obj_y = self.detection_state.object_pos.1;
            let obj_dist = sqrtf(obj_x * obj_x + obj_y * obj_y);

            let detection_failing = self.detector.arena_w == 0.0 || obj_x == 100.0;

            if detection_failing {
                // Fallback: move away from closest wall towards open space
                let away_x = -wall_x;
                let away_y = -wall_y;
                let target_x = (away_x + self.detection_state.open_space.0) / 2.0;
                let target_y = (away_y + self.detection_state.open_space.1) / 2.0;
                let target_len = sqrtf(target_x * target_x + target_y * target_y);
                let throttle = ((target_len - 50.0 + (wall_dist - 300.0).max(0.0)) / 200.0)
                    .max(0.1)
                    .min(1.0);
                if target_len > 0.0 {
                    (
                        target_x / target_len * MOVEMENT_SPEED_CENTER * throttle,
                        target_y / target_len * MOVEMENT_SPEED_CENTER * throttle,
                    )
                } else {
                    (0.0, 0.0)
                }
            } else {
                // Normal mode: alternate between center and object
                let cycle_ts = ts - self.autonomous_start_ts.unwrap_or(0);
                let phase = (cycle_ts % self.autonomous_cycle_period_us) as f32
                    / self.autonomous_cycle_period_us as f32;
                // Pick a target and a speed depending on what the target is
                let (target_x, target_y, speed_suggestion) = if phase < self.autonomous_duty_cycle
                    || fabsf(self.detector.rpm) < MIN_ATTACK_RPM
                {
                    // Towards center
                    let center_x = self.detection_state.open_space.0;
                    let center_y = self.detection_state.open_space.1;
                    let center_len = sqrtf(center_x * center_x + center_y * center_y);
                    let obj_len = sqrtf(obj_x * obj_x + obj_y * obj_y);

                    // Check if object is nearly aligned with center and
                    // closer than center, meaning we have to go around the
                    // object in order to reach center
                    let angle_center = atan2f(center_y, center_x);
                    let angle_object = atan2f(obj_y, obj_x);
                    let angle_diff =
                        fabsf(rem_euclid_f32(angle_center - angle_object + PI, 2.0 * PI) - PI);
                    if obj_len < center_len
                        && angle_diff < PI / 4.0
                        && center_len > 0.0
                        && obj_len > 0.0
                    {
                        // Compute perpendicular vectors
                        let perp_x1 = -center_y;
                        let perp_y1 = center_x;
                        let perp_x2 = center_y;
                        let perp_y2 = -center_x;

                        // Choose the one farther from object
                        let dot1 = perp_x1 * obj_x + perp_y1 * obj_y;
                        let dot2 = perp_x2 * obj_x + perp_y2 * obj_y;
                        let (perp_x, perp_y) = if dot1.abs() > dot2.abs() {
                            (perp_x1, perp_y1)
                        } else {
                            (perp_x2, perp_y2)
                        };

                        // Normalize perp
                        let perp_len = sqrtf(perp_x * perp_x + perp_y * perp_y);
                        let norm_perp_x = if perp_len > 0.0 {
                            perp_x / perp_len
                        } else {
                            0.0
                        };
                        let norm_perp_y = if perp_len > 0.0 {
                            perp_y / perp_len
                        } else {
                            0.0
                        };

                        // Blend
                        let k = 0.5; // Bias factor
                        let blended_x = center_x + k * norm_perp_x * center_len;
                        let blended_y = center_y + k * norm_perp_y * center_len;
                        let throttle = ((center_len - 50.0) / 200.0).max(0.1).min(1.0);
                        (blended_x, blended_y, MOVEMENT_SPEED_CENTER * throttle)
                    } else {
                        // Not aligned, use center directly
                        let throttle = ((center_len - 50.0) / 200.0).max(0.1).min(1.0);
                        (center_x, center_y, MOVEMENT_SPEED_CENTER * throttle)
                    }
                } else {
                    // Towards object
                    (obj_x, obj_y, MOVEMENT_SPEED_ATTACK)
                };
                let target_len = sqrtf(target_x * target_x + target_y * target_y);
                let intended_speed = if target_len > 0.0 {
                    speed_suggestion
                } else {
                    0.0
                };
                let intended_x = if target_len > 0.0 {
                    target_x / target_len * intended_speed
                } else {
                    0.0
                };
                let intended_y = if target_len > 0.0 {
                    target_y / target_len * intended_speed
                } else {
                    0.0
                };

                // Cancel unwanted velocity
                let v_factor = 10.0;
                let velocity_diff_x = intended_x * v_factor - self.detection_state.velocity.0;
                let velocity_diff_y = intended_y * v_factor - self.detection_state.velocity.1;
                let gain = 0.3;
                let max = 0.7;
                let adjust_x = (velocity_diff_x * gain).max(-max).min(max);
                let adjust_y = (velocity_diff_y * gain).max(-max).min(max);
                /*println!(
                    "velocity_diff: {:.0}, {:.0}, adjust: {:.2}, {:.2}",
                    velocity_diff_x, velocity_diff_y, adjust_x, adjust_y,
                );*/
                (intended_x + adjust_x, intended_y + adjust_y)

                /*// NOTE: This doesn't work properly so it is commented out
                let vel_x = self.detection_state.velocity.0;
                let vel_y = self.detection_state.velocity.1;
                let vel_mag = sqrtf(vel_x * vel_x + vel_y * vel_y);
                if vel_mag > 0.0 {
                    let intended_dot_vel = intended_x * vel_x + intended_y * vel_y;
                    let proj = intended_dot_vel / (vel_mag * intended_speed);
                    let unwanted_vel_x = vel_x - proj * intended_x;
                    let unwanted_vel_y = vel_y - proj * intended_y;
                    let gain = 0.5;
                    let movement_x = intended_x - gain * unwanted_vel_x / vel_mag;
                    let movement_y = intended_y - gain * unwanted_vel_y / vel_mag;
                    // Re-normalize
                    let new_len = sqrtf(movement_x * movement_x + movement_y * movement_y);
                    if new_len > 0.0 {
                        (movement_x / new_len, movement_y / new_len)
                    } else {
                        (0.0, 0.0)
                    }
                } else {
                    (intended_x, intended_y)
                }*/
            }
        }

        fn update_autonomous_targets(&mut self, ts: u64) {
            if !self.vbat_ok {
                self.reset_targets();
                return;
            }

            // Handle rotation reversal if stuck
            if ts > self.reverse_rotation_ts + 5_000_000
                && self.detector.rpm.abs() < REVERSE_ROTATION_MAX_RPM
            {
                self.reverse_rotation_ts = ts;
                self.reverse_rotation = !self.reverse_rotation;
            }

            self.target.rotation_speed =
                TARGET_RPM * if self.reverse_rotation { -1.0 } else { 1.0 };

            if fabsf(self.detector.rpm) >= MIN_MOVE_RPM {
                let (x, y) = self.compute_autonomous_movement(ts);
                self.target.movement_x = x;
                self.target.movement_y = y;
            } else {
                self.target.movement_x = 0.0;
                self.target.movement_y = 0.0;
            }
            if self.autonomous_start_ts.is_none() {
                self.autonomous_start_ts = Some(ts);
            }
        }

        pub fn control_rpm(&mut self, timestamp: u64) {
            let dt = if let Some(last) = self.last_rpm_update_ts {
                ((timestamp as i64 - last as i64) as f32) / 1_000_000.0
            } else {
                0.0
            };
            let max_rpm_delta = MAX_RPM_RAMP_RATE * dt.max(0.0);
            let target = self.target.rotation_speed;
            self.current_rotation_speed +=
                (target - self.current_rotation_speed).clamp(-max_rpm_delta, max_rpm_delta);

            // Allow initial jump in RPM within RPM_INITIAL_JUMP
            let jump_to_if_needed = target.max(-RPM_INITIAL_JUMP).min(RPM_INITIAL_JUMP);
            if jump_to_if_needed.signum() != self.current_rotation_speed.signum()
                || (self.current_rotation_speed.abs() < RPM_INITIAL_JUMP
                    && target.abs() > self.current_rotation_speed.abs())
            {
                self.current_rotation_speed = jump_to_if_needed;
            }

            self.last_rpm_update_ts = Some(timestamp);
        }

        pub fn step(
            &mut self,
            timestamp: u64,
            publisher: Option<
                &mut embassy_sync::pubsub::Publisher<CriticalSectionRawMutex, InputEvent, 32, 3, 6>,
            >,
            debug: bool,
        ) {
            let step_start = get_current_timestamp();

            // Determine control mode and enabled state based on timeouts and last inputs
            let wifi_active = self.last_wifi_ts != 0
                && timestamp as i64 - (self.last_wifi_ts as i64) < FAILSAFE_TIMEOUT_US as i64;
            if wifi_active {
                self.control_mode = ControlMode::from(self.last_wifi_mode);
                self.control_rotation_speed = self.last_wifi_r;
                self.control_movement_speed = self.last_wifi_m;
                self.control_turning_speed = self.last_wifi_t;
                match self.control_mode {
                    ControlMode::Autonomous => {
                        self.autonomous_enabled = true;
                    }
                    ControlMode::ManualWifi => {
                        self.autonomous_enabled = false;
                    }
                    ControlMode::RcReceiver => {
                        // Do not change autonomous_enabled; it will be handled in RC logic
                    }
                }
                if !self.autonomous_enabled {
                    self.autonomous_start_ts = None;
                }
            } else {
                self.control_mode = ControlMode::RcReceiver;
                // autonomous_enabled will be handled below
            }

            if self.control_mode == ControlMode::RcReceiver {
                let receiver_active = self.last_receiver_event_ts != 0
                    && timestamp as i64 - (self.last_receiver_event_ts as i64)
                        < FAILSAFE_TIMEOUT_US as i64;
                let valid_pulse_active = self.last_valid_receiver_ts.map_or(false, |ts| {
                    timestamp as i64 - (ts as i64) < FAILSAFE_TIMEOUT_US as i64
                });
                if !receiver_active || !valid_pulse_active {
                    self.autonomous_enabled = false;
                } else {
                    let pl = self.last_receiver_pulse;
                    let stick_percent = (pl - 1500.0) * 0.2;
                    self.autonomous_enabled = stick_percent >= 5.0;
                }
                if !self.autonomous_enabled {
                    self.autonomous_start_ts = None;
                }
            }

            self.update_targets();

            if (timestamp as i64 - self.last_planner_ts as i64) >= 50_000 {
                self.last_planner_ts = timestamp;

                let result = self.detector.detect_objects(debug);
                self.detection_state = DetectionState {
                    closest_wall: result.closest_wall,
                    open_space: result.open_space,
                    object_pos: result.object_pos,
                    wall_distances: result.wall_distances,
                    position: result.position,
                    velocity: result.velocity,
                };
                if debug {
                    println!(
                        "Detected: Wall {:?}, Open {:?}, Object {:?} (bins={})",
                        result.closest_wall,
                        result.open_space,
                        result.object_pos,
                        self.detector.bin_count()
                    );
                }

                if self.autonomous_enabled {
                    self.update_autonomous_targets(timestamp);
                }

                // Flip detection

                let intended_movement_mag =
                    sqrtf(self.target.movement_x.powi(2) + self.target.movement_y.powi(2));

                if intended_movement_mag < 0.1 || self.detector.rpm.abs() < MIN_MOVE_RPM {
                    self.no_intended_movement_timestamp = timestamp;
                }

                // Compare intended movement with measured velocity using dot
                // product
                let dot = self.detection_state.velocity.0 * self.target.movement_x
                    + self.detection_state.velocity.1 * self.target.movement_y;
                self.intended_movement_comparison = if intended_movement_mag > 0.01 {
                    sqrtf(dot) / intended_movement_mag
                } else {
                    -f32::INFINITY
                };
                if self.intended_movement_comparison > 0.5 {
                    self.intended_movement_velocity_timestamp = timestamp;
                }

                const D: f32 = 250.0;
                if (self.detection_state.wall_distances.0 > D
                    && self.detection_state.wall_distances.1 > D
                    && self.detection_state.wall_distances.2 > D
                    && self.detection_state.wall_distances.3 > D)
                {
                    self.away_from_wall_timestamp = timestamp;
                } else {
                    self.close_to_wall_timestamp = timestamp;
                }

                // Flip angular correction 180° if robot moves to the wrong
                // direction or seems stuck to a wall (i.e. it might be flipped)
                if timestamp - self.angular_correction_flip_ts > 6_000_000
                    && self.detector.rpm.abs() > MIN_MOVE_RPM
                    && timestamp - self.no_intended_movement_timestamp > 200_000
                    && ((timestamp - self.intended_movement_velocity_timestamp > 2_500_000
                        && timestamp - self.away_from_wall_timestamp > 1_500_000)
                        || timestamp - self.away_from_wall_timestamp > 3_000_000)
                {
                    self.angular_correction_flip_ts = timestamp;
                    self.angular_correction_flip = !self.angular_correction_flip;
                    if debug {
                        println!("angular_correction_flip: {}", self.angular_correction_flip);
                    }
                }

                if debug {
                    println!(
                        "intended: {:.0},{:.0}, vel: {:.0},{:.0}, angular_correction_flip: {}",
                        self.target.movement_x,
                        self.target.movement_y,
                        self.detection_state.velocity.0,
                        self.detection_state.velocity.1,
                        self.angular_correction_flip
                    );
                }

                // This is passed to MotorModulator later
                // We only do 0° and 180° now
                self.angular_correction_total = if self.angular_correction_flip {
                    //PI
                    // Disable flip detection while focusing on other things
                    0.0
                } else {
                    0.0
                };
            }

            self.control_rpm(timestamp);

            // Motor controller reset logic
            if let Some(plan) = &self.motor_control_plan {
                let target_rpm_abs = plan.rotation_speed.abs();
                let measured_rpm_abs = self.detector.rpm.abs();
                if target_rpm_abs > 0.0 && measured_rpm_abs < 0.25 * target_rpm_abs {
                    if let Some(start_ts) = self.low_rpm_start_ts {
                        if timestamp as i64 - start_ts as i64 >= 2_000_000
                            && timestamp as i64 - self.last_reset_attempt_ts as i64 >= 6_000_000
                        {
                            self.reset_start_ts = Some(timestamp);
                            self.last_reset_attempt_ts = timestamp;
                            self.low_rpm_start_ts = None;
                            if debug {
                                println!("Initiating motor controller reset at ts={}", timestamp);
                            }
                        }
                    } else {
                        self.low_rpm_start_ts = Some(timestamp);
                    }
                } else {
                    self.low_rpm_start_ts = None;
                }
            } else {
                self.low_rpm_start_ts = None;
            }

            let is_in_reset = self.reset_start_ts.is_some()
                && (timestamp as i64 - self.reset_start_ts.unwrap() as i64) < 1_500_000;

            let plan_opt: Option<MotorControlPlan> = if is_in_reset {
                self.current_rotation_speed = 0.0;
                if debug {
                    println!("Motor controller reset in progress at ts={}", timestamp);
                }
                Some(MotorControlPlan {
                    timestamp,
                    rotation_speed: 0.0,
                    movement_x: 0.0,
                    movement_y: 0.0,
                })
            } else {
                if self.reset_start_ts.is_some() {
                    self.reset_start_ts = None;
                }
                if !self.vbat_ok {
                    None
                } else {
                    Some(MotorControlPlan {
                        timestamp,
                        rotation_speed: self.current_rotation_speed,
                        movement_x: self.target.movement_x,
                        movement_y: self.target.movement_y,
                    })
                }
            };

            self.motor_control_plan = plan_opt.clone();

            if let Some(plan) = plan_opt {
                if timestamp as i64 - self.last_publish_ts as i64 >= 50_000 {
                    self.last_publish_ts = timestamp;
                    if let Some(ref publisher) = publisher {
                        publisher.publish_immediate(InputEvent::Planner(
                            timestamp,
                            plan,
                            self.detection_state.closest_wall,
                            self.detection_state.open_space,
                            self.detection_state.object_pos,
                            self.detector.theta,
                            self.detector.rpm,
                        ));
                    }
                }
            }

            let step_duration_us = get_current_timestamp() - step_start;
            self.stats_step_sum_us += step_duration_us;
            self.stats_step_min_us = self.stats_step_min_us.min(step_duration_us);
            self.stats_step_max_us = self.stats_step_max_us.max(step_duration_us);

            self.stats_step_count += 1;

            if self.stats_step_count >= 50 {
                // ~1 second at 20ms loop
                if let Some(publisher) = publisher {
                    let avg_step_us = self.stats_step_sum_us / self.stats_step_count;

                    publisher.publish_immediate(InputEvent::Stats(
                        timestamp,
                        Stats {
                            step_min_duration_us: self.stats_step_min_us,
                            step_max_duration_us: self.stats_step_max_us,
                            step_avg_duration_us: avg_step_us,
                        },
                    ));
                }
                // Reset stats
                self.stats_step_count = 0;
                self.stats_step_sum_us = 0;
                self.stats_step_min_us = u64::MAX;
                self.stats_step_max_us = 0;
            }
        }
    }

    const TAG_XOR: u16 = 0x5555;

    pub fn serialize_event(event: &InputEvent) -> Vec<u8> {
        let mut buf = Vec::with_capacity(32);
        match event {
            InputEvent::Lidar(ts, d1, d2, d3, d4) => {
                let tag = (0u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&d1.to_le_bytes());
                buf.extend_from_slice(&d2.to_le_bytes());
                buf.extend_from_slice(&d3.to_le_bytes());
                buf.extend_from_slice(&d4.to_le_bytes());
            }
            InputEvent::Accelerometer(ts, accel_y, accel_z) => {
                let tag = (1u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&accel_y.to_le_bytes());
                buf.extend_from_slice(&accel_z.to_le_bytes());
            }
            InputEvent::Receiver(ts, channel, pulse_length) => {
                let tag = (2u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.push(*channel);
                match pulse_length {
                    Some(pl) => {
                        buf.push(1); // Flag 1
                        buf.extend_from_slice(&pl.to_le_bytes());
                    }
                    None => {
                        buf.push(0); // Flag 0
                    }
                }
            }
            InputEvent::Vbat(ts, voltage) => {
                let tag = (3u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&voltage.to_le_bytes());
            }
            InputEvent::WifiControl(ts, mode, r, m, t) => {
                let tag = (4u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.push(*mode);
                buf.extend_from_slice(&r.to_le_bytes());
                buf.extend_from_slice(&m.to_le_bytes());
                buf.extend_from_slice(&t.to_le_bytes());
            }
            InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) => {
                let tag = (5u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&plan.rotation_speed.to_le_bytes());
                buf.extend_from_slice(&plan.movement_x.to_le_bytes());
                buf.extend_from_slice(&plan.movement_y.to_le_bytes());
                buf.extend_from_slice(&cw.0.to_le_bytes());
                buf.extend_from_slice(&cw.1.to_le_bytes());
                buf.extend_from_slice(&os.0.to_le_bytes());
                buf.extend_from_slice(&os.1.to_le_bytes());
                buf.extend_from_slice(&op.0.to_le_bytes());
                buf.extend_from_slice(&op.1.to_le_bytes());
                buf.extend_from_slice(&theta.to_le_bytes());
                buf.extend_from_slice(&rpm.to_le_bytes());
            }
            InputEvent::Stats(ts, stats) => {
                let tag = (6u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&stats.step_min_duration_us.to_le_bytes());
                buf.extend_from_slice(&stats.step_max_duration_us.to_le_bytes());
                buf.extend_from_slice(&stats.step_avg_duration_us.to_le_bytes());
            }
        }
        buf
    }

    pub const MODULATION_AMPLITUDE: f32 = 250.0; // RPM

    pub struct MotorModulator {
        last_ts: u64,
        theta: f32,
        rpm: f32,
        pub mcp: Option<MotorControlPlan>,
        angular_correction: f32,
    }

    impl MotorModulator {
        pub fn new() -> Self {
            Self {
                last_ts: 0,
                theta: 0.0,
                rpm: 0.0,
                mcp: None,
                angular_correction: 0.0,
            }
        }

        pub fn sync(
            &mut self,
            ts: u64,
            theta: f32,
            plan: MotorControlPlan,
            angular_correction: f32,
        ) {
            self.last_ts = ts;
            self.theta = theta;
            self.mcp = Some(plan);
            self.angular_correction = angular_correction;
        }

        pub fn step(&mut self, ts: u64) -> (f32, f32) {
            let base_rpm = self.mcp.as_ref().map_or(0.0, |p| p.rotation_speed);

            // Cast to i64 so that ts sometimes going backwards is fine
            let dt = (ts as i64 - self.last_ts as i64) as f32 / 1_000_000.0;
            self.theta += base_rpm / 60.0 * 2.0 * PI * dt;
            self.theta = rem_euclid_f32(self.theta, 2.0 * PI);
            self.last_ts = ts;

            if let Some(plan) = self.mcp.as_ref() {
                // Cast to i64 so that ts sometimes going backwards is fine
                if ts as i64 - plan.timestamp as i64 > 500_000 {
                    self.mcp = None;
                }
            }

            let target_rotation_speed = self.mcp.as_ref().map_or(0.0, |p| p.rotation_speed);
            let target_movement_x = self.mcp.as_ref().map_or(0.0, |p| p.movement_x);
            let target_movement_y = self.mcp.as_ref().map_or(0.0, |p| p.movement_y);

            let mag = sqrtf(
                target_movement_x * target_movement_x + target_movement_y * target_movement_y,
            );
            if mag == 0.0 {
                return (base_rpm, base_rpm);
            }

            let phase = atan2f(target_movement_y, target_movement_x) + self.angular_correction;

            let mod_half = MODULATION_AMPLITUDE * mag * cosf(self.theta - phase);

            let left_rpm = base_rpm - mod_half;
            let right_rpm = base_rpm + mod_half;

            (left_rpm, right_rpm)
        }
    }
}
