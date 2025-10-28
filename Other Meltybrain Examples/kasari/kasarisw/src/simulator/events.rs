use kasarisw::shared::kasari::InputEvent;
use kasarisw::shared::kasari::MotorControlPlan;
use kasarisw::shared::kasari::Stats;
use serde_json::Value;
use std::error::Error;

pub fn parse_event(line: &str) -> Result<InputEvent, Box<dyn Error>> {
    let v: Vec<Value> = serde_json::from_str(line)?;

    if v.is_empty() {
        return Err("Empty event".into());
    }

    let typ = v[0].as_str().ok_or("No type")?;
    let ts = v[1].as_u64().ok_or("No timestamp")?;

    match typ {
        "Lidar" => {
            if v.len() != 6 {
                return Err("Invalid Lidar event length".into());
            }
            let d1 = v[2].as_f64().ok_or("Invalid d1")? as f32;
            let d2 = v[3].as_f64().ok_or("Invalid d2")? as f32;
            let d3 = v[4].as_f64().ok_or("Invalid d3")? as f32;
            let d4 = v[5].as_f64().ok_or("Invalid d4")? as f32;
            Ok(InputEvent::Lidar(ts, d1, d2, d3, d4))
        }
        "Accelerometer" => {
            if v.len() != 4 {
                return Err("Invalid Accelerometer event length".into());
            }
            let ay = v[2].as_f64().ok_or("Invalid ay")? as f32;
            let az = v[3].as_f64().ok_or("Invalid az")? as f32;
            Ok(InputEvent::Accelerometer(ts, ay, az))
        }
        "Receiver" => {
            if v.len() != 4 {
                return Err("Invalid Receiver event length".into());
            }
            let ch = v[2].as_u64().ok_or("Invalid channel")? as u8;
            let pulse = if v[3].is_null() {
                None
            } else {
                Some(v[3].as_f64().ok_or("Invalid pulse")? as f32)
            };
            Ok(InputEvent::Receiver(ts, ch, pulse))
        }
        "Vbat" => {
            if v.len() != 3 {
                return Err("Invalid Vbat event length".into());
            }
            let voltage = v[2].as_f64().ok_or("Invalid voltage")? as f32;
            Ok(InputEvent::Vbat(ts, voltage))
        }
        "WifiControl" => {
            if v.len() != 6 {
                return Err("Invalid WifiControl event length".into());
            }
            let mode = v[2].as_u64().ok_or("Invalid mode")? as u8;
            let r = v[3].as_f64().ok_or("Invalid r")? as f32;
            let m = v[4].as_f64().ok_or("Invalid m")? as f32;
            let t = v[5].as_f64().ok_or("Invalid t")? as f32;
            Ok(InputEvent::WifiControl(ts, mode, r, m, t))
        }
        "Planner" => {
            if v.len() != 12 && v.len() != 13 {
                return Err("Invalid Planner event length".into());
            }
            let rotation_speed = v[2].as_f64().ok_or("Invalid rotation_speed")? as f32;
            let movement_x = v[3].as_f64().ok_or("Invalid movement_x")? as f32;
            let movement_y = v[4].as_f64().ok_or("Invalid movement_y")? as f32;
            let cw_x = v[5].as_f64().ok_or("Invalid cw_x")? as f32;
            let cw_y = v[6].as_f64().ok_or("Invalid cw_y")? as f32;
            let os_x = v[7].as_f64().ok_or("Invalid os_x")? as f32;
            let os_y = v[8].as_f64().ok_or("Invalid os_y")? as f32;
            let op_x = v[9].as_f64().ok_or("Invalid op_x")? as f32;
            let op_y = v[10].as_f64().ok_or("Invalid op_y")? as f32;
            let theta = v[11].as_f64().ok_or("Invalid theta")? as f32;
            let rpm = if v.len() >= 13 {
                v[12].as_f64().ok_or("Invalid rpm")? as f32
            } else {
                0.0
            };
            Ok(InputEvent::Planner(
                ts,
                MotorControlPlan {
                    timestamp: 0,
                    rotation_speed,
                    movement_x,
                    movement_y,
                },
                (cw_x, cw_y),
                (os_x, os_y),
                (op_x, op_y),
                theta,
                rpm,
            ))
        }
        "Stats" => {
            if v.len() != 5 {
                return Err("Invalid Stats event length".into());
            }
            let min_dur = v[2].as_u64().ok_or("Invalid min_dur")?;
            let max_dur = v[3].as_u64().ok_or("Invalid max_dur")?;
            let avg_dur = v[4].as_u64().ok_or("Invalid avg_dur")?;
            Ok(InputEvent::Stats(
                ts,
                Stats {
                    step_min_duration_us: min_dur,
                    step_max_duration_us: max_dur,
                    step_avg_duration_us: avg_dur,
                },
            ))
        }
        _ => Err("Unknown event type".into()),
    }
}

pub fn get_ts(event: &InputEvent) -> u64 {
    match event {
        InputEvent::Lidar(ts, ..) => *ts,
        InputEvent::Accelerometer(ts, ..) => *ts,
        InputEvent::Receiver(ts, ..) => *ts,
        InputEvent::Vbat(ts, ..) => *ts,
        InputEvent::WifiControl(ts, ..) => *ts,
        InputEvent::Planner(ts, ..) => *ts,
        InputEvent::Stats(ts, stats) => *ts,
    }
}
