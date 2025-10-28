use clap::Parser;
use eframe::{self, egui};
use kasarisw::shared::kasari::InputEvent;
use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Error as IoError};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod app;
mod events;
mod physics;
mod sources;

use app::MyApp;
use events::get_ts;
use sources::{EventSource, FileEventSource, SimEventSource};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Log file path or '-' for stdin
    #[arg(required_unless_present = "sim")]
    source: Option<String>,

    /// Enable debug prints
    #[arg(long, short)]
    debug: bool,

    /// Remove all WifiControl events and inject new ones with mode=2 every 100ms starting after first Lidar event
    #[arg(long, short)]
    inject_autonomous: bool,

    /// Offset LIDAR's distance values (mm)
    #[arg(long, short, default_value_t = 0.0)]
    lidar_distance_offset: f32,

    /// Run in true simulation mode (ignores source)
    #[arg(long, short)]
    sim: bool,

    /// Arena width in mm (simulation only)
    #[arg(long, default_value_t = 1200.0)]
    arena_width: f32,

    /// Arena height in mm (simulation only)
    #[arg(long, default_value_t = 1200.0)]
    arena_height: f32,

    /// Whether to include the object in the arena (simulation only)
    #[arg(long)]
    no_object: bool,

    /// Whether to set the robot to rotate in reverse
    #[arg(long)]
    reverse_rotation: bool,

    /// Whether to run the robot in the flipped state
    #[arg(long)]
    robot_flipped: bool,

    /// Headless mode for profiling
    #[arg(long)]
    headless: bool,

    /// Time at which simulation ends (seconds) (only applies in headless mode)
    #[arg(long)]
    end_time: Option<f32>,

    #[arg(long)]
    listen: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let lines: Box<dyn Iterator<Item = Result<String, IoError>>> = if args.sim {
        Box::new(std::iter::empty())
    } else {
        match args.source {
            Some(source) => {
                if source == "-" {
                    Box::new(io::stdin().lines())
                } else {
                    let file = File::open(&source)?;
                    let reader = BufReader::new(file);
                    Box::new(reader.lines())
                }
            }
            None => {
                return Err("Source argument required when not in simulation mode".into());
            }
        }
    };

    let mut app = MyApp::new(
        lines,
        args.debug,
        args.inject_autonomous,
        args.lidar_distance_offset,
        args.sim,
        args.arena_width,
        args.arena_height,
        args.no_object,
        args.reverse_rotation,
        args.robot_flipped,
        args.listen,
    );

    if args.listen {
        use crossbeam_channel::{unbounded, Receiver, Sender};
        use std::io::{Read, Write};
        use std::net::{Shutdown, TcpListener};
        use std::sync::{Arc, Mutex};
        use std::thread;

        let (incoming_tx, incoming_rx): (Sender<InputEvent>, Receiver<InputEvent>) = unbounded();
        let (outgoing_tx, outgoing_rx): (Sender<InputEvent>, Receiver<InputEvent>) = unbounded();

        app.outgoing_tx = Some(outgoing_tx);
        app.incoming_rx = Some(incoming_rx);

        let current_conns: Arc<Mutex<Vec<Sender<Vec<u8>>>>> = Arc::new(Mutex::new(Vec::new()));

        // Broadcaster thread for outgoing events
        let current_conns_clone = current_conns.clone();
        let outgoing_rx_clone = outgoing_rx.clone();
        thread::spawn(move || loop {
            if let Ok(event) = outgoing_rx_clone.recv() {
                let serialized = kasarisw::shared::kasari::serialize_event(&event);
                let mut guard = current_conns_clone.lock().unwrap();
                let mut i = 0;
                while i < guard.len() {
                    if guard[i].send(serialized.clone()).is_err() {
                        guard.remove(i);
                    } else {
                        i += 1;
                    }
                }
            }
        });

        // Listener thread for port 8080
        let incoming_tx_clone = incoming_tx.clone();
        thread::spawn(move || {
            let listener = match TcpListener::bind("127.0.0.1:8080") {
                Ok(l) => l,
                Err(e) => {
                    eprintln!("Failed to bind 8080: {}", e);
                    return;
                }
            };
            for stream in listener.incoming() {
                if let Ok(socket) = stream {
                    println!("New connection on 8080");
                    let incoming_tx_c = incoming_tx_clone.clone();
                    let mut read_socket = match socket.try_clone() {
                        Ok(s) => s,
                        Err(e) => {
                            eprintln!("Socket clone failed: {}", e);
                            continue;
                        }
                    };
                    let current_conns_c = current_conns.clone();
                    let (conn_tx, conn_rx) = unbounded::<Vec<u8>>();
                    {
                        let mut guard = current_conns_c.lock().unwrap();
                        guard.push(conn_tx);
                    }
                    // Sender sub-thread (per connection)
                    let mut write_socket = socket;
                    thread::spawn(move || loop {
                        if let Ok(data) = conn_rx.recv() {
                            if write_socket.write_all(&data).is_err() {
                                break;
                            }
                            let _ = write_socket.flush();
                        } else {
                            break;
                        }
                    });
                    // Receiver sub-thread (per connection)
                    thread::spawn(move || {
                        let mut buf = [0u8; 512];
                        let mut rx_pos: usize = 0;
                        loop {
                            match read_socket.read(&mut buf[rx_pos..]) {
                                Ok(0) => break,
                                Ok(n) => {
                                    rx_pos += n;
                                    while rx_pos >= 1 {
                                        let tag = buf[0];
                                        if tag == 4 {
                                            if rx_pos >= 22 {
                                                let _ts_orig = u64::from_le_bytes(
                                                    buf[1..9].try_into().unwrap(),
                                                );
                                                let mode = buf[9];
                                                let r = f32::from_le_bytes(
                                                    buf[10..14].try_into().unwrap(),
                                                );
                                                let m = f32::from_le_bytes(
                                                    buf[14..18].try_into().unwrap(),
                                                );
                                                let t = f32::from_le_bytes(
                                                    buf[18..22].try_into().unwrap(),
                                                );
                                                let ts = 0; // Ignored in logic
                                                let event = kasarisw::shared::kasari::InputEvent::WifiControl(ts, mode, r, m, t);
                                                let _ = incoming_tx_c.send(event);
                                                buf.copy_within(22.., 0);
                                                rx_pos -= 22;
                                            } else {
                                                break;
                                            }
                                        } else {
                                            buf.copy_within(1.., 0);
                                            rx_pos -= 1;
                                        }
                                    }
                                }
                                Err(_) => break,
                            }
                        }
                    });
                }
            }
        });

        // Listener thread for port 8081 (dummy: accept and close)
        thread::spawn(move || {
            let listener = match TcpListener::bind("127.0.0.1:8081") {
                Ok(l) => l,
                Err(e) => {
                    eprintln!("Failed to bind 8081: {}", e);
                    return;
                }
            };
            for stream in listener.incoming() {
                if let Ok(mut socket) = stream {
                    println!("New connection on 8081");
                    let msg = "This was a simulator run (not a real panic)";
                    let mut batch = vec![0u8; 4096];
                    batch[0..4].copy_from_slice(&0xffffffffu32.to_le_bytes());
                    let msg_bytes = msg.as_bytes();
                    batch[4..4 + msg_bytes.len()].copy_from_slice(msg_bytes);
                    if socket.write_all(&batch).is_ok() {
                        let _ = socket.flush();
                    }
                    let _ = socket.shutdown(Shutdown::Both);
                }
            }
        });
    }

    if args.headless {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
            println!("Received SIGINT, shutting down gracefully...");
        })?;

        let mut processed_events = 0;
        let mut latest_ts = 0u64;
        let mut last_print = Instant::now();
        let print_interval = Duration::from_secs(1);

        while running.load(Ordering::SeqCst) {
            if let Some(event) = app.event_source.get_next_event() {
                app.process_event(&event); // Optional: process as in GUI mode
                processed_events += 1;
                latest_ts = get_ts(&event);
            } else {
                // No more events; could break or sleep
                std::thread::sleep(Duration::from_millis(100));
            }

            let now = Instant::now();
            if now - last_print >= print_interval {
                println!(
                    "Processed events: {}, Latest timestamp: {} ms",
                    processed_events,
                    latest_ts / 1000
                );
                last_print = now;
            }

            if let Some(end_time) = args.end_time {
                if latest_ts >= (end_time * 1_000_000.0) as u64 {
                    break;
                }
            }
        }

        // Final print on exit
        println!(
            "Final: Processed events: {}, Latest timestamp: {} ms",
            processed_events,
            latest_ts / 1000
        );
    } else {
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([2000.0, 2000.0]),
            ..Default::default()
        };
        eframe::run_native(
            "Robot Simulator",
            options,
            Box::new(move |_cc| Ok(Box::new(app))),
        )?;
    }

    Ok(())
}
