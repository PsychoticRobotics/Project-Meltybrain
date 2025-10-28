#![no_std]
use crate::shared::{kasari::InputEvent, EventChannel, LOG_LIDAR};
use crate::LIDAR_EVENT_QUEUE_REF;
use crate::SIGNAL_LIDAR_REF;
use alloc::collections::VecDeque;
use core::cell::RefCell;
use core::ops::DerefMut;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Timer;
use esp_hal::{
    gpio::Level,
    rmt::{Channel, ConstChannelAccess, PulseCode, Rx, RxChannelAsync},
    spi::master::Spi,
    uart::{UartRx, UartTx},
    Async,
};
use esp_println::println;
use ringbuffer::ConstGenericRingBuffer;
use ringbuffer::RingBuffer;

// LIDAR constants
pub const PACKET_SIZE: usize = 22;
pub const HEAD_BYTE: u8 = 0xFA;
// Experimentally tuned to maximize baud usage. This minimizes the effect
// buffering at the sending end has in terms of timing noise. The LIDAR buffers
// at least a couple of packets before sending them.
// * 86 Hz is too fast and 85 Hz works in terms of data transfer, so we could
//   use 83 Hz in terms of that. However...
// * 83 Hz doesn't work because at that rate the LIDAR gives up actually
//   measuring stuff and returns 17mm for the last half of its data buffering
//   cycle (the data buffering cycle is about 7 messages). In terms of this, 81
//   Hz appears to not work and 80 Hz appears to work, so we use 80 Hz.
pub const LIDAR_ENCODER_HZ: f32 = 80.0;
// Distance offset based on sensor's mounting position on the robot
pub const LIDAR_DISTANCE_OFFSET: f32 = 36.0;

#[embassy_executor::task]
pub async fn lidar_writer(
    mut tx: UartTx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(&mut tx, b"Hello async serial\r\n")
        .await
        .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
pub async fn lidar_publisher(event_channel: &'static EventChannel) {
    let publisher = event_channel.publisher().unwrap();
    loop {
        unsafe {
            SIGNAL_LIDAR_REF.unwrap().wait().await;
        }
        critical_section::with(|cs| {
            let mut queue = unsafe { LIDAR_EVENT_QUEUE_REF.unwrap().borrow(cs).borrow_mut() };
            while let Some(event) = queue.dequeue() {
                publisher.publish_immediate(event);
            }
        });
    }
}

pub struct ParsedPacket {
    pub distances: [u16; 4], // mm
}

pub fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE || packet[0] != HEAD_BYTE {
        return None;
    }

    let a0 = (packet[1] as u16).wrapping_sub(0xA0);
    if a0 > 360 / 4 {
        return None;
    }
    let angle = a0 * 4;

    let mut distances = [0u16; 4];
    for (i, dist) in distances.iter_mut().enumerate() {
        let idx = 4 + i * 4;
        *dist = ((packet[idx + 1] as u16 & 0x3F) << 8) | (packet[idx] as u16);
    }

    let received_sum = ((packet[21] as u16) << 8) | (packet[20] as u16);
    let computed_sum = compute_checksum(&packet[..PACKET_SIZE - 2]);
    if computed_sum != received_sum {
        return None;
    }

    Some(ParsedPacket { distances })
}

pub fn compute_checksum(data: &[u8]) -> u16 {
    let mut chk32: u32 = 0;
    for chunk in data.chunks(2) {
        let word = if chunk.len() == 2 {
            ((chunk[1] as u16) << 8) + (chunk[0] as u16)
        } else {
            chunk[0] as u16
        };
        chk32 = (chk32 << 1) + (word as u32);
    }
    let checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    (checksum & 0x7FFF) as u16
}

#[embassy_executor::task]
pub async fn accelerometer_task(
    mut spi: Spi<'static, esp_hal::Blocking>,
    event_channel: &'static EventChannel,
) {
    let publisher = event_channel.publisher().unwrap();

    let mut buf = [(0x00 << 1) | 1, 0]; // DEVID_AD
    spi.transfer(&mut buf).unwrap();
    println!("DEVID_AD = {:#02x} (should be 0xad)", buf[1]);

    let mut buf = [(0x3f << 1) | 0, 0b00010111]; // POWER_CTL
    spi.transfer(&mut buf).unwrap();

    loop {
        let mut buf = [(0x08 << 1) | 1, 0, 0, 0, 0, 0, 0]; // XDATA
        spi.transfer(&mut buf).unwrap();
        let x_raw = (((buf[1] as i16) << 8) | buf[2] as i16) >> 4;
        let y_raw = (((buf[3] as i16) << 8) | buf[4] as i16) >> 4;
        let z_raw = (((buf[5] as i16) << 8) | buf[6] as i16) >> 4;
        let x_g = x_raw as f32 * 0.2;
        let y_g = y_raw as f32 * 0.2;
        let z_g = z_raw as f32 * 0.2;

        let timestamp = embassy_time::Instant::now().as_ticks();
        let event = InputEvent::Accelerometer(timestamp, -y_g, -z_g);
        publisher.publish_immediate(event);

        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
pub async fn rmt_task(
    mut ch0: Channel<Async, ConstChannelAccess<Rx, 0>>,
    event_channel: &'static EventChannel,
) {
    let publisher = event_channel.publisher().unwrap();

    let mut buffer0: [u32; 1] = [PulseCode::empty(); 1];

    loop {
        for x in buffer0.iter_mut() {
            x.reset();
        }
        let result = ch0.receive(&mut buffer0).await;
        let mut ch0_final_result: Option<f32> = None;
        match result {
            Ok(()) => {
                for entry in buffer0.iter().take_while(|e| e.length1() != 0) {
                    let length_raw = if entry.level1() == Level::High && entry.length1() > 10 {
                        entry.length1()
                    } else if entry.level2() == Level::High && entry.length2() > 10 {
                        entry.length2()
                    } else {
                        0
                    };

                    if length_raw > 10 {
                        let pulse_width_us = length_raw as f32;
                        ch0_final_result = Some(pulse_width_us);
                    }
                }
            }
            Err(_err) => {}
        }

        let timestamp = embassy_time::Instant::now().as_ticks();
        let event = InputEvent::Receiver(timestamp, 0, ch0_final_result);
        publisher.publish_immediate(event);
    }
}
