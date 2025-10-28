#![no_std]
#![no_main]

use core::net::Ipv4Addr;
use embassy_executor::Spawner;
use embassy_net::{
    tcp::TcpSocket, IpListenEndpoint, Ipv4Cidr, Runner, StackResources, StaticConfigV4,
};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::interrupt;
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::timer::Timer as HwTimer;
use esp_hal::{
    analog::{adc, adc::Adc, adc::AdcConfig},
    gpio::{Level, Output, OutputConfig},
    ledc::{channel as ledc_channel, channel::ChannelHW, timer as ledc_timer, Ledc},
    rmt::{RxChannelConfig, RxChannelCreator},
    spi::master::Spi,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Uart, UartInterrupt},
    Blocking,
};
use esp_println::println;
use esp_wifi::wifi::WifiDevice;
use esp_wifi::{
    init,
    wifi::{
        AccessPointConfiguration, ClientConfiguration, Configuration, WifiController, WifiEvent,
        WifiState,
    },
    EspWifiController,
};
use ringbuffer::ConstGenericRingBuffer;
use static_cell::StaticCell;
extern crate alloc;
use alloc::boxed::Box;
use arrayvec::{ArrayString, ArrayVec};
use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use critical_section::Mutex;
use ringbuffer::RingBuffer;

use esp_bootloader_esp_idf::esp_app_desc; // Added for app descriptor
esp_app_desc!(); // Populates the ESP_APP_DESC static with defaults

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_storage::FlashStorage;
use libm::fabsf;

mod sensors;
mod shared;

use shared::kasari;
use kasari::InputEvent;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const PWM_HZ: f32 = 400.0;

const RPM_PER_THROTTLE_PERCENT: f32 = 40.0;

const MOTOR_UPDATE_HZ: f32 = 200.0;

// ~2083; generalize from consts LIDAR_ENCODER_HZ as u64, PULSES_PER_REV=15,
// PACKETS_PER_REV=90
const LIDAR_PACKET_INTERVAL_US: u64 =
    1_000_000u64 / (((sensors::LIDAR_ENCODER_HZ as u64) * 90u64) / 15u64);

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// ESC target speed (bidirectional) to servo signal PWM duty cycle percent
fn target_speed_to_pwm_duty(speed_percent: f32, duty_range: u32) -> u32 {
    let center_pwm = 0.00149 * PWM_HZ;
    let pwm_amplitude = 0.000350 * PWM_HZ;
    let duty_percent = (center_pwm * 100.0 + pwm_amplitude * speed_percent)
        .min(100.0)
        .max(-100.0);
    (duty_range * duty_percent as u32) / 100
}

const LIDAR_BUFFER_SIZE: usize = sensors::PACKET_SIZE * 6;
const LIDAR_EVENT_QUEUE_SIZE: usize = 128;

static UART2: StaticCell<Mutex<RefCell<Option<Uart<'static, Blocking>>>>> = StaticCell::new();
static LIDAR_BYTE_BUFFER: StaticCell<
    Mutex<RefCell<ConstGenericRingBuffer<u8, LIDAR_BUFFER_SIZE>>>,
> = StaticCell::new();
static LIDAR_EVENT_QUEUE: StaticCell<
    Mutex<RefCell<ConstGenericRingBuffer<kasari::InputEvent, LIDAR_EVENT_QUEUE_SIZE>>>,
> = StaticCell::new();
static SIGNAL_LIDAR: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();

static mut UART2_REF: Option<&'static Mutex<RefCell<Option<Uart<'static, Blocking>>>>> = None;
static mut LIDAR_BYTE_BUFFER_REF: Option<
    &'static Mutex<RefCell<ConstGenericRingBuffer<u8, LIDAR_BUFFER_SIZE>>>,
> = None;
static mut LIDAR_EVENT_QUEUE_REF: Option<
    &'static Mutex<RefCell<ConstGenericRingBuffer<kasari::InputEvent, LIDAR_EVENT_QUEUE_SIZE>>>,
> = None;
static mut SIGNAL_LIDAR_REF: Option<&'static Signal<NoopRawMutex, ()>> = None;

static LIDAR_PACKET_COUNT: AtomicU32 = AtomicU32::new(0);
static LIDAR_SKIPPED_BYTE_COUNT: AtomicU32 = AtomicU32::new(0);
static LIDAR_VALID_BYTE_COUNT: AtomicU32 = AtomicU32::new(0);

type LedcTimer = ledc_timer::Timer<'static, esp_hal::ledc::LowSpeed>;
type LedcChannel = ledc_channel::Channel<'static, esp_hal::ledc::LowSpeed>;
type TimgTimer = esp_hal::timer::timg::Timer<'static>;

static MODULATOR: StaticCell<Mutex<RefCell<shared::kasari::MotorModulator>>> = StaticCell::new();
static CHANNEL0: StaticCell<Mutex<RefCell<Option<LedcChannel>>>> = StaticCell::new();
static CHANNEL1: StaticCell<Mutex<RefCell<Option<LedcChannel>>>> = StaticCell::new();
static MOTOR_TIMER: StaticCell<Mutex<RefCell<Option<TimgTimer>>>> = StaticCell::new();

static mut MODULATOR_REF: Option<&'static Mutex<RefCell<shared::kasari::MotorModulator>>> = None;
static mut CHANNEL0_REF: Option<&'static Mutex<RefCell<Option<LedcChannel>>>> = None;
static mut CHANNEL1_REF: Option<&'static Mutex<RefCell<Option<LedcChannel>>>> = None;
static mut MOTOR_TIMER_REF: Option<&'static Mutex<RefCell<Option<TimgTimer>>>> = None;

static MOTOR_UPDATE_COUNT: AtomicU32 = AtomicU32::new(0);

static BATTERY_PRESENT: AtomicBool = AtomicBool::new(false);

const LOG_FLASH_OFFSET: u32 = 0x300000;
const LOG_FLASH_SIZE: u32 = 0x100000;

const BATCH_FLUSH_SIZE: usize = 4096;
const BATCH_DATA_SIZE: usize = BATCH_FLUSH_SIZE - 4; // Reserve 4 bytes for sequence number
const RAM_BUFFER_SIZE: usize = 8192;

static LOG_BUFFER: StaticCell<Mutex<RefCell<ConstGenericRingBuffer<u8, RAM_BUFFER_SIZE>>>> =
    StaticCell::new();
static LOGGING_ACTIVE: AtomicBool = AtomicBool::new(false);
static LOG_WRITE_POS: AtomicU32 = AtomicU32::new(0);
static LOG_SEQ: AtomicU32 = AtomicU32::new(0);
static LIDAR_COUNT: AtomicU32 = AtomicU32::new(0);
static LAST_HIGH_RPM_TS: AtomicU32 = AtomicU32::new(0);

static FLASH_STORAGE: StaticCell<Mutex<RefCell<FlashStorage>>> = StaticCell::new();

const PANIC_OFFSET: u32 = 0x2ff000;
const PANIC_SIZE: u32 = 0x1000;

static mut FLASH_STORAGE_REF: Option<&'static Mutex<RefCell<FlashStorage>>> = None;

static LED_PIN: StaticCell<Mutex<RefCell<Option<Output<'static>>>>> = StaticCell::new();

static mut LED_PIN_REF: Option<&'static Mutex<RefCell<Option<Output<'static>>>>> = None;

use core::panic::PanicInfo;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    use core::fmt::Write;
    use esp_hal::delay::Delay;

    esp_println::println!("PANIC: {}", info);

    let mut msg = ArrayString::<1024>::new();
    let _ = write!(&mut msg, "{}", info);
    let bytes = msg.as_bytes();
    let len = bytes.len() as u32;
    let len_bytes = len.to_le_bytes();

    let data_len = bytes.len();
    let total_data_len = 4 + data_len;
    let padded_len = ((total_data_len + 31) / 32 * 32) as usize;
    let mut combined = [0xffu8; 1056]; // Enough for 4 + 1024 + padding
    combined[0..4].copy_from_slice(&len_bytes);
    combined[4..4 + data_len].copy_from_slice(bytes);

    let delay = Delay::new();

    critical_section::with(|cs| {
        let flash_storage = unsafe { FLASH_STORAGE_REF.unwrap() };
        let mut storage = flash_storage.borrow(cs).borrow_mut();
        let _ = storage.erase(PANIC_OFFSET, PANIC_OFFSET + PANIC_SIZE);
    });

    delay.delay_millis(10);

    critical_section::with(|cs| {
        let flash_storage = unsafe { FLASH_STORAGE_REF.unwrap() };
        let mut storage = flash_storage.borrow(cs).borrow_mut();
        let _ = storage.write(PANIC_OFFSET, &combined[0..padded_len]);
    });

    for _ in 0..5 {
        critical_section::with(|cs| {
            let mut led = unsafe { LED_PIN_REF.unwrap().borrow(cs).borrow_mut() };
            if let Some(pin) = led.as_mut() {
                pin.set_high();
            }
        });
        esp_println::println!("PANIC: {}", info);
        delay.delay_millis(500);
        critical_section::with(|cs| {
            let mut led = unsafe { LED_PIN_REF.unwrap().borrow(cs).borrow_mut() };
            if let Some(pin) = led.as_mut() {
                pin.set_low();
            }
        });
        delay.delay_millis(500);
    }

    esp_hal::system::software_reset();
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    // This has to be initialized before any tasks are started
    let event_channel = &*shared::EVENT_CHANNEL.init(PubSubChannel::new());

    // GPIO
    let rc_receiver_ch1_pinmap = peripherals.GPIO34;
    let mut led_pin = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());

    let led_pin_static = LED_PIN.init(Mutex::new(RefCell::new(Some(led_pin))));
    unsafe {
        LED_PIN_REF = Some(led_pin_static);
    }

    // ADC (battery voltage monitoring)
    let mut adc1_config = AdcConfig::new();
    let mut vbat_pin = adc1_config.enable_pin(peripherals.GPIO39, adc::Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    // PWM output to ESCs (LEDC channels 0 and 1)
    let esc_right_pin = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let esc_left_pin = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    let mut lstimer0 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer0);
    lstimer0
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_HZ as u32),
        })
        .unwrap();
    let lstimer0 = Box::leak(Box::new(lstimer0));

    let mut lstimer1 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer1);
    lstimer1
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_HZ as u32),
        })
        .unwrap();
    let lstimer1 = Box::leak(Box::new(lstimer1));

    let mut channel0 = ledc.channel(ledc_channel::Number::Channel0, esc_right_pin);
    channel0
        .configure(ledc_channel::config::Config {
            timer: &*lstimer0,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut channel1 = ledc.channel(ledc_channel::Number::Channel1, esc_left_pin);
    channel1
        .configure(ledc_channel::config::Config {
            timer: &*lstimer1,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let neutral_duty = target_speed_to_pwm_duty(0.0, 255);
    _ = channel0.set_duty_hw(neutral_duty);
    _ = channel1.set_duty_hw(neutral_duty);

    let channel0_static = CHANNEL0.init(Mutex::new(RefCell::new(Some(channel0))));
    unsafe {
        CHANNEL0_REF = Some(channel0_static);
    }

    let channel1_static = CHANNEL1.init(Mutex::new(RefCell::new(Some(channel1))));
    unsafe {
        CHANNEL1_REF = Some(channel1_static);
    }

    // Create MotorModulator instance
    let modulator_static = MODULATOR.init(Mutex::new(RefCell::new(
        shared::kasari::MotorModulator::new(),
    )));
    unsafe {
        MODULATOR_REF = Some(modulator_static);
    }

    // Setup timer interrupt (after esp_hal_embassy::init(timg1.timer0);)
    let mut motor_timer = timg0.timer1;
    interrupt::enable(
        motor_timer.peripheral_interrupt(),
        interrupt::Priority::Priority1,
    )
    .unwrap();
    motor_timer.load_value(esp_hal::time::Duration::from_micros(
        (1_000_000.0 / MOTOR_UPDATE_HZ) as u64,
    ));
    motor_timer.enable_auto_reload(true);
    motor_timer.enable_interrupt(true);
    motor_timer.set_interrupt_handler(motor_update_handler);
    motor_timer.start();

    let motor_timer_static = MOTOR_TIMER.init(Mutex::new(RefCell::new(Some(motor_timer))));
    unsafe {
        MOTOR_TIMER_REF = Some(motor_timer_static);
    }

    // 50Hz square wave to LIDAR encoder input (LEDC channel 2)
    let lidar_encoder_output_pin =
        Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());

    let mut lstimer2 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer2);
    lstimer2
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(sensors::LIDAR_ENCODER_HZ as u32),
        })
        .unwrap();

    let mut channel2 = ledc.channel(ledc_channel::Number::Channel2, lidar_encoder_output_pin);
    channel2
        .configure(ledc_channel::config::Config {
            timer: &lstimer2,
            duty_pct: 50,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // UART2
    let (tx_pin, rx_pin) = (peripherals.GPIO17, peripherals.GPIO16);
    let config = esp_hal::uart::Config::default()
        .with_rx(
            esp_hal::uart::RxConfig::default()
                .with_fifo_full_threshold((sensors::PACKET_SIZE * 4) as u16),
        )
        .with_baudrate(115200);
    let mut uart2 = Uart::new(peripherals.UART2, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin);

    uart2.set_interrupt_handler(uart2_handler);
    uart2.listen(UartInterrupt::RxFifoFull);

    let uart_static = UART2.init(Mutex::new(RefCell::new(Some(uart2))));
    unsafe {
        UART2_REF = Some(uart_static);
    }
    let byte_buffer =
        LIDAR_BYTE_BUFFER.init(Mutex::new(RefCell::new(ConstGenericRingBuffer::new())));
    unsafe {
        LIDAR_BYTE_BUFFER_REF = Some(byte_buffer);
    }
    let lidar_event_queue =
        LIDAR_EVENT_QUEUE.init(Mutex::new(RefCell::new(ConstGenericRingBuffer::new())));
    unsafe {
        LIDAR_EVENT_QUEUE_REF = Some(lidar_event_queue);
    }
    let signal_lidar = SIGNAL_LIDAR.init(Signal::new());
    unsafe {
        SIGNAL_LIDAR_REF = Some(signal_lidar);
    }

    // SPI (ADXL373)
    let sclk = peripherals.GPIO18;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO23;
    let cs = peripherals.GPIO5;
    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_cs(cs);

    // RC receiver PWM input
    let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt = rmt.into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80)
        .with_idle_threshold(3000)
        .with_filter_threshold(100)
        .with_carrier_modulation(false);
    let rmt_ch0 = rmt
        .channel0
        .configure_rx(rc_receiver_ch1_pinmap, rx_config)
        .unwrap();

    // Flash storage
    let flash_storage = FLASH_STORAGE.init(Mutex::new(RefCell::new(FlashStorage::new())));
    unsafe {
        FLASH_STORAGE_REF = Some(flash_storage);
    }

    // Scan flash to find next write position and sequence
    let mut max_seq: u32 = 0;
    let mut max_pos: u32 = 0;
    let mut all_erased = true;
    for i in 0..(LOG_FLASH_SIZE / BATCH_FLUSH_SIZE as u32) {
        let pos = i * BATCH_FLUSH_SIZE as u32;
        let mut buf4 = [0u8; 4];
        critical_section::with(|cs| {
            let mut storage = flash_storage.borrow(cs).borrow_mut();
            let _ = storage.read(LOG_FLASH_OFFSET + pos, &mut buf4);
        });
        let seq = u32::from_le_bytes(buf4);
        if seq != 0xFFFFFFFF {
            all_erased = false;
            if seq > max_seq || (max_seq == 0 && seq == 0) {
                // Handle wrap-around if needed, but u32 is large
                max_seq = seq;
                max_pos = pos;
            }
        }
    }
    let next_pos = if all_erased {
        0
    } else {
        (max_pos + BATCH_FLUSH_SIZE as u32) % LOG_FLASH_SIZE
    };
    let next_seq = if all_erased {
        0
    } else {
        max_seq.wrapping_add(1)
    };
    LOG_WRITE_POS.store(next_pos, Ordering::Relaxed);
    LOG_SEQ.store(next_seq, Ordering::Relaxed);

    // Spawn tasks
    spawner.spawn(sensors::lidar_publisher(event_channel)).ok();
    spawner
        .spawn(sensors::accelerometer_task(spi, event_channel))
        .ok();
    spawner
        .spawn(sensors::rmt_task(rmt_ch0, event_channel))
        .ok();

    spawner.spawn(log_task(event_channel, flash_storage)).ok();

    // Initialize the Wi-Fi controller
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng.clone()).unwrap()
    );
    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();
    let wifi_ap_device = interfaces.ap;
    let wifi_sta_device = interfaces.sta;

    // Configure Wi-Fi in AP and STA mode
    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stacks
    let (ap_stack, ap_runner) = embassy_net::new(
        wifi_ap_device,
        ap_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    let client_config = Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.into(),
            password: PASSWORD.into(),
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "kasarisw".into(),
            ..Default::default()
        },
    );
    controller.set_configuration(&client_config).unwrap();

    spawner.spawn(connection_task(controller)).ok();
    spawner.spawn(net_task(ap_runner)).ok();
    spawner.spawn(net_task(sta_runner)).ok();
    spawner.spawn(print_ap_link_task(ap_stack)).ok();
    spawner.spawn(print_sta_ip_task(sta_stack, SSID)).ok();
    spawner
        .spawn(listener_task(ap_stack, sta_stack, event_channel))
        .ok();
    spawner
        .spawn(download_task(ap_stack, sta_stack, flash_storage))
        .ok();

    // Main loop
    let mut logic = shared::kasari::MainLogic::new(false);
    let mut publisher = event_channel.publisher().unwrap();
    let mut subscriber = event_channel.subscriber().unwrap();
    let mut loop_i: u64 = 0;
    let mut lidar_report_ts: u64 = 0;

    loop {
        loop_i += 1;

        // Measure and publish Vbat
        if loop_i % 10 == 0 {
            let vbat_raw = nb::block!(adc1.read_oneshot(&mut vbat_pin)).unwrap();
            let vbat = (4095 - vbat_raw) as f32 * 0.01045;
            if shared::LOG_VBAT {
                esp_println::println!("vbat_raw: {:?}, vbat: {:?} V", vbat_raw, vbat);
            }

            publisher.publish_immediate(shared::kasari::InputEvent::Vbat(
                embassy_time::Instant::now().as_ticks(),
                vbat,
            ));
        }

        while let Some(event) = subscriber.try_next_message_pure() {
            logic.feed_event(event);
        }

        logic.step(
            embassy_time::Instant::now().as_ticks(),
            Some(&mut publisher),
            shared::LOG_DETECTION,
        );

        BATTERY_PRESENT.store(logic.battery_present, Ordering::Relaxed);

        // Pass motor control plan to motor modulator interrupt
        critical_section::with(|cs| {
            let mut modulator = unsafe { MODULATOR_REF.unwrap().borrow(cs).borrow_mut() };
            if let Some(ref plan) = logic.motor_control_plan {
                modulator.sync(
                    embassy_time::Instant::now().as_ticks(),
                    logic.detector.theta,
                    plan.clone(),
                    logic.angular_correction_total,
                );
            } else {
                modulator.mcp = None;
            }
        });

        // This allows checking that the accelerometer has been calibrated when
        // the robot is idling
        if libm::fabsf(logic.detector.rpm) < 100.0 {
            critical_section::with(|cs| {
                let mut led = unsafe { LED_PIN_REF.unwrap().borrow(cs).borrow_mut() };
                if let Some(pin) = led.as_mut() {
                    pin.set_high();
                }
            });
        } else {
            critical_section::with(|cs| {
                let mut led = unsafe { LED_PIN_REF.unwrap().borrow(cs).borrow_mut() };
                if let Some(pin) = led.as_mut() {
                    pin.set_low();
                }
            });
        }

        let current_ts = embassy_time::Instant::now().as_millis() as u32;
        if libm::fabsf(logic.detector.rpm) > 500.0 {
            LAST_HIGH_RPM_TS.store(current_ts, Ordering::Relaxed);
        }
        let time_since_high_rpm = current_ts.wrapping_sub(LAST_HIGH_RPM_TS.load(Ordering::Relaxed));
        if time_since_high_rpm < 3000 || embassy_time::Instant::now().as_millis() < 3000 {
            LOGGING_ACTIVE.store(true, Ordering::Relaxed);
        } else {
            LOGGING_ACTIVE.store(false, Ordering::Relaxed);
        }

        let current_ts = embassy_time::Instant::now().as_ticks();
        critical_section::with(|cs| {
            if current_ts > lidar_report_ts + 1_000_000 {
                let skipped = LIDAR_SKIPPED_BYTE_COUNT.swap(0, Ordering::Relaxed);
                let valid = LIDAR_VALID_BYTE_COUNT.swap(0, Ordering::Relaxed);
                println!("LIDAR: {} skipped / {} valid bytes", skipped, valid);
                lidar_report_ts = current_ts;
            }
        });

        Timer::after_millis(20).await;
    }
}

// Motor modulator interrupt
#[esp_hal::handler]
fn motor_update_handler() {
    //println!("motor_update_handler()");
    critical_section::with(|cs| {
        // Clear interrupt
        let mut timer_guard = unsafe { MOTOR_TIMER_REF.unwrap().borrow(cs).borrow_mut() };
        if let Some(timer) = timer_guard.as_mut() {
            timer.clear_interrupt();
        }

        let update_i = MOTOR_UPDATE_COUNT.fetch_add(1, Ordering::Relaxed);

        let ts = embassy_time::Instant::now().as_ticks();

        // Step modulator
        let mut modulator_guard = unsafe { MODULATOR_REF.unwrap().borrow(cs).borrow_mut() };
        let (left_rpm, right_rpm) = modulator_guard.step(ts);

        let left_percent = left_rpm / RPM_PER_THROTTLE_PERCENT;
        let right_percent = right_rpm / RPM_PER_THROTTLE_PERCENT;

        let mut duty_left = target_speed_to_pwm_duty(left_percent, 255);
        let mut duty_right = target_speed_to_pwm_duty(right_percent, 255);

        if !BATTERY_PRESENT.load(Ordering::Relaxed) {
            duty_left = 0;
            duty_right = 0;
            if update_i % 1000 == 0 {
                esp_println::println!(
                    "[{}] Shutting down motor controllers (battery not present)",
                    update_i
                );
            }
        } else if shared::LOG_MOTOR_CONTROL && update_i % 100 == 0 {
            esp_println::println!(
                "[{}] Setting motor duties: left={}%, right={}%",
                update_i,
                left_percent,
                right_percent,
            );
        }

        // Update channels (right on channel0, left on channel1)
        let mut ch0_guard = unsafe { CHANNEL0_REF.unwrap().borrow(cs).borrow_mut() };
        if let Some(ch0) = ch0_guard.as_mut() {
            _ = ch0.set_duty_hw(duty_right);
        }

        let mut ch1_guard = unsafe { CHANNEL1_REF.unwrap().borrow(cs).borrow_mut() };
        if let Some(ch1) = ch1_guard.as_mut() {
            _ = ch1.set_duty_hw(duty_left);
        }
    });
}

fn process_raw_lidar_distance(d0: u16) -> f32 {
    if d0 < 50 {
        0.0 // The LIDAR doesn't see anything
    } else {
        d0 as f32 + sensors::LIDAR_DISTANCE_OFFSET
    }
}

// LIDAR UART handler
#[esp_hal::handler]
fn uart2_handler() {
    //println!("uart2_handler");
    critical_section::with(|cs| {
        let mut uart_ref = unsafe { UART2_REF.unwrap().borrow(cs).borrow_mut() };
        if let Some(uart) = uart_ref.as_mut() {
            let mut temp = [0u8; 128];
            let read_bytes = uart.read_buffered(&mut temp).unwrap_or(0);
            uart.clear_interrupts(UartInterrupt::RxFifoFull.into());

            // Accumulate new bytes into ring buffer
            let mut buf_guard = unsafe { LIDAR_BYTE_BUFFER_REF.unwrap().borrow(cs).borrow_mut() };
            for &byte in &temp[0..read_bytes] {
                if buf_guard.len() < 256 {
                    buf_guard.push(byte);
                } else {
                    println!("LIDAR: Byte buffer overflow");
                }
            }
            drop(buf_guard); // Release before parsing

            // Now parse from ring buffer
            let timestamp_end = embassy_time::Instant::now().as_ticks();
            let mut parsed_packets: ArrayVec<sensors::ParsedPacket, 6> = ArrayVec::new();
            let mut buf_guard = unsafe { LIDAR_BYTE_BUFFER_REF.unwrap().borrow(cs).borrow_mut() };

            while buf_guard.len() >= sensors::PACKET_SIZE {
                // Peek at first byte without dequeue
                if *buf_guard.get(0).unwrap_or(&0) != sensors::HEAD_BYTE {
                    // Skip invalid until head or end
                    while let Some(&byte) = buf_guard.get(0) {
                        if byte == sensors::HEAD_BYTE {
                            break;
                        }
                        buf_guard.dequeue();
                        let skipped = LIDAR_SKIPPED_BYTE_COUNT.fetch_add(1, Ordering::Relaxed);
                        // Report skipped bytes for one packet's length
                        if skipped < sensors::PACKET_SIZE as u32 {
                            println!("LISKIP {:02x}", byte);
                        } else if skipped == sensors::PACKET_SIZE as u32 {
                            println!("LISKIP {:02x} (hiding futher reports)", byte);
                        }
                    }
                    continue;
                }

                // Extract packet (dequeue 22 bytes)
                let mut packet = [0u8; sensors::PACKET_SIZE];
                for i in 0..sensors::PACKET_SIZE {
                    LIDAR_VALID_BYTE_COUNT.fetch_add(1, Ordering::Relaxed);
                    packet[i] = buf_guard.dequeue().unwrap_or(0);
                }

                if let Some(parsed) = sensors::parse_packet(&packet) {
                    if parsed_packets.try_push(parsed).is_err() {
                        println!("LIDAR: Batch overflow");
                    }
                } else {
                    println!("LIDAR: Invalid packet in batch");
                }
            }

            // Extrapolate and queue events (as before)
            let batch_size = parsed_packets.len() as u64;
            let mut queue = unsafe { LIDAR_EVENT_QUEUE_REF.unwrap().borrow(cs).borrow_mut() };
            for (i, parsed) in parsed_packets.into_iter().enumerate() {
                let ts = timestamp_end
                    .saturating_sub((batch_size - i as u64 - 1) * LIDAR_PACKET_INTERVAL_US);
                let event = kasari::InputEvent::Lidar(
                    ts,
                    process_raw_lidar_distance(parsed.distances[0]),
                    process_raw_lidar_distance(parsed.distances[1]),
                    process_raw_lidar_distance(parsed.distances[2]),
                    process_raw_lidar_distance(parsed.distances[3]),
                );
                queue.push(event);

                let packet_i = LIDAR_PACKET_COUNT.fetch_add(1, Ordering::Relaxed);
                if (packet_i % 100 == 1 || shared::LOG_ALL_LIDAR) && shared::LOG_LIDAR {
                    // This message has to be quite short when logging all
                    // LIDAR messages instead of just every 20th
                    println!(
                        "Lidar {}: t={}, d={},{},{},{}",
                        packet_i,
                        ts,
                        parsed.distances[0],
                        parsed.distances[1],
                        parsed.distances[2],
                        parsed.distances[3]
                    );
                }
            }
        }
    });
    unsafe {
        SIGNAL_LIDAR_REF.unwrap().signal(());
    }
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());

    println!("Starting wifi");
    controller.start_async().await.unwrap();
    println!("Wifi started!");

    loop {
        match esp_wifi::wifi::ap_state() {
            WifiState::ApStarted => {
                println!("About to connect...");

                match controller.connect_async().await {
                    Ok(_) => {
                        // wait until we're no longer connected
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        println!("STA disconnected");
                    }
                    Err(e) => {
                        println!("Failed to connect to wifi: {e:?}");
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return,
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

#[embassy_executor::task]
async fn print_ap_link_task(ap_stack: embassy_net::Stack<'static>) {
    loop {
        if ap_stack.is_link_up() {
            println!("Connect to the AP `kasarisw` and connect to TCP 192.168.2.1:8080");
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::task]
async fn print_sta_ip_task(sta_stack: embassy_net::Stack<'static>, ssid: &'static str) {
    let mut sta_address = None;
    loop {
        if let Some(config) = sta_stack.config_v4() {
            sta_address = Some(config.address.address());
            println!("Got IP: {}", sta_address.unwrap());
            println!(
                "Or connect to the ap `{ssid}` and connect to TCP {}:8080",
                sta_address.unwrap()
            );
            break;
        }
        println!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    }
}

use crate::shared::EventChannel;
use embassy_futures::select::{select, Either};
use embedded_io_async::Write;

#[embassy_executor::task]
async fn listener_task(
    ap_stack: embassy_net::Stack<'static>,
    sta_stack: embassy_net::Stack<'static>,
    event_channel: &'static EventChannel,
) {
    let mut subscriber = event_channel.subscriber().unwrap();
    let publisher = event_channel.publisher().unwrap();

    let mut ap_rx_buffer = [0; 1536];
    let mut ap_tx_buffer = [0; 1536];
    let mut sta_rx_buffer = [0; 1536];
    let mut sta_tx_buffer = [0; 1536];

    let mut ap_socket = TcpSocket::new(ap_stack, &mut ap_rx_buffer, &mut ap_tx_buffer);
    let mut sta_socket = TcpSocket::new(sta_stack, &mut sta_rx_buffer, &mut sta_tx_buffer);

    let mut limit_streaming = false;
    let mut last_lidar_sent: u64 = 0;
    let mut last_accel_sent: u64 = 0;
    const LIDAR_INTERVAL_US: u64 = 500_000; // 2 per second
    const ACCEL_INTERVAL_US: u64 = 500_000; // 2 per second

    loop {
        println!("Wait for connection...");
        let either = select(
            ap_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
            sta_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8080,
            }),
        )
        .await;

        let (r, socket) = match either {
            Either::First(r) => (r, &mut ap_socket),
            Either::Second(r) => (r, &mut sta_socket),
        };

        if let Err(e) = r {
            println!("Accept error: {:?}", e);
            continue;
        }

        println!("Connected!");
        socket.set_timeout(None);

        let mut rx_buffer = [0u8; 512];
        let mut rx_pos = 0;

        loop {
            let read_fut = socket.read(&mut rx_buffer[rx_pos..]);
            let event_fut = subscriber.next_message_pure();

            match select(read_fut, event_fut).await {
                Either::Second(event) => {
                    let should_send = if limit_streaming {
                        match &event {
                            InputEvent::Lidar(..) => {
                                let ts = event.timestamp();
                                if ts >= last_lidar_sent + LIDAR_INTERVAL_US {
                                    last_lidar_sent = ts;
                                    true
                                } else {
                                    false
                                }
                            }
                            InputEvent::Accelerometer(..) => {
                                let ts = event.timestamp();
                                if ts >= last_accel_sent + ACCEL_INTERVAL_US {
                                    last_accel_sent = ts;
                                    true
                                } else {
                                    false
                                }
                            }
                            _ => true,
                        }
                    } else {
                        true
                    };

                    if should_send {
                        let serialized = kasari::serialize_event(&event);
                        if let Err(e) = socket.write_all(&serialized).await {
                            println!("Write error: {:?}", e);
                            break;
                        }
                        // Don't flush here. Events are too small and throughput
                        // will be severely limited.
                    }
                }
                Either::First(Ok(0)) => {
                    println!("Client disconnected");
                    break;
                }
                Either::First(Ok(len)) => {
                    rx_pos += len;
                    // Process binary events
                    while rx_pos >= 1 {
                        let tag = rx_buffer[0];
                        if tag == 4 {
                            // Assuming tag 4 for WifiControl
                            if rx_pos >= 1 + 8 + 1 + 4 + 4 + 4 {
                                // u8 tag + u64 ts + u8 mode + 3*f32
                                let _ts_orig =
                                    u64::from_le_bytes(rx_buffer[1..9].try_into().unwrap());
                                let mode = rx_buffer[9];
                                let r = f32::from_le_bytes(rx_buffer[10..14].try_into().unwrap());
                                let m = f32::from_le_bytes(rx_buffer[14..18].try_into().unwrap());
                                let t = f32::from_le_bytes(rx_buffer[18..22].try_into().unwrap());

                                limit_streaming = mode == 3;

                                // Use local timestamp in order to make all
                                // event log timestamps have the same local
                                // timebase
                                let ts = embassy_time::Instant::now().as_ticks();
                                publisher.publish_immediate(
                                    shared::kasari::InputEvent::WifiControl(ts, mode, r, m, t),
                                );
                                // Shift buffer
                                let shift_len = 22;
                                rx_buffer.copy_within(shift_len.., 0);
                                rx_pos -= shift_len;

                                // Flush the sending socket as we receive valid
                                // events. This should be a good interval for
                                // flushing
                                if let Err(e) = socket.flush().await {
                                    println!("Flush error: {:?}", e);
                                    break;
                                }
                            } else {
                                break;
                            }
                        } else {
                            // Unknown tag, skip or handle error
                            println!("Unknown tag: {}", tag);
                            // Shift by 1 to skip invalid tag
                            rx_buffer.copy_within(1.., 0);
                            rx_pos -= 1;
                        }
                    }
                    if rx_pos == rx_buffer.len() {
                        println!("Buffer full without complete event, dropping data");
                        rx_pos = 0;
                    }
                }
                Either::First(Err(e)) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            }
        }

        socket.close();
        Timer::after(Duration::from_millis(1000)).await;
        socket.abort();
    }
}

#[embassy_executor::task]
async fn download_task(
    ap_stack: embassy_net::Stack<'static>,
    sta_stack: embassy_net::Stack<'static>,
    flash_storage: &'static Mutex<RefCell<FlashStorage>>,
) {
    let mut ap_rx_buffer = [0; 512];
    let mut ap_tx_buffer = [0; 512];
    let mut sta_rx_buffer = [0; 512];
    let mut sta_tx_buffer = [0; 512];

    let mut ap_socket = TcpSocket::new(ap_stack, &mut ap_rx_buffer, &mut ap_tx_buffer);
    let mut sta_socket = TcpSocket::new(sta_stack, &mut sta_rx_buffer, &mut sta_tx_buffer);

    loop {
        println!("Wait for download connection on port 8081...");
        let either = select(
            ap_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8081,
            }),
            sta_socket.accept(IpListenEndpoint {
                addr: None,
                port: 8081,
            }),
        )
        .await;

        let (r, socket) = match either {
            Either::First(r) => (r, &mut ap_socket),
            Either::Second(r) => (r, &mut sta_socket),
        };

        if let Err(e) = r {
            println!("Accept error: {:?}", e);
            continue;
        }

        println!("Download connected!");
        socket.set_timeout(None);

        // Send panic info if available
        let mut len_buf = [0u8; 4];
        critical_section::with(|cs| {
            let mut storage = flash_storage.borrow(cs).borrow_mut();
            let _ = storage.read(PANIC_OFFSET, &mut len_buf);
        });
        let len = u32::from_le_bytes(len_buf);

        if len > 0 && len <= (PANIC_SIZE - 4) {
            let padded_len = (((len as usize) + 3) / 4 * 4) as usize;
            let mut panic_buf = alloc::vec![0u8; padded_len];
            critical_section::with(|cs| {
                let mut storage = flash_storage.borrow(cs).borrow_mut();
                let read_result = storage.read(PANIC_OFFSET + 4, &mut panic_buf);
                if read_result.is_err() {
                    println!("Panic message read failed: {:?}", read_result);
                }
            });

            let mut special_batch = [0u8; BATCH_FLUSH_SIZE as usize];
            special_batch[0..4].copy_from_slice(&0xffffffffu32.to_le_bytes());
            special_batch[4..4 + (len as usize)].copy_from_slice(&panic_buf[0..(len as usize)]);

            if let Err(e) = socket.write_all(&special_batch).await {
                println!("Write error during panic send: {:?}", e);
            } else {
                // Clear panic after sending
                critical_section::with(|cs| {
                    let mut storage = flash_storage.borrow(cs).borrow_mut();
                    let _ = storage.erase(PANIC_OFFSET, PANIC_OFFSET + PANIC_SIZE);
                });
            }
        }

        // Dump all flash log data
        for i in 0..(LOG_FLASH_SIZE / BATCH_FLUSH_SIZE as u32) {
            let pos = i * BATCH_FLUSH_SIZE as u32;
            let mut buf = [0u8; BATCH_FLUSH_SIZE];
            critical_section::with(|cs| {
                let mut storage = flash_storage.borrow(cs).borrow_mut();
                let _ = storage.read(LOG_FLASH_OFFSET + pos, &mut buf);
            });
            let mut all_ff = true;
            for b in buf {
                if b != 0xff {
                    all_ff = false;
                    break;
                }
            }
            if all_ff {
                break;
            }
            if let Err(e) = socket.write_all(&buf).await {
                println!("Write error during download: {:?}", e);
                break;
            }
        }

        socket.close();

        println!("Download complete. Clearing storage...");

        // Clear the storage
        for i in 0..(LOG_FLASH_SIZE / BATCH_FLUSH_SIZE as u32) {
            let pos = i * BATCH_FLUSH_SIZE as u32;
            critical_section::with(|cs| {
                let mut storage = flash_storage.borrow(cs).borrow_mut();
                let _ = storage.erase(
                    LOG_FLASH_OFFSET + pos,
                    LOG_FLASH_OFFSET + pos + BATCH_FLUSH_SIZE as u32,
                );
            });
        }
        LOG_WRITE_POS.store(0, Ordering::Relaxed);
        LOG_SEQ.store(0, Ordering::Relaxed);

        println!("Storage cleared.");

        // Abort socket after some time. It's best not to do this too quickly
        // because then the client wouldn't get informed about the socket
        // closing.
        Timer::after(Duration::from_millis(5000)).await;
        socket.abort();
    }
}

#[embassy_executor::task]
async fn log_task(
    event_channel: &'static shared::EventChannel,
    flash_storage: &'static Mutex<RefCell<FlashStorage>>,
) {
    use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

    let mut subscriber = event_channel.subscriber().unwrap();
    let ram_buffer = LOG_BUFFER.init(Mutex::new(RefCell::new(ConstGenericRingBuffer::new())));

    loop {
        let event = match subscriber.next_message().await {
            embassy_sync::pubsub::WaitResult::Message(event) => event,
            _ => continue,
        };

        if !LOGGING_ACTIVE.load(Ordering::Relaxed) {
            continue;
        }

        let serialized = kasari::serialize_event(&event);
        critical_section::with(|cs| {
            let mut buf = ram_buffer.borrow(cs).borrow_mut();
            for &byte in &serialized {
                if buf.is_full() {
                    break;
                }
                buf.push(byte);
            }
        });

        critical_section::with(|cs| {
            let mut buf = ram_buffer.borrow(cs).borrow_mut();
            if buf.len() >= BATCH_DATA_SIZE {
                let pos = LOG_WRITE_POS.load(Ordering::Relaxed);
                let seq = LOG_SEQ.fetch_add(1, Ordering::Relaxed);

                critical_section::with(|inner_cs| {
                    let mut storage = flash_storage.borrow(inner_cs).borrow_mut();
                    let _ = storage.erase(
                        LOG_FLASH_OFFSET + pos,
                        LOG_FLASH_OFFSET + pos + BATCH_FLUSH_SIZE as u32,
                    );
                });

                let mut temp = [0u8; BATCH_FLUSH_SIZE];
                temp[0..4].copy_from_slice(&seq.to_le_bytes());
                for i in 0..BATCH_DATA_SIZE {
                    temp[4 + i] = buf.dequeue().unwrap();
                }

                critical_section::with(|inner_cs| {
                    let mut storage = flash_storage.borrow(inner_cs).borrow_mut();
                    let _ = storage.write(LOG_FLASH_OFFSET + pos, &temp);
                });

                let new_pos = (pos + BATCH_FLUSH_SIZE as u32) % LOG_FLASH_SIZE;
                LOG_WRITE_POS.store(new_pos, Ordering::Relaxed);
                println!(
                    "Logged {} bytes at 0x{:x} with seq {}",
                    BATCH_DATA_SIZE, pos, seq
                );
            }
        });
    }
}
