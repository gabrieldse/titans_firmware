#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
// For embassy (RTOS)
use embassy_executor::Spawner;
// use embassy_time::{Duration, Timer};
use esp_hal::timer::timg::TimerGroup;

// For messaging
use defmt::info;

use esp_hal::clock::CpuClock;
use esp_hal::gpio::DriveMode;
use esp_hal::time::Rate;
use {esp_backtrace as _, esp_println as _};

// For LEDC
use esp_hal::ledc::channel::ChannelIFace;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::{HighSpeed, Ledc, channel, timer};

// For the motor
use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::{Level, Output, OutputConfig};

// For the Radio
use esp_hal::uart::{Config, Uart};
// use embedded_io_async::Read;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // General ESP initialization
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Start timergroup on hardware TIMG0
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);
    info!("Embassy initialized!");
    // TODO: Spawn some tasks
    let _ = spawner;

    // Radio setup
    let uart_config = Config::default().with_baudrate(420_000); // ExpressLRS default baudrate
    let uart1 = Uart::new(peripherals.UART1, uart_config)
        .unwrap()
        .with_rx(peripherals.GPIO16)
        .with_tx(peripherals.GPIO17);
    let (mut rx, _tx) = uart1.into_async().split();
    let mut buf = [0u8; 100];

    //Motors setup
    let mut lmotor = peripherals.GPIO32; // Left motor
    let mut rmotor = peripherals.GPIO33; // Right motor

    let mut in1_pin = Output::new(peripherals.GPIO25, Level::Low, OutputConfig::default());
    let mut in2_pin = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let mut in3_pin = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());
    let mut in4_pin = Output::new(peripherals.GPIO14, Level::Low, OutputConfig::default());

    // LEDC setup - LEDC PWM controller
    let ledc = Ledc::new(peripherals.LEDC);
    //let led = peripherals.GPIO5;

    // Timer that will be used for motor PWM signal
    let mut hstimer0 = ledc.timer::<HighSpeed>(timer::Number::Timer0);

    hstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty12Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: Rate::from_hz(50), // 50Hz - frequency of motor PWM signal
        })
        .unwrap();

    // The LEDC has 16 channels, we will use channel 0 for the left motor and channel 1 for the right motor
    let mut channel0 = ledc.channel(channel::Number::Channel0, lmotor.reborrow());
    let mut channel1 = ledc.channel(channel::Number::Channel1, rmotor.reborrow());

    channel0
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    channel1
        .configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    // Radio values to motor duty cycle mapping
    let max_radio = 1811;
    let min_radio = 175;
    let deadzone_radio = 1;

    let max_duty_cycle = channel0.max_duty_cycle() as u32;
    let min_duty_cycle = 0;

    let mut lmotor_pwm: u16;
    let mut rmotor_pwm: u16;

    let mut in1_state;
    let mut in2_state;
    let mut in3_state;
    let mut in4_state;

    // let duty = 512;
    // channel0.set_duty_cycle(duty).unwrap();
    // let min_duty = (25 * max_duty_cycle) / 1000; //  2.5% duty cycle
    // let max_duty = (125 * max_duty_cycle) / 1000; // 12.5% duty cycle

    // let duty_gap = max_duty - min_duty; // 512 - 102 = 410

    loop {
        // Read radio commands
        match rx.read_async(&mut buf).await {
            Ok(count) => {
                if count > 0 {
                    // info!("Received {} bytes: {:?}", count, &buf[0..count]);

                    let (yaw, throttle, pitch, roll) = parse_channels(&buf);

                    let gas = cast_to_u16(pitch, max_radio, min_radio);
                    let steering = cast_to_u16(roll, max_radio, min_radio);

                    if buf[0] == 200 && buf[2] == 22 && count >= 24 {
                        info!("R: {} | P: {} | T: {} | Y: {}", roll, pitch, throttle, yaw);

                        (
                            lmotor_pwm, rmotor_pwm, in1_state, in2_state, in3_state, in4_state,
                        ) = gas_steering_to_diff_wheel(
                            gas,
                            steering,
                            min_duty_cycle,
                            max_duty_cycle,
                            deadzone_radio,
                        );

                        if in1_state {
                            in1_pin.set_high()
                        } else {
                            in1_pin.set_low()
                        }
                        if in2_state {
                            in2_pin.set_high()
                        } else {
                            in2_pin.set_low()
                        }
                        if in3_state {
                            in3_pin.set_high()
                        } else {
                            in3_pin.set_low()
                        }
                        if in4_state {
                            in4_pin.set_high()
                        } else {
                            in4_pin.set_low()
                        }

                        channel0.set_duty_cycle(lmotor_pwm).unwrap(); // Control left motor
                        channel1.set_duty_cycle(rmotor_pwm).unwrap(); // Control right motor
                    }
                }
            }
            Err(e) => info!("UART Error: {:?}", e),
            // Idealy blink a led to debub
        }
    }
}

fn gas_steering_to_diff_wheel(
    gas: u16,
    steering: u16,
    min_duty: u32,
    max_duty: u32,
    deadzone: u32,
) -> (u16, u16, bool, bool, bool, bool) {
    let mean_u16 = u16::MAX / 2;

    let mut lmotor_pwm: u32 = min_duty;
    let mut rmotor_pwm: u32 = min_duty;

    let mut in1: bool = false;
    let mut in2: bool = false;
    let mut in3: bool = false;
    let mut in4: bool = false;

    // Dont move if in deadzone
    if gas < (mean_u16 + deadzone as u16) && gas > (mean_u16 - deadzone as u16)
        && gas | steering > (mean_u16 - deadzone as u16)
    {
        return (lmotor_pwm as u16, rmotor_pwm as u16, in1, in2, in3, in4);
    }

    if steering > mean_u16 {
        // Turn right
        let steering_offset = steering - mean_u16;

        lmotor_pwm = min_duty
            + ((max_duty - min_duty) as u32 * (steering_offset as u32) / (mean_u16 as u32));
        rmotor_pwm = min_duty;
    } else {
        // Turn left
        let steering_offset = mean_u16 - steering;
        lmotor_pwm = min_duty;
        rmotor_pwm = min_duty
            + ((max_duty - min_duty) as u32 * (steering_offset as u32) / (mean_u16 as u32));
    }

    // Decide if forward or backward differential drive
    if gas >= (mean_u16) {
        // Forward
        in1 = true;
        in3 = true;
        in2 = false;
        in4 = false;
    } else {
        // Backward
        in1 = false;
        in3 = false;
        in2 = true;
        in4 = true;
    }

    (lmotor_pwm as u16, rmotor_pwm as u16, in1, in2, in3, in4)
}

fn cast_to_u16(value: u16, max: u16, min: u16) -> u16 {
    if value >= max {
        u16::MAX
    } else if value <= min {
        u16::MIN
    } else {
        (value / (max - min) * (u16::MAX)) as u16
    }
}

// Crossfire Serial Protocol (CRSF) channel parsing (developed by Team BlackSheep)
fn parse_channels(buf: &[u8]) -> (u16, u16, u16, u16) {
    let b = &buf[3..]; // Slice starting at index 3

    // Channel 1 (Roll) - 11 bits
    // 8 bits from b[0] | 3 bits from b[1]
    let ch1 = (b[0] as u16) | ((b[1] as u16 & 0x07) << 8);

    // Channel 2 (Pitch) - 11 bits
    // 5 bits from b[1] (shifted down) | 6 bits from b[2]
    let ch2 = ((b[1] as u16) >> 3) | ((b[2] as u16 & 0x3F) << 5);

    // Channel 3 (Throttle) - 11 bits
    // 2 bits from b[2] (shifted down) | 8 bits from b[3] | 1 bit from b[4]
    let ch3 = ((b[2] as u16) >> 6) | ((b[3] as u16) << 2) | ((b[4] as u16 & 0x01) << 10);

    // Channel 4 (Yaw) - 11 bits
    // 7 bits from b[4] (shifted down) | 4 bits from b[5]
    let ch4 = ((b[4] as u16) >> 1) | ((b[5] as u16 & 0x0F) << 7);

    (ch1, ch2, ch3, ch4)
}
