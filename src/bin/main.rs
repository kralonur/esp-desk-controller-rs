#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    gpio::{DriveMode, Level, Output, OutputConfig},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use {esp_backtrace as _, esp_println as _};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const PWM_FREQUENCY_KHZ: u32 = 20;
const PWM_MIN_DUTY_PERCENT: u8 = 10;
const PWM_MAX_DUTY_PERCENT: u8 = 100;
const RAMP_STEP_MS: u64 = 50;
const DIRECTION_CHANGE_DELAY_MS: u64 = 500;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let left_enable_pin = peripherals.GPIO4;
    let right_enable_pin = peripherals.GPIO3;
    let left_pwm_pin = peripherals.GPIO2;
    let right_pwm_pin = peripherals.GPIO1;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let mut left_enable = Output::new(left_enable_pin, Level::High, OutputConfig::default());
    let mut right_enable = Output::new(right_enable_pin, Level::High, OutputConfig::default());

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut pwm_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    pwm_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(PWM_FREQUENCY_KHZ),
        })
        .unwrap();

    let mut left_pwm = ledc.channel(channel::Number::Channel0, left_pwm_pin);
    left_pwm
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    let mut right_pwm = ledc.channel(channel::Number::Channel1, right_pwm_pin);
    right_pwm
        .configure(channel::config::Config {
            timer: &pwm_timer,
            duty_pct: 0,
            drive_mode: DriveMode::PushPull,
        })
        .unwrap();

    info!(
        "motor ramp active, left/right pwm alternate, duty {}-{}%, freq={}kHz",
        PWM_MIN_DUTY_PERCENT, PWM_MAX_DUTY_PERCENT, PWM_FREQUENCY_KHZ,
    );

    let _ = spawner;

    loop {
        right_pwm.set_duty(0).unwrap();
        for duty in PWM_MIN_DUTY_PERCENT..=PWM_MAX_DUTY_PERCENT {
            left_pwm.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        for duty in (PWM_MIN_DUTY_PERCENT..PWM_MAX_DUTY_PERCENT).rev() {
            left_pwm.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        left_pwm.set_duty(0).unwrap();
        left_enable.set_low();
        right_enable.set_low();
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
        left_enable.set_high();
        right_enable.set_high();

        for duty in PWM_MIN_DUTY_PERCENT..=PWM_MAX_DUTY_PERCENT {
            right_pwm.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        for duty in (PWM_MIN_DUTY_PERCENT..PWM_MAX_DUTY_PERCENT).rev() {
            right_pwm.set_duty(duty).unwrap();
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        right_pwm.set_duty(0).unwrap();
        left_enable.set_low();
        right_enable.set_low();
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
        left_enable.set_high();
        right_enable.set_high();
    }
}
