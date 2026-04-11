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
    gpio::{Level, Output, OutputConfig},
    mcpwm::{McPwm, PeripheralClockConfig},
    time::Rate,
    timer::timg::TimerGroup,
};
use {esp_backtrace as _, esp_println as _};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const PWM_PERIOD_TICKS: u16 = 99;
const PWM_FREQUENCY_KHZ: u32 = 20;
const PWM_MIN_DUTY_PERCENT: u16 = 10;
const PWM_MAX_DUTY_PERCENT: u16 = 100;
const RAMP_STEP_MS: u64 = 50;
const DIRECTION_CHANGE_DELAY_MS: u64 = 500;

fn duty_pct_to_timestamp(duty_pct: u16) -> u16 {
    (PWM_PERIOD_TICKS * duty_pct) / 100
}

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

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);

    let mut left_pwm = mcpwm.operator0.with_pin_a(
        left_pwm_pin,
        esp_hal::mcpwm::operator::PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let mut right_pwm = mcpwm.operator1.with_pin_a(
        right_pwm_pin,
        esp_hal::mcpwm::operator::PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
            Rate::from_khz(PWM_FREQUENCY_KHZ),
        )
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let mut left_enable = Output::new(left_enable_pin, Level::High, OutputConfig::default());
    let mut right_enable = Output::new(right_enable_pin, Level::High, OutputConfig::default());

    info!(
        "motor ramp active, left/right pwm alternate, duty {}-{}%, freq={}kHz, period={}",
        PWM_MIN_DUTY_PERCENT, PWM_MAX_DUTY_PERCENT, PWM_FREQUENCY_KHZ, PWM_PERIOD_TICKS,
    );

    let _ = spawner;

    loop {
        right_pwm.set_timestamp(0);
        for duty in PWM_MIN_DUTY_PERCENT..=PWM_MAX_DUTY_PERCENT {
            left_pwm.set_timestamp(duty_pct_to_timestamp(duty));
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        for duty in (PWM_MIN_DUTY_PERCENT..PWM_MAX_DUTY_PERCENT).rev() {
            left_pwm.set_timestamp(duty_pct_to_timestamp(duty));
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        left_pwm.set_timestamp(0);
        left_enable.set_low();
        right_enable.set_low();
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
        left_enable.set_high();
        right_enable.set_high();

        for duty in PWM_MIN_DUTY_PERCENT..=PWM_MAX_DUTY_PERCENT {
            right_pwm.set_timestamp(duty_pct_to_timestamp(duty));
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        for duty in (PWM_MIN_DUTY_PERCENT..PWM_MAX_DUTY_PERCENT).rev() {
            right_pwm.set_timestamp(duty_pct_to_timestamp(duty));
            Timer::after(Duration::from_millis(RAMP_STEP_MS)).await;
        }

        right_pwm.set_timestamp(0);
        left_enable.set_low();
        right_enable.set_low();
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
        left_enable.set_high();
        right_enable.set_high();
    }
}
