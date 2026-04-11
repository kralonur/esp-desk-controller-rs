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
    mcpwm::{McPwm, PeripheralClockConfig},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_pwm_motor::{
    motor::Motor,
    quadrature::{Quadrature, QuadratureDirection, QuadratureWatcher},
};
use {esp_backtrace as _, esp_println as _};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const HALL1_GPIO: u8 = 5;
const HALL2_GPIO: u8 = 6;
const PWM_PERIOD_TICKS: u16 = 99;
const PWM_FREQUENCY_KHZ: u32 = 20;
const DIRECTION_CHANGE_DELAY_MS: u64 = 500;

#[embassy_executor::task]
async fn watch_quadrature(mut watcher: QuadratureWatcher) {
    loop {
        let event = watcher.wait_for_change().await;
        let snapshot = event.snapshot;

        match event.direction {
            QuadratureDirection::Positive => {
                info!(
                    "quad pos={=i32} dir={=str} a={=bool} b={=bool} state={=u8} via_hall{=u8}",
                    snapshot.position,
                    "positive",
                    snapshot.hall1_high(),
                    snapshot.hall2_high(),
                    snapshot.state,
                    event.channel,
                );
            }
            QuadratureDirection::Negative => {
                info!(
                    "quad pos={=i32} dir={=str} a={=bool} b={=bool} state={=u8} via_hall{=u8}",
                    snapshot.position,
                    "negative",
                    snapshot.hall1_high(),
                    snapshot.hall2_high(),
                    snapshot.state,
                    event.channel,
                );
            }
            QuadratureDirection::Invalid => {
                info!(
                    "quad invalid state={=u8} hall{=u8}={:?} errors={=u32}",
                    snapshot.state, event.channel, event.level, snapshot.invalid_transitions,
                );
            }
        }
    }
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

    let mut motor = Motor::new(
        left_enable_pin,
        right_enable_pin,
        left_pwm_pin,
        right_pwm_pin,
        mcpwm.operator0,
    );

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
            Rate::from_khz(PWM_FREQUENCY_KHZ),
        )
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let (quadrature, snapshot) = Quadrature::new(peripherals.GPIO5, peripherals.GPIO6);
    let log_watcher = quadrature.watcher();
    let leg_watcher = quadrature.watcher();
    quadrature.spawn(&spawner);
    spawner.must_spawn(watch_quadrature(log_watcher));

    motor.enable();

    let mut leg = esp_pwm_motor::leg::Leg::new(motor, leg_watcher);

    info!(
        "quadrature monitor initialized on GPIO{=u8} and GPIO{=u8}",
        HALL1_GPIO, HALL2_GPIO
    );
    info!(
        "quadrature initial a={=bool} b={=bool} state={=u8} pos={=i32} errors={=u32}",
        snapshot.hall1_high(),
        snapshot.hall2_high(),
        snapshot.state,
        snapshot.position,
        snapshot.invalid_transitions,
    );

    info!(
        "motor control active, freq={}kHz, period={}",
        PWM_FREQUENCY_KHZ, PWM_PERIOD_TICKS,
    );

    loop {
        leg.move_up_step(100).await;
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
        leg.move_down_step(100).await;
        Timer::after(Duration::from_millis(DIRECTION_CHANGE_DELAY_MS)).await;
    }
}
