#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    clock::CpuClock,
    mcpwm::{McPwm, PeripheralClockConfig},
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_pwm_motor::{
    desk::{Desk, DeskStatusStorage, DeskStatusWatcher},
    leg::{DriveSide, LegConfig, LegStatusStorage},
    motor::Motor,
    quadrature::{Quadrature, QuadratureDirection, QuadratureStorage},
};
use {esp_backtrace as _, esp_println as _};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const PWM_PERIOD_TICKS: u16 = 99;
const PWM_FREQUENCY_KHZ: u32 = 20;
const RUN_MOVE_TEST_SEQUENCE: bool = true;

#[embassy_executor::task]
async fn watch_desk_status(mut watcher: DeskStatusWatcher) {
    loop {
        let status = watcher.wait_for_change().await;
        info!(
            "desk homed={=bool} rehome={=bool} motion={:?} target_active={=bool} target={=i32} left={=i32} right={=i32} avg={=i32} skew={=i32} min={=i32} max={=i32}",
            status.homed,
            status.needs_rehome,
            status.motion,
            status.target_active,
            status.target_position,
            status.left_position,
            status.right_position,
            status.average_position,
            status.skew_counts,
            status.min_position,
            status.max_position,
        );
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0
    static QUADRATURE1_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static QUADRATURE2_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static LEG1_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static LEG2_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static DESK_STATUS_STORAGE: DeskStatusStorage = DeskStatusStorage::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let leg_1_left_enable_pin = peripherals.GPIO4;
    let leg_1_right_enable_pin = peripherals.GPIO3;
    let leg_1_left_pwm_pin = peripherals.GPIO2;
    let leg_1_right_pwm_pin = peripherals.GPIO1;
    let leg_1_hall1_pin = peripherals.GPIO5;
    let leg_1_hall2_pin = peripherals.GPIO6;

    let leg_2_left_enable_pin = peripherals.GPIO10;
    let leg_2_right_enable_pin = peripherals.GPIO11;
    let leg_2_left_pwm_pin = peripherals.GPIO12;
    let leg_2_right_pwm_pin = peripherals.GPIO13;
    let leg_2_hall1_pin = peripherals.GPIO9;
    let leg_2_hall2_pin = peripherals.GPIO8;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    mcpwm.operator1.set_timer(&mcpwm.timer0);

    let mut motor_1 = Motor::new(
        leg_1_left_enable_pin,
        leg_1_right_enable_pin,
        leg_1_left_pwm_pin,
        leg_1_right_pwm_pin,
        mcpwm.operator0,
    );

    let mut motor_2 = Motor::new(
        leg_2_left_enable_pin,
        leg_2_right_enable_pin,
        leg_2_left_pwm_pin,
        leg_2_right_pwm_pin,
        mcpwm.operator1,
    );

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(
            PWM_PERIOD_TICKS,
            esp_hal::mcpwm::timer::PwmWorkingMode::Increase,
            Rate::from_khz(PWM_FREQUENCY_KHZ),
        )
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let (quadrature_1, _snapshot_1) =
        Quadrature::new(&QUADRATURE1_STORAGE, leg_1_hall1_pin, leg_1_hall2_pin);
    let leg_watcher_1 = quadrature_1.watcher();
    let leg_status_source_1 = quadrature_1.watcher();
    quadrature_1.spawn(&spawner);

    let (quadrature_2, _snapshot_2) =
        Quadrature::new(&QUADRATURE2_STORAGE, leg_2_hall1_pin, leg_2_hall2_pin);
    let leg_watcher_2 = quadrature_2.watcher();
    let leg_status_source_2 = quadrature_2.watcher();
    quadrature_2.spawn(&spawner);

    motor_1.enable();
    motor_2.enable();

    let leg_1 = esp_pwm_motor::leg::Leg::new(
        &LEG1_STATUS_STORAGE,
        LegConfig {
            up_drive: DriveSide::Left,
            up_direction: QuadratureDirection::Positive,
        },
        motor_1,
        leg_watcher_1,
        leg_status_source_1,
        &spawner,
    );
    let desk_left_status_watcher = leg_1.status_watcher();

    let leg_2 = esp_pwm_motor::leg::Leg::new(
        &LEG2_STATUS_STORAGE,
        LegConfig {
            up_drive: DriveSide::Left,
            up_direction: QuadratureDirection::Positive,
        },
        motor_2,
        leg_watcher_2,
        leg_status_source_2,
        &spawner,
    );
    let desk_right_status_watcher = leg_2.status_watcher();

    let desk = Desk::new(
        &DESK_STATUS_STORAGE,
        leg_1,
        leg_2,
        desk_left_status_watcher,
        desk_right_status_watcher,
        &spawner,
    );
    let desk_status_watcher = desk.status_watcher();
    spawner.must_spawn(watch_desk_status(desk_status_watcher));

    let mut desk = match desk.home_all().await {
        Ok(desk) => desk,
        Err((_desk, error)) => {
            warn!("desk homing failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    if RUN_MOVE_TEST_SEQUENCE {
        Timer::after(Duration::from_secs(5)).await;
        desk = match desk.move_to(500).await {
            Ok(desk) => desk,
            Err((_desk, error)) => {
                warn!("move to 500 failed: {:?}", error);
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        };

        Timer::after(Duration::from_secs(1)).await;

        desk = match desk.move_to(0).await {
            Ok(desk) => desk,
            Err((_desk, error)) => {
                warn!("move to 0 failed: {:?}", error);
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        };

        for cycle in 1..=2 {
            Timer::after(Duration::from_secs(1)).await;

            desk = match desk.move_to(500).await {
                Ok(desk) => desk,
                Err((_desk, error)) => {
                    warn!("cycle {} move to 500 failed: {:?}", cycle, error);
                    loop {
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
            };

            Timer::after(Duration::from_secs(1)).await;

            desk = match desk.move_to(0).await {
                Ok(desk) => desk,
                Err((_desk, error)) => {
                    warn!("cycle {} move to 0 failed: {:?}", cycle, error);
                    loop {
                        Timer::after(Duration::from_secs(1)).await;
                    }
                }
            };
        }
    }

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
