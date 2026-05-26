//! Firmware startup orchestration.
//!
//! This module owns the embedded application wiring that used to live directly
//! in `main`: persistent config loading, WiFi, motor/PWM/quadrature setup, desk
//! construction, controller state, MQTT, and finally the desk control loop.
//!
//! File layout:
//!
//! - `mod`: startup wiring, storage allocation, peripheral assignment, task
//!   spawning, and handoff into the desk control loop.
//! - `control`: desk command execution. It consumes controller commands, drives
//!   desk typestate operations, records lifecycle transitions, and logs
//!   completion/stop/fault outcomes.

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    interrupt::software::SoftwareInterruptControl,
    mcpwm::{McPwm, PeripheralClockConfig},
    pcnt::Pcnt,
    peripherals::Peripherals,
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use static_cell::StaticCell;

use crate::{
    config::{RuntimeConfig, RuntimeConfigStorage},
    controller::DeskControllerState,
    desk::{Desk, DeskStatusStorage},
    leg::{DriveSide, LegConfig, LegStatusStorage},
    motor::Motor,
    persistent_config::{PersistError, RuntimeConfigPersistence},
    quadrature::{Quadrature, QuadratureDirection, QuadratureStorage},
    units::PWM_TIMER_MAX_TICKS,
};

mod control;

const PWM_PERIOD_TICKS: u16 = PWM_TIMER_MAX_TICKS;
const PWM_FREQUENCY_KHZ: u32 = 20;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. during firmware startup"
)]
pub async fn run(spawner: Spawner, peripherals: Peripherals) -> ! {
    static QUADRATURE1_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static QUADRATURE2_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static LEG1_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static LEG2_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static DESK_STATUS_STORAGE: DeskStatusStorage = DeskStatusStorage::new();
    static RUNTIME_CONFIG_STORAGE: RuntimeConfigStorage = RuntimeConfigStorage::new();
    static RUNTIME_CONFIG_PERSISTENCE_STORAGE: StaticCell<RuntimeConfigPersistence> =
        StaticCell::new();
    static DESK_CONTROL_STATE_STORAGE: StaticCell<DeskControllerState> = StaticCell::new();

    let mut runtime_config_persistence: Option<&'static mut RuntimeConfigPersistence> =
        match RuntimeConfigPersistence::new(peripherals.FLASH) {
            Ok(persistence) => Some(RUNTIME_CONFIG_PERSISTENCE_STORAGE.init(persistence)),
            Err(error) => {
                warn!("persistent config storage unavailable: {:?}", error);
                None
            }
        };
    let initial_runtime_config = match runtime_config_persistence.as_deref_mut() {
        Some(persistence) => match persistence.load() {
            Ok(config) => {
                info!("loaded runtime config from flash");
                config
            }
            Err(PersistError::Missing) => {
                info!("no saved runtime config found");
                RuntimeConfig::default()
            }
            Err(error) => {
                warn!("saved runtime config ignored: {:?}", error);
                RuntimeConfig::default()
            }
        },
        None => RuntimeConfig::default(),
    };

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
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    let rng = Rng::new();
    let stack = match crate::wifi::start_wifi(peripherals.WIFI, rng, &spawner).await {
        Ok(stack) => stack,
        Err(error) => {
            warn!("wifi setup failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };
    info!("main continuing after wifi setup");

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

    let mut pcnt = Pcnt::new(peripherals.PCNT);
    pcnt.set_interrupt_handler(crate::quadrature::pcnt_interrupt_handler);

    let (quadrature_1, _snapshot_1) = Quadrature::<0>::new(
        &QUADRATURE1_STORAGE,
        pcnt.unit0,
        leg_1_hall1_pin,
        leg_1_hall2_pin,
    );
    let leg_watcher_1 = quadrature_1.watcher();
    let leg_status_source_1 = quadrature_1.watcher();
    quadrature_1.spawn(&spawner);

    let (quadrature_2, _snapshot_2) = Quadrature::<1>::new(
        &QUADRATURE2_STORAGE,
        pcnt.unit1,
        leg_2_hall1_pin,
        leg_2_hall2_pin,
    );
    let leg_watcher_2 = quadrature_2.watcher();
    let leg_status_source_2 = quadrature_2.watcher();
    quadrature_2.spawn(&spawner);

    motor_1.enable();
    motor_2.enable();
    info!("motors and quadrature initialized");

    let runtime_config_state = RUNTIME_CONFIG_STORAGE.init(initial_runtime_config);
    let runtime_config_reader = runtime_config_state.reader();

    let leg_1 = crate::leg::Leg::new(
        &LEG1_STATUS_STORAGE,
        LegConfig {
            up_drive: DriveSide::Left,
            up_direction: QuadratureDirection::Positive,
        },
        runtime_config_reader,
        motor_1,
        leg_watcher_1,
        leg_status_source_1,
        &spawner,
    );
    let desk_left_status_watcher = leg_1.status_watcher();

    let leg_2 = crate::leg::Leg::new(
        &LEG2_STATUS_STORAGE,
        LegConfig {
            up_drive: DriveSide::Left,
            up_direction: QuadratureDirection::Positive,
        },
        runtime_config_reader,
        motor_2,
        leg_watcher_2,
        leg_status_source_2,
        &spawner,
    );
    let desk_right_status_watcher = leg_2.status_watcher();

    let desk = Desk::new(
        &DESK_STATUS_STORAGE,
        runtime_config_reader,
        leg_1,
        leg_2,
        desk_left_status_watcher,
        desk_right_status_watcher,
        &spawner,
    );
    let control_state = DESK_CONTROL_STATE_STORAGE
        .uninit()
        .write(DeskControllerState::new(
            desk.status_reader(),
            runtime_config_reader,
        ));
    info!("desk controller initialized");
    let mqtt_status_watcher = desk.status_watcher();
    info!("spawning mqtt task");
    crate::mqtt::spawn_mqtt(
        &spawner,
        stack,
        control_state,
        runtime_config_state,
        runtime_config_persistence,
        mqtt_status_watcher,
    );
    info!("entering desk control loop");

    control::run_desk_control(control_state, desk).await
}
