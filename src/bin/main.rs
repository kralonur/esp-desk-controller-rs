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
    mcpwm::{McPwm, PeripheralClockConfig, PwmPeripheral},
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_pwm_motor::{
    config::RuntimeConfigStorage,
    controller::{DeskCommand, DeskControllerState},
    desk::{Desk, DeskMoveOutcome, DeskStatusStorage, ReadyDesk, UnhomedDesk},
    leg::{DriveSide, LegConfig, LegStatusStorage},
    motor::Motor,
    quadrature::{Quadrature, QuadratureDirection, QuadratureStorage},
    units::{PWM_TIMER_MAX_TICKS, PositionCounts, RelativeCounts},
};
use static_cell::StaticCell;
use {esp_backtrace as _, esp_println as _};

esp_bootloader_esp_idf::esp_app_desc!();

const PWM_PERIOD_TICKS: u16 = PWM_TIMER_MAX_TICKS;
const PWM_FREQUENCY_KHZ: u32 = 20;

fn clamp_relative_target(
    current_position: i32,
    min_position: i32,
    max_position: i32,
    delta: RelativeCounts,
) -> PositionCounts {
    PositionCounts::new(current_position)
        .saturating_add(delta)
        .clamp(
            PositionCounts::new(min_position),
            PositionCounts::new(max_position),
        )
}

enum DeskRuntime<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
> {
    Unhomed(Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>),
    Ready(Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>),
}

async fn run_desk_control<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
>(
    control_state: &'static DeskControllerState,
    mut desk: DeskRuntime<'a, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
) -> ! {
    loop {
        match control_state.next_command().await {
            DeskCommand::Home => {
                if control_state.stop_requested() {
                    control_state.clear_stop_request();
                    info!("home command cancelled before start");
                    continue;
                }

                info!("received /home command");
                control_state.begin_homing();

                desk = match desk {
                    DeskRuntime::Unhomed(desk) => {
                        match desk.home_all(|| control_state.stop_requested()).await {
                            Ok(desk) => {
                                info!("desk homed");
                                control_state.finish_ready();
                                DeskRuntime::Ready(desk)
                            }
                            Err((desk, esp_pwm_motor::desk::DeskError::Stopped)) => {
                                info!("desk homing stopped");
                                control_state.finish_unhomed();
                                DeskRuntime::Unhomed(desk)
                            }
                            Err((desk, error)) => {
                                warn!("desk homing failed: {:?}", error);
                                control_state.fault(error);
                                DeskRuntime::Unhomed(desk)
                            }
                        }
                    }
                    DeskRuntime::Ready(desk) => {
                        match desk.home_all(|| control_state.stop_requested()).await {
                            Ok(desk) => {
                                info!("desk re-homed");
                                control_state.finish_ready();
                                DeskRuntime::Ready(desk)
                            }
                            Err((desk, esp_pwm_motor::desk::DeskError::Stopped)) => {
                                info!("desk re-home stopped");
                                control_state.finish_unhomed();
                                DeskRuntime::Unhomed(desk)
                            }
                            Err((desk, error)) => {
                                warn!("desk re-home failed: {:?}", error);
                                control_state.fault(error);
                                DeskRuntime::Unhomed(desk)
                            }
                        }
                    }
                };
            }
            DeskCommand::MoveTo(target_position) => {
                if control_state.stop_requested() {
                    control_state.clear_stop_request();
                    info!("move_to command cancelled before start");
                    continue;
                }

                desk = match desk {
                    DeskRuntime::Unhomed(desk) => {
                        warn!("move_to ignored while desk is unhomed");
                        control_state.finish_unhomed();
                        DeskRuntime::Unhomed(desk)
                    }
                    DeskRuntime::Ready(desk) => {
                        control_state.begin_move();
                        match desk
                            .move_to(target_position, || control_state.stop_requested())
                            .await
                        {
                            Ok((desk, outcome)) => {
                                let position = desk.status().average_position;
                                match outcome {
                                    DeskMoveOutcome::Completed => {
                                        info!("move_to command finished at {}", position);
                                    }
                                    DeskMoveOutcome::StoppedByRequest => {
                                        info!("move_to command stopped by request at {}", position);
                                    }
                                    DeskMoveOutcome::StoppedByObstruction => {
                                        warn!(
                                            "move_to command stopped by obstruction at {}",
                                            position
                                        );
                                    }
                                }
                                control_state.finish_ready();
                                DeskRuntime::Ready(desk)
                            }
                            Err((desk, error)) => {
                                warn!("move to {} failed: {:?}", target_position.get(), error);
                                control_state.fault(error);
                                DeskRuntime::Unhomed(desk)
                            }
                        }
                    }
                };
            }
            DeskCommand::MoveBy(delta) => {
                if control_state.stop_requested() {
                    control_state.clear_stop_request();
                    info!("move_by command cancelled before start");
                    continue;
                }

                desk = match desk {
                    DeskRuntime::Unhomed(desk) => {
                        warn!("relative move ignored while desk is unhomed");
                        control_state.finish_unhomed();
                        DeskRuntime::Unhomed(desk)
                    }
                    DeskRuntime::Ready(desk) => {
                        control_state.begin_move();
                        let status = desk.status();
                        let target_position = clamp_relative_target(
                            status.average_position,
                            status.min_position,
                            status.max_position,
                            delta,
                        );

                        if target_position.get() == status.average_position {
                            info!("relative move {} ignored at desk limit", delta.get());
                            control_state.finish_ready();
                            DeskRuntime::Ready(desk)
                        } else {
                            match desk
                                .move_to(target_position, || control_state.stop_requested())
                                .await
                            {
                                Ok((desk, outcome)) => {
                                    let position = desk.status().average_position;
                                    match outcome {
                                        DeskMoveOutcome::Completed => {
                                            info!("move_by command finished at {}", position);
                                        }
                                        DeskMoveOutcome::StoppedByRequest => {
                                            info!(
                                                "move_by command stopped by request at {}",
                                                position
                                            );
                                        }
                                        DeskMoveOutcome::StoppedByObstruction => {
                                            warn!(
                                                "move_by command stopped by obstruction at {}",
                                                position
                                            );
                                        }
                                    }
                                    control_state.finish_ready();
                                    DeskRuntime::Ready(desk)
                                }
                                Err((desk, error)) => {
                                    warn!("move by {} failed: {:?}", delta.get(), error);
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                    }
                };
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
    static QUADRATURE1_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static QUADRATURE2_STORAGE: QuadratureStorage = QuadratureStorage::new();
    static LEG1_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static LEG2_STATUS_STORAGE: LegStatusStorage = LegStatusStorage::new();
    static DESK_STATUS_STORAGE: DeskStatusStorage = DeskStatusStorage::new();
    static RUNTIME_CONFIG_STORAGE: RuntimeConfigStorage = RuntimeConfigStorage::new();
    static DESK_CONTROL_STATE_STORAGE: StaticCell<DeskControllerState> = StaticCell::new();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 65536);

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

    static RADIO_INIT: StaticCell<esp_radio::Controller<'static>> = StaticCell::new();
    let radio_init = RADIO_INIT
        .uninit()
        .write(esp_radio::init().expect("Failed to initialize Wi-Fi controller"));
    let rng = Rng::new();
    let stack =
        match esp_pwm_motor::wifi::start_wifi(radio_init, peripherals.WIFI, rng, &spawner).await {
            Ok(stack) => stack,
            Err(error) => {
                warn!("wifi setup failed: {:?}", error);
                loop {
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        };

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

    let runtime_config_reader = RUNTIME_CONFIG_STORAGE.init(Default::default()).reader();

    let leg_1 = esp_pwm_motor::leg::Leg::new(
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

    let leg_2 = esp_pwm_motor::leg::Leg::new(
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
    let mqtt_status_watcher = desk.status_watcher();
    esp_pwm_motor::mqtt::spawn_mqtt(&spawner, stack, control_state, mqtt_status_watcher);

    run_desk_control(control_state, DeskRuntime::Unhomed(desk)).await
}
