use defmt::{info, warn};
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    controller::{DeskCommand, DeskControllerState, OverrideCommand},
    desk::{
        Desk, DeskError, DeskMoveError, DeskMoveOutcome, DeskOverrideOutcome, ReadyDesk,
        UnhomedDesk,
    },
    units::{PositionCounts, RelativeCounts},
};

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

pub(super) async fn run_desk_control<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
>(
    control_state: &'static DeskControllerState,
    desk: Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
) -> ! {
    let mut desk = DeskRuntime::Unhomed(desk);

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
                            Err((desk, DeskError::Stopped)) => {
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
                            Err((desk, DeskError::Stopped)) => {
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
            DeskCommand::ForceHome => {
                if control_state.stop_requested() {
                    control_state.clear_stop_request();
                    info!("force_home command cancelled before start");
                    continue;
                }

                info!("received /force_home command");
                desk = match desk {
                    DeskRuntime::Unhomed(desk) => {
                        let desk = desk.force_home();
                        info!("desk force-homed");
                        control_state.finish_ready();
                        DeskRuntime::Ready(desk)
                    }
                    DeskRuntime::Ready(desk) => {
                        let desk = desk.force_home();
                        info!("desk force-rehomed");
                        control_state.finish_ready();
                        DeskRuntime::Ready(desk)
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
                            Err(DeskMoveError::FaultedReady(desk, error)) => {
                                warn!("move to {} failed: {:?}", target_position.get(), error);
                                control_state.fault(error);
                                DeskRuntime::Ready(desk)
                            }
                            Err(DeskMoveError::FaultedUnhomed(desk, error)) => {
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
                                Err(DeskMoveError::FaultedReady(desk, error)) => {
                                    warn!("move by {} failed: {:?}", delta.get(), error);
                                    control_state.fault(error);
                                    DeskRuntime::Ready(desk)
                                }
                                Err(DeskMoveError::FaultedUnhomed(desk, error)) => {
                                    warn!("move by {} failed: {:?}", delta.get(), error);
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                    }
                };
            }
            DeskCommand::Override(command) => {
                if control_state.stop_requested() {
                    control_state.clear_stop_request();
                    info!("override command cancelled before start");
                    continue;
                }

                control_state.begin_override();
                desk = match command {
                    OverrideCommand::Home { side } => match desk {
                        DeskRuntime::Unhomed(desk) => {
                            match desk
                                .override_home_leg(side, || control_state.stop_requested())
                                .await
                            {
                                Ok((desk, DeskOverrideOutcome::Completed)) => {
                                    info!("override home completed for {:?}", side);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Ok((desk, DeskOverrideOutcome::StoppedByRequest)) => {
                                    info!("override home stopped for {:?}", side);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Err((desk, error)) => {
                                    warn!("override home failed for {:?}: {:?}", side, error);
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                        DeskRuntime::Ready(desk) => {
                            match desk
                                .override_home_leg(side, || control_state.stop_requested())
                                .await
                            {
                                Ok((desk, DeskOverrideOutcome::Completed)) => {
                                    info!("override home completed for {:?}", side);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Ok((desk, DeskOverrideOutcome::StoppedByRequest)) => {
                                    info!("override home stopped for {:?}", side);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Err((desk, error)) => {
                                    warn!("override home failed for {:?}: {:?}", side, error);
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                    },
                    OverrideCommand::Move {
                        side,
                        direction,
                        steps,
                    } => match desk {
                        DeskRuntime::Unhomed(desk) => {
                            match desk
                                .override_move_leg(side, direction, steps, || {
                                    control_state.stop_requested()
                                })
                                .await
                            {
                                Ok((desk, DeskOverrideOutcome::Completed)) => {
                                    info!(
                                        "override move completed for {:?} {:?} {}",
                                        side,
                                        direction,
                                        steps.get()
                                    );
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Ok((desk, DeskOverrideOutcome::StoppedByRequest)) => {
                                    info!("override move stopped for {:?} {:?}", side, direction);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Err((desk, error)) => {
                                    warn!(
                                        "override move failed for {:?} {:?}: {:?}",
                                        side, direction, error
                                    );
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                        DeskRuntime::Ready(desk) => {
                            match desk
                                .override_move_leg(side, direction, steps, || {
                                    control_state.stop_requested()
                                })
                                .await
                            {
                                Ok((desk, DeskOverrideOutcome::Completed)) => {
                                    info!(
                                        "override move completed for {:?} {:?} {}",
                                        side,
                                        direction,
                                        steps.get()
                                    );
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Ok((desk, DeskOverrideOutcome::StoppedByRequest)) => {
                                    info!("override move stopped for {:?} {:?}", side, direction);
                                    control_state.finish_unhomed();
                                    DeskRuntime::Unhomed(desk)
                                }
                                Err((desk, error)) => {
                                    warn!(
                                        "override move failed for {:?} {:?}: {:?}",
                                        side, direction, error
                                    );
                                    control_state.fault(error);
                                    DeskRuntime::Unhomed(desk)
                                }
                            }
                        }
                    },
                };
            }
        }
    }
}
