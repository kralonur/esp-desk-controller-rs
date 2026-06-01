use defmt::Format;
use embassy_time::{Instant, Timer};
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::LegRuntimeConfig,
    leg::{DriveMode, Leg, LegError, Unhomed},
    quadrature::QuadratureDirection,
    units::CountDelta,
};

use super::{
    position::{progressed_in_direction, travel_in_direction},
    state::{Desk, DeskError, UnhomedDesk},
    status::{DeskMotionState, DeskStopReason},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Selects which physical leg a manual override command should operate on.
pub enum DeskLegSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Manual override travel direction for one selected leg.
pub enum OverrideLegDirection {
    Up,
    Down,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Non-fault outcome from a manual override operation.
pub enum DeskOverrideOutcome {
    Completed,
    StoppedByRequest,
}

impl<
    'a,
    State,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
> Desk<'a, State, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    /// Home one selected leg as a manual service/recovery operation.
    ///
    /// The non-selected leg is stopped and the desk is always returned unhomed,
    /// so coordinated movement requires a later full `home_all`.
    pub async fn override_home_leg<StopRequested>(
        self,
        side: DeskLegSide,
        stop_requested: StopRequested,
    ) -> Result<
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskOverrideOutcome,
        ),
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    >
    where
        StopRequested: Fn() -> bool,
    {
        let mut desk = self.force_unhomed();
        let runtime_config = desk.runtime_config_reader.current();
        let leg_config = runtime_config.leg();
        desk.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = DeskMotionState::Homing;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
        });

        let (mut left_leg, mut right_leg) = desk.take_unhomed_legs();
        stop_non_selected_leg(side, &mut left_leg, &mut right_leg, leg_config);

        let result = match side {
            DeskLegSide::Left => {
                override_home_one_leg(&mut left_leg, leg_config, &stop_requested).await
            }
            DeskLegSide::Right => {
                override_home_one_leg(&mut right_leg, leg_config, &stop_requested).await
            }
        };

        match result {
            Ok(outcome) => {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                let reason = match outcome {
                    DeskOverrideOutcome::Completed => DeskStopReason::None,
                    DeskOverrideOutcome::StoppedByRequest => DeskStopReason::UserStop,
                };
                Ok((
                    desk.finish_override_unhomed(left_leg, right_leg, reason),
                    outcome,
                ))
            }
            Err(error) => {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                Err(desk.restore_unhomed(left_leg, right_leg, side_error(side, error)))
            }
        }
    }

    /// Move one selected leg by a fixed number of encoder steps.
    ///
    /// This is an override/recovery path, not coordinated desk movement. The
    /// returned desk is unhomed even when the override completes successfully.
    pub async fn override_move_leg<StopRequested>(
        self,
        side: DeskLegSide,
        direction: OverrideLegDirection,
        steps: CountDelta,
        stop_requested: StopRequested,
    ) -> Result<
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskOverrideOutcome,
        ),
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    >
    where
        StopRequested: Fn() -> bool,
    {
        let mut desk = self.force_unhomed();
        let runtime_config = desk.runtime_config_reader.current();
        let leg_config = runtime_config.leg();
        desk.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = match direction {
                OverrideLegDirection::Up => DeskMotionState::MovingUp,
                OverrideLegDirection::Down => DeskMotionState::MovingDown,
            };
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
        });

        let (mut left_leg, mut right_leg) = desk.take_unhomed_legs();
        stop_non_selected_leg(side, &mut left_leg, &mut right_leg, leg_config);

        let result = match side {
            DeskLegSide::Left => {
                override_move_one_leg(&mut left_leg, direction, steps, leg_config, &stop_requested)
                    .await
            }
            DeskLegSide::Right => {
                override_move_one_leg(
                    &mut right_leg,
                    direction,
                    steps,
                    leg_config,
                    &stop_requested,
                )
                .await
            }
        };

        match result {
            Ok(outcome) => {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                let reason = match outcome {
                    DeskOverrideOutcome::Completed => DeskStopReason::None,
                    DeskOverrideOutcome::StoppedByRequest => DeskStopReason::UserStop,
                };
                Ok((
                    desk.finish_override_unhomed(left_leg, right_leg, reason),
                    outcome,
                ))
            }
            Err(error) => {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                Err(desk.restore_unhomed(left_leg, right_leg, side_error(side, error)))
            }
        }
    }
}

async fn override_home_one_leg<'a, StopRequested, const OP: u8, PWM: PwmPeripheral>(
    leg: &mut Leg<'a, Unhomed, OP, PWM>,
    leg_config: LegRuntimeConfig,
    stop_requested: &StopRequested,
) -> Result<DeskOverrideOutcome, LegError>
where
    StopRequested: Fn() -> bool,
{
    if stop_requested() {
        leg.apply_drive_mode(DriveMode::Stop, leg_config);
        return Ok(DeskOverrideOutcome::StoppedByRequest);
    }

    let start_position = leg.encoder_position();
    let start_deadline = Instant::now() + leg_config.homing_start_timeout();
    let down_direction = leg.down_direction();
    let up_direction = leg.up_direction();

    leg.apply_drive_mode(DriveMode::DownBoost, leg_config);

    loop {
        if stop_requested() {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Ok(DeskOverrideOutcome::StoppedByRequest);
        }

        let current_position = leg.encoder_position();
        if progressed_in_direction(start_position, current_position, down_direction) {
            break;
        }
        if progressed_in_direction(start_position, current_position, up_direction) {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Err(LegError::PolarityMismatch);
        }
        if Instant::now() >= start_deadline {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Err(LegError::HomingStartTimeout);
        }

        Timer::after(leg_config.homing_poll_interval()).await;
    }

    let mut last_position = leg.encoder_position();
    let mut last_progress_at = Instant::now();
    leg.apply_drive_mode(DriveMode::HomeDown, leg_config);

    loop {
        if stop_requested() {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Ok(DeskOverrideOutcome::StoppedByRequest);
        }

        Timer::after(leg_config.homing_poll_interval()).await;
        let current_position = leg.encoder_position();
        if progressed_in_direction(last_position, current_position, down_direction) {
            last_position = current_position;
            last_progress_at = Instant::now();
            continue;
        }

        if Instant::now().saturating_duration_since(last_progress_at)
            >= leg_config.homing_stall_timeout()
        {
            break;
        }
    }

    leg.apply_drive_mode(DriveMode::Stop, leg_config);
    match override_move_one_leg(
        leg,
        OverrideLegDirection::Up,
        leg_config.homing_backoff_steps(),
        leg_config,
        stop_requested,
    )
    .await?
    {
        DeskOverrideOutcome::Completed => {
            leg.reset_position();
            leg.publish_status();
            Ok(DeskOverrideOutcome::Completed)
        }
        DeskOverrideOutcome::StoppedByRequest => Ok(DeskOverrideOutcome::StoppedByRequest),
    }
}

async fn override_move_one_leg<'a, StopRequested, const OP: u8, PWM: PwmPeripheral>(
    leg: &mut Leg<'a, Unhomed, OP, PWM>,
    direction: OverrideLegDirection,
    steps: CountDelta,
    leg_config: LegRuntimeConfig,
    stop_requested: &StopRequested,
) -> Result<DeskOverrideOutcome, LegError>
where
    StopRequested: Fn() -> bool,
{
    if steps.get() == 0 {
        leg.apply_drive_mode(DriveMode::Stop, leg_config);
        return Ok(DeskOverrideOutcome::Completed);
    }

    if stop_requested() {
        leg.apply_drive_mode(DriveMode::Stop, leg_config);
        return Ok(DeskOverrideOutcome::StoppedByRequest);
    }

    let expected_direction = match direction {
        OverrideLegDirection::Up => leg.up_direction(),
        OverrideLegDirection::Down => leg.down_direction(),
    };
    let wrong_direction = match direction {
        OverrideLegDirection::Up => leg.down_direction(),
        OverrideLegDirection::Down => leg.up_direction(),
    };
    if expected_direction == QuadratureDirection::Invalid {
        leg.apply_drive_mode(DriveMode::Stop, leg_config);
        return Ok(DeskOverrideOutcome::Completed);
    }

    let target_steps = steps.get_i32();
    let startup_steps = target_steps.min(leg_config.startup_events().get_i32());
    let start_position = leg.encoder_position();
    let mut last_position = start_position;
    let mut last_progress_at = Instant::now();
    let mut running = startup_steps == 0;

    leg.apply_drive_mode(override_boost_mode(direction), leg_config);
    if running {
        leg.apply_drive_mode(override_run_mode(direction), leg_config);
    }

    loop {
        if stop_requested() {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Ok(DeskOverrideOutcome::StoppedByRequest);
        }

        Timer::after(leg_config.homing_poll_interval()).await;
        let current_position = leg.encoder_position();
        let total_progress =
            travel_in_direction(start_position, current_position, expected_direction);
        if total_progress >= target_steps {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Ok(DeskOverrideOutcome::Completed);
        }

        if progressed_in_direction(last_position, current_position, expected_direction) {
            last_position = current_position;
            last_progress_at = Instant::now();
            if !running && total_progress >= startup_steps {
                leg.apply_drive_mode(override_run_mode(direction), leg_config);
                running = true;
            }
            continue;
        }

        if progressed_in_direction(last_position, current_position, wrong_direction) {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Err(LegError::PolarityMismatch);
        }

        if Instant::now().saturating_duration_since(last_progress_at)
            >= leg_config.move_stall_timeout()
        {
            leg.apply_drive_mode(DriveMode::Stop, leg_config);
            return Err(LegError::MoveTimeout);
        }
    }
}

fn override_boost_mode(direction: OverrideLegDirection) -> DriveMode {
    match direction {
        OverrideLegDirection::Up => DriveMode::UpBoost,
        OverrideLegDirection::Down => DriveMode::DownBoost,
    }
}

fn override_run_mode(direction: OverrideLegDirection) -> DriveMode {
    match direction {
        OverrideLegDirection::Up => DriveMode::UpRun,
        OverrideLegDirection::Down => DriveMode::DownRun,
    }
}

fn side_error(side: DeskLegSide, error: LegError) -> DeskError {
    match side {
        DeskLegSide::Left => DeskError::LeftLeg(error),
        DeskLegSide::Right => DeskError::RightLeg(error),
    }
}

fn stop_non_selected_leg<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
>(
    side: DeskLegSide,
    left_leg: &mut Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
    right_leg: &mut Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
    leg_config: LegRuntimeConfig,
) {
    match side {
        DeskLegSide::Left => right_leg.apply_drive_mode(DriveMode::Stop, leg_config),
        DeskLegSide::Right => left_leg.apply_drive_mode(DriveMode::Stop, leg_config),
    }
}
