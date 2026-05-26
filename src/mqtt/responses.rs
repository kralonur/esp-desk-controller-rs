use alloc::{format, string::String};

use crate::{
    config::{ConfigError, ObstructionSensitivity, RuntimeConfigState},
    controller::{
        CommandSubmission, DeskControllerMode, DeskControllerSnapshot, DeskControllerState,
        DeskFault,
    },
    desk::{DeskLegSide, DeskMotionState, DeskMoveInvariant, DeskStopReason, OverrideLegDirection},
};

pub(super) fn status_response(state: &DeskControllerState) -> String {
    let snapshot = state.snapshot();
    let status = state.status();
    format!(
        "mode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\nhomed={}\nneeds_rehome={}\nmotion={}\nlast_stop_reason={}\nobstruction_sensitivity={}\ntarget_active={}\ntarget_position={}\naverage_position={}\nleft_position={}\nright_position={}\nleft_duty={}\nright_duty={}\nmin_position={}\nmax_position={}\nskew_counts={}\n",
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
        bool_name(status.homed),
        bool_name(status.needs_rehome),
        motion_name(status.motion),
        stop_reason_name(status.last_stop_reason),
        obstruction_sensitivity_name(state.obstruction_sensitivity()),
        bool_name(status.target_active),
        status.target_position,
        status.average_position,
        status.left_position,
        status.right_position,
        status.left_duty,
        status.right_duty,
        status.min_position,
        status.max_position,
        status.skew_counts,
    )
}

pub(super) fn submission_response(
    command: &'static str,
    submission: CommandSubmission,
    snapshot: DeskControllerSnapshot,
) -> String {
    let result = match submission {
        CommandSubmission::Accepted => "accepted",
        CommandSubmission::RejectedBusy => "rejected_busy",
        CommandSubmission::RejectedUnhomed => "rejected_unhomed",
        CommandSubmission::RejectedFaulted => "rejected_faulted",
        CommandSubmission::RejectedLocked => "rejected_locked",
        CommandSubmission::RejectedOverrideUnlocked => "rejected_override_unlocked",
    };

    response_body(command, result, snapshot)
}

pub(super) fn override_submission_response(
    command: &'static str,
    submission: CommandSubmission,
    snapshot: DeskControllerSnapshot,
    override_unlocked: bool,
    side: Option<DeskLegSide>,
    direction: Option<OverrideLegDirection>,
) -> String {
    let result = match submission {
        CommandSubmission::Accepted => "accepted",
        CommandSubmission::RejectedBusy => "rejected_busy",
        CommandSubmission::RejectedUnhomed => "rejected_unhomed",
        CommandSubmission::RejectedFaulted => "rejected_faulted",
        CommandSubmission::RejectedLocked => "rejected_locked",
        CommandSubmission::RejectedOverrideUnlocked => "rejected_override_unlocked",
    };

    let mut body = override_response_body(command, result, snapshot, override_unlocked);
    if let Some(side) = side {
        body.push_str("leg=");
        body.push_str(leg_side_name(side));
        body.push('\n');
    }
    if let Some(direction) = direction {
        body.push_str("direction=");
        body.push_str(override_direction_name(direction));
        body.push('\n');
    }
    body
}

pub(super) fn response_body(
    command: &'static str,
    result: &'static str,
    snapshot: DeskControllerSnapshot,
) -> String {
    format!(
        "command={}\nresult={}\nmode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\n",
        command,
        result,
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
    )
}

pub(super) fn override_response_body(
    command: &'static str,
    result: &'static str,
    snapshot: DeskControllerSnapshot,
    override_unlocked: bool,
) -> String {
    format!(
        "command={}\nresult={}\nmode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\noverride_unlocked={}\n",
        command,
        result,
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
        bool_name(override_unlocked),
    )
}

pub(super) fn config_response(runtime_config_state: &RuntimeConfigState) -> String {
    let config = runtime_config_state.current();
    let desk = config.desk();
    let leg = config.leg();
    let low_profile = desk
        .obstruction_profile(ObstructionSensitivity::Low)
        .expect("low obstruction profile must exist");
    let medium_profile = desk
        .obstruction_profile(ObstructionSensitivity::Medium)
        .expect("medium obstruction profile must exist");
    let high_profile = desk
        .obstruction_profile(ObstructionSensitivity::High)
        .expect("high obstruction profile must exist");

    format!(
        "desk.target_tolerance={}\n\
desk.target_slow_zone={}\n\
desk.move_timeout_ms={}\n\
desk.obstruction_sample_window_ms={}\n\
desk.obstruction_warmup_duration_ms={}\n\
desk.obstruction_warmup_counts={}\n\
desk.min_move_duty={}\n\
desk.move_run_duty={}\n\
desk.move_slow_duty={}\n\
desk.move_sync_duty_step={}\n\
desk.homing_poll_interval_ms={}\n\
desk.homing_start_timeout_ms={}\n\
desk.homing_stall_timeout_ms={}\n\
desk.homing_backoff_steps={}\n\
desk.homing_run_duty={}\n\
desk.homing_sync_duty_step={}\n\
desk.sync_speedup_enter_counts={}\n\
desk.sync_speedup_exit_counts={}\n\
desk.catch_up_enter_counts={}\n\
desk.catch_up_exit_counts={}\n\
desk.fault_skew_counts={}\n\
desk.homing_fault_skew_counts={}\n\
desk.obstruction_sensitivity={}\n\
desk.low_obstruction_min_percent={}\n\
desk.low_obstruction_windows={}\n\
desk.medium_obstruction_min_percent={}\n\
desk.medium_obstruction_windows={}\n\
desk.high_obstruction_min_percent={}\n\
desk.high_obstruction_windows={}\n\
desk.override_unlock_timeout_ms={}\n\
leg.startup_duty={}\n\
leg.max_duty={}\n\
leg.run_duty={}\n\
leg.slow_duty={}\n\
leg.homing_duty={}\n\
leg.startup_events={}\n\
leg.homing_start_timeout_ms={}\n\
leg.homing_stall_timeout_ms={}\n\
leg.homing_poll_interval_ms={}\n\
leg.homing_backoff_steps={}\n\
leg.default_max_position={}\n\
leg.move_stall_timeout_ms={}\n\
leg.target_slow_zone={}\n\
leg.target_tolerance={}\n",
        desk.target_tolerance().get(),
        desk.target_slow_zone().get(),
        desk.move_timeout().as_millis(),
        desk.obstruction_sample_window().as_millis(),
        desk.obstruction_warmup_duration().as_millis(),
        desk.obstruction_warmup_counts().get(),
        desk.min_move_duty().get(),
        desk.move_run_duty().get(),
        desk.move_slow_duty().get(),
        desk.move_sync_duty_step().get(),
        desk.homing_poll_interval().as_millis(),
        desk.homing_start_timeout().as_millis(),
        desk.homing_stall_timeout().as_millis(),
        desk.homing_backoff_steps().get(),
        desk.homing_run_duty().get(),
        desk.homing_sync_duty_step().get(),
        desk.sync_speedup_enter_counts().get(),
        desk.sync_speedup_exit_counts().get(),
        desk.catch_up_enter_counts().get(),
        desk.catch_up_exit_counts().get(),
        desk.fault_skew_counts().get(),
        desk.homing_fault_skew_counts().get(),
        obstruction_sensitivity_name(desk.obstruction_sensitivity()),
        low_profile.minimum_baseline_percent().get(),
        low_profile.consecutive_windows(),
        medium_profile.minimum_baseline_percent().get(),
        medium_profile.consecutive_windows(),
        high_profile.minimum_baseline_percent().get(),
        high_profile.consecutive_windows(),
        desk.override_unlock_timeout().as_millis(),
        leg.startup_duty().get(),
        leg.max_duty().get(),
        leg.run_duty().get(),
        leg.slow_duty().get(),
        leg.homing_duty().get(),
        leg.startup_events().get(),
        leg.homing_start_timeout().as_millis(),
        leg.homing_stall_timeout().as_millis(),
        leg.homing_poll_interval().as_millis(),
        leg.homing_backoff_steps().get(),
        leg.default_max_position().get(),
        leg.move_stall_timeout().as_millis(),
        leg.target_slow_zone().get(),
        leg.target_tolerance().get(),
    )
}

pub(super) fn config_error_name(error: ConfigError) -> &'static str {
    match error {
        ConfigError::InvalidDeskConfig | ConfigError::InvalidLegConfig => "invalid_config",
    }
}

pub(super) fn controller_mode_name(mode: DeskControllerMode) -> &'static str {
    match mode {
        DeskControllerMode::Unhomed => "unhomed",
        DeskControllerMode::Ready => "ready",
        DeskControllerMode::Homing => "homing",
        DeskControllerMode::Moving => "moving",
        DeskControllerMode::Override => "override",
        DeskControllerMode::Faulted => "faulted",
    }
}

pub(super) fn fault_name(fault: Option<DeskFault>) -> &'static str {
    match fault {
        None => "none",
        Some(DeskFault::LeftLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "left_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "left_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "left_leg_move_timeout",
        },
        Some(DeskFault::RightLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "right_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "right_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "right_leg_move_timeout",
        },
        Some(DeskFault::MoveTimeout) => "move_timeout",
        Some(DeskFault::SkewFault) => "skew_fault",
        Some(DeskFault::InvariantViolation(DeskMoveInvariant::DirectionMismatch)) => {
            "invariant_direction_mismatch"
        }
        Some(DeskFault::InvariantViolation(DeskMoveInvariant::WrongWayProgress)) => {
            "invariant_wrong_way_progress"
        }
        Some(DeskFault::RehomeRequired) => "rehome_required",
    }
}

pub(super) fn motion_name(motion: DeskMotionState) -> &'static str {
    match motion {
        DeskMotionState::Idle => "idle",
        DeskMotionState::Homing => "homing",
        DeskMotionState::MovingUp => "moving_up",
        DeskMotionState::MovingDown => "moving_down",
    }
}

pub(super) fn stop_reason_name(reason: DeskStopReason) -> &'static str {
    match reason {
        DeskStopReason::None => "none",
        DeskStopReason::TargetReached => "target_reached",
        DeskStopReason::UserStop => "user_stop",
        DeskStopReason::Obstruction => "obstruction",
    }
}

pub(super) fn obstruction_sensitivity_name(sensitivity: ObstructionSensitivity) -> &'static str {
    match sensitivity {
        ObstructionSensitivity::None => "none",
        ObstructionSensitivity::Low => "low",
        ObstructionSensitivity::Medium => "medium",
        ObstructionSensitivity::High => "high",
    }
}

pub(super) fn leg_side_name(side: DeskLegSide) -> &'static str {
    match side {
        DeskLegSide::Left => "left",
        DeskLegSide::Right => "right",
    }
}

pub(super) fn override_direction_name(direction: OverrideLegDirection) -> &'static str {
    match direction {
        OverrideLegDirection::Up => "up",
        OverrideLegDirection::Down => "down",
    }
}

pub(super) fn bool_name(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}
