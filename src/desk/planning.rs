use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::{DeskConfig, LegRuntimeConfig},
    leg::{DriveMode, Leg},
    units::{DutyPercent, DutyPercentTrim, positive_position_delta},
};

use super::{
    monitor::{AxisTargetState, MoveSnapshot},
    status::DeskMotionState,
};

#[derive(Clone, Copy)]
pub(super) enum SyncPhase {
    Balanced,
    SpeedMatch,
    PauseLead,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum TravelDirection {
    Up,
    Down,
}

#[derive(Clone, Copy)]
pub(super) enum LegPlan {
    Stop,
    Up(DutyPercent),
    Down(DutyPercent),
    HomeDown(DutyPercent),
}

#[derive(Clone, Copy)]
pub(super) struct DualLegPlan {
    pub(super) left: LegPlan,
    pub(super) right: LegPlan,
}

impl TravelDirection {
    pub(super) fn from_target(target: i32, current: i32) -> Option<Self> {
        if target > current {
            Some(Self::Up)
        } else if target < current {
            Some(Self::Down)
        } else {
            None
        }
    }

    pub(super) fn motion_state(self) -> DeskMotionState {
        match self {
            Self::Up => DeskMotionState::MovingUp,
            Self::Down => DeskMotionState::MovingDown,
        }
    }

    pub(super) fn lead_left(self, observed_skew: i32) -> bool {
        match self {
            Self::Up => observed_skew > 0,
            Self::Down => observed_skew < 0,
        }
    }

    pub(super) fn travel(self, start_position: i32, current_position: i32) -> i32 {
        match self {
            Self::Up => positive_position_delta(current_position, start_position),
            Self::Down => positive_position_delta(start_position, current_position),
        }
    }
}

pub(super) fn next_sync_phase(
    config: DeskConfig,
    current: SyncPhase,
    effective_skew_abs: i32,
) -> SyncPhase {
    // Sync uses hysteresis: enter thresholds are higher than exit thresholds so
    // encoder noise near a boundary does not make the two legs chatter.
    match current {
        SyncPhase::Balanced => {
            if effective_skew_abs >= config.catch_up_enter_counts().get_i32() {
                SyncPhase::PauseLead
            } else if effective_skew_abs >= config.sync_speedup_enter_counts().get_i32() {
                SyncPhase::SpeedMatch
            } else {
                SyncPhase::Balanced
            }
        }
        SyncPhase::SpeedMatch => {
            if effective_skew_abs >= config.catch_up_enter_counts().get_i32() {
                SyncPhase::PauseLead
            } else if effective_skew_abs <= config.sync_speedup_exit_counts().get_i32() {
                SyncPhase::Balanced
            } else {
                SyncPhase::SpeedMatch
            }
        }
        SyncPhase::PauseLead => {
            if effective_skew_abs <= config.catch_up_exit_counts().get_i32() {
                // Leaving catch-up can still require speed matching if skew is
                // below the pause threshold but above the speed-match threshold.
                if effective_skew_abs >= config.sync_speedup_enter_counts().get_i32() {
                    SyncPhase::SpeedMatch
                } else {
                    SyncPhase::Balanced
                }
            } else {
                SyncPhase::PauseLead
            }
        }
    }
}

pub(super) fn clamp_duty(base: DutyPercent, trim: DutyPercentTrim) -> DutyPercent {
    DutyPercent::from_clamped_i32(base.get() as i32 + trim.get_i32())
}

pub(super) fn clamp_drive_duty(
    config: DeskConfig,
    base: DutyPercent,
    trim: DutyPercentTrim,
) -> DutyPercent {
    clamp_duty(base, trim).max(config.min_move_duty())
}

pub(super) fn sync_trim(
    phase: SyncPhase,
    is_leader: bool,
    step: DutyPercentTrim,
) -> DutyPercentTrim {
    match phase {
        SyncPhase::Balanced => DutyPercentTrim::new(0),
        SyncPhase::SpeedMatch if is_leader => DutyPercentTrim::new(-step.get()),
        SyncPhase::SpeedMatch => step,
        SyncPhase::PauseLead if is_leader => DutyPercentTrim::new(0),
        SyncPhase::PauseLead => step,
    }
}

pub(super) fn plan_move_leg(
    config: DeskConfig,
    direction: TravelDirection,
    axis: AxisTargetState,
    phase: SyncPhase,
    is_leader: bool,
) -> LegPlan {
    if axis.done || (matches!(phase, SyncPhase::PauseLead) && is_leader) {
        return LegPlan::Stop;
    }

    let duty = clamp_drive_duty(
        config,
        axis.base_duty,
        sync_trim(phase, is_leader, config.move_sync_duty_step()),
    );
    match direction {
        TravelDirection::Up => LegPlan::Up(duty),
        TravelDirection::Down => LegPlan::Down(duty),
    }
}

pub(super) fn plan_move(
    config: DeskConfig,
    direction: TravelDirection,
    phase: SyncPhase,
    lead_left: bool,
    snapshot: MoveSnapshot,
) -> DualLegPlan {
    DualLegPlan {
        left: plan_move_leg(config, direction, snapshot.left, phase, lead_left),
        right: plan_move_leg(config, direction, snapshot.right, phase, !lead_left),
    }
}

pub(super) fn plan_homing_down(
    config: DeskConfig,
    phase: SyncPhase,
    lead_left: bool,
) -> DualLegPlan {
    DualLegPlan {
        left: plan_homing_leg(config, phase, lead_left),
        right: plan_homing_leg(config, phase, !lead_left),
    }
}

pub(super) fn plan_homing_leg(config: DeskConfig, phase: SyncPhase, is_leader: bool) -> LegPlan {
    if matches!(phase, SyncPhase::PauseLead) && is_leader {
        return LegPlan::HomeDown(config.min_move_duty());
    }

    LegPlan::HomeDown(clamp_drive_duty(
        config,
        config.homing_run_duty(),
        sync_trim(phase, is_leader, config.homing_sync_duty_step()),
    ))
}

pub(super) fn apply_leg_plan<State, const OP: u8, PWM: PwmPeripheral>(
    leg: &mut Leg<'_, State, OP, PWM>,
    plan: LegPlan,
    leg_config: LegRuntimeConfig,
) {
    match plan {
        LegPlan::Stop => leg.apply_drive_mode(DriveMode::Stop, leg_config),
        LegPlan::Up(duty) => leg.drive_up_duty(duty, leg_config),
        LegPlan::Down(duty) => leg.drive_down_duty(duty, leg_config),
        LegPlan::HomeDown(duty) => leg.drive_home_down_duty(duty, leg_config),
    }
}

pub(super) fn apply_dual_plan<
    LeftState,
    RightState,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
>(
    left_leg: &mut Leg<'_, LeftState, LEFT_OP, LeftPwm>,
    right_leg: &mut Leg<'_, RightState, RIGHT_OP, RightPwm>,
    plan: DualLegPlan,
    leg_config: LegRuntimeConfig,
) {
    apply_leg_plan(left_leg, plan.left, leg_config);
    apply_leg_plan(right_leg, plan.right, leg_config);
}
