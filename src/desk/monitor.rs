use embassy_time::Instant;

use crate::{
    config::{DeskConfig, ObstructionProfileConfig},
    quadrature::QuadratureDirection,
    units::{DutyPercent, PositionCounts, abs_position_delta, average_position, position_delta},
};

use super::{
    movement::{DeskMoveInvariant, DeskMoveOutcome},
    planning::{SyncPhase, TravelDirection},
    state::DeskError,
    status::DeskStatus,
};

#[derive(Clone, Copy)]
pub(super) struct AxisTargetState {
    pub(super) done: bool,
    pub(super) near_target: bool,
    pub(super) base_duty: DutyPercent,
}

#[derive(Clone, Copy)]
pub(super) struct MoveSnapshot {
    pub(super) observed_skew: i32,
    pub(super) observed_skew_abs: i32,
    pub(super) left: AxisTargetState,
    pub(super) right: AxisTargetState,
}

pub(super) struct ObstructionMonitor {
    config: DeskConfig,
    pub(super) direction: TravelDirection,
    move_started_at: Instant,
    start_left: i32,
    start_right: i32,
    max_left_travel: i32,
    max_right_travel: i32,
    speed_drop: SpeedDropMonitor,
}

pub(super) struct HomingContactMonitor {
    config: DeskConfig,
    direction: QuadratureDirection,
    started_at: Instant,
    start_position: i32,
    max_travel: i32,
    speed_drop: SpeedDropMonitor,
}

struct SpeedDropMonitor {
    profile: ObstructionProfileConfig,
    warmed_up: bool,
    window_started_at: Instant,
    window_start_total_travel: i32,
    baseline_speed: u32,
    consecutive_slow_windows: u8,
}

pub(super) struct MoveProgressMonitor {
    warmup_counts: i32,
    pub(super) direction: TravelDirection,
    start_average_position: i32,
    max_forward_travel: i32,
    retreat_counts: i32,
    consecutive_wrong_way_samples: u8,
}

pub(super) struct MoveStart {
    pub(super) target: i32,
    pub(super) direction: TravelDirection,
}

pub(super) enum MoveStepValidation {
    Continue(MoveSnapshot),
    Completed,
}

impl MoveProgressMonitor {
    pub(super) fn new(
        config: DeskConfig,
        direction: TravelDirection,
        average_position: i32,
    ) -> Self {
        let warmup_counts = config.obstruction_warmup_counts().get_i32();
        Self {
            warmup_counts,
            direction,
            start_average_position: average_position,
            max_forward_travel: 0,
            retreat_counts: warmup_counts,
            consecutive_wrong_way_samples: 0,
        }
    }

    pub(super) fn observe(&mut self, average_position: i32) -> Option<DeskMoveInvariant> {
        let forward_travel = match self.direction {
            TravelDirection::Up => position_delta(average_position, self.start_average_position),
            TravelDirection::Down => position_delta(self.start_average_position, average_position),
        }
        .max(0);
        self.max_forward_travel = self.max_forward_travel.max(forward_travel);

        // Ignore early samples until normal startup movement has had room to
        // overcome encoder jitter and mechanical slack.
        if self.max_forward_travel < self.warmup_counts {
            self.consecutive_wrong_way_samples = 0;
            return None;
        }

        // Compare against the best forward progress, not the start point, so a
        // retreat after valid movement is caught in either travel direction.
        let moved_wrong_way =
            self.max_forward_travel.saturating_sub(forward_travel) >= self.retreat_counts;

        if moved_wrong_way {
            self.consecutive_wrong_way_samples =
                self.consecutive_wrong_way_samples.saturating_add(1);
        } else {
            self.consecutive_wrong_way_samples = 0;
        }

        (self.consecutive_wrong_way_samples >= 2).then_some(DeskMoveInvariant::WrongWayProgress)
    }
}

impl MoveSnapshot {
    pub(super) fn new(
        config: DeskConfig,
        direction: TravelDirection,
        target: i32,
        left_position: i32,
        right_position: i32,
    ) -> Self {
        let tolerance = config.target_tolerance().get_i32();
        let left_done = axis_done(direction, target, left_position, tolerance);
        let right_done = axis_done(direction, target, right_position, tolerance);
        let left_near =
            abs_position_delta(target, left_position) <= config.target_slow_zone().get_i32();
        let right_near =
            abs_position_delta(target, right_position) <= config.target_slow_zone().get_i32();

        Self {
            observed_skew: position_delta(left_position, right_position),
            observed_skew_abs: abs_position_delta(left_position, right_position),
            left: AxisTargetState {
                done: left_done,
                near_target: left_near,
                base_duty: axis_base_duty(config, left_near, left_done, right_done),
            },
            right: AxisTargetState {
                done: right_done,
                near_target: right_near,
                base_duty: axis_base_duty(config, right_near, right_done, left_done),
            },
        }
    }

    pub(super) fn is_complete(self) -> bool {
        self.left.done && self.right.done
    }

    pub(super) fn is_skew_fault(self, config: DeskConfig) -> bool {
        self.observed_skew_abs > config.fault_skew_counts().get_i32()
    }

    pub(super) fn suspends_obstruction_detection(self, phase: SyncPhase) -> bool {
        // Sync correction and near-target slowing intentionally reduce speed,
        // so obstruction windows collected there would look like false stalls.
        !matches!(phase, SyncPhase::Balanced) || self.left.near_target || self.right.near_target
    }
}

pub(super) fn validate_move_start(
    status: DeskStatus,
    target_position: PositionCounts,
    config: DeskConfig,
) -> Result<MoveStart, DeskMoveOutcome> {
    let target = target_position
        .clamp(
            PositionCounts::new(status.min_position),
            PositionCounts::new(status.max_position),
        )
        .get();

    if abs_position_delta(target, status.average_position) <= config.target_tolerance().get_i32() {
        return Err(DeskMoveOutcome::Completed);
    }

    let Some(direction) = TravelDirection::from_target(target, status.average_position) else {
        return Err(DeskMoveOutcome::Completed);
    };

    match direction {
        TravelDirection::Up if target <= status.average_position => Err(DeskMoveOutcome::Completed),
        TravelDirection::Down if target >= status.average_position => {
            Err(DeskMoveOutcome::Completed)
        }
        _ => Ok(MoveStart { target, direction }),
    }
}

pub(super) fn validate_move_step(
    config: DeskConfig,
    direction: TravelDirection,
    target: i32,
    left_position: i32,
    right_position: i32,
    progress_monitor: &mut MoveProgressMonitor,
) -> Result<MoveStepValidation, DeskError> {
    let snapshot = MoveSnapshot::new(config, direction, target, left_position, right_position);
    if snapshot.is_complete() {
        return Ok(MoveStepValidation::Completed);
    }

    if snapshot.is_skew_fault(config) {
        return Err(DeskError::SkewFault);
    }

    let average_position = average_position(left_position, right_position);
    if let Some(invariant) = progress_monitor.observe(average_position) {
        return Err(DeskError::InvariantViolation(invariant));
    }

    Ok(MoveStepValidation::Continue(snapshot))
}

impl ObstructionMonitor {
    pub(super) fn new(
        config: DeskConfig,
        direction: TravelDirection,
        started_at: Instant,
        left_position: i32,
        right_position: i32,
    ) -> Option<Self> {
        let profile = config.obstruction_profile(config.obstruction_sensitivity())?;

        Some(Self {
            config,
            direction,
            move_started_at: started_at,
            start_left: left_position,
            start_right: right_position,
            max_left_travel: 0,
            max_right_travel: 0,
            speed_drop: SpeedDropMonitor::new(profile, started_at),
        })
    }

    pub(super) fn observe(
        &mut self,
        now: Instant,
        left_position: i32,
        right_position: i32,
        snapshot: MoveSnapshot,
        phase: SyncPhase,
    ) -> bool {
        let left_travel = self.direction.travel(self.start_left, left_position);
        let right_travel = self.direction.travel(self.start_right, right_position);
        let total_travel = left_travel.saturating_add(right_travel);

        self.max_left_travel = self.max_left_travel.max(left_travel);
        self.max_right_travel = self.max_right_travel.max(right_travel);

        let warmed_up = now.saturating_duration_since(self.move_started_at)
            >= self.config.obstruction_warmup_duration()
            && self.max_left_travel >= self.config.obstruction_warmup_counts().get_i32()
            && self.max_right_travel >= self.config.obstruction_warmup_counts().get_i32();
        self.speed_drop.observe(
            self.config.obstruction_sample_window(),
            now,
            total_travel,
            snapshot.suspends_obstruction_detection(phase),
            warmed_up,
        )
    }
}

impl HomingContactMonitor {
    pub(super) fn new(
        config: DeskConfig,
        started_at: Instant,
        start_position: i32,
        direction: QuadratureDirection,
    ) -> Option<Self> {
        let profile = config.homing_obstruction_profile()?;

        Some(Self {
            config,
            direction,
            started_at,
            start_position,
            max_travel: 0,
            speed_drop: SpeedDropMonitor::new(profile, started_at),
        })
    }

    pub(super) fn observe(&mut self, now: Instant, current_position: i32, suspended: bool) -> bool {
        let travel = homing_travel(self.direction, self.start_position, current_position);
        self.max_travel = self.max_travel.max(travel);

        let warmed_up = now.saturating_duration_since(self.started_at)
            >= self.config.obstruction_warmup_duration()
            && self.max_travel >= self.config.obstruction_warmup_counts().get_i32();
        self.speed_drop.observe(
            self.config.obstruction_sample_window(),
            now,
            travel,
            suspended,
            warmed_up,
        )
    }
}

impl SpeedDropMonitor {
    fn new(profile: ObstructionProfileConfig, started_at: Instant) -> Self {
        Self {
            profile,
            warmed_up: false,
            window_started_at: started_at,
            window_start_total_travel: 0,
            baseline_speed: 0,
            consecutive_slow_windows: 0,
        }
    }

    fn observe(
        &mut self,
        sample_window: embassy_time::Duration,
        now: Instant,
        total_travel: i32,
        suspended: bool,
        warmed_up: bool,
    ) -> bool {
        if suspended {
            self.consecutive_slow_windows = 0;
            self.restart_window(now, total_travel);
            return false;
        }

        if !self.warmed_up {
            if warmed_up {
                self.warmed_up = true;
                self.consecutive_slow_windows = 0;
                self.restart_window(now, total_travel);
            }
            return false;
        }

        let elapsed = now.saturating_duration_since(self.window_started_at);
        if elapsed < sample_window {
            return false;
        }

        let elapsed_ms = elapsed.as_millis();
        if elapsed_ms == 0 {
            self.consecutive_slow_windows = 0;
            self.restart_window(now, total_travel);
            return false;
        }

        let window_travel = total_travel.saturating_sub(self.window_start_total_travel);
        let window_speed = (window_travel as u32).saturating_mul(1_000) / elapsed_ms as u32;

        if self.baseline_speed == 0 || window_speed >= self.baseline_speed {
            self.baseline_speed = window_speed;
            self.consecutive_slow_windows = 0;
        } else {
            let threshold_speed = self
                .baseline_speed
                .saturating_mul(self.profile.minimum_baseline_percent().get_u32())
                / 100;
            if window_speed < threshold_speed {
                self.consecutive_slow_windows = self.consecutive_slow_windows.saturating_add(1);
            } else {
                self.consecutive_slow_windows = 0;
            }
        }

        let speed_dropped = self.consecutive_slow_windows >= self.profile.consecutive_windows();
        self.restart_window(now, total_travel);
        speed_dropped
    }

    fn restart_window(&mut self, now: Instant, total_travel: i32) {
        self.window_started_at = now;
        self.window_start_total_travel = total_travel;
    }
}

fn homing_travel(
    direction: QuadratureDirection,
    start_position: i32,
    current_position: i32,
) -> i32 {
    match direction {
        QuadratureDirection::Positive => position_delta(current_position, start_position).max(0),
        QuadratureDirection::Negative => position_delta(start_position, current_position).max(0),
        QuadratureDirection::Invalid => 0,
    }
}

pub(super) fn axis_base_duty(
    config: DeskConfig,
    near: bool,
    done: bool,
    other_done: bool,
) -> DutyPercent {
    if other_done && !done {
        config.move_run_duty()
    } else if near {
        config.move_slow_duty()
    } else {
        config.move_run_duty()
    }
}

pub(super) fn axis_done(
    direction: TravelDirection,
    target: i32,
    position: i32,
    tolerance: i32,
) -> bool {
    match direction {
        TravelDirection::Up => position >= target.saturating_sub(tolerance),
        TravelDirection::Down => position <= target.saturating_add(tolerance),
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use embassy_time::Duration;

    use crate::config::HomingObstructionSensitivity;

    use super::*;

    #[test]
    fn homing_contact_monitor_is_disabled_with_no_obstruction_sensitivity() {
        let mut config = DeskConfig::default();
        config
            .set_homing_obstruction_sensitivity(HomingObstructionSensitivity::Off)
            .unwrap();

        assert!(
            HomingContactMonitor::new(
                config,
                Instant::from_ticks(0),
                0,
                QuadratureDirection::Positive,
            )
            .is_none()
        );
    }

    #[test]
    fn homing_contact_monitor_detects_sustained_slowdown() {
        let config = DeskConfig::default();
        let started_at = Instant::from_ticks(0);
        let mut monitor =
            HomingContactMonitor::new(config, started_at, 0, QuadratureDirection::Positive)
                .unwrap();

        assert!(!monitor.observe(started_at + Duration::from_millis(300), 8, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(450), 23, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(600), 38, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(750), 40, false));
        assert!(monitor.observe(started_at + Duration::from_millis(900), 41, false));
    }

    #[test]
    fn homing_contact_monitor_ignores_suspended_sync_windows() {
        let config = DeskConfig::default();
        let started_at = Instant::from_ticks(0);
        let mut monitor =
            HomingContactMonitor::new(config, started_at, 0, QuadratureDirection::Positive)
                .unwrap();

        assert!(!monitor.observe(started_at + Duration::from_millis(300), 8, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(450), 23, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(600), 38, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(750), 40, true));
        assert!(!monitor.observe(started_at + Duration::from_millis(900), 41, false));
    }

    #[test]
    fn homing_contact_monitor_supports_negative_encoder_direction() {
        let config = DeskConfig::default();
        let started_at = Instant::from_ticks(0);
        let mut monitor =
            HomingContactMonitor::new(config, started_at, 100, QuadratureDirection::Negative)
                .unwrap();

        assert!(!monitor.observe(started_at + Duration::from_millis(300), 92, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(450), 77, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(600), 62, false));
        assert!(!monitor.observe(started_at + Duration::from_millis(750), 60, false));
        assert!(monitor.observe(started_at + Duration::from_millis(900), 59, false));
    }
}
