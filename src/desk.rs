use core::cell::RefCell;

use defmt::Format;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use embassy_time::{Instant, Timer, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;
use static_cell::StaticCell;

pub use crate::config::ObstructionSensitivity;
use crate::config::{DeskConfig, LegRuntimeConfig, ObstructionProfileConfig, RuntimeConfigReader};
use crate::leg::{DriveMode, Leg, LegError, LegStatus, LegStatusWatcher, Ready, Unhomed};
use crate::quadrature::QuadratureDirection;
use crate::units::{CountDelta, DutyPercent, DutyPercentTrim, PositionCounts};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskMotionState {
    Idle,
    Homing,
    MovingUp,
    MovingDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskStopReason {
    None,
    TargetReached,
    UserStop,
    Obstruction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct DeskStatus {
    pub homed: bool,
    pub needs_rehome: bool,
    pub motion: DeskMotionState,
    pub last_stop_reason: DeskStopReason,
    pub target_active: bool,
    pub target_position: i32,
    pub left_position: i32,
    pub right_position: i32,
    pub left_duty: u8,
    pub right_duty: u8,
    pub average_position: i32,
    pub skew_counts: i32,
    pub min_position: i32,
    pub max_position: i32,
    left_min_position: i32,
    left_max_position: i32,
    right_min_position: i32,
    right_max_position: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskError {
    LeftLeg(LegError),
    RightLeg(LegError),
    MoveTimeout,
    SkewFault,
    InvariantViolation(DeskMoveInvariant),
    RehomeRequired,
    Stopped,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskMoveOutcome {
    Completed,
    StoppedByRequest,
    StoppedByObstruction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskMoveInvariant {
    DirectionMismatch,
    WrongWayProgress,
}

pub enum DeskMoveError<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
> {
    FaultedReady(
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        DeskError,
    ),
    FaultedUnhomed(
        Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        DeskError,
    ),
}

pub struct UnhomedDesk;
pub struct ReadyDesk;

pub struct DeskStatusStorage {
    state: StaticCell<DeskStatusState>,
}

pub struct DeskStatusWatcher {
    receiver: Receiver<'static, CriticalSectionRawMutex, DeskStatus, 4>,
}

#[derive(Clone, Copy)]
pub struct DeskStatusReader {
    status_state: &'static DeskStatusState,
}

struct DeskStatusState {
    status: Mutex<CriticalSectionRawMutex, RefCell<DeskStatus>>,
    watch: Watch<CriticalSectionRawMutex, DeskStatus, 4>,
}

#[derive(Clone, Copy)]
enum DeskSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskLegSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum OverrideLegDirection {
    Up,
    Down,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskOverrideOutcome {
    Completed,
    StoppedByRequest,
}

enum ManagedLeg<'a, const OP: u8, PWM: PwmPeripheral> {
    Unhomed(Leg<'a, Unhomed, OP, PWM>),
    Ready(Leg<'a, Ready, OP, PWM>),
}

#[derive(Clone, Copy)]
enum SyncPhase {
    Balanced,
    SpeedMatch,
    PauseLead,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TravelDirection {
    Up,
    Down,
}

#[derive(Clone, Copy)]
struct AxisTargetState {
    done: bool,
    near_target: bool,
    base_duty: DutyPercent,
}

#[derive(Clone, Copy)]
struct MoveSnapshot {
    observed_skew: i32,
    observed_skew_abs: i32,
    left: AxisTargetState,
    right: AxisTargetState,
}

struct ObstructionMonitor {
    config: DeskConfig,
    direction: TravelDirection,
    profile: ObstructionProfileConfig,
    move_started_at: Instant,
    start_left: i32,
    start_right: i32,
    max_left_travel: i32,
    max_right_travel: i32,
    warmed_up: bool,
    window_started_at: Instant,
    window_start_total_travel: i32,
    baseline_speed: u32,
    consecutive_slow_windows: u8,
}

struct MoveProgressMonitor {
    warmup_counts: i32,
    direction: TravelDirection,
    start_average_position: i32,
    max_forward_travel: i32,
    retreat_counts: i32,
    consecutive_wrong_way_samples: u8,
}

struct MoveStart {
    target: i32,
    direction: TravelDirection,
}

enum MoveStepValidation {
    Continue(MoveSnapshot),
    Completed,
}

#[derive(Clone, Copy)]
enum LegPlan {
    Stop,
    Up(DutyPercent),
    Down(DutyPercent),
    HomeDown(DutyPercent),
}

#[derive(Clone, Copy)]
struct DualLegPlan {
    left: LegPlan,
    right: LegPlan,
}

type ReadyLegPair<'desk, 'leg, const LEFT_OP: u8, LeftPwm, const RIGHT_OP: u8, RightPwm> = (
    &'desk mut Leg<'leg, Ready, LEFT_OP, LeftPwm>,
    &'desk mut Leg<'leg, Ready, RIGHT_OP, RightPwm>,
);

pub struct Desk<
    'a,
    State,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
> {
    left: Option<ManagedLeg<'a, LEFT_OP, LeftPwm>>,
    right: Option<ManagedLeg<'a, RIGHT_OP, RightPwm>>,
    runtime_config_reader: RuntimeConfigReader,
    status_state: &'static DeskStatusState,
    _state: State,
}

impl Default for DeskStatusStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl DeskStatusStorage {
    pub const fn new() -> Self {
        Self {
            state: StaticCell::new(),
        }
    }
}

impl DeskStatusWatcher {
    pub async fn wait_for_change(&mut self) -> DeskStatus {
        self.receiver.changed().await
    }
}

impl DeskStatusReader {
    pub fn current(&self) -> DeskStatus {
        self.status_state.current()
    }
}

impl TravelDirection {
    fn from_target(target: i32, current: i32) -> Option<Self> {
        if target > current {
            Some(Self::Up)
        } else if target < current {
            Some(Self::Down)
        } else {
            None
        }
    }

    fn motion_state(self) -> DeskMotionState {
        match self {
            Self::Up => DeskMotionState::MovingUp,
            Self::Down => DeskMotionState::MovingDown,
        }
    }

    fn lead_left(self, observed_skew: i32) -> bool {
        match self {
            Self::Up => observed_skew > 0,
            Self::Down => observed_skew < 0,
        }
    }

    fn travel(self, start_position: i32, current_position: i32) -> i32 {
        match self {
            Self::Up => (current_position - start_position).max(0),
            Self::Down => (start_position - current_position).max(0),
        }
    }
}

impl MoveProgressMonitor {
    fn new(config: DeskConfig, direction: TravelDirection, average_position: i32) -> Self {
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

    fn observe(&mut self, average_position: i32) -> Option<DeskMoveInvariant> {
        let forward_travel = match self.direction {
            TravelDirection::Up => average_position - self.start_average_position,
            TravelDirection::Down => self.start_average_position - average_position,
        }
        .max(0);
        self.max_forward_travel = self.max_forward_travel.max(forward_travel);

        if self.max_forward_travel < self.warmup_counts {
            self.consecutive_wrong_way_samples = 0;
            return None;
        }

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
    fn new(
        config: DeskConfig,
        direction: TravelDirection,
        target: i32,
        left_position: i32,
        right_position: i32,
    ) -> Self {
        let tolerance = config.target_tolerance().get_i32();
        let left_error = target - left_position;
        let right_error = target - right_position;
        let left_done = axis_done(direction, target, left_position, tolerance);
        let right_done = axis_done(direction, target, right_position, tolerance);
        let left_near = left_error.abs() <= config.target_slow_zone().get_i32();
        let right_near = right_error.abs() <= config.target_slow_zone().get_i32();

        Self {
            observed_skew: left_position - right_position,
            observed_skew_abs: (left_position - right_position).abs(),
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

    fn is_complete(self) -> bool {
        self.left.done && self.right.done
    }

    fn is_skew_fault(self, config: DeskConfig) -> bool {
        self.observed_skew_abs > config.fault_skew_counts().get_i32()
    }

    fn suspends_obstruction_detection(self, phase: SyncPhase) -> bool {
        !matches!(phase, SyncPhase::Balanced) || self.left.near_target || self.right.near_target
    }
}

fn average_position(left_position: i32, right_position: i32) -> i32 {
    (left_position + right_position) / 2
}

fn validate_move_start(
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

    if (target - status.average_position).abs() <= config.target_tolerance().get_i32() {
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

fn validate_move_step(
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
    fn new(
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
            profile,
            move_started_at: started_at,
            start_left: left_position,
            start_right: right_position,
            max_left_travel: 0,
            max_right_travel: 0,
            warmed_up: false,
            window_started_at: started_at,
            window_start_total_travel: 0,
            baseline_speed: 0,
            consecutive_slow_windows: 0,
        })
    }

    fn observe(
        &mut self,
        now: Instant,
        left_position: i32,
        right_position: i32,
        snapshot: MoveSnapshot,
        phase: SyncPhase,
    ) -> bool {
        let left_travel = self.direction.travel(self.start_left, left_position);
        let right_travel = self.direction.travel(self.start_right, right_position);
        let total_travel = left_travel + right_travel;

        self.max_left_travel = self.max_left_travel.max(left_travel);
        self.max_right_travel = self.max_right_travel.max(right_travel);

        if snapshot.suspends_obstruction_detection(phase) {
            self.consecutive_slow_windows = 0;
            self.restart_window(now, total_travel);
            return false;
        }

        if !self.warmed_up {
            let warmed_up = now.saturating_duration_since(self.move_started_at)
                >= self.config.obstruction_warmup_duration()
                && self.max_left_travel >= self.config.obstruction_warmup_counts().get_i32()
                && self.max_right_travel >= self.config.obstruction_warmup_counts().get_i32();
            if warmed_up {
                self.warmed_up = true;
                self.consecutive_slow_windows = 0;
                self.restart_window(now, total_travel);
            }
            return false;
        }

        let elapsed = now.saturating_duration_since(self.window_started_at);
        if elapsed < self.config.obstruction_sample_window() {
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

        let obstructed = self.consecutive_slow_windows >= self.profile.consecutive_windows();
        self.restart_window(now, total_travel);
        obstructed
    }

    fn restart_window(&mut self, now: Instant, total_travel: i32) {
        self.window_started_at = now;
        self.window_start_total_travel = total_travel;
    }
}

impl DeskStatusState {
    fn new(initial_status: DeskStatus) -> Self {
        Self {
            status: Mutex::new(RefCell::new(initial_status)),
            watch: Watch::new(),
        }
    }

    fn current(&self) -> DeskStatus {
        self.status.lock(|status| *status.borrow())
    }

    fn publish(&self, status: DeskStatus) {
        let changed = self.status.lock(|cached_status| {
            let mut cached_status = cached_status.borrow_mut();
            if *cached_status == status {
                false
            } else {
                *cached_status = status;
                true
            }
        });
        if changed {
            self.watch.sender().send(status);
        }
    }

    fn update(&self, update_fn: impl FnOnce(&mut DeskStatus)) -> DeskStatus {
        let (status, changed) = self.status.lock(|cached_status| {
            let mut cached_status = cached_status.borrow_mut();
            let previous = *cached_status;
            update_fn(&mut cached_status);
            cached_status.average_position =
                (cached_status.left_position + cached_status.right_position) / 2;
            cached_status.skew_counts =
                (cached_status.left_position - cached_status.right_position).abs();
            cached_status.min_position = cached_status
                .left_min_position
                .max(cached_status.right_min_position);
            cached_status.max_position = cached_status
                .left_max_position
                .min(cached_status.right_max_position);
            (*cached_status, *cached_status != previous)
        });
        if changed {
            self.watch.sender().send(status);
        }
        status
    }

    fn update_from_leg(&self, side: DeskSide, leg_status: LegStatus) {
        self.update(|status| match side {
            DeskSide::Left => {
                status.left_position = leg_status.position;
                status.left_duty = leg_status.duty;
                status.left_min_position = leg_status.min_position;
                status.left_max_position = leg_status.max_position;
            }
            DeskSide::Right => {
                status.right_position = leg_status.position;
                status.right_duty = leg_status.duty;
                status.right_min_position = leg_status.min_position;
                status.right_max_position = leg_status.max_position;
            }
        });
    }
}

#[embassy_executor::task(pool_size = 4)]
async fn mirror_leg_to_desk_status(
    side: DeskSide,
    mut watcher: LegStatusWatcher,
    status_state: &'static DeskStatusState,
) {
    loop {
        let leg_status = watcher.wait_for_change().await;
        status_state.update_from_leg(side, leg_status);
    }
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
    pub fn status(&self) -> DeskStatus {
        self.status_state.current()
    }

    pub fn status_watcher(&self) -> DeskStatusWatcher {
        let receiver = self
            .status_state
            .watch
            .receiver()
            .expect("desk status watch receiver limit reached");

        DeskStatusWatcher { receiver }
    }

    pub fn status_reader(&self) -> DeskStatusReader {
        DeskStatusReader {
            status_state: self.status_state,
        }
    }

    fn update_status(&self, update_fn: impl FnOnce(&mut DeskStatus)) -> DeskStatus {
        self.status_state.update(update_fn)
    }

    fn stop_ready_legs(&mut self) {
        if let Some(ManagedLeg::Ready(left)) = &mut self.left {
            left.stop_for_desk();
        }
        if let Some(ManagedLeg::Ready(right)) = &mut self.right {
            right.stop_for_desk();
        }
    }

    fn force_unhomed(mut self) -> Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        let left = match self.left.take().expect("left leg missing") {
            ManagedLeg::Ready(left) => ManagedLeg::Unhomed(left.into_unhomed()),
            ManagedLeg::Unhomed(left) => ManagedLeg::Unhomed(left),
        };
        let right = match self.right.take().expect("right leg missing") {
            ManagedLeg::Ready(right) => ManagedLeg::Unhomed(right.into_unhomed()),
            ManagedLeg::Unhomed(right) => ManagedLeg::Unhomed(right),
        };
        self.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });

        Desk {
            left: Some(left),
            right: Some(right),
            runtime_config_reader: self.runtime_config_reader,
            status_state: self.status_state,
            _state: UnhomedDesk,
        }
    }

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

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    fn take_unhomed_legs(
        &mut self,
    ) -> (
        Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
    ) {
        let left = match self.left.take().expect("left leg missing") {
            ManagedLeg::Unhomed(left) => left,
            ManagedLeg::Ready(_) => unreachable!("override must force left leg unhomed first"),
        };
        let right = match self.right.take().expect("right leg missing") {
            ManagedLeg::Unhomed(right) => right,
            ManagedLeg::Ready(_) => unreachable!("override must force right leg unhomed first"),
        };
        (left, right)
    }

    fn finish_override_unhomed(
        mut self,
        left_leg: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        right_leg: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
        stop_reason: DeskStopReason,
    ) -> Self {
        self.left = Some(ManagedLeg::Unhomed(left_leg));
        self.right = Some(ManagedLeg::Unhomed(right_leg));
        self.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = stop_reason;
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });
        self
    }

    pub fn new(
        storage: &'static DeskStatusStorage,
        runtime_config_reader: RuntimeConfigReader,
        left: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        right: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
        left_status_watcher: LegStatusWatcher,
        right_status_watcher: LegStatusWatcher,
        spawner: &Spawner,
    ) -> Self {
        let left_position = left.logical_position();
        let right_position = right.logical_position();
        let initial_status = DeskStatus {
            homed: false,
            needs_rehome: true,
            motion: DeskMotionState::Idle,
            last_stop_reason: DeskStopReason::None,
            target_active: false,
            target_position: 0,
            left_position,
            right_position,
            left_duty: 0,
            right_duty: 0,
            average_position: (left_position + right_position) / 2,
            skew_counts: (left_position - right_position).abs(),
            min_position: 0,
            max_position: 0,
            left_min_position: 0,
            left_max_position: 0,
            right_min_position: 0,
            right_max_position: 0,
        };
        let status_state = storage.state.init(DeskStatusState::new(initial_status));
        spawner.must_spawn(mirror_leg_to_desk_status(
            DeskSide::Left,
            left_status_watcher,
            status_state,
        ));
        spawner.must_spawn(mirror_leg_to_desk_status(
            DeskSide::Right,
            right_status_watcher,
            status_state,
        ));

        let desk = Self {
            left: Some(ManagedLeg::Unhomed(left)),
            right: Some(ManagedLeg::Unhomed(right)),
            runtime_config_reader,
            status_state,
            _state: UnhomedDesk,
        };
        desk.status_state.publish(desk.status());
        desk
    }

    fn restore_unhomed(
        mut self,
        left_leg: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        right_leg: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
        error: DeskError,
    ) -> (Self, DeskError) {
        self.left = Some(ManagedLeg::Unhomed(left_leg));
        self.right = Some(ManagedLeg::Unhomed(right_leg));
        self.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });
        (self, error)
    }

    pub async fn home_all<StopRequested>(
        mut self,
        stop_requested: StopRequested,
    ) -> Result<
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    >
    where
        StopRequested: Fn() -> bool,
    {
        let runtime_config = self.runtime_config_reader.current();
        let desk_config = runtime_config.desk();
        let leg_config = runtime_config.leg();
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Homing;
            status.homed = false;
            status.needs_rehome = true;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
        });

        let left_leg = self.left.take().expect("left leg missing");
        let right_leg = self.right.take().expect("right leg missing");

        let (mut left_leg, mut right_leg) = match (left_leg, right_leg) {
            (ManagedLeg::Unhomed(left_leg), ManagedLeg::Unhomed(right_leg)) => {
                (left_leg, right_leg)
            }
            (left_leg, right_leg) => {
                self.left = Some(left_leg);
                self.right = Some(right_leg);
                self.update_status(|status| status.motion = DeskMotionState::Idle);
                return Err((self, DeskError::RehomeRequired));
            }
        };

        let left_direction = left_leg.down_direction();
        let right_direction = right_leg.down_direction();
        let left_up_direction = left_leg.up_direction();
        let right_up_direction = right_leg.up_direction();
        let alignment_tolerance = desk_config.sync_speedup_exit_counts().get_i32();

        let initial_left_position = left_leg.logical_encoder_position();
        let initial_right_position = right_leg.logical_encoder_position();
        if initial_left_position > initial_right_position + alignment_tolerance {
            right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            left_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);
            let mut last_position = left_leg.encoder_position();
            let mut last_progress = Instant::now();

            while left_leg.logical_encoder_position() > initial_right_position + alignment_tolerance
            {
                if stop_requested() {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(left_leg, right_leg, DeskError::Stopped));
                }

                Timer::after(desk_config.homing_poll_interval()).await;
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(last_position, current_position, left_direction) {
                    last_position = current_position;
                    last_progress = Instant::now();
                } else if progressed_in_direction(
                    last_position,
                    current_position,
                    left_up_direction,
                ) {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::LeftLeg(LegError::PolarityMismatch),
                    ));
                } else if Instant::now().saturating_duration_since(last_progress)
                    >= desk_config.homing_stall_timeout()
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::LeftLeg(LegError::MoveTimeout),
                    ));
                }
            }

            left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
        } else if initial_right_position > initial_left_position + alignment_tolerance {
            left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            right_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);
            let mut last_position = right_leg.encoder_position();
            let mut last_progress = Instant::now();

            while right_leg.logical_encoder_position() > initial_left_position + alignment_tolerance
            {
                if stop_requested() {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(left_leg, right_leg, DeskError::Stopped));
                }

                Timer::after(desk_config.homing_poll_interval()).await;
                let current_position = right_leg.encoder_position();
                if progressed_in_direction(last_position, current_position, right_direction) {
                    last_position = current_position;
                    last_progress = Instant::now();
                } else if progressed_in_direction(
                    last_position,
                    current_position,
                    right_up_direction,
                ) {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::RightLeg(LegError::PolarityMismatch),
                    ));
                } else if Instant::now().saturating_duration_since(last_progress)
                    >= desk_config.homing_stall_timeout()
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::RightLeg(LegError::MoveTimeout),
                    ));
                }
            }

            left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
        }

        let left_start = left_leg.encoder_position();
        let right_start = right_leg.encoder_position();
        let homing_start_deadline = Instant::now() + desk_config.homing_start_timeout();

        left_leg.apply_drive_mode(DriveMode::DownBoost, leg_config);
        right_leg.apply_drive_mode(DriveMode::DownBoost, leg_config);

        let mut left_started = false;
        let mut right_started = false;

        loop {
            if stop_requested() {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::Stopped));
            }

            if !left_started {
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(left_start, current_position, left_direction) {
                    left_started = true;
                } else if progressed_in_direction(left_start, current_position, left_up_direction) {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::LeftLeg(LegError::PolarityMismatch),
                    ));
                }
            }

            if !right_started {
                let current_position = right_leg.encoder_position();
                if progressed_in_direction(right_start, current_position, right_direction) {
                    right_started = true;
                } else if progressed_in_direction(right_start, current_position, right_up_direction)
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::RightLeg(LegError::PolarityMismatch),
                    ));
                }
            }

            if left_started && right_started {
                break;
            }

            if Instant::now() >= homing_start_deadline {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                let error = if !left_started {
                    DeskError::LeftLeg(LegError::HomingStartTimeout)
                } else {
                    DeskError::RightLeg(LegError::HomingStartTimeout)
                };
                return Err(self.restore_unhomed(left_leg, right_leg, error));
            }

            Timer::after(desk_config.homing_poll_interval()).await;
        }

        left_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);
        right_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);

        let mut left_last_position = left_leg.encoder_position();
        let mut right_last_position = right_leg.encoder_position();
        let mut left_last_progress = Instant::now();
        let mut right_last_progress = Instant::now();
        let mut left_stalled = false;
        let mut right_stalled = false;
        let mut homing_sync_phase = SyncPhase::Balanced;

        while !left_stalled || !right_stalled {
            if stop_requested() {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::Stopped));
            }

            Timer::after(desk_config.homing_poll_interval()).await;

            if !left_stalled {
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(left_last_position, current_position, left_direction) {
                    left_last_position = current_position;
                    left_last_progress = Instant::now();
                } else if Instant::now().saturating_duration_since(left_last_progress)
                    >= desk_config.homing_stall_timeout()
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    left_stalled = true;
                }
            }

            if !right_stalled {
                let current_position = right_leg.encoder_position();
                if progressed_in_direction(right_last_position, current_position, right_direction) {
                    right_last_position = current_position;
                    right_last_progress = Instant::now();
                } else if Instant::now().saturating_duration_since(right_last_progress)
                    >= desk_config.homing_stall_timeout()
                {
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_stalled = true;
                }
            }

            let left_travel =
                travel_in_direction(left_start, left_leg.encoder_position(), left_direction);
            let right_travel =
                travel_in_direction(right_start, right_leg.encoder_position(), right_direction);
            let observed_skew = left_travel - right_travel;
            let homing_skew = observed_skew.abs();

            if homing_skew > desk_config.homing_fault_skew_counts().get_i32() {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::SkewFault));
            }

            homing_sync_phase = next_sync_phase(desk_config, homing_sync_phase, homing_skew);
            let lead_left = observed_skew > 0;
            let active_phase = if !left_stalled && !right_stalled {
                homing_sync_phase
            } else {
                SyncPhase::Balanced
            };
            let mut plan = plan_homing_down(desk_config, active_phase, lead_left);

            if left_stalled {
                plan.left = LegPlan::Stop;
            }
            if right_stalled {
                plan.right = LegPlan::Stop;
            }

            apply_dual_plan(&mut left_leg, &mut right_leg, plan, leg_config);
        }

        let mut left_backoff_position = left_leg.encoder_position();
        let mut right_backoff_position = right_leg.encoder_position();
        let mut left_backoff_progress_at = Instant::now();
        let mut right_backoff_progress_at = Instant::now();
        let mut left_backoff_done = false;
        let mut right_backoff_done = false;
        let mut left_backoff = 0;
        let mut right_backoff = 0;
        let mut left_boosting = true;
        let mut right_boosting = true;

        left_leg.apply_drive_mode(DriveMode::UpBoost, leg_config);
        right_leg.apply_drive_mode(DriveMode::UpBoost, leg_config);

        while !left_backoff_done || !right_backoff_done {
            if stop_requested() {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::Stopped));
            }

            Timer::after(desk_config.homing_poll_interval()).await;

            if !left_backoff_done {
                let current_position = left_leg.encoder_position();
                let progress =
                    travel_in_direction(left_backoff_position, current_position, left_up_direction);
                if progress > 0 {
                    left_backoff_progress_at = Instant::now();
                    left_backoff += progress;
                    left_backoff_position = current_position;
                    if left_boosting {
                        left_leg.apply_drive_mode(DriveMode::UpRun, leg_config);
                        left_boosting = false;
                    }
                }

                if left_backoff >= desk_config.homing_backoff_steps().get_i32() {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    left_backoff_done = true;
                } else if Instant::now().saturating_duration_since(left_backoff_progress_at)
                    >= desk_config.move_timeout()
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::LeftLeg(LegError::MoveTimeout),
                    ));
                }
            }

            if !right_backoff_done {
                let current_position = right_leg.encoder_position();
                let progress = travel_in_direction(
                    right_backoff_position,
                    current_position,
                    right_up_direction,
                );
                if progress > 0 {
                    right_backoff_progress_at = Instant::now();
                    right_backoff += progress;
                    right_backoff_position = current_position;
                    if right_boosting {
                        right_leg.apply_drive_mode(DriveMode::UpRun, leg_config);
                        right_boosting = false;
                    }
                }

                if right_backoff >= desk_config.homing_backoff_steps().get_i32() {
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_backoff_done = true;
                } else if Instant::now().saturating_duration_since(right_backoff_progress_at)
                    >= desk_config.move_timeout()
                {
                    left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::RightLeg(LegError::MoveTimeout),
                    ));
                }
            }

            if (left_backoff - right_backoff).abs() > desk_config.fault_skew_counts().get_i32() {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::SkewFault));
            }
        }

        left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
        right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
        left_leg.reset_position();
        right_leg.reset_position();
        let left_leg = left_leg.into_ready_with_config(leg_config);
        let right_leg = right_leg.into_ready_with_config(leg_config);
        let left_status = left_leg.status();
        let right_status = right_leg.status();
        left_leg.publish_status();
        right_leg.publish_status();

        self.update_status(|status| {
            status.homed = true;
            status.needs_rehome = false;
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
            status.left_position = left_status.position;
            status.right_position = right_status.position;
            status.left_min_position = left_status.min_position;
            status.left_max_position = left_status.max_position;
            status.right_min_position = right_status.min_position;
            status.right_max_position = right_status.max_position;
        });

        Ok(Desk {
            left: Some(ManagedLeg::Ready(left_leg)),
            right: Some(ManagedLeg::Ready(right_leg)),
            runtime_config_reader: self.runtime_config_reader,
            status_state: self.status_state,
            _state: ReadyDesk,
        })
    }
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    pub async fn home_all<StopRequested>(
        self,
        stop_requested: StopRequested,
    ) -> Result<
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    >
    where
        StopRequested: Fn() -> bool,
    {
        self.into_unhomed().home_all(stop_requested).await
    }

    fn ready_legs_mut(&mut self) -> ReadyLegPair<'_, 'a, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        match (&mut self.left, &mut self.right) {
            (Some(ManagedLeg::Ready(left)), Some(ManagedLeg::Ready(right))) => (left, right),
            _ => unreachable!("ready desk must contain ready legs"),
        }
    }

    fn into_unhomed(mut self) -> Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        self.stop_ready_legs();

        let left = match self.left.take().expect("left leg missing") {
            ManagedLeg::Ready(left) => ManagedLeg::Unhomed(left.into_unhomed()),
            ManagedLeg::Unhomed(left) => ManagedLeg::Unhomed(left),
        };
        let right = match self.right.take().expect("right leg missing") {
            ManagedLeg::Ready(right) => ManagedLeg::Unhomed(right.into_unhomed()),
            ManagedLeg::Unhomed(right) => ManagedLeg::Unhomed(right),
        };

        self.update_status(|status| {
            status.homed = false;
            status.needs_rehome = true;
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });

        Desk {
            left: Some(left),
            right: Some(right),
            runtime_config_reader: self.runtime_config_reader,
            status_state: self.status_state,
            _state: UnhomedDesk,
        }
    }

    fn fail_move(
        self,
        error: DeskError,
    ) -> (
        Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        DeskError,
    ) {
        (self.into_unhomed(), error)
    }

    fn fault_ready_move(
        mut self,
        error: DeskError,
    ) -> (
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        DeskError,
    ) {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Idle;
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = false;
        });
        (self, error)
    }

    pub fn stop_with_reason(&mut self, reason: DeskStopReason) {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Idle;
            status.target_active = false;
            status.last_stop_reason = reason;
        });
    }

    pub async fn move_to<StopRequested>(
        mut self,
        target_position: PositionCounts,
        stop_requested: StopRequested,
    ) -> Result<
        (
            Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskMoveOutcome,
        ),
        DeskMoveError<'a, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
    >
    where
        StopRequested: Fn() -> bool,
    {
        let runtime_config = self.runtime_config_reader.current();
        let desk_config = runtime_config.desk();
        let leg_config = runtime_config.leg();
        let status = self.status();
        if !status.homed || status.needs_rehome {
            let (desk, error) = self.fail_move(DeskError::RehomeRequired);
            return Err(DeskMoveError::FaultedUnhomed(desk, error));
        }

        if stop_requested() {
            self.stop_with_reason(DeskStopReason::UserStop);
            return Ok((self, DeskMoveOutcome::StoppedByRequest));
        }

        let MoveStart {
            target: shared_target,
            direction,
        } = match validate_move_start(status, target_position, desk_config) {
            Ok(start) => start,
            Err(DeskMoveOutcome::Completed) => {
                self.stop_with_reason(DeskStopReason::TargetReached);
                return Ok((self, DeskMoveOutcome::Completed));
            }
            Err(outcome) => return Ok((self, outcome)),
        };

        self.update_status(|status| {
            status.last_stop_reason = DeskStopReason::None;
            status.target_active = true;
            status.target_position = shared_target;
            status.motion = direction.motion_state();
        });

        {
            let (left_leg, right_leg) = self.ready_legs_mut();
            match direction {
                TravelDirection::Up => {
                    left_leg.start_up_boost(leg_config);
                    right_leg.start_up_boost(leg_config);
                }
                TravelDirection::Down => {
                    left_leg.start_down_boost(leg_config);
                    right_leg.start_down_boost(leg_config);
                }
            }
        }

        let (
            mut left_progress_watcher,
            mut right_progress_watcher,
            mut left_status,
            mut right_status,
        ) = {
            let (left_leg, right_leg) = self.ready_legs_mut();
            (
                left_leg.progress_watcher(),
                right_leg.progress_watcher(),
                left_leg.status(),
                right_leg.status(),
            )
        };
        let mut sync_phase = SyncPhase::Balanced;
        let move_started_at = Instant::now();
        let mut obstruction_monitor = ObstructionMonitor::new(
            desk_config,
            direction,
            move_started_at,
            left_status.position,
            right_status.position,
        );
        let mut progress_monitor = MoveProgressMonitor::new(
            desk_config,
            direction,
            average_position(left_status.position, right_status.position),
        );

        loop {
            if stop_requested() {
                self.stop_with_reason(DeskStopReason::UserStop);
                return Ok((self, DeskMoveOutcome::StoppedByRequest));
            }

            match with_deadline(
                Instant::now() + desk_config.move_timeout(),
                select(
                    left_progress_watcher.wait_for_change(),
                    right_progress_watcher.wait_for_change(),
                ),
            )
            .await
            {
                Ok(Either::First(progress)) => left_status.position = progress.position,
                Ok(Either::Second(progress)) => right_status.position = progress.position,
                Err(_) => {
                    let (desk, error) = self.fail_move(DeskError::MoveTimeout);
                    return Err(DeskMoveError::FaultedUnhomed(desk, error));
                }
            };

            let snapshot = match validate_move_step(
                desk_config,
                direction,
                shared_target,
                left_status.position,
                right_status.position,
                &mut progress_monitor,
            ) {
                Ok(MoveStepValidation::Completed) => {
                    self.stop_with_reason(DeskStopReason::TargetReached);
                    return Ok((self, DeskMoveOutcome::Completed));
                }
                Ok(MoveStepValidation::Continue(snapshot)) => snapshot,
                Err(DeskError::InvariantViolation(invariant)) => {
                    let error = DeskError::InvariantViolation(invariant);
                    let (desk, error) = self.fault_ready_move(error);
                    return Err(DeskMoveError::FaultedReady(desk, error));
                }
                Err(error) => {
                    let (desk, error) = self.fail_move(error);
                    return Err(DeskMoveError::FaultedUnhomed(desk, error));
                }
            };

            sync_phase = next_sync_phase(desk_config, sync_phase, snapshot.observed_skew_abs);
            let (left_leg, right_leg) = self.ready_legs_mut();
            let plan = plan_move(
                desk_config,
                direction,
                sync_phase,
                direction.lead_left(snapshot.observed_skew),
                snapshot,
            );
            apply_dual_plan(left_leg, right_leg, plan, leg_config);

            if obstruction_monitor.as_mut().is_some_and(|monitor| {
                monitor.observe(
                    Instant::now(),
                    left_status.position,
                    right_status.position,
                    snapshot,
                    sync_phase,
                )
            }) {
                self.stop_with_reason(DeskStopReason::Obstruction);
                return Ok((self, DeskMoveOutcome::StoppedByObstruction));
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

fn progressed_in_direction(
    last_position: i32,
    current_position: i32,
    direction: QuadratureDirection,
) -> bool {
    match direction {
        QuadratureDirection::Positive => current_position > last_position,
        QuadratureDirection::Negative => current_position < last_position,
        QuadratureDirection::Invalid => false,
    }
}

fn travel_in_direction(
    start_position: i32,
    current_position: i32,
    direction: QuadratureDirection,
) -> i32 {
    match direction {
        QuadratureDirection::Positive => (current_position - start_position).max(0),
        QuadratureDirection::Negative => (start_position - current_position).max(0),
        QuadratureDirection::Invalid => 0,
    }
}

fn next_sync_phase(config: DeskConfig, current: SyncPhase, effective_skew_abs: i32) -> SyncPhase {
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

fn axis_base_duty(config: DeskConfig, near: bool, done: bool, other_done: bool) -> DutyPercent {
    if other_done && !done {
        config.move_run_duty()
    } else if near {
        config.move_slow_duty()
    } else {
        config.move_run_duty()
    }
}

fn clamp_duty(base: DutyPercent, trim: DutyPercentTrim) -> DutyPercent {
    DutyPercent::from_clamped_i32(base.get() as i32 + trim.get_i32())
}

fn clamp_drive_duty(config: DeskConfig, base: DutyPercent, trim: DutyPercentTrim) -> DutyPercent {
    clamp_duty(base, trim).max(config.min_move_duty())
}

fn sync_trim(phase: SyncPhase, is_leader: bool, step: DutyPercentTrim) -> DutyPercentTrim {
    match phase {
        SyncPhase::Balanced => DutyPercentTrim::new(0),
        SyncPhase::SpeedMatch if is_leader => DutyPercentTrim::new(-step.get()),
        SyncPhase::SpeedMatch => step,
        SyncPhase::PauseLead if is_leader => DutyPercentTrim::new(0),
        SyncPhase::PauseLead => step,
    }
}

fn plan_move_leg(
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

fn axis_done(direction: TravelDirection, target: i32, position: i32, tolerance: i32) -> bool {
    match direction {
        TravelDirection::Up => position >= target.saturating_sub(tolerance),
        TravelDirection::Down => position <= target.saturating_add(tolerance),
    }
}

fn plan_move(
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

fn plan_homing_down(config: DeskConfig, phase: SyncPhase, lead_left: bool) -> DualLegPlan {
    DualLegPlan {
        left: plan_homing_leg(config, phase, lead_left),
        right: plan_homing_leg(config, phase, !lead_left),
    }
}

fn plan_homing_leg(config: DeskConfig, phase: SyncPhase, is_leader: bool) -> LegPlan {
    if matches!(phase, SyncPhase::PauseLead) && is_leader {
        return LegPlan::HomeDown(config.min_move_duty());
    }

    LegPlan::HomeDown(clamp_drive_duty(
        config,
        config.homing_run_duty(),
        sync_trim(phase, is_leader, config.homing_sync_duty_step()),
    ))
}

fn apply_leg_plan<State, const OP: u8, PWM: PwmPeripheral>(
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

fn apply_dual_plan<
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
