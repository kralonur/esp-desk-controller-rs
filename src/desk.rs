use core::cell::RefCell;

use defmt::Format;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;
use static_cell::StaticCell;

use crate::leg::{DriveMode, Leg, LegError, LegStatus, LegStatusWatcher, Ready, Unhomed};
use crate::quadrature::QuadratureDirection;

const DESK_TARGET_TOLERANCE: i32 = 5;
const DESK_TARGET_SLOW_ZONE: i32 = 10;
const DESK_MOVE_TIMEOUT: Duration = Duration::from_millis(1200);
const MIN_MOVE_DUTY: u16 = 10;
const MOVE_RUN_DUTY: u16 = 30;
const MOVE_SLOW_DUTY: u16 = 15;
const MOVE_SYNC_DUTY_STEP: i16 = 8;
const HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
const HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
const HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const HOMING_BACKOFF_STEPS: i32 = 20;
const HOMING_RUN_DUTY: u16 = 20;
const HOMING_SYNC_DUTY_STEP: i16 = 4;
const SYNC_SPEEDUP_ENTER_COUNTS: i32 = 10;
const SYNC_SPEEDUP_EXIT_COUNTS: i32 = 4;
const CATCH_UP_ENTER_COUNTS: i32 = 30;
const CATCH_UP_EXIT_COUNTS: i32 = 12;
const FAULT_SKEW_COUNTS: i32 = 80;
const HOMING_FAULT_SKEW_COUNTS: i32 = 160;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskMotionState {
    Idle,
    Homing,
    MovingUp,
    MovingDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct DeskStatus {
    pub homed: bool,
    pub needs_rehome: bool,
    pub motion: DeskMotionState,
    pub target_active: bool,
    pub target_position: i32,
    pub left_position: i32,
    pub right_position: i32,
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
    RehomeRequired,
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

#[derive(Clone, Copy)]
enum TravelDirection {
    Up,
    Down,
}

#[derive(Clone, Copy)]
struct AxisTargetState {
    at_target: bool,
    base_duty: u16,
}

#[derive(Clone, Copy)]
struct MoveSnapshot {
    observed_skew: i32,
    observed_skew_abs: i32,
    left: AxisTargetState,
    right: AxisTargetState,
}

#[derive(Clone, Copy)]
enum LegPlan {
    Stop,
    Up(u16),
    Down(u16),
    HomeDown(u16),
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
}

impl MoveSnapshot {
    fn new(target: i32, left_position: i32, right_position: i32) -> Self {
        let left_error = target - left_position;
        let right_error = target - right_position;
        let left_at_target = left_error.abs() <= DESK_TARGET_TOLERANCE;
        let right_at_target = right_error.abs() <= DESK_TARGET_TOLERANCE;
        let left_near = left_error.abs() <= DESK_TARGET_SLOW_ZONE;
        let right_near = right_error.abs() <= DESK_TARGET_SLOW_ZONE;

        Self {
            observed_skew: left_position - right_position,
            observed_skew_abs: (left_position - right_position).abs(),
            left: AxisTargetState {
                at_target: left_at_target,
                base_duty: axis_base_duty(left_near, left_at_target, right_at_target),
            },
            right: AxisTargetState {
                at_target: right_at_target,
                base_duty: axis_base_duty(right_near, right_at_target, left_at_target),
            },
        }
    }

    fn is_complete(self) -> bool {
        self.left.at_target && self.right.at_target && self.observed_skew_abs == 0
    }

    fn is_skew_fault(self) -> bool {
        self.observed_skew_abs > FAULT_SKEW_COUNTS
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
                status.left_min_position = leg_status.min_position;
                status.left_max_position = leg_status.max_position;
            }
            DeskSide::Right => {
                status.right_position = leg_status.position;
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
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    pub fn new(
        storage: &'static DeskStatusStorage,
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
            target_active: false,
            target_position: 0,
            left_position,
            right_position,
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
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });
        (self, error)
    }

    pub async fn home_all(
        mut self,
    ) -> Result<
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    > {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Homing;
            status.homed = false;
            status.needs_rehome = true;
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

        let left_start = left_leg.encoder_position();
        let right_start = right_leg.encoder_position();
        let homing_start_deadline = Instant::now() + HOMING_START_TIMEOUT;

        left_leg.apply_drive_mode(DriveMode::DownBoost);
        right_leg.apply_drive_mode(DriveMode::DownBoost);

        let left_direction = left_leg.down_direction();
        let right_direction = right_leg.down_direction();
        let left_up_direction = left_leg.up_direction();
        let right_up_direction = right_leg.up_direction();
        let mut left_started = false;
        let mut right_started = false;

        loop {
            if !left_started {
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(left_start, current_position, left_direction) {
                    left_started = true;
                } else if progressed_in_direction(left_start, current_position, left_up_direction) {
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    right_leg.apply_drive_mode(DriveMode::Stop);
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
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    right_leg.apply_drive_mode(DriveMode::Stop);
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
                left_leg.apply_drive_mode(DriveMode::Stop);
                right_leg.apply_drive_mode(DriveMode::Stop);
                let error = if !left_started {
                    DeskError::LeftLeg(LegError::HomingStartTimeout)
                } else {
                    DeskError::RightLeg(LegError::HomingStartTimeout)
                };
                return Err(self.restore_unhomed(left_leg, right_leg, error));
            }

            Timer::after(HOMING_POLL_INTERVAL).await;
        }

        left_leg.apply_drive_mode(DriveMode::HomeDown);
        right_leg.apply_drive_mode(DriveMode::HomeDown);

        let mut left_last_position = left_leg.encoder_position();
        let mut right_last_position = right_leg.encoder_position();
        let mut left_last_progress = Instant::now();
        let mut right_last_progress = Instant::now();
        let mut left_stalled = false;
        let mut right_stalled = false;
        let mut homing_sync_phase = SyncPhase::Balanced;

        while !left_stalled || !right_stalled {
            Timer::after(HOMING_POLL_INTERVAL).await;

            if !left_stalled {
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(left_last_position, current_position, left_direction) {
                    left_last_position = current_position;
                    left_last_progress = Instant::now();
                } else if Instant::now().saturating_duration_since(left_last_progress)
                    >= HOMING_STALL_TIMEOUT
                {
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    left_stalled = true;
                }
            }

            if !right_stalled {
                let current_position = right_leg.encoder_position();
                if progressed_in_direction(right_last_position, current_position, right_direction) {
                    right_last_position = current_position;
                    right_last_progress = Instant::now();
                } else if Instant::now().saturating_duration_since(right_last_progress)
                    >= HOMING_STALL_TIMEOUT
                {
                    right_leg.apply_drive_mode(DriveMode::Stop);
                    right_stalled = true;
                }
            }

            let left_travel =
                travel_in_direction(left_start, left_leg.encoder_position(), left_direction);
            let right_travel =
                travel_in_direction(right_start, right_leg.encoder_position(), right_direction);
            let observed_skew = left_travel - right_travel;
            let homing_skew = observed_skew.abs();

            if homing_skew > HOMING_FAULT_SKEW_COUNTS {
                left_leg.apply_drive_mode(DriveMode::Stop);
                right_leg.apply_drive_mode(DriveMode::Stop);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::SkewFault));
            }

            homing_sync_phase = next_sync_phase(homing_sync_phase, homing_skew);
            let lead_left = observed_skew > 0;
            let active_phase = if !left_stalled && !right_stalled {
                homing_sync_phase
            } else {
                SyncPhase::Balanced
            };
            let mut plan = plan_homing_down(active_phase, lead_left);

            if left_stalled {
                plan.left = LegPlan::Stop;
            }
            if right_stalled {
                plan.right = LegPlan::Stop;
            }

            apply_dual_plan(&mut left_leg, &mut right_leg, plan);
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

        left_leg.apply_drive_mode(DriveMode::UpBoost);
        right_leg.apply_drive_mode(DriveMode::UpBoost);

        while !left_backoff_done || !right_backoff_done {
            Timer::after(HOMING_POLL_INTERVAL).await;

            if !left_backoff_done {
                let current_position = left_leg.encoder_position();
                let progress =
                    travel_in_direction(left_backoff_position, current_position, left_up_direction);
                if progress > 0 {
                    left_backoff_progress_at = Instant::now();
                    left_backoff += progress;
                    left_backoff_position = current_position;
                    if left_boosting {
                        left_leg.apply_drive_mode(DriveMode::UpRun);
                        left_boosting = false;
                    }
                }

                if left_backoff >= HOMING_BACKOFF_STEPS {
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    left_backoff_done = true;
                } else if Instant::now().saturating_duration_since(left_backoff_progress_at)
                    >= DESK_MOVE_TIMEOUT
                {
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    right_leg.apply_drive_mode(DriveMode::Stop);
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
                        right_leg.apply_drive_mode(DriveMode::UpRun);
                        right_boosting = false;
                    }
                }

                if right_backoff >= HOMING_BACKOFF_STEPS {
                    right_leg.apply_drive_mode(DriveMode::Stop);
                    right_backoff_done = true;
                } else if Instant::now().saturating_duration_since(right_backoff_progress_at)
                    >= DESK_MOVE_TIMEOUT
                {
                    left_leg.apply_drive_mode(DriveMode::Stop);
                    right_leg.apply_drive_mode(DriveMode::Stop);
                    return Err(self.restore_unhomed(
                        left_leg,
                        right_leg,
                        DeskError::RightLeg(LegError::MoveTimeout),
                    ));
                }
            }

            if (left_backoff - right_backoff).abs() > FAULT_SKEW_COUNTS {
                left_leg.apply_drive_mode(DriveMode::Stop);
                right_leg.apply_drive_mode(DriveMode::Stop);
                return Err(self.restore_unhomed(left_leg, right_leg, DeskError::SkewFault));
            }
        }

        left_leg.apply_drive_mode(DriveMode::Stop);
        right_leg.apply_drive_mode(DriveMode::Stop);
        left_leg.reset_position();
        right_leg.reset_position();
        let left_leg = left_leg.into_ready();
        let right_leg = right_leg.into_ready();
        let left_status = left_leg.status();
        let right_status = right_leg.status();
        left_leg.publish_status();
        right_leg.publish_status();

        self.update_status(|status| {
            status.homed = true;
            status.needs_rehome = false;
            status.motion = DeskMotionState::Idle;
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
            status_state: self.status_state,
            _state: ReadyDesk,
        })
    }
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    pub async fn home_all(
        self,
    ) -> Result<
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    > {
        self.into_unhomed().home_all().await
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
            status.target_active = false;
            status.left_min_position = 0;
            status.left_max_position = 0;
            status.right_min_position = 0;
            status.right_max_position = 0;
        });

        Desk {
            left: Some(left),
            right: Some(right),
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

    pub fn stop(&mut self) {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Idle;
            status.target_active = false;
        });
    }

    pub async fn move_to(
        mut self,
        target_position: i32,
    ) -> Result<
        Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        (
            Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
            DeskError,
        ),
    > {
        let status = self.status();
        if !status.homed || status.needs_rehome {
            return Err(self.fail_move(DeskError::RehomeRequired));
        }

        let shared_target = target_position.clamp(status.min_position, status.max_position);
        let Some(direction) = TravelDirection::from_target(shared_target, status.average_position)
        else {
            self.stop();
            return Ok(self);
        };

        self.update_status(|status| {
            status.target_active = true;
            status.target_position = shared_target;
            status.motion = direction.motion_state();
        });

        {
            let (left_leg, right_leg) = self.ready_legs_mut();
            match direction {
                TravelDirection::Up => {
                    left_leg.start_up_boost();
                    right_leg.start_up_boost();
                }
                TravelDirection::Down => {
                    left_leg.start_down_boost();
                    right_leg.start_down_boost();
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

        loop {
            match with_deadline(
                Instant::now() + DESK_MOVE_TIMEOUT,
                select(
                    left_progress_watcher.wait_for_change(),
                    right_progress_watcher.wait_for_change(),
                ),
            )
            .await
            {
                Ok(Either::First(progress)) => left_status.position = progress.position,
                Ok(Either::Second(progress)) => right_status.position = progress.position,
                Err(_) => return Err(self.fail_move(DeskError::MoveTimeout)),
            };

            let snapshot =
                MoveSnapshot::new(shared_target, left_status.position, right_status.position);

            if snapshot.is_complete() {
                self.stop();
                return Ok(self);
            }

            if snapshot.is_skew_fault() {
                return Err(self.fail_move(DeskError::SkewFault));
            }

            sync_phase = next_sync_phase(sync_phase, snapshot.observed_skew_abs);
            let (left_leg, right_leg) = self.ready_legs_mut();
            let plan = plan_move(
                direction,
                sync_phase,
                direction.lead_left(snapshot.observed_skew),
                snapshot,
            );
            apply_dual_plan(left_leg, right_leg, plan);
        }
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

fn next_sync_phase(current: SyncPhase, effective_skew_abs: i32) -> SyncPhase {
    match current {
        SyncPhase::Balanced => {
            if effective_skew_abs >= CATCH_UP_ENTER_COUNTS {
                SyncPhase::PauseLead
            } else if effective_skew_abs >= SYNC_SPEEDUP_ENTER_COUNTS {
                SyncPhase::SpeedMatch
            } else {
                SyncPhase::Balanced
            }
        }
        SyncPhase::SpeedMatch => {
            if effective_skew_abs >= CATCH_UP_ENTER_COUNTS {
                SyncPhase::PauseLead
            } else if effective_skew_abs <= SYNC_SPEEDUP_EXIT_COUNTS {
                SyncPhase::Balanced
            } else {
                SyncPhase::SpeedMatch
            }
        }
        SyncPhase::PauseLead => {
            if effective_skew_abs <= CATCH_UP_EXIT_COUNTS {
                if effective_skew_abs >= SYNC_SPEEDUP_ENTER_COUNTS {
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

fn axis_base_duty(near: bool, at_target: bool, other_at_target: bool) -> u16 {
    if other_at_target && !at_target {
        MOVE_RUN_DUTY
    } else if near {
        MOVE_SLOW_DUTY
    } else {
        MOVE_RUN_DUTY
    }
}

fn clamp_duty(base: u16, trim: i16) -> u16 {
    (base as i16 + trim).clamp(0, 99) as u16
}

fn clamp_drive_duty(base: u16, trim: i16) -> u16 {
    clamp_duty(base, trim).max(MIN_MOVE_DUTY)
}

fn sync_trim(phase: SyncPhase, is_leader: bool, step: i16) -> i16 {
    match phase {
        SyncPhase::Balanced => 0,
        SyncPhase::SpeedMatch if is_leader => -step,
        SyncPhase::SpeedMatch => step,
        SyncPhase::PauseLead if is_leader => 0,
        SyncPhase::PauseLead => step,
    }
}

fn plan_move_leg(
    direction: TravelDirection,
    axis: AxisTargetState,
    phase: SyncPhase,
    is_leader: bool,
) -> LegPlan {
    if axis.at_target || (matches!(phase, SyncPhase::PauseLead) && is_leader) {
        return LegPlan::Stop;
    }

    let duty = clamp_drive_duty(
        axis.base_duty,
        sync_trim(phase, is_leader, MOVE_SYNC_DUTY_STEP),
    );
    match direction {
        TravelDirection::Up => LegPlan::Up(duty),
        TravelDirection::Down => LegPlan::Down(duty),
    }
}

fn plan_move(
    direction: TravelDirection,
    phase: SyncPhase,
    lead_left: bool,
    snapshot: MoveSnapshot,
) -> DualLegPlan {
    DualLegPlan {
        left: plan_move_leg(direction, snapshot.left, phase, lead_left),
        right: plan_move_leg(direction, snapshot.right, phase, !lead_left),
    }
}

fn plan_homing_down(phase: SyncPhase, lead_left: bool) -> DualLegPlan {
    let left_plan = if matches!(phase, SyncPhase::PauseLead) && lead_left {
        LegPlan::Stop
    } else {
        LegPlan::HomeDown(clamp_drive_duty(
            HOMING_RUN_DUTY,
            sync_trim(phase, lead_left, HOMING_SYNC_DUTY_STEP),
        ))
    };
    let right_plan = if matches!(phase, SyncPhase::PauseLead) && !lead_left {
        LegPlan::Stop
    } else {
        LegPlan::HomeDown(clamp_drive_duty(
            HOMING_RUN_DUTY,
            sync_trim(phase, !lead_left, HOMING_SYNC_DUTY_STEP),
        ))
    };

    DualLegPlan {
        left: left_plan,
        right: right_plan,
    }
}

fn apply_leg_plan<State, const OP: u8, PWM: PwmPeripheral>(
    leg: &mut Leg<'_, State, OP, PWM>,
    plan: LegPlan,
) {
    match plan {
        LegPlan::Stop => leg.apply_drive_mode(DriveMode::Stop),
        LegPlan::Up(duty) => leg.drive_up_duty(duty),
        LegPlan::Down(duty) => leg.drive_down_duty(duty),
        LegPlan::HomeDown(duty) => leg.drive_home_down_duty(duty),
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
) {
    apply_leg_plan(left_leg, plan.left);
    apply_leg_plan(right_leg, plan.right);
}
