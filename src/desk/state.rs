use defmt::Format;
use embassy_executor::Spawner;
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::RuntimeConfigReader,
    leg::{Leg, LegError, LegStatusWatcher, Ready, Unhomed},
    units::{abs_position_delta, average_position},
};

use super::{
    movement::DeskMoveInvariant,
    status::{
        DeskMotionState, DeskSide, DeskStatus, DeskStatusReader, DeskStatusState,
        DeskStatusStorage, DeskStatusWatcher, DeskStopReason,
    },
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Desk-level failures returned by homing, movement, and override operations.
///
/// `Stopped` means the caller-requested stop was handled as a normal outcome;
/// controller fault reporting intentionally ignores it.
pub enum DeskError {
    LeftLeg(LegError),
    RightLeg(LegError),
    MoveTimeout,
    SkewFault,
    InvariantViolation(DeskMoveInvariant),
    RehomeRequired,
    Stopped,
}

/// Typestate marker for a desk whose shared coordinate frame is not trusted.
///
/// A desk starts unhomed and returns to this state after override operations or
/// faults that can invalidate synchronized two-leg positioning.
pub struct UnhomedDesk;

/// Typestate marker for a desk that has completed full two-leg homing.
///
/// Only `Desk<ReadyDesk, ...>` can run coordinated target moves.
pub struct ReadyDesk;

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

pub(super) enum ManagedLeg<'a, const OP: u8, PWM: PwmPeripheral> {
    Unhomed(Leg<'a, Unhomed, OP, PWM>),
    Ready(Leg<'a, Ready, OP, PWM>),
}

pub(super) type ReadyLegPair<
    'desk,
    'leg,
    const LEFT_OP: u8,
    LeftPwm,
    const RIGHT_OP: u8,
    RightPwm,
> = (
    &'desk mut Leg<'leg, Ready, LEFT_OP, LeftPwm>,
    &'desk mut Leg<'leg, Ready, RIGHT_OP, RightPwm>,
);

/// Coordinated two-leg desk.
///
/// The `State` typestate controls which operations are available: unhomed desks
/// can home or run manual override, while ready desks can run target movement.
pub struct Desk<
    'a,
    State,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
> {
    pub(super) left: Option<ManagedLeg<'a, LEFT_OP, LeftPwm>>,
    pub(super) right: Option<ManagedLeg<'a, RIGHT_OP, RightPwm>>,
    pub(super) runtime_config_reader: RuntimeConfigReader,
    pub(super) status_state: &'static DeskStatusState,
    pub(super) _state: State,
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

    /// Subscribe to desk status changes.
    ///
    /// Watchers are intended for async tasks such as MQTT publication.
    pub fn status_watcher(&self) -> DeskStatusWatcher {
        let receiver = self
            .status_state
            .watch
            .receiver()
            .expect("desk status watch receiver limit reached");

        DeskStatusWatcher { receiver }
    }

    /// Create a cheap reader for polling the latest desk status.
    pub fn status_reader(&self) -> DeskStatusReader {
        DeskStatusReader {
            status_state: self.status_state,
        }
    }

    pub(super) fn update_status(&self, update_fn: impl FnOnce(&mut DeskStatus)) -> DeskStatus {
        self.status_state.update(update_fn)
    }

    pub(super) fn stop_ready_legs(&mut self) {
        if let Some(ManagedLeg::Ready(left)) = &mut self.left {
            left.stop_for_desk();
        }
        if let Some(ManagedLeg::Ready(right)) = &mut self.right {
            right.stop_for_desk();
        }
    }

    pub(super) fn force_unhomed(
        mut self,
    ) -> Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
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
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    /// Trust the current physical desk position as home without moving.
    ///
    /// This is a manual recovery operation: the caller is responsible for only
    /// using it when both legs are visually at the lower reference position.
    pub fn force_home(mut self) -> Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        let runtime_config = self.runtime_config_reader.current();
        let leg_config = runtime_config.leg();

        let left_leg = self.left.take().expect("left leg missing");
        let right_leg = self.right.take().expect("right leg missing");
        let (left_leg, right_leg) = match (left_leg, right_leg) {
            (ManagedLeg::Unhomed(left_leg), ManagedLeg::Unhomed(right_leg)) => {
                (left_leg, right_leg)
            }
            _ => unreachable!("unhomed desk must contain unhomed legs"),
        };

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
            status.target_position = 0;
            status.left_position = left_status.position;
            status.right_position = right_status.position;
            status.left_min_position = left_status.min_position;
            status.left_max_position = left_status.max_position;
            status.right_min_position = right_status.min_position;
            status.right_max_position = right_status.max_position;
            status.min_position = left_status.min_position.max(right_status.min_position);
            status.max_position = left_status.max_position.min(right_status.max_position);
            status.average_position = average_position(left_status.position, right_status.position);
            status.skew_counts = abs_position_delta(left_status.position, right_status.position);
        });

        Desk {
            left: Some(ManagedLeg::Ready(left_leg)),
            right: Some(ManagedLeg::Ready(right_leg)),
            runtime_config_reader: self.runtime_config_reader,
            status_state: self.status_state,
            _state: ReadyDesk,
        }
    }

    pub(super) fn take_unhomed_legs(
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

    pub(super) fn finish_override_unhomed(
        mut self,
        left_leg: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        right_leg: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
        stop_reason: DeskStopReason,
    ) -> Self {
        let (left_leg, right_leg) = reset_unhomed_leg_positions(left_leg, right_leg);
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

    /// Build a desk from two unhomed legs and start status mirroring tasks.
    ///
    /// The returned desk must be homed before coordinated target movement is
    /// available.
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
            average_position: average_position(left_position, right_position),
            skew_counts: abs_position_delta(left_position, right_position),
            min_position: 0,
            max_position: 0,
            left_min_position: 0,
            left_max_position: 0,
            right_min_position: 0,
            right_max_position: 0,
        };
        let status_state = storage.state.init(DeskStatusState::new(initial_status));
        spawner.spawn(
            mirror_leg_to_desk_status(DeskSide::Left, left_status_watcher, status_state)
                .expect("spawn left leg status mirror task"),
        );
        spawner.spawn(
            mirror_leg_to_desk_status(DeskSide::Right, right_status_watcher, status_state)
                .expect("spawn right leg status mirror task"),
        );

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

    pub(super) fn restore_unhomed(
        mut self,
        left_leg: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
        right_leg: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
        error: DeskError,
    ) -> (Self, DeskError) {
        let (left_leg, right_leg) = reset_unhomed_leg_positions(left_leg, right_leg);
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
}

fn reset_unhomed_leg_positions<
    'a,
    const LEFT_OP: u8,
    LeftPwm: PwmPeripheral,
    const RIGHT_OP: u8,
    RightPwm: PwmPeripheral,
>(
    left_leg: Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
    right_leg: Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
) -> (
    Leg<'a, Unhomed, LEFT_OP, LeftPwm>,
    Leg<'a, Unhomed, RIGHT_OP, RightPwm>,
) {
    left_leg.reset_position();
    right_leg.reset_position();
    left_leg.publish_status();
    right_leg.publish_status();
    (left_leg, right_leg)
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    /// Trust the current physical desk position as home without moving.
    pub fn force_home(self) -> Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        self.into_unhomed().force_home()
    }

    pub(super) fn ready_legs_mut(
        &mut self,
    ) -> ReadyLegPair<'_, 'a, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
        match (&mut self.left, &mut self.right) {
            (Some(ManagedLeg::Ready(left)), Some(ManagedLeg::Ready(right))) => (left, right),
            _ => unreachable!("ready desk must contain ready legs"),
        }
    }

    pub(super) fn into_unhomed(
        mut self,
    ) -> Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm> {
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

    pub(super) fn fail_move(
        self,
        error: DeskError,
    ) -> (
        Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>,
        DeskError,
    ) {
        (self.into_unhomed(), error)
    }

    pub(super) fn fault_ready_move(
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

    /// Stop both ready legs and publish the supplied stop reason.
    pub fn stop_with_reason(&mut self, reason: DeskStopReason) {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Idle;
            status.target_active = false;
            status.last_stop_reason = reason;
        });
    }
}
