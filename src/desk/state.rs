use defmt::Format;
use embassy_executor::Spawner;
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::RuntimeConfigReader,
    leg::{Leg, LegError, LegStatusWatcher, Ready, Unhomed},
};

use super::{
    movement::DeskMoveInvariant,
    status::{
        DeskMotionState, DeskSide, DeskStatus, DeskStatusReader, DeskStatusState,
        DeskStatusStorage, DeskStatusWatcher, DeskStopReason,
    },
};

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

pub struct UnhomedDesk;
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

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
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

    pub fn stop_with_reason(&mut self, reason: DeskStopReason) {
        self.stop_ready_legs();
        self.update_status(|status| {
            status.motion = DeskMotionState::Idle;
            status.target_active = false;
            status.last_stop_reason = reason;
        });
    }
}
