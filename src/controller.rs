use core::cell::Cell;

use defmt::Format;
use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};

use crate::config::{ObstructionSensitivity, RuntimeConfigReader};
use crate::desk::{DeskError, DeskMoveInvariant, DeskStatus, DeskStatusReader};
use crate::units::{PositionCounts, RelativeCounts};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskCommand {
    Home,
    MoveTo(PositionCounts),
    MoveBy(RelativeCounts),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskControllerMode {
    Unhomed,
    Ready,
    Homing,
    Moving,
    Faulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskFault {
    LeftLeg(crate::leg::LegError),
    RightLeg(crate::leg::LegError),
    MoveTimeout,
    SkewFault,
    InvariantViolation(DeskMoveInvariant),
    RehomeRequired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeskControllerSnapshot {
    pub mode: DeskControllerMode,
    pub command_pending: bool,
    pub stop_requested: bool,
    pub last_fault: Option<DeskFault>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandSubmission {
    Accepted,
    RejectedBusy,
    RejectedUnhomed,
    RejectedFaulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopSubmission {
    Accepted,
    IgnoredIdle,
}

pub struct DeskControllerState {
    commands: Channel<CriticalSectionRawMutex, DeskCommand, 1>,
    snapshot: Mutex<CriticalSectionRawMutex, Cell<DeskControllerSnapshot>>,
    status_reader: DeskStatusReader,
    runtime_config_reader: RuntimeConfigReader,
}

impl DeskControllerState {
    pub fn new(
        status_reader: DeskStatusReader,
        runtime_config_reader: RuntimeConfigReader,
    ) -> Self {
        Self {
            commands: Channel::new(),
            snapshot: Mutex::new(Cell::new(DeskControllerSnapshot {
                mode: DeskControllerMode::Unhomed,
                command_pending: false,
                stop_requested: false,
                last_fault: None,
            })),
            status_reader,
            runtime_config_reader,
        }
    }

    pub fn snapshot(&self) -> DeskControllerSnapshot {
        self.snapshot.lock(|snapshot| snapshot.get())
    }

    pub fn status(&self) -> DeskStatus {
        self.status_reader.current()
    }

    pub fn obstruction_sensitivity(&self) -> ObstructionSensitivity {
        self.runtime_config_reader
            .current()
            .desk()
            .obstruction_sensitivity()
    }

    pub fn submit_home(&self) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing | DeskControllerMode::Moving
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        if self.try_queue(DeskCommand::Home) {
            CommandSubmission::Accepted
        } else {
            CommandSubmission::RejectedBusy
        }
    }

    pub fn submit_move_to(&self, position: PositionCounts) -> CommandSubmission {
        self.submit_motion(DeskCommand::MoveTo(position))
    }

    pub fn submit_move_by(&self, delta: RelativeCounts) -> CommandSubmission {
        self.submit_motion(DeskCommand::MoveBy(delta))
    }

    pub fn submit_stop(&self) -> StopSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing | DeskControllerMode::Moving
            )
        {
            self.update_snapshot(|snapshot| snapshot.stop_requested = true);
            StopSubmission::Accepted
        } else {
            StopSubmission::IgnoredIdle
        }
    }

    pub async fn next_command(&self) -> DeskCommand {
        let command = self.commands.receive().await;
        self.update_snapshot(|snapshot| snapshot.command_pending = false);
        command
    }

    pub fn begin_homing(&self) {
        self.update_snapshot(|snapshot| {
            snapshot.mode = DeskControllerMode::Homing;
            snapshot.command_pending = false;
            snapshot.stop_requested = false;
        });
    }

    pub fn begin_move(&self) {
        self.update_snapshot(|snapshot| {
            snapshot.mode = DeskControllerMode::Moving;
            snapshot.command_pending = false;
            snapshot.stop_requested = false;
        });
    }

    pub fn finish_ready(&self) {
        self.update_snapshot(|snapshot| {
            snapshot.mode = DeskControllerMode::Ready;
            snapshot.command_pending = false;
            snapshot.stop_requested = false;
            snapshot.last_fault = None;
        });
    }

    pub fn finish_unhomed(&self) {
        self.update_snapshot(|snapshot| {
            snapshot.mode = DeskControllerMode::Unhomed;
            snapshot.command_pending = false;
            snapshot.stop_requested = false;
            snapshot.last_fault = None;
        });
    }

    pub fn fault(&self, error: DeskError) {
        if let Some(fault) = DeskFault::from_desk_error(error) {
            self.update_snapshot(|snapshot| {
                snapshot.mode = DeskControllerMode::Faulted;
                snapshot.command_pending = false;
                snapshot.stop_requested = false;
                snapshot.last_fault = Some(fault);
            });
        }
    }

    pub fn stop_requested(&self) -> bool {
        self.snapshot().stop_requested
    }

    pub fn clear_stop_request(&self) {
        self.update_snapshot(|snapshot| snapshot.stop_requested = false);
    }

    pub fn can_persist_config(&self) -> bool {
        let snapshot = self.snapshot();
        !snapshot.command_pending
            && !matches!(
                snapshot.mode,
                DeskControllerMode::Homing | DeskControllerMode::Moving
            )
    }

    fn submit_motion(&self, command: DeskCommand) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing | DeskControllerMode::Moving
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        match snapshot.mode {
            DeskControllerMode::Ready => {
                if self.try_queue(command) {
                    CommandSubmission::Accepted
                } else {
                    CommandSubmission::RejectedBusy
                }
            }
            DeskControllerMode::Faulted => CommandSubmission::RejectedFaulted,
            DeskControllerMode::Unhomed => CommandSubmission::RejectedUnhomed,
            DeskControllerMode::Homing | DeskControllerMode::Moving => {
                CommandSubmission::RejectedBusy
            }
        }
    }

    fn try_queue(&self, command: DeskCommand) -> bool {
        if self.commands.try_send(command).is_ok() {
            self.update_snapshot(|snapshot| snapshot.command_pending = true);
            true
        } else {
            false
        }
    }

    fn update_snapshot(
        &self,
        update_fn: impl FnOnce(&mut DeskControllerSnapshot),
    ) -> DeskControllerSnapshot {
        self.snapshot.lock(|snapshot| {
            let mut current = snapshot.get();
            update_fn(&mut current);
            snapshot.set(current);
            current
        })
    }
}

impl DeskFault {
    fn from_desk_error(error: DeskError) -> Option<Self> {
        match error {
            DeskError::LeftLeg(error) => Some(Self::LeftLeg(error)),
            DeskError::RightLeg(error) => Some(Self::RightLeg(error)),
            DeskError::MoveTimeout => Some(Self::MoveTimeout),
            DeskError::SkewFault => Some(Self::SkewFault),
            DeskError::InvariantViolation(invariant) => Some(Self::InvariantViolation(invariant)),
            DeskError::RehomeRequired => Some(Self::RehomeRequired),
            DeskError::Stopped => None,
        }
    }
}
