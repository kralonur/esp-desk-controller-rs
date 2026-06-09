use core::cell::Cell;

use embassy_sync::{
    blocking_mutex::{Mutex, raw::CriticalSectionRawMutex},
    channel::Channel,
};
use embassy_time::{Duration, Instant};

use crate::config::{HomingObstructionSensitivity, ObstructionSensitivity, RuntimeConfigReader};
use crate::controller::{
    command::{CommandSubmission, DeskCommand, OverrideCommand, StopSubmission},
    status::{DeskControllerMode, DeskControllerSnapshot, DeskFault},
};
use crate::desk::{DeskError, DeskLegSide, DeskStatus, DeskStatusReader, OverrideLegDirection};
use crate::units::{CountDelta, PositionCounts, RelativeCounts};

/// Shared command queue and state machine for desk control.
///
/// MQTT submits commands here, while the app control loop consumes them and
/// reports lifecycle transitions back through this state.
pub struct DeskControllerState {
    commands: Channel<CriticalSectionRawMutex, DeskCommand, 1>,
    snapshot: Mutex<CriticalSectionRawMutex, Cell<DeskControllerSnapshot>>,
    override_unlocked_until: Mutex<CriticalSectionRawMutex, Cell<Option<Instant>>>,
    status_reader: DeskStatusReader,
    runtime_config_reader: RuntimeConfigReader,
}

impl DeskControllerState {
    /// Create controller state from desk status and runtime config readers.
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
            override_unlocked_until: Mutex::new(Cell::new(None)),
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

    pub fn homing_obstruction_sensitivity(&self) -> HomingObstructionSensitivity {
        self.runtime_config_reader
            .current()
            .desk()
            .homing_obstruction_sensitivity()
    }

    pub fn override_unlocked(&self) -> bool {
        let now = Instant::now();
        self.override_unlocked_until.lock(|until| {
            let unlocked = until.get().is_some_and(|expires_at| now < expires_at);
            if !unlocked {
                until.set(None);
            }
            unlocked
        })
    }

    /// Unlock manual override commands for the configured timeout.
    pub fn unlock_override(&self) -> Duration {
        let timeout = self
            .runtime_config_reader
            .current()
            .desk()
            .override_unlock_timeout();
        self.override_unlocked_until
            .lock(|until| until.set(Some(Instant::now() + timeout)));
        timeout
    }

    pub fn lock_override(&self) {
        self.override_unlocked_until.lock(|until| until.set(None));
    }

    /// Queue a full desk home command if the controller is idle enough.
    pub fn submit_home(&self) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        if self.override_unlocked() {
            return CommandSubmission::RejectedOverrideUnlocked;
        }

        if self.try_queue(DeskCommand::Home) {
            CommandSubmission::Accepted
        } else {
            CommandSubmission::RejectedBusy
        }
    }

    /// Queue a no-motion home command that trusts the current physical position as zero.
    pub fn submit_force_home(&self) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        if self.override_unlocked() {
            return CommandSubmission::RejectedOverrideUnlocked;
        }

        if self.try_queue(DeskCommand::ForceHome) {
            CommandSubmission::Accepted
        } else {
            CommandSubmission::RejectedBusy
        }
    }

    /// Queue an absolute target move for a ready desk.
    pub fn submit_move_to(&self, position: PositionCounts) -> CommandSubmission {
        self.submit_motion(DeskCommand::MoveTo(position))
    }

    /// Queue a relative target move for a ready desk.
    pub fn submit_move_by(&self, delta: RelativeCounts) -> CommandSubmission {
        self.submit_motion(DeskCommand::MoveBy(delta))
    }

    /// Queue a manual one-leg home override while override mode is unlocked.
    pub fn submit_override_home(&self, side: DeskLegSide) -> CommandSubmission {
        self.submit_override(OverrideCommand::Home { side })
    }

    /// Queue a manual one-leg step override while override mode is unlocked.
    pub fn submit_override_move(
        &self,
        side: DeskLegSide,
        direction: OverrideLegDirection,
        steps: CountDelta,
    ) -> CommandSubmission {
        self.submit_override(OverrideCommand::Move {
            side,
            direction,
            steps,
        })
    }

    /// Request the active or pending command to stop.
    pub fn submit_stop(&self) -> StopSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
        {
            self.update_snapshot(|snapshot| snapshot.stop_requested = true);
            StopSubmission::Accepted
        } else {
            StopSubmission::IgnoredIdle
        }
    }

    /// Wait for the next queued command and clear the pending flag.
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

    pub fn begin_override(&self) {
        self.update_snapshot(|snapshot| {
            snapshot.mode = DeskControllerMode::Override;
            snapshot.command_pending = false;
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
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
    }

    pub fn can_update_hardware_config(&self) -> bool {
        let snapshot = self.snapshot();
        !snapshot.command_pending
            && !snapshot.stop_requested
            && matches!(
                snapshot.mode,
                DeskControllerMode::Unhomed | DeskControllerMode::Faulted
            )
    }

    fn submit_motion(&self, command: DeskCommand) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        if self.override_unlocked() {
            return CommandSubmission::RejectedOverrideUnlocked;
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
            DeskControllerMode::Homing
            | DeskControllerMode::Moving
            | DeskControllerMode::Override => CommandSubmission::RejectedBusy,
        }
    }

    fn submit_override(&self, command: OverrideCommand) -> CommandSubmission {
        let snapshot = self.snapshot();
        if snapshot.command_pending
            || matches!(
                snapshot.mode,
                DeskControllerMode::Homing
                    | DeskControllerMode::Moving
                    | DeskControllerMode::Override
            )
        {
            return CommandSubmission::RejectedBusy;
        }

        if !self.override_unlocked() {
            return CommandSubmission::RejectedLocked;
        }

        if self.try_queue(DeskCommand::Override(command)) {
            CommandSubmission::Accepted
        } else {
            CommandSubmission::RejectedBusy
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
