use defmt::Format;
use embassy_futures::select::{Either, select};
use embassy_time::{Instant, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::{DeskConfig, LegRuntimeConfig},
    leg::{LegProgressWatcher, LegStatus},
    units::{PositionCounts, average_position},
};

use super::{
    monitor::{
        MoveProgressMonitor, MoveSnapshot, MoveStart, MoveStepValidation, ObstructionMonitor,
        validate_move_start, validate_move_step,
    },
    planning::{SyncPhase, TravelDirection, apply_dual_plan, next_sync_phase, plan_move},
    state::{Desk, DeskError, ReadyDesk, UnhomedDesk},
    status::DeskStopReason,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Non-fault outcomes from a ready-state desk move.
pub enum DeskMoveOutcome {
    Completed,
    StoppedByRequest,
    StoppedByObstruction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Internal movement invariant violations that require fault handling.
pub enum DeskMoveInvariant {
    DirectionMismatch,
    WrongWayProgress,
}

/// Fault result from ready-state movement.
///
/// Some faults preserve the ready typestate, while faults that invalidate the
/// shared coordinate frame return an unhomed desk so the caller must rehome.
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

struct ReadyMoveLoop {
    desk_config: DeskConfig,
    leg_config: LegRuntimeConfig,
    shared_target: i32,
    direction: TravelDirection,
    left_progress_watcher: LegProgressWatcher,
    right_progress_watcher: LegProgressWatcher,
    left_status: LegStatus,
    right_status: LegStatus,
    sync_phase: SyncPhase,
    obstruction_monitor: Option<ObstructionMonitor>,
    progress_monitor: MoveProgressMonitor,
}

impl ReadyMoveLoop {
    fn new(
        desk_config: DeskConfig,
        leg_config: LegRuntimeConfig,
        shared_target: i32,
        direction: TravelDirection,
        left_progress_watcher: LegProgressWatcher,
        right_progress_watcher: LegProgressWatcher,
        left_status: LegStatus,
        right_status: LegStatus,
    ) -> Self {
        let move_started_at = Instant::now();
        let obstruction_monitor = ObstructionMonitor::new(
            desk_config,
            direction,
            move_started_at,
            left_status.position,
            right_status.position,
        );
        let progress_monitor = MoveProgressMonitor::new(
            desk_config,
            direction,
            average_position(left_status.position, right_status.position),
        );

        Self {
            desk_config,
            leg_config,
            shared_target,
            direction,
            left_progress_watcher,
            right_progress_watcher,
            left_status,
            right_status,
            sync_phase: SyncPhase::Balanced,
            obstruction_monitor,
            progress_monitor,
        }
    }

    async fn wait_for_progress(&mut self) -> Result<(), DeskError> {
        match with_deadline(
            Instant::now() + self.desk_config.move_timeout(),
            select(
                self.left_progress_watcher.wait_for_change(),
                self.right_progress_watcher.wait_for_change(),
            ),
        )
        .await
        {
            Ok(Either::First(progress)) => {
                self.left_status.position = progress.position;
                Ok(())
            }
            Ok(Either::Second(progress)) => {
                self.right_status.position = progress.position;
                Ok(())
            }
            Err(_) => Err(DeskError::MoveTimeout),
        }
    }

    fn validate_step(&mut self) -> Result<MoveStepValidation, DeskError> {
        validate_move_step(
            self.desk_config,
            self.direction,
            self.shared_target,
            self.left_status.position,
            self.right_status.position,
            &mut self.progress_monitor,
        )
    }

    fn next_plan(&mut self, snapshot: MoveSnapshot) -> super::planning::DualLegPlan {
        self.sync_phase = next_sync_phase(
            self.desk_config,
            self.sync_phase,
            snapshot.observed_skew_abs,
        );
        plan_move(
            self.desk_config,
            self.direction,
            self.sync_phase,
            self.direction.lead_left(snapshot.observed_skew),
            snapshot,
        )
    }

    fn obstruction_detected(&mut self, snapshot: MoveSnapshot) -> bool {
        self.obstruction_monitor.as_mut().is_some_and(|monitor| {
            monitor.observe(
                Instant::now(),
                self.left_status.position,
                self.right_status.position,
                snapshot,
                self.sync_phase,
            )
        })
    }
}

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, ReadyDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    /// Move a homed desk to an absolute target within the configured limits.
    ///
    /// The move may complete, stop by user request, stop by obstruction, or
    /// fault. Faults that make synchronization untrustworthy return an unhomed
    /// desk through `DeskMoveError::FaultedUnhomed`.
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

        let mut move_loop = {
            let (left_leg, right_leg) = self.ready_legs_mut();
            ReadyMoveLoop::new(
                desk_config,
                leg_config,
                shared_target,
                direction,
                left_leg.progress_watcher(),
                right_leg.progress_watcher(),
                left_leg.status(),
                right_leg.status(),
            )
        };

        loop {
            if stop_requested() {
                self.stop_with_reason(DeskStopReason::UserStop);
                return Ok((self, DeskMoveOutcome::StoppedByRequest));
            }

            if let Err(error) = move_loop.wait_for_progress().await {
                match error {
                    DeskError::MoveTimeout => {
                        let (desk, error) = self.fail_move(DeskError::MoveTimeout);
                        return Err(DeskMoveError::FaultedUnhomed(desk, error));
                    }
                    error => {
                        let (desk, error) = self.fail_move(error);
                        return Err(DeskMoveError::FaultedUnhomed(desk, error));
                    }
                }
            }

            let snapshot = match move_loop.validate_step() {
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

            let plan = move_loop.next_plan(snapshot);
            let (left_leg, right_leg) = self.ready_legs_mut();
            apply_dual_plan(left_leg, right_leg, plan, move_loop.leg_config);

            if move_loop.obstruction_detected(snapshot) {
                self.stop_with_reason(DeskStopReason::Obstruction);
                return Ok((self, DeskMoveOutcome::StoppedByObstruction));
            }
        }
    }
}
