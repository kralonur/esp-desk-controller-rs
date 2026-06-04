use embassy_time::{Instant, Timer};
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    leg::{DriveMode, LegError},
    units::{abs_position_delta, position_delta},
};

use super::{
    monitor::HomingContactMonitor,
    planning::{LegPlan, SyncPhase, apply_dual_plan, next_sync_phase, plan_homing_down},
    position::{progressed_in_direction, travel_in_direction},
    state::{Desk, DeskError, ManagedLeg, ReadyDesk, UnhomedDesk},
    status::{DeskMotionState, DeskStopReason},
};

impl<'a, const LEFT_OP: u8, LeftPwm: PwmPeripheral, const RIGHT_OP: u8, RightPwm: PwmPeripheral>
    Desk<'a, UnhomedDesk, LEFT_OP, LeftPwm, RIGHT_OP, RightPwm>
{
    /// Home both legs and establish the ready desk coordinate frame.
    ///
    /// This operation drives both legs downward until end-stop/stall detection,
    /// backs off, resets positions, and returns `Desk<ReadyDesk, ...>`. A stop
    /// request returns the desk still unhomed with `DeskError::Stopped`.
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
        // If the legs start uneven, lower only the leading leg until the pair
        // is close enough to enter synchronized homing.
        if position_delta(initial_left_position, initial_right_position) > alignment_tolerance {
            right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            left_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);
            let mut last_position = left_leg.encoder_position();
            let mut last_progress = Instant::now();

            while position_delta(left_leg.logical_encoder_position(), initial_right_position)
                > alignment_tolerance
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
        } else if position_delta(initial_right_position, initial_left_position)
            > alignment_tolerance
        {
            left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
            right_leg.apply_drive_mode(DriveMode::HomeDown, leg_config);
            let mut last_position = right_leg.encoder_position();
            let mut last_progress = Instant::now();

            while position_delta(right_leg.logical_encoder_position(), initial_left_position)
                > alignment_tolerance
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

        // Confirm both encoders move downward before relying on stall timing;
        // upward progress here means the configured polarity is wrong.
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

        // Drive both legs down until each stops making progress, while keeping
        // skew bounded so one side cannot keep moving far after the other stalls.
        let mut left_last_position = left_leg.encoder_position();
        let mut right_last_position = right_leg.encoder_position();
        let homing_contact_started_at = Instant::now();
        let mut left_last_progress = homing_contact_started_at;
        let mut right_last_progress = homing_contact_started_at;
        let mut left_contact_monitor = HomingContactMonitor::new(
            desk_config,
            homing_contact_started_at,
            left_last_position,
            left_direction,
        );
        let mut right_contact_monitor = HomingContactMonitor::new(
            desk_config,
            homing_contact_started_at,
            right_last_position,
            right_direction,
        );
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
            let now = Instant::now();

            if !left_stalled {
                let current_position = left_leg.encoder_position();
                if progressed_in_direction(left_last_position, current_position, left_direction) {
                    left_last_position = current_position;
                    left_last_progress = now;
                } else if now.saturating_duration_since(left_last_progress)
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
                    right_last_progress = now;
                } else if now.saturating_duration_since(right_last_progress)
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
            let observed_skew = position_delta(left_travel, right_travel);
            let homing_skew = abs_position_delta(left_travel, right_travel);

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
            let contact_detection_suspended = !matches!(active_phase, SyncPhase::Balanced);

            if !left_stalled
                && left_contact_monitor.as_mut().is_some_and(|monitor| {
                    monitor.observe(
                        now,
                        left_leg.encoder_position(),
                        contact_detection_suspended,
                    )
                })
            {
                left_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                left_stalled = true;
            }

            if !right_stalled
                && right_contact_monitor.as_mut().is_some_and(|monitor| {
                    monitor.observe(
                        now,
                        right_leg.encoder_position(),
                        contact_detection_suspended,
                    )
                })
            {
                right_leg.apply_drive_mode(DriveMode::Stop, leg_config);
                right_stalled = true;
            }

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
        let mut left_backoff: i32 = 0;
        let mut right_backoff: i32 = 0;
        let mut left_boosting = true;
        let mut right_boosting = true;

        left_leg.apply_drive_mode(DriveMode::UpBoost, leg_config);
        right_leg.apply_drive_mode(DriveMode::UpBoost, leg_config);

        // Lift both legs off the lower end stop before zeroing the coordinate
        // frame; this avoids treating a loaded hard stop as the ready position.
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
                    left_backoff = left_backoff.saturating_add(progress);
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
                    right_backoff = right_backoff.saturating_add(progress);
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

            if abs_position_delta(left_backoff, right_backoff)
                > desk_config.fault_skew_counts().get_i32()
            {
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
}
