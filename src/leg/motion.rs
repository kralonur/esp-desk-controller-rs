use defmt::Format;
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::LegRuntimeConfig,
    quadrature::{QuadratureDirection, QuadratureEvent},
};

use super::{
    drive::{DriveMode, progressed_in_direction, travel_in_direction},
    state::{Leg, Ready, Unhomed},
    status::{LegStatus, MotionState},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum LegError {
    HomingStartTimeout,
    PolarityMismatch,
    MoveTimeout,
}

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    async fn move_up_steps(
        &mut self,
        expected_direction: QuadratureDirection,
        steps: u16,
        runtime_config: LegRuntimeConfig,
    ) {
        if steps == 0 {
            return;
        }

        if expected_direction == QuadratureDirection::Invalid {
            return;
        }

        self.apply_drive_mode(DriveMode::UpBoost, runtime_config);

        let startup_steps = steps.min(runtime_config.startup_events().get());
        let start_position = self.encoder_position();

        while travel_in_direction(start_position, self.encoder_position(), expected_direction)
            < startup_steps as i32
        {
            let event = self
                .quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            if travel_in_direction(start_position, event.snapshot.position, expected_direction)
                >= startup_steps as i32
            {
                break;
            }
        }

        self.apply_drive_mode(DriveMode::UpRun, runtime_config);

        while travel_in_direction(start_position, self.encoder_position(), expected_direction)
            < steps as i32
        {
            let event = self
                .quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            if travel_in_direction(start_position, event.snapshot.position, expected_direction)
                >= steps as i32
            {
                break;
            }
        }

        self.apply_drive_mode(DriveMode::Stop, runtime_config);
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Unhomed, OP, PWM> {
    pub async fn home_down(mut self) -> Result<Leg<'a, Ready, OP, PWM>, (Self, LegError)> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let start_position = self.encoder_position();
        let start_deadline = Instant::now() + runtime_config.homing_start_timeout();
        let down_direction = self.configured_down_direction();
        let up_direction = self.configured_up_direction();

        self.apply_drive_mode(DriveMode::DownBoost, runtime_config);

        loop {
            let current_position = self.encoder_position();

            if progressed_in_direction(start_position, current_position, down_direction) {
                break;
            }

            if progressed_in_direction(start_position, current_position, up_direction) {
                self.coast();
                return Err((self, LegError::PolarityMismatch));
            }

            if Instant::now() >= start_deadline {
                self.coast();
                return Err((self, LegError::HomingStartTimeout));
            }

            Timer::after(runtime_config.homing_poll_interval()).await;
        }

        let mut last_position = self.encoder_position();
        let mut last_progress_at = Instant::now();
        self.apply_drive_mode(DriveMode::HomeDown, runtime_config);

        loop {
            Timer::after(runtime_config.homing_poll_interval()).await;

            let current_position = self.encoder_position();
            let progressed =
                progressed_in_direction(last_position, current_position, down_direction);

            if progressed {
                last_position = current_position;
                last_progress_at = Instant::now();
                continue;
            }

            if Instant::now().saturating_duration_since(last_progress_at)
                >= runtime_config.homing_stall_timeout()
            {
                break;
            }
        }

        self.apply_drive_mode(DriveMode::Stop, runtime_config);
        self.move_up_steps(
            up_direction,
            runtime_config.homing_backoff_steps().get(),
            runtime_config,
        )
        .await;
        self.quadrature_watcher.reset_position();
        let leg = self.into_ready_with_config(runtime_config);
        leg.send_status(leg.status());

        Ok(leg)
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Ready, OP, PWM> {
    pub fn start_up_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingUp;
        self.apply_drive_mode(DriveMode::UpBoost, runtime_config);
    }

    pub fn start_up_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingUp;
        self.apply_drive_mode(DriveMode::UpSlow, runtime_config);
    }

    pub fn start_down_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingDown;
        self.apply_drive_mode(DriveMode::DownBoost, runtime_config);
    }

    pub fn start_down_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingDown;
        self.apply_drive_mode(DriveMode::DownSlow, runtime_config);
    }

    pub fn stop_for_desk(&mut self) {
        self.stop();
    }

    pub fn move_up(&mut self) {
        let runtime_config = self.runtime_config_reader.current().leg();
        if self.logical_position() >= self.state.max_position {
            self.stop();
            return;
        }

        let duty = self.drive_up_run(runtime_config);
        self.state.motion = MotionState::MovingUp;
        self.send_status(LegStatus {
            duty: duty.get(),
            ..self.status()
        });
    }

    pub fn move_down(&mut self) {
        let runtime_config = self.runtime_config_reader.current().leg();
        if self.logical_position() <= self.state.min_position {
            self.stop();
            return;
        }

        let duty = self.drive_down_run(runtime_config);
        self.state.motion = MotionState::MovingDown;
        self.send_status(LegStatus {
            duty: duty.get(),
            ..self.status()
        });
    }

    pub fn stop(&mut self) {
        self.coast();
        self.state.motion = MotionState::Idle;
        self.send_status(LegStatus {
            duty: 0,
            ..self.status()
        });
    }

    async fn wait_for_progress(
        &mut self,
        direction: QuadratureDirection,
        timeout: Duration,
    ) -> Result<QuadratureEvent, LegError> {
        let deadline = Instant::now() + timeout;

        loop {
            match with_deadline(deadline, self.quadrature_watcher.wait_for_change()).await {
                Ok(event) if event.direction == direction => return Ok(event),
                Ok(_) => {}
                Err(_) => {
                    self.stop();
                    return Err(LegError::MoveTimeout);
                }
            }
        }
    }

    async fn move_steps(
        &mut self,
        direction: QuadratureDirection,
        steps: u16,
    ) -> Result<(), LegError> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let allowed_steps = if direction == self.state.up_direction {
            (self.state.max_position - self.logical_position()).max(0) as u16
        } else if direction == self.state.down_direction {
            (self.logical_position() - self.state.min_position).max(0) as u16
        } else {
            0
        };
        let steps = steps.min(allowed_steps);

        if steps == 0 {
            self.stop();
            return Ok(());
        }

        let startup_steps = steps.min(runtime_config.startup_events().get());
        let start_position = self.encoder_position();

        if direction == self.state.up_direction {
            self.start_up_boost(runtime_config);
        } else if direction == self.state.down_direction {
            self.start_down_boost(runtime_config);
        } else {
            self.stop();
            return Ok(());
        }

        while travel_in_direction(start_position, self.encoder_position(), direction)
            < startup_steps as i32
        {
            let event = self
                .wait_for_progress(direction, runtime_config.move_stall_timeout())
                .await?;
            if travel_in_direction(start_position, event.snapshot.position, direction)
                >= startup_steps as i32
            {
                break;
            }
        }

        if direction == self.state.up_direction {
            let duty = self.drive_up_run(runtime_config);
            self.state.motion = MotionState::MovingUp;
            self.send_status(LegStatus {
                duty: duty.get(),
                ..self.status()
            });
        } else if direction == self.state.down_direction {
            let duty = self.drive_down_run(runtime_config);
            self.state.motion = MotionState::MovingDown;
            self.send_status(LegStatus {
                duty: duty.get(),
                ..self.status()
            });
        }

        while travel_in_direction(start_position, self.encoder_position(), direction) < steps as i32
        {
            let event = self
                .wait_for_progress(direction, runtime_config.move_stall_timeout())
                .await?;
            if travel_in_direction(start_position, event.snapshot.position, direction)
                >= steps as i32
            {
                break;
            }
        }

        self.stop();
        Ok(())
    }

    pub async fn move_up_step(&mut self, steps: u16) -> Result<(), LegError> {
        self.move_steps(self.state.up_direction, steps).await
    }

    pub async fn move_down_step(&mut self, steps: u16) -> Result<(), LegError> {
        self.move_steps(self.state.down_direction, steps).await
    }

    pub async fn move_to(&mut self, target_position: i32) -> Result<(), LegError> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let target_position = self.clamp_target(target_position);
        let current_position = self.logical_position();

        if (target_position - current_position).abs() <= runtime_config.target_tolerance().get_i32()
        {
            self.stop();
            return Ok(());
        }

        if target_position > current_position {
            self.start_up_boost(runtime_config);

            loop {
                let event = self
                    .wait_for_progress(self.state.up_direction, runtime_config.move_stall_timeout())
                    .await?;
                let error =
                    target_position - self.logical_position_from_raw(event.snapshot.position);

                if error <= runtime_config.target_tolerance().get_i32() {
                    self.stop();
                    break;
                }

                if error <= runtime_config.target_slow_zone().get_i32() {
                    self.start_up_slow(runtime_config);
                } else {
                    let duty = self.drive_up_run(runtime_config);
                    self.state.motion = MotionState::MovingUp;
                    self.send_status(LegStatus {
                        duty: duty.get(),
                        ..self.status()
                    });
                }
            }
        } else if target_position < current_position {
            self.start_down_boost(runtime_config);

            loop {
                let event = self
                    .wait_for_progress(
                        self.state.down_direction,
                        runtime_config.move_stall_timeout(),
                    )
                    .await?;
                let error =
                    self.logical_position_from_raw(event.snapshot.position) - target_position;

                if error <= runtime_config.target_tolerance().get_i32() {
                    self.stop();
                    break;
                }

                if error <= runtime_config.target_slow_zone().get_i32() {
                    self.start_down_slow(runtime_config);
                } else {
                    let duty = self.drive_down_run(runtime_config);
                    self.state.motion = MotionState::MovingDown;
                    self.send_status(LegStatus {
                        duty: duty.get(),
                        ..self.status()
                    });
                }
            }
        } else {
            self.stop();
        }

        Ok(())
    }
}
