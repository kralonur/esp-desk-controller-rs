use defmt::Format;
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::LegRuntimeConfig,
    quadrature::QuadratureDirection,
    units::{DutyPercent, PWM_TIMER_MAX_TICKS},
};

use super::{
    state::Leg,
    status::{LegStatus, MotionState},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Physical motor side used for upward travel.
pub enum DriveSide {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Requested low-level drive behavior for a leg motor.
pub enum DriveMode {
    Stop,
    UpBoost,
    UpRun,
    UpSlow,
    DownBoost,
    DownRun,
    DownSlow,
    HomeDown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct DriveOutput {
    side: Option<DriveSide>,
    duty: DutyPercent,
    timestamp: u16,
    motion: MotionState,
}

impl DriveOutput {
    pub(super) const fn stopped() -> Self {
        Self {
            side: None,
            duty: DutyPercent::new(0),
            timestamp: 0,
            motion: MotionState::Idle,
        }
    }
}

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    pub fn apply_drive_mode(&mut self, mode: DriveMode, runtime_config: LegRuntimeConfig) {
        match mode {
            DriveMode::Stop => self.apply_drive_output(DriveOutput::stopped()),
            DriveMode::UpBoost => {
                self.apply_up_drive_output(runtime_config.startup_duty(), runtime_config)
            }
            DriveMode::UpRun => {
                self.apply_up_drive_output(runtime_config.run_duty(), runtime_config)
            }
            DriveMode::UpSlow => {
                self.apply_up_drive_output(runtime_config.slow_duty(), runtime_config)
            }
            DriveMode::DownBoost => {
                self.apply_down_drive_output(runtime_config.startup_duty(), runtime_config)
            }
            DriveMode::DownRun => {
                self.apply_down_drive_output(runtime_config.run_duty(), runtime_config)
            }
            DriveMode::DownSlow => {
                self.apply_down_drive_output(runtime_config.slow_duty(), runtime_config)
            }
            DriveMode::HomeDown => {
                self.apply_down_drive_output(runtime_config.homing_duty(), runtime_config)
            }
        }
    }

    pub fn drive_up_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self
            .drive_output
            .is_none_or(|output| output.motion != MotionState::MovingUp)
        {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.apply_up_drive_output(duty, runtime_config);
    }

    pub fn drive_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self
            .drive_output
            .is_none_or(|output| output.motion != MotionState::MovingDown)
        {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.apply_down_drive_output(duty, runtime_config);
    }

    pub fn drive_home_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self
            .drive_output
            .is_none_or(|output| output.motion != MotionState::MovingDown)
        {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.apply_down_drive_output(duty, runtime_config);
    }

    fn apply_up_drive_output(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        self.apply_directional_drive_output(
            self.config.up_drive,
            duty,
            MotionState::MovingUp,
            runtime_config,
        );
    }

    fn apply_down_drive_output(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        self.apply_directional_drive_output(
            opposite_drive_side(self.config.up_drive),
            duty,
            MotionState::MovingDown,
            runtime_config,
        );
    }

    fn apply_directional_drive_output(
        &mut self,
        side: DriveSide,
        duty: DutyPercent,
        motion: MotionState,
        runtime_config: LegRuntimeConfig,
    ) {
        let duty = duty
            .min(runtime_config.max_duty())
            .min(DutyPercent::new(100));
        let output = DriveOutput {
            side: Some(side),
            duty,
            timestamp: duty.to_pwm_timestamp(PWM_TIMER_MAX_TICKS),
            motion,
        };
        self.apply_drive_output(output);
    }

    fn apply_drive_output(&mut self, output: DriveOutput) {
        if self.drive_output == Some(output) {
            return;
        }

        match output.side {
            Some(DriveSide::Left) => self.motor.drive_left(output.timestamp),
            Some(DriveSide::Right) => self.motor.drive_right(output.timestamp),
            None => self.motor.coast(),
        }

        self.drive_output = Some(output);
        self.send_status(LegStatus {
            duty: output.duty.get(),
            motion: output.motion,
            ..self.current_status()
        });
    }
}

pub(super) fn opposite_direction(direction: QuadratureDirection) -> QuadratureDirection {
    match direction {
        QuadratureDirection::Positive => QuadratureDirection::Negative,
        QuadratureDirection::Negative => QuadratureDirection::Positive,
        QuadratureDirection::Invalid => QuadratureDirection::Invalid,
    }
}

pub(super) fn opposite_drive_side(side: DriveSide) -> DriveSide {
    match side {
        DriveSide::Left => DriveSide::Right,
        DriveSide::Right => DriveSide::Left,
    }
}

pub(super) fn progressed_in_direction(
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

pub(super) fn travel_in_direction(
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
