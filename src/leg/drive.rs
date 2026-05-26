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

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    pub fn apply_drive_mode(&mut self, mode: DriveMode, runtime_config: LegRuntimeConfig) {
        let duty = match mode {
            DriveMode::Stop => {
                self.coast();
                DutyPercent::new(0)
            }
            DriveMode::UpBoost => self.drive_up_boost(runtime_config),
            DriveMode::UpRun => self.drive_up_run(runtime_config),
            DriveMode::UpSlow => self.drive_up_slow(runtime_config),
            DriveMode::DownBoost => self.drive_down_boost(runtime_config),
            DriveMode::DownRun => self.drive_down_run(runtime_config),
            DriveMode::DownSlow => self.drive_down_slow(runtime_config),
            DriveMode::HomeDown => self.drive_home_down(runtime_config),
        };

        let motion = match mode {
            DriveMode::Stop => MotionState::Idle,
            DriveMode::UpBoost | DriveMode::UpRun | DriveMode::UpSlow => MotionState::MovingUp,
            DriveMode::DownBoost
            | DriveMode::DownRun
            | DriveMode::DownSlow
            | DriveMode::HomeDown => MotionState::MovingDown,
        };

        self.send_status(LegStatus {
            duty: duty.get(),
            motion,
            ..self.current_status()
        });
    }

    pub fn drive_up_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingUp {
            runtime_config.startup_duty()
        } else {
            duty
        };
        let duty = self.drive_side(self.config.up_drive, duty, runtime_config);
        self.send_status(LegStatus {
            duty: duty.get(),
            motion: MotionState::MovingUp,
            ..self.current_status()
        });
    }

    pub fn drive_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingDown {
            runtime_config.startup_duty()
        } else {
            duty
        };
        let duty = self.drive_side(
            opposite_drive_side(self.config.up_drive),
            duty,
            runtime_config,
        );
        self.send_status(LegStatus {
            duty: duty.get(),
            motion: MotionState::MovingDown,
            ..self.current_status()
        });
    }

    pub fn drive_home_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingDown {
            runtime_config.startup_duty()
        } else {
            duty
        };
        let duty = self.drive_side(
            opposite_drive_side(self.config.up_drive),
            duty,
            runtime_config,
        );
        self.send_status(LegStatus {
            duty: duty.get(),
            motion: MotionState::MovingDown,
            ..self.current_status()
        });
    }

    pub(super) fn drive_up_boost(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            self.config.up_drive,
            runtime_config.startup_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_down_boost(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.startup_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_up_run(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            self.config.up_drive,
            runtime_config.run_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_down_run(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.run_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_up_slow(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            self.config.up_drive,
            runtime_config.slow_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_down_slow(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.slow_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_home_down(&mut self, runtime_config: LegRuntimeConfig) -> DutyPercent {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.homing_duty(),
            runtime_config,
        )
    }

    pub(super) fn drive_side(
        &mut self,
        side: DriveSide,
        duty: DutyPercent,
        runtime_config: LegRuntimeConfig,
    ) -> DutyPercent {
        let duty = duty
            .min(runtime_config.max_duty())
            .min(DutyPercent::new(100));
        let timestamp = duty.to_pwm_timestamp(PWM_TIMER_MAX_TICKS);
        match side {
            DriveSide::Left => self.motor.drive_left(timestamp),
            DriveSide::Right => self.motor.drive_right(timestamp),
        }
        duty
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
