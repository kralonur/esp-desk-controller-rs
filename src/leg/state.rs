use embassy_executor::Spawner;
use esp_hal::mcpwm::PwmPeripheral;

use crate::{
    config::{LegRuntimeConfig, RuntimeConfigReader},
    motor::Motor,
    quadrature::{QuadratureDirection, QuadratureWatcher},
    units::PositionSign,
};

use super::{
    drive::{DriveOutput, DriveSide, opposite_direction},
    status::{
        LegProgressWatcher, LegStatus, LegStatusState, LegStatusStorage, LegStatusWatcher,
        MotionState, mirror_quadrature_to_leg_status,
    },
};

/// Typestate marker for a leg that has not established its travel limits.
pub struct Unhomed;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Static direction configuration for one physical leg.
///
/// `up_drive` selects which motor side raises the leg, while `up_direction`
/// records which quadrature direction corresponds to upward travel.
pub struct LegConfig {
    /// Motor drive side that physically raises this leg.
    pub up_drive: DriveSide,
    /// Encoder direction observed while this leg moves upward.
    pub up_direction: QuadratureDirection,
}

/// Typestate data for a homed leg with known travel limits.
pub struct Ready {
    pub(super) min_position: i32,
    pub(super) max_position: i32,
    pub(super) position_sign: PositionSign,
    pub(super) up_direction: QuadratureDirection,
    pub(super) down_direction: QuadratureDirection,
    pub(super) motion: MotionState,
}

/// One motorized desk leg.
///
/// The `State` typestate controls whether movement may use configured travel
/// limits. Unhomed legs can home; ready legs can move to targets.
pub struct Leg<'a, State, const OP: u8, PWM: PwmPeripheral> {
    pub(super) config: LegConfig,
    pub(super) runtime_config_reader: RuntimeConfigReader,
    pub(super) motor: Motor<'a, OP, PWM>,
    pub(super) quadrature_watcher: QuadratureWatcher,
    pub(super) status_state: &'static LegStatusState,
    pub(super) drive_output: Option<DriveOutput>,
    pub(super) state: State,
}

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    pub(super) fn configured_up_direction(&self) -> QuadratureDirection {
        self.config.up_direction
    }

    pub(super) fn configured_down_direction(&self) -> QuadratureDirection {
        opposite_direction(self.config.up_direction)
    }

    pub(super) fn position_sign(&self) -> PositionSign {
        match self.config.up_direction {
            QuadratureDirection::Positive | QuadratureDirection::Invalid => PositionSign::Positive,
            QuadratureDirection::Negative => PositionSign::Negative,
        }
    }

    /// Current logical position after applying configured encoder polarity.
    pub fn logical_position(&self) -> i32 {
        self.current_status().position
    }

    pub fn encoder_position(&self) -> i32 {
        self.quadrature_watcher.snapshot().position
    }

    pub fn logical_encoder_position(&self) -> i32 {
        self.position_sign().apply(self.encoder_position())
    }

    /// Reset the underlying quadrature position accumulator to zero.
    pub fn reset_position(&self) {
        self.quadrature_watcher.reset_position();
    }

    pub fn up_direction(&self) -> QuadratureDirection {
        self.configured_up_direction()
    }

    pub fn down_direction(&self) -> QuadratureDirection {
        self.configured_down_direction()
    }

    pub(super) fn current_status(&self) -> LegStatus {
        self.status_state.current()
    }

    pub(super) fn coast(&mut self) {
        self.motor.coast();
        self.drive_output = None;
    }

    pub(super) fn send_status(&self, status: LegStatus) {
        self.status_state.publish(status);
    }

    /// Subscribe to published leg status changes.
    pub fn status_watcher(&self) -> LegStatusWatcher {
        let receiver = self
            .status_state
            .watch
            .receiver()
            .expect("leg status watch receiver limit reached");

        LegStatusWatcher { receiver }
    }

    /// Subscribe to raw quadrature progress using the leg's logical direction.
    pub fn progress_watcher(&self) -> LegProgressWatcher {
        LegProgressWatcher {
            quadrature_watcher: self.quadrature_watcher.resubscribe(),
            position_sign: self.position_sign(),
        }
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Unhomed, OP, PWM> {
    fn status(&self) -> LegStatus {
        LegStatus {
            position: self.logical_encoder_position(),
            min_position: 0,
            max_position: 0,
            duty: 0,
            motion: MotionState::Idle,
            homed: false,
        }
    }

    pub fn publish_status(&self) {
        self.send_status(self.status());
    }

    /// Build an unhomed leg and start mirroring quadrature events into status.
    pub fn new(
        storage: &'static LegStatusStorage,
        config: LegConfig,
        runtime_config_reader: RuntimeConfigReader,
        motor: Motor<'a, OP, PWM>,
        quadrature_watcher: QuadratureWatcher,
        status_quadrature_watcher: QuadratureWatcher,
        spawner: &Spawner,
    ) -> Self {
        let initial_status = LegStatus {
            position: quadrature_watcher.snapshot().position,
            min_position: 0,
            max_position: 0,
            duty: 0,
            motion: MotionState::Idle,
            homed: false,
        };
        let status_state = storage.state.init(LegStatusState::new(
            initial_status,
            match config.up_direction {
                QuadratureDirection::Positive | QuadratureDirection::Invalid => {
                    PositionSign::Positive
                }
                QuadratureDirection::Negative => PositionSign::Negative,
            },
        ));
        spawner.spawn(
            mirror_quadrature_to_leg_status(status_quadrature_watcher, status_state)
                .expect("spawn quadrature status mirror task"),
        );
        let leg = Self {
            config,
            runtime_config_reader,
            motor,
            quadrature_watcher,
            status_state,
            drive_output: Some(DriveOutput::stopped()),
            state: Unhomed,
        };
        leg.send_status(leg.status());
        leg
    }

    pub fn into_ready_with_config(
        self,
        runtime_config: LegRuntimeConfig,
    ) -> Leg<'a, Ready, OP, PWM> {
        let position_sign = self.position_sign();
        let up_direction = self.configured_up_direction();
        let down_direction = self.configured_down_direction();

        self.status_state
            .publish_position(self.quadrature_watcher.snapshot().position);

        Leg {
            config: self.config,
            runtime_config_reader: self.runtime_config_reader,
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            status_state: self.status_state,
            drive_output: self.drive_output,
            state: Ready {
                min_position: 0,
                max_position: runtime_config.default_max_position().get(),
                position_sign,
                up_direction,
                down_direction,
                motion: MotionState::Idle,
            },
        }
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Ready, OP, PWM> {
    pub(super) fn logical_position_from_raw(&self, raw_position: i32) -> i32 {
        self.state.position_sign.apply(raw_position)
    }

    pub fn status(&self) -> LegStatus {
        let mut status = self.current_status();
        status.min_position = self.state.min_position;
        status.max_position = self.state.max_position;
        status.homed = true;
        status
    }

    pub fn motion(&self) -> MotionState {
        self.current_status().motion
    }

    pub fn publish_status(&self) {
        self.send_status(self.status());
    }

    pub fn into_unhomed(mut self) -> Leg<'a, Unhomed, OP, PWM> {
        self.coast();
        self.drive_output = Some(DriveOutput::stopped());
        self.send_status(LegStatus {
            position: self.logical_position(),
            min_position: 0,
            max_position: 0,
            duty: 0,
            motion: MotionState::Idle,
            homed: false,
        });

        Leg {
            config: self.config,
            runtime_config_reader: self.runtime_config_reader,
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            status_state: self.status_state,
            drive_output: self.drive_output,
            state: Unhomed,
        }
    }

    pub fn min_position(&self) -> i32 {
        self.state.min_position
    }

    pub fn max_position(&self) -> i32 {
        self.state.max_position
    }

    pub fn set_max_position(&mut self, max_position: i32) {
        self.state.max_position = max_position.max(self.state.min_position);
        self.send_status(self.status());
    }

    pub fn clamp_target(&self, target_position: i32) -> i32 {
        target_position.clamp(self.state.min_position, self.state.max_position)
    }
}
