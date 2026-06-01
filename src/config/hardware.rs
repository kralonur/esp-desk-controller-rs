use crate::{
    config::ConfigError,
    leg::{DriveSide, LegConfig},
    quadrature::QuadratureDirection,
};

/// Stable identifier for the two physical desk legs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareLegSide {
    Left,
    Right,
}

// Default motor side that physically moves the left leg upward.
const DEFAULT_LEFT_UP_DRIVE: DriveSide = DriveSide::Left;
// Default encoder direction observed while the left leg moves upward.
const DEFAULT_LEFT_UP_DIRECTION: QuadratureDirection = QuadratureDirection::Positive;
// Default motor side that physically moves the right leg upward.
const DEFAULT_RIGHT_UP_DRIVE: DriveSide = DriveSide::Left;
// Default encoder direction observed while the right leg moves upward.
const DEFAULT_RIGHT_UP_DIRECTION: QuadratureDirection = QuadratureDirection::Positive;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Hardware calibration for motor side and encoder polarity.
///
/// These values describe physical wiring/assembly, not normal runtime tuning.
/// MQTT updates are intentionally accepted only while the desk is idle and
/// unhomed, because changing them invalidates the coordinate frame established
/// during homing.
pub struct HardwareConfig {
    // MQTT `hardware/left/up_drive`: H-bridge side that moves the left leg upward.
    left_up_drive: DriveSide,
    // MQTT `hardware/left/up_direction`: encoder direction observed while the left leg moves up.
    left_up_direction: QuadratureDirection,
    // MQTT `hardware/right/up_drive`: H-bridge side that moves the right leg upward.
    right_up_drive: DriveSide,
    // MQTT `hardware/right/up_direction`: encoder direction observed while the right leg moves up.
    right_up_direction: QuadratureDirection,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct HardwareConfigParts {
    pub(crate) left_up_drive: DriveSide,
    pub(crate) left_up_direction: QuadratureDirection,
    pub(crate) right_up_drive: DriveSide,
    pub(crate) right_up_direction: QuadratureDirection,
}

impl HardwareConfig {
    pub const fn new() -> Self {
        let config = Self {
            left_up_drive: DEFAULT_LEFT_UP_DRIVE,
            left_up_direction: DEFAULT_LEFT_UP_DIRECTION,
            right_up_drive: DEFAULT_RIGHT_UP_DRIVE,
            right_up_direction: DEFAULT_RIGHT_UP_DIRECTION,
        };
        assert!(config.is_valid(), "invalid default hardware config");
        config
    }

    pub(crate) fn from_parts(parts: HardwareConfigParts) -> Result<Self, ConfigError> {
        Self {
            left_up_drive: parts.left_up_drive,
            left_up_direction: parts.left_up_direction,
            right_up_drive: parts.right_up_drive,
            right_up_direction: parts.right_up_direction,
        }
        .validate()
    }

    /// Validate hardware-only invariants before this config is installed or stored.
    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.is_valid() {
            Ok(self)
        } else {
            Err(ConfigError::InvalidHardwareConfig)
        }
    }

    const fn is_valid(self) -> bool {
        // Invalid encoder direction cannot establish up/down movement.
        !matches!(self.left_up_direction, QuadratureDirection::Invalid)
            // Invalid encoder direction cannot establish up/down movement.
            && !matches!(self.right_up_direction, QuadratureDirection::Invalid)
    }

    fn update_checked(&mut self, update_fn: impl FnOnce(&mut Self)) -> Result<(), ConfigError> {
        let previous = *self;
        update_fn(self);
        if self.is_valid() {
            Ok(())
        } else {
            *self = previous;
            Err(ConfigError::InvalidHardwareConfig)
        }
    }

    pub const fn left_up_drive(self) -> DriveSide {
        self.left_up_drive
    }

    pub const fn left_up_direction(self) -> QuadratureDirection {
        self.left_up_direction
    }

    pub const fn right_up_drive(self) -> DriveSide {
        self.right_up_drive
    }

    pub const fn right_up_direction(self) -> QuadratureDirection {
        self.right_up_direction
    }

    pub const fn leg_config(self, side: HardwareLegSide) -> LegConfig {
        match side {
            HardwareLegSide::Left => LegConfig {
                up_drive: self.left_up_drive,
                up_direction: self.left_up_direction,
            },
            HardwareLegSide::Right => LegConfig {
                up_drive: self.right_up_drive,
                up_direction: self.right_up_direction,
            },
        }
    }

    pub fn set_left_up_drive(&mut self, value: DriveSide) -> Result<(), ConfigError> {
        self.update_checked(|config| config.left_up_drive = value)
    }

    pub fn set_left_up_direction(&mut self, value: QuadratureDirection) -> Result<(), ConfigError> {
        self.update_checked(|config| config.left_up_direction = value)
    }

    pub fn set_right_up_drive(&mut self, value: DriveSide) -> Result<(), ConfigError> {
        self.update_checked(|config| config.right_up_drive = value)
    }

    pub fn set_right_up_direction(
        &mut self,
        value: QuadratureDirection,
    ) -> Result<(), ConfigError> {
        self.update_checked(|config| config.right_up_direction = value)
    }
}

impl From<HardwareConfig> for HardwareConfigParts {
    fn from(config: HardwareConfig) -> Self {
        Self {
            left_up_drive: config.left_up_drive,
            left_up_direction: config.left_up_direction,
            right_up_drive: config.right_up_drive,
            right_up_direction: config.right_up_direction,
        }
    }
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self::new()
    }
}
