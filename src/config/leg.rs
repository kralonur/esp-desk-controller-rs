use embassy_time::Duration;

use crate::config::ConfigError;
use crate::units::{CountDelta, DutyPercent, PositionCounts};

const DEFAULT_LEG_STARTUP_DUTY: DutyPercent = DutyPercent::new(100);
const DEFAULT_LEG_MAX_DUTY: DutyPercent = DutyPercent::new(100);
const DEFAULT_LEG_RUN_DUTY: DutyPercent = DutyPercent::new(30);
const DEFAULT_LEG_SLOW_DUTY: DutyPercent = DutyPercent::new(15);
const DEFAULT_LEG_HOMING_DUTY: DutyPercent = DutyPercent::new(20);
const DEFAULT_LEG_STARTUP_EVENTS: CountDelta = CountDelta::new(8);
const DEFAULT_LEG_HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
const DEFAULT_LEG_HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const DEFAULT_LEG_HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
const DEFAULT_LEG_HOMING_BACKOFF_STEPS: CountDelta = CountDelta::new(20);
const DEFAULT_LEG_MAX_POSITION: PositionCounts = PositionCounts::new(2_000);
const DEFAULT_LEG_MOVE_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const DEFAULT_LEG_TARGET_SLOW_ZONE: CountDelta = CountDelta::new(10);
const DEFAULT_LEG_TARGET_TOLERANCE: CountDelta = CountDelta::new(5);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Runtime settings for individual leg movement and homing.
///
/// Validation keeps travel limits sane and rejects zero movement
/// timeouts/intervals.
pub struct LegRuntimeConfig {
    startup_duty: DutyPercent,
    max_duty: DutyPercent,
    run_duty: DutyPercent,
    slow_duty: DutyPercent,
    homing_duty: DutyPercent,
    startup_events: CountDelta,
    homing_start_timeout: Duration,
    homing_stall_timeout: Duration,
    homing_poll_interval: Duration,
    homing_backoff_steps: CountDelta,
    default_max_position: PositionCounts,
    move_stall_timeout: Duration,
    target_slow_zone: CountDelta,
    target_tolerance: CountDelta,
}

impl LegRuntimeConfig {
    pub const fn new() -> Self {
        let config = Self {
            startup_duty: DEFAULT_LEG_STARTUP_DUTY,
            max_duty: DEFAULT_LEG_MAX_DUTY,
            run_duty: DEFAULT_LEG_RUN_DUTY,
            slow_duty: DEFAULT_LEG_SLOW_DUTY,
            homing_duty: DEFAULT_LEG_HOMING_DUTY,
            startup_events: DEFAULT_LEG_STARTUP_EVENTS,
            homing_start_timeout: DEFAULT_LEG_HOMING_START_TIMEOUT,
            homing_stall_timeout: DEFAULT_LEG_HOMING_STALL_TIMEOUT,
            homing_poll_interval: DEFAULT_LEG_HOMING_POLL_INTERVAL,
            homing_backoff_steps: DEFAULT_LEG_HOMING_BACKOFF_STEPS,
            default_max_position: DEFAULT_LEG_MAX_POSITION,
            move_stall_timeout: DEFAULT_LEG_MOVE_STALL_TIMEOUT,
            target_slow_zone: DEFAULT_LEG_TARGET_SLOW_ZONE,
            target_tolerance: DEFAULT_LEG_TARGET_TOLERANCE,
        };
        assert!(config.is_valid(), "invalid default leg config");
        config
    }

    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.is_valid() {
            Ok(self)
        } else {
            Err(ConfigError::InvalidLegConfig)
        }
    }

    const fn is_valid(self) -> bool {
        self.default_max_position.get() >= 0
            && self.homing_start_timeout.as_millis() > 0
            && self.homing_stall_timeout.as_millis() > 0
            && self.homing_poll_interval.as_millis() > 0
            && self.move_stall_timeout.as_millis() > 0
    }

    fn update_checked(&mut self, update_fn: impl FnOnce(&mut Self)) -> Result<(), ConfigError> {
        let previous = *self;
        update_fn(self);
        if self.is_valid() {
            Ok(())
        } else {
            *self = previous;
            Err(ConfigError::InvalidLegConfig)
        }
    }

    pub const fn startup_duty(self) -> DutyPercent {
        self.startup_duty
    }

    pub const fn max_duty(self) -> DutyPercent {
        self.max_duty
    }

    pub const fn run_duty(self) -> DutyPercent {
        self.run_duty
    }

    pub const fn slow_duty(self) -> DutyPercent {
        self.slow_duty
    }

    pub const fn homing_duty(self) -> DutyPercent {
        self.homing_duty
    }

    pub const fn startup_events(self) -> CountDelta {
        self.startup_events
    }

    pub const fn homing_start_timeout(self) -> Duration {
        self.homing_start_timeout
    }

    pub const fn homing_stall_timeout(self) -> Duration {
        self.homing_stall_timeout
    }

    pub const fn homing_poll_interval(self) -> Duration {
        self.homing_poll_interval
    }

    pub const fn homing_backoff_steps(self) -> CountDelta {
        self.homing_backoff_steps
    }

    pub const fn default_max_position(self) -> PositionCounts {
        self.default_max_position
    }

    pub const fn move_stall_timeout(self) -> Duration {
        self.move_stall_timeout
    }

    pub const fn target_slow_zone(self) -> CountDelta {
        self.target_slow_zone
    }

    pub const fn target_tolerance(self) -> CountDelta {
        self.target_tolerance
    }

    pub fn set_startup_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.startup_duty = value)
    }

    pub fn set_max_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.max_duty = value)
    }

    pub fn set_run_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.run_duty = value)
    }

    pub fn set_slow_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.slow_duty = value)
    }

    pub fn set_homing_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_duty = value)
    }

    pub fn set_startup_events(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.startup_events = value)
    }

    pub fn set_homing_start_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_start_timeout = value)
    }

    pub fn set_homing_stall_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_stall_timeout = value)
    }

    pub fn set_homing_poll_interval(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_poll_interval = value)
    }

    pub fn set_homing_backoff_steps(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_backoff_steps = value)
    }

    pub fn set_default_max_position(&mut self, value: PositionCounts) -> Result<(), ConfigError> {
        self.update_checked(|config| config.default_max_position = value)
    }

    pub fn set_move_stall_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.move_stall_timeout = value)
    }

    pub fn set_target_slow_zone(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.target_slow_zone = value)
    }

    pub fn set_target_tolerance(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.target_tolerance = value)
    }
}

impl Default for LegRuntimeConfig {
    fn default() -> Self {
        Self::new()
    }
}
