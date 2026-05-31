use embassy_time::Duration;

use crate::config::ConfigError;
use crate::units::{CountDelta, DutyPercent, PositionCounts};

// Initial one-leg duty, in percent, used for the startup boost phase.
const DEFAULT_LEG_STARTUP_DUTY: DutyPercent = DutyPercent::new(100);
// Hard cap, in percent, applied to every duty requested by one-leg control.
const DEFAULT_LEG_MAX_DUTY: DutyPercent = DutyPercent::new(100);
// Normal one-leg movement duty, in percent, after startup boost ends.
const DEFAULT_LEG_RUN_DUTY: DutyPercent = DutyPercent::new(30);
// Reduced one-leg duty, in percent, used inside the target slow zone.
const DEFAULT_LEG_SLOW_DUTY: DutyPercent = DutyPercent::new(15);
// One-leg downward homing duty, in percent, used while searching for the end stop.
const DEFAULT_LEG_HOMING_DUTY: DutyPercent = DutyPercent::new(20);
// Encoder-count travel before one-leg control leaves startup boost.
const DEFAULT_LEG_STARTUP_EVENTS: CountDelta = CountDelta::new(8);
// Maximum time for a homing leg to show initial movement before faulting.
const DEFAULT_LEG_HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
// Maximum time without encoder progress before one-leg homing treats the leg as stalled.
const DEFAULT_LEG_HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
// Sleep interval for one-leg homing and manual override progress checks.
const DEFAULT_LEG_HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
// Encoder-count lift after hitting the lower end stop before declaring zero.
const DEFAULT_LEG_HOMING_BACKOFF_STEPS: CountDelta = CountDelta::new(20);
// Default logical travel range, in encoder counts, after homing.
const DEFAULT_LEG_MAX_POSITION: PositionCounts = PositionCounts::new(2_000);
// Maximum time without encoder progress during one-leg ready movement.
const DEFAULT_LEG_MOVE_STALL_TIMEOUT: Duration = Duration::from_millis(600);
// Encoder-count distance from target where one-leg movement switches to slow duty.
const DEFAULT_LEG_TARGET_SLOW_ZONE: CountDelta = CountDelta::new(10);
// Encoder-count error band accepted as reached for one-leg movement.
const DEFAULT_LEG_TARGET_TOLERANCE: CountDelta = CountDelta::new(5);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Runtime settings for individual leg movement and homing.
///
/// Validation keeps travel limits sane and rejects zero movement
/// timeouts/intervals. Setters are transactional: rejected values leave the
/// previous config unchanged.
pub struct LegRuntimeConfig {
    // MQTT `leg/startup_duty`: startup boost duty for individual-leg control, in percent.
    startup_duty: DutyPercent,
    // MQTT `leg/max_duty`: maximum allowed individual-leg duty, in percent.
    max_duty: DutyPercent,
    // MQTT `leg/run_duty`: normal individual-leg movement duty, in percent.
    run_duty: DutyPercent,
    // MQTT `leg/slow_duty`: individual-leg duty inside the target slow zone, in percent.
    slow_duty: DutyPercent,
    // MQTT `leg/homing_duty`: individual-leg downward homing duty, in percent.
    homing_duty: DutyPercent,
    // MQTT `leg/startup_events`: encoder counts to stay in startup boost.
    startup_events: CountDelta,
    // MQTT `leg/homing_start_timeout_ms`: initial homing movement timeout, in milliseconds.
    homing_start_timeout: Duration,
    // MQTT `leg/homing_stall_timeout_ms`: no-progress homing stall timeout, in milliseconds.
    homing_stall_timeout: Duration,
    // MQTT `leg/homing_poll_interval_ms`: individual-leg homing/override poll interval, in milliseconds.
    homing_poll_interval: Duration,
    // MQTT `leg/homing_backoff_steps`: encoder counts to back off after homing stall.
    homing_backoff_steps: CountDelta,
    // MQTT `leg/default_max_position`: logical travel limit after homing, in encoder counts.
    default_max_position: PositionCounts,
    // MQTT `leg/move_stall_timeout_ms`: individual-leg ready-move no-progress timeout.
    move_stall_timeout: Duration,
    // MQTT `leg/target_slow_zone`: distance from target where individual-leg moves slow down.
    target_slow_zone: CountDelta,
    // MQTT `leg/target_tolerance`: acceptable individual-leg target error, in encoder counts.
    target_tolerance: CountDelta,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct LegRuntimeConfigParts {
    pub(crate) startup_duty: DutyPercent,
    pub(crate) max_duty: DutyPercent,
    pub(crate) run_duty: DutyPercent,
    pub(crate) slow_duty: DutyPercent,
    pub(crate) homing_duty: DutyPercent,
    pub(crate) startup_events: CountDelta,
    pub(crate) homing_start_timeout: Duration,
    pub(crate) homing_stall_timeout: Duration,
    pub(crate) homing_poll_interval: Duration,
    pub(crate) homing_backoff_steps: CountDelta,
    pub(crate) default_max_position: PositionCounts,
    pub(crate) move_stall_timeout: Duration,
    pub(crate) target_slow_zone: CountDelta,
    pub(crate) target_tolerance: CountDelta,
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

    pub(crate) fn from_parts(parts: LegRuntimeConfigParts) -> Result<Self, ConfigError> {
        Self {
            startup_duty: parts.startup_duty,
            max_duty: parts.max_duty,
            run_duty: parts.run_duty,
            slow_duty: parts.slow_duty,
            homing_duty: parts.homing_duty,
            startup_events: parts.startup_events,
            homing_start_timeout: parts.homing_start_timeout,
            homing_stall_timeout: parts.homing_stall_timeout,
            homing_poll_interval: parts.homing_poll_interval,
            homing_backoff_steps: parts.homing_backoff_steps,
            default_max_position: parts.default_max_position,
            move_stall_timeout: parts.move_stall_timeout,
            target_slow_zone: parts.target_slow_zone,
            target_tolerance: parts.target_tolerance,
        }
        .validate()
    }

    /// Validate leg-only invariants before this config is installed or stored.
    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.is_valid() {
            Ok(self)
        } else {
            Err(ConfigError::InvalidLegConfig)
        }
    }

    const fn is_valid(self) -> bool {
        // Every leg move begins with boost.
        self.startup_duty.get() > 0
            // Max duty must not clamp all active drive modes to stopped.
            && self.max_duty.get() > 0
            // Normal leg movement needs drive.
            && self.run_duty.get() > 0
            // Near-target leg movement still needs drive.
            && self.slow_duty.get() > 0
            // Homing must drive downward until stall.
            && self.homing_duty.get() > 0
            // Startup duty should not be silently clamped by max duty.
            && self.startup_duty.get() <= self.max_duty.get()
            // Run duty should not be silently clamped by max duty.
            && self.run_duty.get() <= self.max_duty.get()
            // Entering the slow zone must not increase speed.
            && self.slow_duty.get() <= self.run_duty.get()
            // Homing duty should not be silently clamped by max duty.
            && self.homing_duty.get() <= self.max_duty.get()
            // A homed leg needs a usable travel range.
            && self.default_max_position.get() > 0
            // Startup phase must fit inside travel range.
            && self.startup_events.get() <= self.default_max_position.get() as u32
            // Backoff must lift off the end stop before resetting position.
            && self.homing_backoff_steps.get() > 0
            // Backoff must fit inside travel range.
            && self.homing_backoff_steps.get() <= self.default_max_position.get() as u32
            // Backoff should not finish entirely in startup boost.
            && self.startup_events.get() <= self.homing_backoff_steps.get()
            // Slow zone should include the final tolerance band.
            && self.target_slow_zone.get() >= self.target_tolerance.get()
            // Slow zone must fit inside travel range.
            && self.target_slow_zone.get() <= self.default_max_position.get() as u32
            // Startup failure detection must be enabled.
            && self.homing_start_timeout.as_millis() > 0
            // End-stop/stall detection must be enabled.
            && self.homing_stall_timeout.as_millis() > 0
            // Homing and override loops sleep for this interval.
            && self.homing_poll_interval.as_millis() > 0
            // Startup timeout must allow at least one scheduled poll.
            && self.homing_start_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Stall timeout must allow at least one scheduled poll.
            && self.homing_stall_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Override moves observe progress on the homing poll cadence.
            && self.move_stall_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Ready leg moves need progress timeout enabled.
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

impl From<LegRuntimeConfig> for LegRuntimeConfigParts {
    fn from(config: LegRuntimeConfig) -> Self {
        Self {
            startup_duty: config.startup_duty,
            max_duty: config.max_duty,
            run_duty: config.run_duty,
            slow_duty: config.slow_duty,
            homing_duty: config.homing_duty,
            startup_events: config.startup_events,
            homing_start_timeout: config.homing_start_timeout,
            homing_stall_timeout: config.homing_stall_timeout,
            homing_poll_interval: config.homing_poll_interval,
            homing_backoff_steps: config.homing_backoff_steps,
            default_max_position: config.default_max_position,
            move_stall_timeout: config.move_stall_timeout,
            target_slow_zone: config.target_slow_zone,
            target_tolerance: config.target_tolerance,
        }
    }
}

impl Default for LegRuntimeConfig {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use super::*;

    #[test]
    fn rejects_zero_default_max_position() {
        let mut config = LegRuntimeConfig::default();
        assert_eq!(
            config.set_default_max_position(PositionCounts::new(0)),
            Err(ConfigError::InvalidLegConfig)
        );
    }

    #[test]
    fn rejects_backoff_beyond_default_max_position() {
        let mut config = LegRuntimeConfig::default();
        assert_eq!(
            config.set_default_max_position(PositionCounts::new(10)),
            Err(ConfigError::InvalidLegConfig)
        );
    }

    #[test]
    fn rejects_slow_duty_above_run_duty() {
        let mut config = LegRuntimeConfig::default();
        assert_eq!(
            config.set_slow_duty(DutyPercent::new(31)),
            Err(ConfigError::InvalidLegConfig)
        );
    }

    #[test]
    fn rejects_timeout_below_poll_interval() {
        let mut config = LegRuntimeConfig::default();
        assert_eq!(
            config.set_homing_poll_interval(Duration::from_millis(700)),
            Err(ConfigError::InvalidLegConfig)
        );
    }
}
