use embassy_time::Duration;

use crate::config::ConfigError;
use crate::units::{CountDelta, DutyPercent, DutyPercentTrim, Percent};

// Encoder-count error band accepted as reached for coordinated desk moves.
const DEFAULT_DESK_TARGET_TOLERANCE: CountDelta = CountDelta::new(5);
// Encoder-count distance from target where coordinated movement switches to slow duty.
const DEFAULT_DESK_TARGET_SLOW_ZONE: CountDelta = CountDelta::new(10);
// Maximum total time allowed for a coordinated move or homing backoff step.
const DEFAULT_DESK_MOVE_TIMEOUT: Duration = Duration::from_millis(1_200);
// Time window used to calculate obstruction speed, in milliseconds.
const DEFAULT_OBSTRUCTION_SAMPLE_WINDOW: Duration = Duration::from_millis(150);
// Startup period ignored by obstruction detection while the desk accelerates.
const DEFAULT_OBSTRUCTION_WARMUP_DURATION: Duration = Duration::from_millis(300);
// Minimum encoder-count travel before obstruction detection can establish a baseline.
const DEFAULT_OBSTRUCTION_WARMUP_COUNTS: CountDelta = CountDelta::new(8);
// Lowest duty, in percent, allowed during coordinated motion after sync trimming.
const DEFAULT_MIN_MOVE_DUTY: DutyPercent = DutyPercent::new(15);
// Normal coordinated movement duty, in percent.
const DEFAULT_MOVE_RUN_DUTY: DutyPercent = DutyPercent::new(30);
// Reduced coordinated movement duty, in percent, used inside the target slow zone.
const DEFAULT_MOVE_SLOW_DUTY: DutyPercent = DutyPercent::new(17);
// Duty-percent adjustment applied when legs are slightly out of sync during movement.
const DEFAULT_MOVE_SYNC_DUTY_STEP: DutyPercentTrim = DutyPercentTrim::new(8);
// Sleep interval for coordinated homing progress and skew checks.
const DEFAULT_DESK_HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
// Maximum time for both legs to show initial downward homing movement.
const DEFAULT_DESK_HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
// Maximum time without encoder progress before coordinated homing treats a leg as stalled.
const DEFAULT_DESK_HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
// Encoder-count lift after coordinated homing hits the lower end stop.
const DEFAULT_DESK_HOMING_BACKOFF_STEPS: CountDelta = CountDelta::new(20);
// Coordinated downward homing duty, in percent, after startup boost ends.
const DEFAULT_HOMING_RUN_DUTY: DutyPercent = DutyPercent::new(20);
// Duty-percent adjustment applied when legs are slightly out of sync during homing.
const DEFAULT_HOMING_SYNC_DUTY_STEP: DutyPercentTrim = DutyPercentTrim::new(4);
// Skew, in encoder counts, where light sync correction starts.
const DEFAULT_SYNC_SPEEDUP_ENTER_COUNTS: CountDelta = CountDelta::new(10);
// Skew, in encoder counts, where light sync correction stops.
const DEFAULT_SYNC_SPEEDUP_EXIT_COUNTS: CountDelta = CountDelta::new(4);
// Skew, in encoder counts, where stronger catch-up correction starts.
const DEFAULT_CATCH_UP_ENTER_COUNTS: CountDelta = CountDelta::new(30);
// Skew, in encoder counts, where stronger catch-up correction stops.
const DEFAULT_CATCH_UP_EXIT_COUNTS: CountDelta = CountDelta::new(12);
// Ready-move skew, in encoder counts, that faults the desk for unsafe mismatch.
const DEFAULT_FAULT_SKEW_COUNTS: CountDelta = CountDelta::new(80);
// Homing skew, in encoder counts, that faults the desk for unsafe mismatch.
const DEFAULT_HOMING_FAULT_SKEW_COUNTS: CountDelta = CountDelta::new(160);
// Default obstruction profile selected at boot before persisted config is loaded.
const DEFAULT_OBSTRUCTION_SENSITIVITY: ObstructionSensitivity = ObstructionSensitivity::High;
// Low sensitivity obstruction threshold: speed may fall to 55 percent for 3 windows.
const DEFAULT_LOW_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(55), 3);
// Medium sensitivity obstruction threshold: speed may fall to 70 percent for 2 windows.
const DEFAULT_MEDIUM_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(70), 2);
// High sensitivity obstruction threshold: speed may fall to 80 percent for 2 windows.
const DEFAULT_HIGH_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(80), 2);
// How long manual override access remains unlocked without a command.
const DEFAULT_OVERRIDE_UNLOCK_TIMEOUT: Duration = Duration::from_millis(120_000);
// Minimum interval between MQTT status publishes during normal operation.
const DEFAULT_MQTT_STATUS_PUBLISH_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
/// Obstruction detection profile selected for coordinated desk movement.
pub enum ObstructionSensitivity {
    /// Disable obstruction detection for coordinated movement.
    None,
    /// Use the least sensitive obstruction profile.
    Low,
    /// Use the middle obstruction profile.
    Medium,
    /// Use the most sensitive obstruction profile.
    High,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Thresholds used by obstruction detection.
///
/// `minimum_baseline_percent` is compared against the best observed movement
/// speed, and `consecutive_windows` controls how long the slowdown must persist.
pub struct ObstructionProfileConfig {
    // Minimum allowed speed as a percent of the best observed speed during the move.
    minimum_baseline_percent: Percent,
    // Number of consecutive slow sample windows required before obstruction faults.
    consecutive_windows: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Runtime settings for coordinated desk behavior.
///
/// Validation keeps the move timeout longer than the obstruction detection
/// window and requires nonzero movement timeouts/intervals. Setters are
/// transactional: rejected values leave the previous config unchanged.
pub struct DeskConfig {
    // MQTT `desk/target_tolerance`: acceptable coordinated target error, in encoder counts.
    target_tolerance: CountDelta,
    // MQTT `desk/target_slow_zone`: distance from target where coordinated moves slow down.
    target_slow_zone: CountDelta,
    // MQTT `desk/move_timeout_ms`: coordinated move/backoff no-progress timeout, in milliseconds.
    move_timeout: Duration,
    // MQTT `desk/obstruction_sample_window_ms`: obstruction speed sample window, in milliseconds.
    obstruction_sample_window: Duration,
    // MQTT `desk/obstruction_warmup_duration_ms`: startup time ignored by obstruction detection.
    obstruction_warmup_duration: Duration,
    // MQTT `desk/obstruction_warmup_counts`: startup travel ignored by obstruction detection.
    obstruction_warmup_counts: CountDelta,
    // MQTT `desk/min_move_duty`: minimum coordinated movement duty after sync trimming, in percent.
    min_move_duty: DutyPercent,
    // MQTT `desk/move_run_duty`: normal coordinated movement duty, in percent.
    move_run_duty: DutyPercent,
    // MQTT `desk/move_slow_duty`: coordinated movement duty inside the slow zone, in percent.
    move_slow_duty: DutyPercent,
    // MQTT `desk/move_sync_duty_step`: duty-percent trim used for movement sync correction.
    move_sync_duty_step: DutyPercentTrim,
    // MQTT `desk/homing_poll_interval_ms`: coordinated homing poll interval, in milliseconds.
    homing_poll_interval: Duration,
    // MQTT `desk/homing_start_timeout_ms`: initial coordinated homing movement timeout.
    homing_start_timeout: Duration,
    // MQTT `desk/homing_stall_timeout_ms`: coordinated homing no-progress stall timeout.
    homing_stall_timeout: Duration,
    // MQTT `desk/homing_backoff_steps`: encoder counts to lift after coordinated homing.
    homing_backoff_steps: CountDelta,
    // MQTT `desk/homing_run_duty`: coordinated homing duty after startup boost, in percent.
    homing_run_duty: DutyPercent,
    // MQTT `desk/homing_sync_duty_step`: duty-percent trim used for homing sync correction.
    homing_sync_duty_step: DutyPercentTrim,
    // MQTT `desk/sync_speedup_enter_counts`: skew where light sync correction starts.
    sync_speedup_enter_counts: CountDelta,
    // MQTT `desk/sync_speedup_exit_counts`: skew where light sync correction stops.
    sync_speedup_exit_counts: CountDelta,
    // MQTT `desk/catch_up_enter_counts`: skew where stronger catch-up correction starts.
    catch_up_enter_counts: CountDelta,
    // MQTT `desk/catch_up_exit_counts`: skew where stronger catch-up correction stops.
    catch_up_exit_counts: CountDelta,
    // MQTT `desk/fault_skew_counts`: ready-move skew that faults the desk.
    fault_skew_counts: CountDelta,
    // MQTT `desk/homing_fault_skew_counts`: homing skew that faults the desk.
    homing_fault_skew_counts: CountDelta,
    // MQTT `desk/obstruction_sensitivity`: selected obstruction profile name.
    obstruction_sensitivity: ObstructionSensitivity,
    // MQTT `desk/low_obstruction_min_percent` and `desk/low_obstruction_windows`.
    low_obstruction_profile: ObstructionProfileConfig,
    // MQTT `desk/medium_obstruction_min_percent` and `desk/medium_obstruction_windows`.
    medium_obstruction_profile: ObstructionProfileConfig,
    // MQTT `desk/high_obstruction_min_percent` and `desk/high_obstruction_windows`.
    high_obstruction_profile: ObstructionProfileConfig,
    // MQTT `desk/override_unlock_timeout_ms`: manual override idle timeout, in milliseconds.
    override_unlock_timeout: Duration,
    // MQTT `desk/mqtt_status_publish_interval_ms`: minimum status publish interval.
    mqtt_status_publish_interval: Duration,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct DeskConfigParts {
    pub(crate) target_tolerance: CountDelta,
    pub(crate) target_slow_zone: CountDelta,
    pub(crate) move_timeout: Duration,
    pub(crate) obstruction_sample_window: Duration,
    pub(crate) obstruction_warmup_duration: Duration,
    pub(crate) obstruction_warmup_counts: CountDelta,
    pub(crate) min_move_duty: DutyPercent,
    pub(crate) move_run_duty: DutyPercent,
    pub(crate) move_slow_duty: DutyPercent,
    pub(crate) move_sync_duty_step: DutyPercentTrim,
    pub(crate) homing_poll_interval: Duration,
    pub(crate) homing_start_timeout: Duration,
    pub(crate) homing_stall_timeout: Duration,
    pub(crate) homing_backoff_steps: CountDelta,
    pub(crate) homing_run_duty: DutyPercent,
    pub(crate) homing_sync_duty_step: DutyPercentTrim,
    pub(crate) sync_speedup_enter_counts: CountDelta,
    pub(crate) sync_speedup_exit_counts: CountDelta,
    pub(crate) catch_up_enter_counts: CountDelta,
    pub(crate) catch_up_exit_counts: CountDelta,
    pub(crate) fault_skew_counts: CountDelta,
    pub(crate) homing_fault_skew_counts: CountDelta,
    pub(crate) obstruction_sensitivity: ObstructionSensitivity,
    pub(crate) low_obstruction_profile: ObstructionProfileConfig,
    pub(crate) medium_obstruction_profile: ObstructionProfileConfig,
    pub(crate) high_obstruction_profile: ObstructionProfileConfig,
    pub(crate) override_unlock_timeout: Duration,
    pub(crate) mqtt_status_publish_interval: Duration,
}

impl DeskConfig {
    pub const fn new() -> Self {
        let config = Self {
            target_tolerance: DEFAULT_DESK_TARGET_TOLERANCE,
            target_slow_zone: DEFAULT_DESK_TARGET_SLOW_ZONE,
            move_timeout: DEFAULT_DESK_MOVE_TIMEOUT,
            obstruction_sample_window: DEFAULT_OBSTRUCTION_SAMPLE_WINDOW,
            obstruction_warmup_duration: DEFAULT_OBSTRUCTION_WARMUP_DURATION,
            obstruction_warmup_counts: DEFAULT_OBSTRUCTION_WARMUP_COUNTS,
            min_move_duty: DEFAULT_MIN_MOVE_DUTY,
            move_run_duty: DEFAULT_MOVE_RUN_DUTY,
            move_slow_duty: DEFAULT_MOVE_SLOW_DUTY,
            move_sync_duty_step: DEFAULT_MOVE_SYNC_DUTY_STEP,
            homing_poll_interval: DEFAULT_DESK_HOMING_POLL_INTERVAL,
            homing_start_timeout: DEFAULT_DESK_HOMING_START_TIMEOUT,
            homing_stall_timeout: DEFAULT_DESK_HOMING_STALL_TIMEOUT,
            homing_backoff_steps: DEFAULT_DESK_HOMING_BACKOFF_STEPS,
            homing_run_duty: DEFAULT_HOMING_RUN_DUTY,
            homing_sync_duty_step: DEFAULT_HOMING_SYNC_DUTY_STEP,
            sync_speedup_enter_counts: DEFAULT_SYNC_SPEEDUP_ENTER_COUNTS,
            sync_speedup_exit_counts: DEFAULT_SYNC_SPEEDUP_EXIT_COUNTS,
            catch_up_enter_counts: DEFAULT_CATCH_UP_ENTER_COUNTS,
            catch_up_exit_counts: DEFAULT_CATCH_UP_EXIT_COUNTS,
            fault_skew_counts: DEFAULT_FAULT_SKEW_COUNTS,
            homing_fault_skew_counts: DEFAULT_HOMING_FAULT_SKEW_COUNTS,
            obstruction_sensitivity: DEFAULT_OBSTRUCTION_SENSITIVITY,
            low_obstruction_profile: DEFAULT_LOW_OBSTRUCTION_PROFILE,
            medium_obstruction_profile: DEFAULT_MEDIUM_OBSTRUCTION_PROFILE,
            high_obstruction_profile: DEFAULT_HIGH_OBSTRUCTION_PROFILE,
            override_unlock_timeout: DEFAULT_OVERRIDE_UNLOCK_TIMEOUT,
            mqtt_status_publish_interval: DEFAULT_MQTT_STATUS_PUBLISH_INTERVAL,
        };
        assert!(config.is_valid(), "invalid default desk config");
        config
    }

    pub(crate) fn from_parts(parts: DeskConfigParts) -> Result<Self, ConfigError> {
        Self {
            target_tolerance: parts.target_tolerance,
            target_slow_zone: parts.target_slow_zone,
            move_timeout: parts.move_timeout,
            obstruction_sample_window: parts.obstruction_sample_window,
            obstruction_warmup_duration: parts.obstruction_warmup_duration,
            obstruction_warmup_counts: parts.obstruction_warmup_counts,
            min_move_duty: parts.min_move_duty,
            move_run_duty: parts.move_run_duty,
            move_slow_duty: parts.move_slow_duty,
            move_sync_duty_step: parts.move_sync_duty_step,
            homing_poll_interval: parts.homing_poll_interval,
            homing_start_timeout: parts.homing_start_timeout,
            homing_stall_timeout: parts.homing_stall_timeout,
            homing_backoff_steps: parts.homing_backoff_steps,
            homing_run_duty: parts.homing_run_duty,
            homing_sync_duty_step: parts.homing_sync_duty_step,
            sync_speedup_enter_counts: parts.sync_speedup_enter_counts,
            sync_speedup_exit_counts: parts.sync_speedup_exit_counts,
            catch_up_enter_counts: parts.catch_up_enter_counts,
            catch_up_exit_counts: parts.catch_up_exit_counts,
            fault_skew_counts: parts.fault_skew_counts,
            homing_fault_skew_counts: parts.homing_fault_skew_counts,
            obstruction_sensitivity: parts.obstruction_sensitivity,
            low_obstruction_profile: parts.low_obstruction_profile,
            medium_obstruction_profile: parts.medium_obstruction_profile,
            high_obstruction_profile: parts.high_obstruction_profile,
            override_unlock_timeout: parts.override_unlock_timeout,
            mqtt_status_publish_interval: parts.mqtt_status_publish_interval,
        }
        .validate()
    }

    /// Validate desk-only invariants before this config is installed or stored.
    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.is_valid() {
            Ok(self)
        } else {
            Err(ConfigError::InvalidDeskConfig)
        }
    }

    const fn is_valid(self) -> bool {
        // Allow enough time for obstruction warmup plus slow-window detection.
        self.move_timeout.as_millis() > default_obstruction_detection_time_ms(self)
            // Avoid zero-duration obstruction speed windows.
            && self.obstruction_sample_window.as_millis() > 0
            // Give motion time to settle before establishing obstruction baseline.
            && self.obstruction_warmup_duration.as_millis() > 0
            // Wrong-way detection uses this as its retreat threshold.
            && self.obstruction_warmup_counts.get() > 0
            // Active synchronized moves must not clamp down to stopped.
            && self.min_move_duty.get() > 0
            // Normal coordinated move duty must drive the motor.
            && self.move_run_duty.get() > 0
            // Near-target move duty must still drive the motor.
            && self.move_slow_duty.get() > 0
            // Coordinated homing needs drive after startup boost.
            && self.homing_run_duty.get() > 0
            // Entering the slow zone must not increase speed.
            && self.move_run_duty.get() >= self.move_slow_duty.get()
            // Keep run duty from being silently clamped by min_move_duty.
            && self.move_run_duty.get() >= self.min_move_duty.get()
            // Keep slow duty from being silently clamped by min_move_duty.
            && self.move_slow_duty.get() >= self.min_move_duty.get()
            // Keep homing duty from being silently clamped by min_move_duty.
            && self.homing_run_duty.get() >= self.min_move_duty.get()
            // Negative trim would reverse leader/follower correction.
            && self.move_sync_duty_step.get() >= 0
            // Homing sync trim follows the same correction direction.
            && self.homing_sync_duty_step.get() >= 0
            // Homing loops sleep for this interval each iteration.
            && self.homing_poll_interval.as_millis() > 0
            // Startup failure detection must be enabled.
            && self.homing_start_timeout.as_millis() > 0
            // End-stop/stall detection must be enabled.
            && self.homing_stall_timeout.as_millis() > 0
            // Startup timeout must allow at least one scheduled poll.
            && self.homing_start_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Stall timeout must allow at least one scheduled poll.
            && self.homing_stall_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Backoff progress timeout must allow at least one scheduled poll.
            && self.move_timeout.as_millis() >= self.homing_poll_interval.as_millis()
            // Backoff must lift off the end stop before resetting position.
            && self.homing_backoff_steps.get() > 0
            // Slow zone should include the final tolerance band.
            && self.target_slow_zone.get() >= self.target_tolerance.get()
            // Catch-up needs enter/exit hysteresis.
            && self.catch_up_enter_counts.get() > self.catch_up_exit_counts.get()
            // Leaving catch-up can still enter speed matching if skew remains.
            && self.catch_up_exit_counts.get() >= self.sync_speedup_enter_counts.get()
            // Speed matching needs enter/exit hysteresis.
            && self.sync_speedup_enter_counts.get() > self.sync_speedup_exit_counts.get()
            // Correction should engage before ready-move skew fault.
            && self.fault_skew_counts.get() > self.catch_up_enter_counts.get()
            // Correction should engage before homing skew fault.
            && self.homing_fault_skew_counts.get() > self.catch_up_enter_counts.get()
            // Final tolerance on both legs must stay within skew fault limit.
            && self.fault_skew_counts.get() / 2 >= self.target_tolerance.get()
            // Low profile can be selected or loaded at runtime.
            && self.low_obstruction_profile.is_valid()
            // Medium profile can be selected or loaded at runtime.
            && self.medium_obstruction_profile.is_valid()
            // High profile can be selected or loaded at runtime.
            && self.high_obstruction_profile.is_valid()
            // Profile names preserve low-to-high sensitivity order.
            && obstruction_profiles_ordered(
                self.low_obstruction_profile,
                self.medium_obstruction_profile,
                self.high_obstruction_profile,
            )
            // Override access must eventually expire.
            && self.override_unlock_timeout.as_millis() > 0
            // Status publishing must not spin continuously.
            && self.mqtt_status_publish_interval.as_millis() > 0
    }

    fn update_checked(&mut self, update_fn: impl FnOnce(&mut Self)) -> Result<(), ConfigError> {
        let previous = *self;
        update_fn(self);
        if self.is_valid() {
            Ok(())
        } else {
            *self = previous;
            Err(ConfigError::InvalidDeskConfig)
        }
    }

    pub const fn target_tolerance(self) -> CountDelta {
        self.target_tolerance
    }

    pub const fn target_slow_zone(self) -> CountDelta {
        self.target_slow_zone
    }

    pub const fn obstruction_warmup_counts(self) -> CountDelta {
        self.obstruction_warmup_counts
    }

    pub const fn min_move_duty(self) -> DutyPercent {
        self.min_move_duty
    }

    pub const fn move_run_duty(self) -> DutyPercent {
        self.move_run_duty
    }

    pub const fn move_slow_duty(self) -> DutyPercent {
        self.move_slow_duty
    }

    pub const fn move_sync_duty_step(self) -> DutyPercentTrim {
        self.move_sync_duty_step
    }

    pub const fn homing_backoff_steps(self) -> CountDelta {
        self.homing_backoff_steps
    }

    pub const fn homing_run_duty(self) -> DutyPercent {
        self.homing_run_duty
    }

    pub const fn homing_sync_duty_step(self) -> DutyPercentTrim {
        self.homing_sync_duty_step
    }

    pub const fn sync_speedup_enter_counts(self) -> CountDelta {
        self.sync_speedup_enter_counts
    }

    pub const fn sync_speedup_exit_counts(self) -> CountDelta {
        self.sync_speedup_exit_counts
    }

    pub const fn catch_up_enter_counts(self) -> CountDelta {
        self.catch_up_enter_counts
    }

    pub const fn catch_up_exit_counts(self) -> CountDelta {
        self.catch_up_exit_counts
    }

    pub const fn fault_skew_counts(self) -> CountDelta {
        self.fault_skew_counts
    }

    pub const fn homing_fault_skew_counts(self) -> CountDelta {
        self.homing_fault_skew_counts
    }

    pub const fn obstruction_sensitivity(self) -> ObstructionSensitivity {
        self.obstruction_sensitivity
    }

    /// Return the active obstruction profile, or `None` when detection is off.
    pub const fn obstruction_profile(
        self,
        sensitivity: ObstructionSensitivity,
    ) -> Option<ObstructionProfileConfig> {
        match sensitivity {
            ObstructionSensitivity::None => None,
            ObstructionSensitivity::Low => Some(self.low_obstruction_profile),
            ObstructionSensitivity::Medium => Some(self.medium_obstruction_profile),
            ObstructionSensitivity::High => Some(self.high_obstruction_profile),
        }
    }

    pub fn override_unlock_timeout(self) -> Duration {
        self.override_unlock_timeout
    }

    pub fn mqtt_status_publish_interval(self) -> Duration {
        self.mqtt_status_publish_interval
    }

    pub fn set_target_tolerance(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.target_tolerance = value)
    }

    pub fn set_target_slow_zone(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.target_slow_zone = value)
    }

    pub fn set_move_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.move_timeout = value)
    }

    pub fn set_obstruction_sample_window(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.obstruction_sample_window = value)
    }

    pub fn set_obstruction_warmup_duration(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.obstruction_warmup_duration = value)
    }

    pub fn set_obstruction_warmup_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.obstruction_warmup_counts = value)
    }

    pub fn set_min_move_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.min_move_duty = value)
    }

    pub fn set_move_run_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.move_run_duty = value)
    }

    pub fn set_move_slow_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.move_slow_duty = value)
    }

    pub fn set_move_sync_duty_step(&mut self, value: DutyPercentTrim) -> Result<(), ConfigError> {
        self.update_checked(|config| config.move_sync_duty_step = value)
    }

    pub fn set_homing_poll_interval(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_poll_interval = value)
    }

    pub fn set_homing_start_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_start_timeout = value)
    }

    pub fn set_homing_stall_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_stall_timeout = value)
    }

    pub fn set_homing_backoff_steps(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_backoff_steps = value)
    }

    pub fn set_homing_run_duty(&mut self, value: DutyPercent) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_run_duty = value)
    }

    pub fn set_homing_sync_duty_step(&mut self, value: DutyPercentTrim) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_sync_duty_step = value)
    }

    pub fn set_sync_speedup_enter_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.sync_speedup_enter_counts = value)
    }

    pub fn set_sync_speedup_exit_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.sync_speedup_exit_counts = value)
    }

    pub fn set_catch_up_enter_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.catch_up_enter_counts = value)
    }

    pub fn set_catch_up_exit_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.catch_up_exit_counts = value)
    }

    pub fn set_fault_skew_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.fault_skew_counts = value)
    }

    pub fn set_homing_fault_skew_counts(&mut self, value: CountDelta) -> Result<(), ConfigError> {
        self.update_checked(|config| config.homing_fault_skew_counts = value)
    }

    pub fn set_obstruction_sensitivity(
        &mut self,
        value: ObstructionSensitivity,
    ) -> Result<(), ConfigError> {
        self.update_checked(|config| config.obstruction_sensitivity = value)
    }

    pub fn set_low_obstruction_profile(
        &mut self,
        value: ObstructionProfileConfig,
    ) -> Result<(), ConfigError> {
        self.update_checked(|config| config.low_obstruction_profile = value)
    }

    pub fn set_medium_obstruction_profile(
        &mut self,
        value: ObstructionProfileConfig,
    ) -> Result<(), ConfigError> {
        self.update_checked(|config| config.medium_obstruction_profile = value)
    }

    pub fn set_high_obstruction_profile(
        &mut self,
        value: ObstructionProfileConfig,
    ) -> Result<(), ConfigError> {
        self.update_checked(|config| config.high_obstruction_profile = value)
    }

    pub fn set_override_unlock_timeout(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.override_unlock_timeout = value)
    }

    pub fn set_mqtt_status_publish_interval(&mut self, value: Duration) -> Result<(), ConfigError> {
        self.update_checked(|config| config.mqtt_status_publish_interval = value)
    }

    pub fn move_timeout(self) -> Duration {
        self.move_timeout
    }

    pub fn obstruction_sample_window(self) -> Duration {
        self.obstruction_sample_window
    }

    pub fn obstruction_warmup_duration(self) -> Duration {
        self.obstruction_warmup_duration
    }

    pub fn homing_poll_interval(self) -> Duration {
        self.homing_poll_interval
    }

    pub fn homing_start_timeout(self) -> Duration {
        self.homing_start_timeout
    }

    pub fn homing_stall_timeout(self) -> Duration {
        self.homing_stall_timeout
    }
}

impl From<DeskConfig> for DeskConfigParts {
    fn from(config: DeskConfig) -> Self {
        Self {
            target_tolerance: config.target_tolerance,
            target_slow_zone: config.target_slow_zone,
            move_timeout: config.move_timeout,
            obstruction_sample_window: config.obstruction_sample_window,
            obstruction_warmup_duration: config.obstruction_warmup_duration,
            obstruction_warmup_counts: config.obstruction_warmup_counts,
            min_move_duty: config.min_move_duty,
            move_run_duty: config.move_run_duty,
            move_slow_duty: config.move_slow_duty,
            move_sync_duty_step: config.move_sync_duty_step,
            homing_poll_interval: config.homing_poll_interval,
            homing_start_timeout: config.homing_start_timeout,
            homing_stall_timeout: config.homing_stall_timeout,
            homing_backoff_steps: config.homing_backoff_steps,
            homing_run_duty: config.homing_run_duty,
            homing_sync_duty_step: config.homing_sync_duty_step,
            sync_speedup_enter_counts: config.sync_speedup_enter_counts,
            sync_speedup_exit_counts: config.sync_speedup_exit_counts,
            catch_up_enter_counts: config.catch_up_enter_counts,
            catch_up_exit_counts: config.catch_up_exit_counts,
            fault_skew_counts: config.fault_skew_counts,
            homing_fault_skew_counts: config.homing_fault_skew_counts,
            obstruction_sensitivity: config.obstruction_sensitivity,
            low_obstruction_profile: config.low_obstruction_profile,
            medium_obstruction_profile: config.medium_obstruction_profile,
            high_obstruction_profile: config.high_obstruction_profile,
            override_unlock_timeout: config.override_unlock_timeout,
            mqtt_status_publish_interval: config.mqtt_status_publish_interval,
        }
    }
}

impl Default for DeskConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl ObstructionProfileConfig {
    /// Build an obstruction profile that requires at least one slow window.
    pub const fn new(minimum_baseline_percent: Percent, consecutive_windows: u8) -> Self {
        assert!(
            consecutive_windows > 0,
            "obstruction profile requires at least one window"
        );
        Self {
            minimum_baseline_percent,
            consecutive_windows,
        }
    }

    pub const fn minimum_baseline_percent(self) -> Percent {
        self.minimum_baseline_percent
    }

    pub const fn consecutive_windows(self) -> u8 {
        self.consecutive_windows
    }

    const fn is_valid(self) -> bool {
        // Zero percent makes the obstruction threshold zero, disabling detection.
        self.minimum_baseline_percent.get() > 0
            // At least one slow window is needed to declare obstruction.
            && self.consecutive_windows > 0
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use super::*;

    #[test]
    fn rejects_target_slow_zone_below_tolerance() {
        let mut config = DeskConfig::default();
        assert_eq!(
            config.set_target_slow_zone(CountDelta::new(1)),
            Err(ConfigError::InvalidDeskConfig)
        );
    }

    #[test]
    fn rejects_negative_sync_trim() {
        let mut config = DeskConfig::default();
        assert_eq!(
            config.set_move_sync_duty_step(DutyPercentTrim::new(-1)),
            Err(ConfigError::InvalidDeskConfig)
        );
    }

    #[test]
    fn rejects_disordered_sync_thresholds() {
        let mut config = DeskConfig::default();
        assert_eq!(
            config.set_catch_up_exit_counts(CountDelta::new(31)),
            Err(ConfigError::InvalidDeskConfig)
        );
    }

    #[test]
    fn rejects_zero_obstruction_profile_percent() {
        let mut config = DeskConfig::default();
        let profile = ObstructionProfileConfig::new(Percent::new(0), 2);
        assert_eq!(
            config.set_high_obstruction_profile(profile),
            Err(ConfigError::InvalidDeskConfig)
        );
    }

    #[test]
    fn rejects_unordered_obstruction_profiles() {
        let mut config = DeskConfig::default();
        let profile = ObstructionProfileConfig::new(Percent::new(90), 3);
        assert_eq!(
            config.set_low_obstruction_profile(profile),
            Err(ConfigError::InvalidDeskConfig)
        );
    }

    #[test]
    fn rejects_invalid_parts() {
        let mut parts = DeskConfigParts::from(DeskConfig::default());
        parts.target_tolerance = CountDelta::new(20);
        parts.target_slow_zone = CountDelta::new(10);

        assert_eq!(
            DeskConfig::from_parts(parts),
            Err(ConfigError::InvalidDeskConfig)
        );
    }
}

const fn obstruction_detection_time_ms_for_windows(
    config: DeskConfig,
    consecutive_windows: u8,
) -> u64 {
    config.obstruction_warmup_duration.as_millis()
        + config.obstruction_sample_window.as_millis() * consecutive_windows as u64
}

const fn default_obstruction_detection_time_ms(config: DeskConfig) -> u64 {
    let low = obstruction_detection_time_ms_from_sensitivity(config, ObstructionSensitivity::Low);
    let medium =
        obstruction_detection_time_ms_from_sensitivity(config, ObstructionSensitivity::Medium);
    let high = obstruction_detection_time_ms_from_sensitivity(config, ObstructionSensitivity::High);

    if low >= medium && low >= high {
        low
    } else if medium >= high {
        medium
    } else {
        high
    }
}

const fn obstruction_detection_time_ms_from_sensitivity(
    config: DeskConfig,
    sensitivity: ObstructionSensitivity,
) -> u64 {
    match config.obstruction_profile(sensitivity) {
        Some(profile) => {
            obstruction_detection_time_ms_for_windows(config, profile.consecutive_windows())
        }
        None => 0,
    }
}

const fn obstruction_profiles_ordered(
    low: ObstructionProfileConfig,
    medium: ObstructionProfileConfig,
    high: ObstructionProfileConfig,
) -> bool {
    low.minimum_baseline_percent().get() <= medium.minimum_baseline_percent().get()
        && medium.minimum_baseline_percent().get() <= high.minimum_baseline_percent().get()
        && low.consecutive_windows() >= medium.consecutive_windows()
        && medium.consecutive_windows() >= high.consecutive_windows()
}
