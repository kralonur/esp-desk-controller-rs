use embassy_time::Duration;

use crate::config::ConfigError;
use crate::units::{CountDelta, DutyPercent, DutyPercentTrim, Percent};

const DEFAULT_DESK_TARGET_TOLERANCE: CountDelta = CountDelta::new(5);
const DEFAULT_DESK_TARGET_SLOW_ZONE: CountDelta = CountDelta::new(10);
const DEFAULT_DESK_MOVE_TIMEOUT: Duration = Duration::from_millis(1_200);
const DEFAULT_OBSTRUCTION_SAMPLE_WINDOW: Duration = Duration::from_millis(150);
const DEFAULT_OBSTRUCTION_WARMUP_DURATION: Duration = Duration::from_millis(300);
const DEFAULT_OBSTRUCTION_WARMUP_COUNTS: CountDelta = CountDelta::new(8);
const DEFAULT_MIN_MOVE_DUTY: DutyPercent = DutyPercent::new(15);
const DEFAULT_MOVE_RUN_DUTY: DutyPercent = DutyPercent::new(30);
const DEFAULT_MOVE_SLOW_DUTY: DutyPercent = DutyPercent::new(17);
const DEFAULT_MOVE_SYNC_DUTY_STEP: DutyPercentTrim = DutyPercentTrim::new(8);
const DEFAULT_DESK_HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
const DEFAULT_DESK_HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
const DEFAULT_DESK_HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const DEFAULT_DESK_HOMING_BACKOFF_STEPS: CountDelta = CountDelta::new(20);
const DEFAULT_HOMING_RUN_DUTY: DutyPercent = DutyPercent::new(20);
const DEFAULT_HOMING_SYNC_DUTY_STEP: DutyPercentTrim = DutyPercentTrim::new(4);
const DEFAULT_SYNC_SPEEDUP_ENTER_COUNTS: CountDelta = CountDelta::new(10);
const DEFAULT_SYNC_SPEEDUP_EXIT_COUNTS: CountDelta = CountDelta::new(4);
const DEFAULT_CATCH_UP_ENTER_COUNTS: CountDelta = CountDelta::new(30);
const DEFAULT_CATCH_UP_EXIT_COUNTS: CountDelta = CountDelta::new(12);
const DEFAULT_FAULT_SKEW_COUNTS: CountDelta = CountDelta::new(80);
const DEFAULT_HOMING_FAULT_SKEW_COUNTS: CountDelta = CountDelta::new(160);
const DEFAULT_OBSTRUCTION_SENSITIVITY: ObstructionSensitivity = ObstructionSensitivity::High;
const DEFAULT_LOW_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(55), 3);
const DEFAULT_MEDIUM_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(70), 2);
const DEFAULT_HIGH_OBSTRUCTION_PROFILE: ObstructionProfileConfig =
    ObstructionProfileConfig::new(Percent::new(80), 2);
const DEFAULT_OVERRIDE_UNLOCK_TIMEOUT: Duration = Duration::from_millis(120_000);
const DEFAULT_MQTT_STATUS_PUBLISH_INTERVAL: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
/// Obstruction detection profile selected for coordinated desk movement.
pub enum ObstructionSensitivity {
    None,
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Thresholds used by obstruction detection.
pub struct ObstructionProfileConfig {
    minimum_baseline_percent: Percent,
    consecutive_windows: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Runtime settings for coordinated desk behavior.
///
/// Validation keeps the move timeout longer than the obstruction detection
/// window and requires nonzero operator-facing timeouts/intervals.
pub struct DeskConfig {
    target_tolerance: CountDelta,
    target_slow_zone: CountDelta,
    move_timeout: Duration,
    obstruction_sample_window: Duration,
    obstruction_warmup_duration: Duration,
    obstruction_warmup_counts: CountDelta,
    min_move_duty: DutyPercent,
    move_run_duty: DutyPercent,
    move_slow_duty: DutyPercent,
    move_sync_duty_step: DutyPercentTrim,
    homing_poll_interval: Duration,
    homing_start_timeout: Duration,
    homing_stall_timeout: Duration,
    homing_backoff_steps: CountDelta,
    homing_run_duty: DutyPercent,
    homing_sync_duty_step: DutyPercentTrim,
    sync_speedup_enter_counts: CountDelta,
    sync_speedup_exit_counts: CountDelta,
    catch_up_enter_counts: CountDelta,
    catch_up_exit_counts: CountDelta,
    fault_skew_counts: CountDelta,
    homing_fault_skew_counts: CountDelta,
    obstruction_sensitivity: ObstructionSensitivity,
    low_obstruction_profile: ObstructionProfileConfig,
    medium_obstruction_profile: ObstructionProfileConfig,
    high_obstruction_profile: ObstructionProfileConfig,
    override_unlock_timeout: Duration,
    mqtt_status_publish_interval: Duration,
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

    pub const fn validate(self) -> Result<Self, ConfigError> {
        if self.is_valid() {
            Ok(self)
        } else {
            Err(ConfigError::InvalidDeskConfig)
        }
    }

    const fn is_valid(self) -> bool {
        self.move_timeout.as_millis() > default_obstruction_detection_time_ms(self)
            && self.override_unlock_timeout.as_millis() > 0
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

impl Default for DeskConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl ObstructionProfileConfig {
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
