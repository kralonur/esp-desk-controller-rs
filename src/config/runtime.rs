use core::cell::Cell;

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use static_cell::StaticCell;

use crate::config::{ConfigError, DeskConfig, LegRuntimeConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Complete runtime configuration snapshot.
pub struct RuntimeConfig {
    desk: DeskConfig,
    leg: LegRuntimeConfig,
}

/// Static storage used to initialize shared runtime config state.
pub struct RuntimeConfigStorage {
    state: StaticCell<RuntimeConfigState>,
}

/// Shared in-memory runtime configuration.
///
/// Updates are validated before being installed so readers never observe an
/// invalid config snapshot.
pub struct RuntimeConfigState {
    config: Mutex<CriticalSectionRawMutex, Cell<RuntimeConfig>>,
}

#[derive(Clone, Copy)]
/// Cheap copyable reader for the current runtime configuration.
pub struct RuntimeConfigReader {
    state: &'static RuntimeConfigState,
}

impl RuntimeConfig {
    pub const fn new() -> Self {
        Self {
            desk: DeskConfig::new(),
            leg: LegRuntimeConfig::new(),
        }
    }

    pub const fn validate(self) -> Result<Self, ConfigError> {
        match self.desk.validate() {
            Ok(_) => match self.leg.validate() {
                Ok(_) => {
                    if self.is_cross_valid() {
                        Ok(self)
                    } else {
                        Err(ConfigError::InvalidDeskConfig)
                    }
                }
                Err(error) => Err(error),
            },
            Err(error) => Err(error),
        }
    }

    const fn is_cross_valid(self) -> bool {
        let max_duty = self.leg.max_duty().get();
        let max_position = self.leg.default_max_position().get() as u32;

        // Leg drive clamps every requested desk duty at max_duty.
        self.desk.min_move_duty().get() <= max_duty
            // Avoid silent clamping during coordinated moves.
            && self.desk.move_run_duty().get() <= max_duty
            // Avoid silent clamping near the target.
            && self.desk.move_slow_duty().get() <= max_duty
            // Avoid silent clamping during coordinated homing.
            && self.desk.homing_run_duty().get() <= max_duty
            // Desk tolerance must fit inside ready travel range.
            && self.desk.target_tolerance().get() <= max_position
            // Desk slow zone must fit inside ready travel range.
            && self.desk.target_slow_zone().get() <= max_position
            // Obstruction warmup travel must be reachable.
            && self.desk.obstruction_warmup_counts().get() <= max_position
            // Coordinated homing backoff must be reachable.
            && self.desk.homing_backoff_steps().get() <= max_position
            // Speedup enter threshold must fit inside travel range.
            && self.desk.sync_speedup_enter_counts().get() <= max_position
            // Speedup exit threshold must fit inside travel range.
            && self.desk.sync_speedup_exit_counts().get() <= max_position
            // Catch-up enter threshold must fit inside travel range.
            && self.desk.catch_up_enter_counts().get() <= max_position
            // Catch-up exit threshold must fit inside travel range.
            && self.desk.catch_up_exit_counts().get() <= max_position
            // Ready-move skew fault threshold must fit inside travel range.
            && self.desk.fault_skew_counts().get() <= max_position
            // Homing skew fault threshold must fit inside travel range.
            && self.desk.homing_fault_skew_counts().get() <= max_position
    }

    pub const fn from_parts(desk: DeskConfig, leg: LegRuntimeConfig) -> Result<Self, ConfigError> {
        Self { desk, leg }.validate()
    }

    pub const fn desk(self) -> DeskConfig {
        self.desk
    }

    pub const fn leg(self) -> LegRuntimeConfig {
        self.leg
    }

    /// Mutate desk config transactionally, reverting if validation fails.
    pub fn update_desk(
        &mut self,
        update_fn: impl FnOnce(&mut DeskConfig) -> Result<(), &'static str>,
    ) -> Result<(), &'static str> {
        let previous = self.desk;
        update_fn(&mut self.desk)?;
        if self.validate().is_ok() {
            Ok(())
        } else {
            self.desk = previous;
            Err("invalid_config")
        }
    }

    /// Mutate leg config transactionally, reverting if validation fails.
    pub fn update_leg(
        &mut self,
        update_fn: impl FnOnce(&mut LegRuntimeConfig) -> Result<(), &'static str>,
    ) -> Result<(), &'static str> {
        let previous = self.leg;
        update_fn(&mut self.leg)?;
        if self.validate().is_ok() {
            Ok(())
        } else {
            self.leg = previous;
            Err("invalid_config")
        }
    }
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for RuntimeConfigStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl RuntimeConfigStorage {
    pub const fn new() -> Self {
        Self {
            state: StaticCell::new(),
        }
    }

    pub fn init(&'static self, initial_config: RuntimeConfig) -> &'static RuntimeConfigState {
        assert!(initial_config.validate().is_ok(), "invalid runtime config");
        self.state.init(RuntimeConfigState::new(initial_config))
    }
}

impl RuntimeConfigState {
    fn new(initial_config: RuntimeConfig) -> Self {
        Self {
            config: Mutex::new(Cell::new(initial_config)),
        }
    }

    pub fn reader(&'static self) -> RuntimeConfigReader {
        RuntimeConfigReader { state: self }
    }

    pub fn current(&self) -> RuntimeConfig {
        self.config.lock(|config| config.get())
    }

    /// Replace the active config after validation and return the previous one.
    pub fn replace(&self, new_config: RuntimeConfig) -> Result<RuntimeConfig, ConfigError> {
        new_config.validate()?;
        Ok(self.config.lock(|config| {
            let previous = config.get();
            config.set(new_config);
            previous
        }))
    }

    /// Mutate the active config in place if the resulting snapshot validates.
    pub fn update(
        &self,
        update_fn: impl FnOnce(&mut RuntimeConfig),
    ) -> Result<RuntimeConfig, ConfigError> {
        self.config.lock(|config| {
            let mut current = config.get();
            update_fn(&mut current);
            match current.validate() {
                Ok(valid) => {
                    config.set(valid);
                    Ok(valid)
                }
                Err(error) => Err(error),
            }
        })
    }
}

impl RuntimeConfigReader {
    pub fn current(self) -> RuntimeConfig {
        self.state.current()
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use super::*;
    use crate::units::{CountDelta, DutyPercent};

    #[test]
    fn rejects_desk_duty_above_leg_max_duty() {
        let mut config = RuntimeConfig::default();
        assert!(
            config
                .update_leg(|leg| {
                    leg.set_startup_duty(DutyPercent::new(25))
                        .map_err(|_| "invalid_config")?;
                    leg.set_run_duty(DutyPercent::new(20))
                        .map_err(|_| "invalid_config")?;
                    leg.set_homing_duty(DutyPercent::new(20))
                        .map_err(|_| "invalid_config")?;
                    leg.set_max_duty(DutyPercent::new(25))
                        .map_err(|_| "invalid_config")
                })
                .is_err()
        );
    }

    #[test]
    fn rejects_desk_count_threshold_above_leg_travel() {
        let mut config = RuntimeConfig::default();
        assert!(
            config
                .update_desk(|desk| {
                    desk.set_homing_fault_skew_counts(CountDelta::new(3_000))
                        .map_err(|_| "invalid_config")
                })
                .is_err()
        );
    }
}
