use core::cell::Cell;

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use static_cell::StaticCell;

use crate::config::{ConfigError, DeskConfig, LegRuntimeConfig};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeConfig {
    desk: DeskConfig,
    leg: LegRuntimeConfig,
}

pub struct RuntimeConfigStorage {
    state: StaticCell<RuntimeConfigState>,
}

pub struct RuntimeConfigState {
    config: Mutex<CriticalSectionRawMutex, Cell<RuntimeConfig>>,
}

#[derive(Clone, Copy)]
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
                Ok(_) => Ok(self),
                Err(error) => Err(error),
            },
            Err(error) => Err(error),
        }
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

    pub fn replace(&self, new_config: RuntimeConfig) -> Result<RuntimeConfig, ConfigError> {
        new_config.validate()?;
        Ok(self.config.lock(|config| {
            let previous = config.get();
            config.set(new_config);
            previous
        }))
    }

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
