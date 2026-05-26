//! Runtime configuration for desk behavior, individual leg behavior, and the
//! shared in-memory config state.
//!
//! `desk` owns all desk-level movement, homing, synchronization, override, and
//! obstruction settings. Obstruction sensitivity/profile types live there too
//! because they directly shape desk movement safety behavior.
//!
//! `leg` owns the per-leg runtime settings used by the leg drive and movement
//! code: duty limits, homing timing, startup boost thresholds, target tolerance,
//! and default travel bounds.
//!
//! `runtime` combines the desk and leg settings into the active firmware config
//! and provides the shared storage/reader types used by MQTT, the controller,
//! and motion code. Callers should import config types from this module rather
//! than from the private child modules.

use defmt::Format;

mod desk;
mod leg;
mod runtime;

pub use desk::{DeskConfig, ObstructionProfileConfig, ObstructionSensitivity};
pub use leg::LegRuntimeConfig;
pub use runtime::{RuntimeConfig, RuntimeConfigReader, RuntimeConfigState, RuntimeConfigStorage};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum ConfigError {
    InvalidDeskConfig,
    InvalidLegConfig,
}
