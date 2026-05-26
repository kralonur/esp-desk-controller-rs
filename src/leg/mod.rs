//! Single-leg motor and quadrature control.
//!
//! The `leg` module owns one motorized desk leg: motor drive direction,
//! quadrature position tracking, status publication, homing, and ready-state
//! movement within configured limits. `desk` composes two legs into coordinated
//! desk behavior.
//!
//! File layout:
//!
//! - `state`: the `Leg` container, typestate markers, construction, shared
//!   accessors, and typestate conversion.
//! - `status`: observer-facing leg status/progress snapshots, watchers, storage,
//!   and quadrature-to-status mirroring.
//! - `drive`: drive modes, drive side selection, duty limiting, motor commands,
//!   and small direction/travel helpers.
//! - `motion`: homing and ready-state movement methods plus movement errors.

mod drive;
mod motion;
mod state;
mod status;

pub use drive::{DriveMode, DriveSide};
pub use motion::LegError;
pub use state::{Leg, LegConfig, Ready, Unhomed};
pub use status::{
    LegProgress, LegProgressWatcher, LegStatus, LegStatusStorage, LegStatusWatcher, MotionState,
};
