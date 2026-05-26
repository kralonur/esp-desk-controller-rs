pub use crate::config::ObstructionSensitivity;

mod homing;
mod monitor;
mod movement;
mod override_control;
mod planning;
mod position;
mod state;
mod status;

pub use movement::{DeskMoveError, DeskMoveInvariant, DeskMoveOutcome};
pub use override_control::{DeskLegSide, DeskOverrideOutcome, OverrideLegDirection};
pub use state::{Desk, DeskError, ReadyDesk, UnhomedDesk};
pub use status::{
    DeskMotionState, DeskStatus, DeskStatusReader, DeskStatusStorage, DeskStatusWatcher,
    DeskStopReason,
};
