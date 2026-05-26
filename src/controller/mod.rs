//! Desk command coordination and externally visible controller state.
//!
//! File layout:
//!
//! - `command`: command and submission result types used by MQTT and the desk
//!   control task.
//! - `status`: controller mode, snapshot, and fault representation. This is the
//!   read-only status surface that MQTT publishes.
//! - `state`: mutable controller state: the command queue, stop request,
//!   override unlock timeout, submission policy, and lifecycle transitions
//!   driven by the desk control task.

mod command;
mod state;
mod status;

pub use command::{CommandSubmission, DeskCommand, OverrideCommand, StopSubmission};
pub use state::DeskControllerState;
pub use status::{DeskControllerMode, DeskControllerSnapshot, DeskFault};
