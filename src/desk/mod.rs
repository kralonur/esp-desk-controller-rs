//! Coordinated two-leg desk control.
//!
//! The `desk` module is the boundary between the lower-level `leg` drivers and
//! the higher-level controller/MQTT command layer. It owns the desk-level
//! typestate API, combines two independent leg status streams into one desk
//! status stream, and exposes the operations that can change the desk's
//! physical state: homing, target movement, stopping, and manual override.
//!
//! The central invariant is that coordinated target movement is only available
//! after the desk has been homed. `Desk<UnhomedDesk, ...>` can home both legs
//! and can run manual override operations, while `Desk<ReadyDesk, ...>` can run
//! target moves. Faults that make the shared coordinate frame untrustworthy
//! convert the desk back to the unhomed typestate so the caller must rehome
//! before moving again.
//!
//! File layout:
//!
//! - `state`: The `Desk` container, typestate markers, shared leg ownership
//!   transitions, status accessors, and the tasks that mirror leg status into
//!   desk status.
//! - `status`: Observer-facing status data: motion state, stop reason, current
//!   leg positions/duties, computed average/skew, and effective travel limits.
//! - `homing`: The full two-leg homing sequence. It validates encoder polarity,
//!   synchronizes downward travel, detects stall/end-stop conditions, backs off,
//!   resets positions, and produces a ready desk.
//! - `movement`: Ready-state target movement. It validates the target, starts
//!   both ready legs, waits for encoder progress, applies sync planning, and
//!   maps completion/fault/stop cases into public move outcomes.
//! - `override_control`: Manual single-leg service/recovery operations. It
//!   stops the non-selected leg, moves or homes one side, and always leaves the
//!   desk needing a full rehome afterward.
//! - `planning`: Pure-ish drive planning: travel direction, sync phase changes,
//!   duty trimming, per-leg plans, and applying those plans to leg drive modes.
//! - `monitor`: Per-move validation state machines: progress direction checks,
//!   target snapshots, skew faults, and obstruction detection windows.
//! - `position`: Small encoder-direction helpers shared by homing and override
//!   flows.
//!
//! Keep public re-exports here stable for callers. New implementation details
//! should live with the behavior they support rather than in a catch-all types
//! or utilities module.
//!
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
