use defmt::Format;

use crate::desk::{DeskLegSide, OverrideLegDirection};
use crate::units::{CountDelta, PositionCounts, RelativeCounts};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Command consumed by the desk control loop.
pub enum DeskCommand {
    Home,
    ForceHome,
    MoveTo(PositionCounts),
    MoveBy(RelativeCounts),
    Override(OverrideCommand),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Manual override command for one selected desk leg.
pub enum OverrideCommand {
    Home {
        side: DeskLegSide,
    },
    Move {
        side: DeskLegSide,
        direction: OverrideLegDirection,
        steps: CountDelta,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Result of trying to queue a controller command.
pub enum CommandSubmission {
    Accepted,
    RejectedBusy,
    RejectedUnhomed,
    RejectedFaulted,
    RejectedLocked,
    RejectedOverrideUnlocked,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Result of trying to request a stop.
pub enum StopSubmission {
    Accepted,
    IgnoredIdle,
}
