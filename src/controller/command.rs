use defmt::Format;

use crate::desk::{DeskLegSide, OverrideLegDirection};
use crate::units::{CountDelta, PositionCounts, RelativeCounts};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskCommand {
    Home,
    MoveTo(PositionCounts),
    MoveBy(RelativeCounts),
    Override(OverrideCommand),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
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
pub enum CommandSubmission {
    Accepted,
    RejectedBusy,
    RejectedUnhomed,
    RejectedFaulted,
    RejectedLocked,
    RejectedOverrideUnlocked,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopSubmission {
    Accepted,
    IgnoredIdle,
}
