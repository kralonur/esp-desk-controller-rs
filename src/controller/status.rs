use defmt::Format;

use crate::desk::{DeskError, DeskMoveInvariant};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// High-level controller mode published to observers.
pub enum DeskControllerMode {
    Unhomed,
    Ready,
    Homing,
    Moving,
    Override,
    Faulted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Fault remembered by the controller after a desk operation fails.
pub enum DeskFault {
    LeftLeg(crate::leg::LegError),
    RightLeg(crate::leg::LegError),
    MoveTimeout,
    SkewFault,
    InvariantViolation(DeskMoveInvariant),
    RehomeRequired,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
/// Snapshot of controller state used by MQTT responses and status publication.
pub struct DeskControllerSnapshot {
    pub mode: DeskControllerMode,
    pub command_pending: bool,
    pub stop_requested: bool,
    pub last_fault: Option<DeskFault>,
}

impl DeskFault {
    pub(super) fn from_desk_error(error: DeskError) -> Option<Self> {
        match error {
            DeskError::LeftLeg(error) => Some(Self::LeftLeg(error)),
            DeskError::RightLeg(error) => Some(Self::RightLeg(error)),
            DeskError::MoveTimeout => Some(Self::MoveTimeout),
            DeskError::SkewFault => Some(Self::SkewFault),
            DeskError::InvariantViolation(invariant) => Some(Self::InvariantViolation(invariant)),
            DeskError::RehomeRequired => Some(Self::RehomeRequired),
            DeskError::Stopped => None,
        }
    }
}
