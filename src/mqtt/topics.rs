use crate::desk::{DeskLegSide, OverrideLegDirection};

#[derive(Clone, Copy)]
pub(super) enum IncomingTopic<'a> {
    Command(CommandTopic),
    Override(OverrideTopic),
    ConfigGet,
    ConfigReset,
    ConfigSet(&'a str),
}

#[derive(Clone, Copy)]
pub(super) enum CommandTopic {
    Home,
    ForceHome,
    Stop,
    Up,
    Down,
    MoveTo,
    MoveBy,
}

#[derive(Clone, Copy)]
pub(super) enum OverrideTopic {
    Unlock,
    Lock,
    LegHome(DeskLegSide),
    LegMove(DeskLegSide, OverrideLegDirection),
}

impl CommandTopic {
    pub(super) const ALL: [Self; 7] = [
        Self::Home,
        Self::ForceHome,
        Self::Stop,
        Self::Up,
        Self::Down,
        Self::MoveTo,
        Self::MoveBy,
    ];

    pub(super) const fn suffix(self) -> &'static str {
        match self {
            Self::Home => "cmd/home",
            Self::ForceHome => "cmd/force_home",
            Self::Stop => "cmd/stop",
            Self::Up => "cmd/up",
            Self::Down => "cmd/down",
            Self::MoveTo => "cmd/move_to",
            Self::MoveBy => "cmd/move_by",
        }
    }

    pub(super) const fn response_name(self) -> &'static str {
        match self {
            Self::Home => "home",
            Self::ForceHome => "force_home",
            Self::Stop => "stop",
            Self::Up => "up",
            Self::Down => "down",
            Self::MoveTo => "move_to",
            Self::MoveBy => "move_by",
        }
    }
}

pub(super) fn parse_override_topic(path: &str) -> Option<OverrideTopic> {
    match path {
        "unlock" => Some(OverrideTopic::Unlock),
        "lock" => Some(OverrideTopic::Lock),
        "leg/left/home" => Some(OverrideTopic::LegHome(DeskLegSide::Left)),
        "leg/right/home" => Some(OverrideTopic::LegHome(DeskLegSide::Right)),
        "leg/left/up" => Some(OverrideTopic::LegMove(
            DeskLegSide::Left,
            OverrideLegDirection::Up,
        )),
        "leg/left/down" => Some(OverrideTopic::LegMove(
            DeskLegSide::Left,
            OverrideLegDirection::Down,
        )),
        "leg/right/up" => Some(OverrideTopic::LegMove(
            DeskLegSide::Right,
            OverrideLegDirection::Up,
        )),
        "leg/right/down" => Some(OverrideTopic::LegMove(
            DeskLegSide::Right,
            OverrideLegDirection::Down,
        )),
        _ => None,
    }
}
