use defmt::Format;

pub const PWM_TIMER_MAX_TICKS: u16 = 99;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct PositionCounts(i32);

impl PositionCounts {
    pub const fn new(value: i32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i32 {
        self.0
    }

    pub const fn clamp(self, min: Self, max: Self) -> Self {
        if self.0 < min.0 {
            min
        } else if self.0 > max.0 {
            max
        } else {
            self
        }
    }

    pub const fn saturating_add(self, delta: RelativeCounts) -> Self {
        Self(self.0.saturating_add(delta.0))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct RelativeCounts(i32);

impl RelativeCounts {
    pub const fn new(value: i32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i32 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct CountDelta(u16);

impl CountDelta {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }
    pub const fn get(self) -> u16 {
        self.0
    }

    pub const fn get_i32(self) -> i32 {
        self.0 as i32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct DutyPercent(u8);

impl DutyPercent {
    pub const MAX: u8 = 100;

    pub const fn new(value: u8) -> Self {
        assert!(value <= Self::MAX, "duty percent must be <= 100");
        Self(value)
    }

    pub const fn from_clamped_i32(value: i32) -> Self {
        let clamped = if value < 0 {
            0
        } else if value > Self::MAX as i32 {
            Self::MAX
        } else {
            value as u8
        };
        Self(clamped)
    }

    pub const fn get(self) -> u8 {
        self.0
    }

    pub const fn get_u16(self) -> u16 {
        self.0 as u16
    }

    pub const fn to_pwm_timestamp(self, max_timestamp: u16) -> u16 {
        let scaled = self.0 as u32 * max_timestamp as u32 + 50;
        (scaled / 100) as u16
    }

    pub const fn min(self, other: Self) -> Self {
        if self.0 <= other.0 { self } else { other }
    }

    pub const fn max(self, other: Self) -> Self {
        if self.0 >= other.0 { self } else { other }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct DutyPercentTrim(i8);

impl DutyPercentTrim {
    pub const fn new(value: i8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i8 {
        self.0
    }

    pub const fn get_i32(self) -> i32 {
        self.0 as i32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Format)]
pub struct Percent(u8);

impl Percent {
    pub const fn new(value: u8) -> Self {
        assert!(value <= 100, "percent must be <= 100");
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }

    pub const fn get_u32(self) -> u32 {
        self.0 as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum PositionSign {
    Positive,
    Negative,
}

impl PositionSign {
    pub const fn apply(self, raw_position: i32) -> i32 {
        match self {
            Self::Positive => raw_position,
            Self::Negative => raw_position.saturating_neg(),
        }
    }
}
