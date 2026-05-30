//! Small typed wrappers for firmware units.
//!
//! These types keep raw encoder counts, relative movement counts, duty percent,
//! trim percent, and generic percent values from being mixed accidentally.

use defmt::Format;

pub const PWM_TIMER_MAX_TICKS: u16 = 99;

/// Clamp a widened encoder calculation back into the signed count range.
pub const fn saturating_i32_from_i64(value: i64) -> i32 {
    if value > i32::MAX as i64 {
        i32::MAX
    } else if value < i32::MIN as i64 {
        i32::MIN
    } else {
        value as i32
    }
}

/// Signed distance between two encoder positions, saturated on overflow.
pub const fn position_delta(end_position: i32, start_position: i32) -> i32 {
    saturating_i32_from_i64(end_position as i64 - start_position as i64)
}

/// Forward-only distance; movement opposite to the expected direction is zero.
pub const fn positive_position_delta(end_position: i32, start_position: i32) -> i32 {
    let delta = position_delta(end_position, start_position);
    if delta < 0 { 0 } else { delta }
}

/// Absolute encoder distance, saturated so `i32::MIN` cannot overflow.
pub const fn abs_position_delta(left_position: i32, right_position: i32) -> i32 {
    let delta = left_position as i64 - right_position as i64;
    if delta == i64::MIN {
        i32::MAX
    } else {
        saturating_i32_from_i64(if delta < 0 { -delta } else { delta })
    }
}

pub const fn average_position(left_position: i32, right_position: i32) -> i32 {
    ((left_position as i64 + right_position as i64) / 2) as i32
}

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
/// Non-negative encoder count distance.
///
/// Stored as `u32`, but capped at `i32::MAX` because motion code compares
/// thresholds against signed encoder deltas.
pub struct CountDelta(u32);

impl CountDelta {
    /// Largest count delta that can be safely converted to signed motion math.
    pub const MAX: u32 = i32::MAX as u32;

    pub const fn new(value: u32) -> Self {
        assert!(value <= Self::MAX, "count delta must fit in i32");
        Self(value)
    }

    pub const fn try_new(value: u32) -> Option<Self> {
        if value <= Self::MAX {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> u32 {
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

    /// Convert arbitrary signed trim math into the legal 0..=100 duty range.
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

    /// Scale percentage duty to a PWM timestamp with nearest-integer rounding.
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
