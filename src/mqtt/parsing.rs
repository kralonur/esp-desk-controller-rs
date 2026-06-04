use core::str::FromStr;

use embassy_time::Duration;

use crate::{
    config::{HomingObstructionSensitivity, ObstructionSensitivity},
    leg::DriveSide,
    quadrature::QuadratureDirection,
    units::{CountDelta, DutyPercent, DutyPercentTrim, Percent, PositionCounts},
};

pub(super) fn parse_i32(value: &str) -> Option<i32> {
    i32::from_str(value).ok()
}

pub(super) fn parse_position_counts(value: &str) -> Result<PositionCounts, &'static str> {
    parse_i32(value)
        .map(PositionCounts::new)
        .ok_or("invalid_payload")
}

pub(super) fn parse_count_delta(value: &str) -> Result<CountDelta, &'static str> {
    match u32::from_str(value) {
        Ok(value) => CountDelta::try_new(value).ok_or("invalid_payload"),
        Err(_) => Err("invalid_payload"),
    }
}

pub(super) fn parse_override_steps(value: &str) -> Result<CountDelta, &'static str> {
    match u32::from_str(value) {
        Ok(0) | Err(_) => Err("invalid_payload"),
        Ok(value) => CountDelta::try_new(value).ok_or("invalid_payload"),
    }
}

pub(super) fn parse_duty_percent(value: &str) -> Result<DutyPercent, &'static str> {
    match u8::from_str(value) {
        Ok(value) if value <= DutyPercent::MAX => Ok(DutyPercent::new(value)),
        _ => Err("invalid_payload"),
    }
}

pub(super) fn parse_duty_trim(value: &str) -> Result<DutyPercentTrim, &'static str> {
    i8::from_str(value)
        .map(DutyPercentTrim::new)
        .map_err(|_| "invalid_payload")
}

pub(super) fn parse_duration_ms(value: &str) -> Result<Duration, &'static str> {
    u64::from_str(value)
        .map(Duration::from_millis)
        .map_err(|_| "invalid_payload")
}

pub(super) fn parse_percent(value: &str) -> Result<Percent, &'static str> {
    match u8::from_str(value) {
        Ok(value) if value <= 100 => Ok(Percent::new(value)),
        _ => Err("invalid_payload"),
    }
}

pub(super) fn parse_windows(value: &str) -> Result<u8, &'static str> {
    match u8::from_str(value) {
        Ok(0) | Err(_) => Err("invalid_payload"),
        Ok(value) => Ok(value),
    }
}

pub(super) fn parse_obstruction_sensitivity(
    value: &str,
) -> Result<ObstructionSensitivity, &'static str> {
    match value {
        "none" => Ok(ObstructionSensitivity::None),
        "low" => Ok(ObstructionSensitivity::Low),
        "medium" => Ok(ObstructionSensitivity::Medium),
        "high" => Ok(ObstructionSensitivity::High),
        _ => Err("invalid_payload"),
    }
}

pub(super) fn parse_homing_obstruction_sensitivity(
    value: &str,
) -> Result<HomingObstructionSensitivity, &'static str> {
    match value {
        "off" => Ok(HomingObstructionSensitivity::Off),
        "on" => Ok(HomingObstructionSensitivity::On),
        _ => Err("invalid_payload"),
    }
}

pub(super) fn parse_drive_side(value: &str) -> Result<DriveSide, &'static str> {
    match value {
        "left" => Ok(DriveSide::Left),
        "right" => Ok(DriveSide::Right),
        _ => Err("invalid_payload"),
    }
}

pub(super) fn parse_quadrature_direction(value: &str) -> Result<QuadratureDirection, &'static str> {
    match value {
        "positive" => Ok(QuadratureDirection::Positive),
        "negative" => Ok(QuadratureDirection::Negative),
        _ => Err("invalid_payload"),
    }
}
