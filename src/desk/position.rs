use crate::quadrature::QuadratureDirection;

pub(super) const fn saturating_i32_from_i64(value: i64) -> i32 {
    if value > i32::MAX as i64 {
        i32::MAX
    } else if value < i32::MIN as i64 {
        i32::MIN
    } else {
        value as i32
    }
}

pub(super) const fn position_delta(end_position: i32, start_position: i32) -> i32 {
    saturating_i32_from_i64(end_position as i64 - start_position as i64)
}

pub(super) const fn abs_position_delta(left_position: i32, right_position: i32) -> i32 {
    let delta = left_position as i64 - right_position as i64;
    if delta == i64::MIN {
        i32::MAX
    } else {
        saturating_i32_from_i64(if delta < 0 { -delta } else { delta })
    }
}

pub(super) const fn average_position(left_position: i32, right_position: i32) -> i32 {
    ((left_position as i64 + right_position as i64) / 2) as i32
}

pub(super) fn progressed_in_direction(
    last_position: i32,
    current_position: i32,
    direction: QuadratureDirection,
) -> bool {
    match direction {
        QuadratureDirection::Positive => current_position > last_position,
        QuadratureDirection::Negative => current_position < last_position,
        QuadratureDirection::Invalid => false,
    }
}

pub(super) fn travel_in_direction(
    start_position: i32,
    current_position: i32,
    direction: QuadratureDirection,
) -> i32 {
    match direction {
        QuadratureDirection::Positive => position_delta(current_position, start_position).max(0),
        QuadratureDirection::Negative => position_delta(start_position, current_position).max(0),
        QuadratureDirection::Invalid => 0,
    }
}
