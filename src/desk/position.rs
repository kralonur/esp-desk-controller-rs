use crate::{quadrature::QuadratureDirection, units::positive_position_delta};

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
        QuadratureDirection::Positive => positive_position_delta(current_position, start_position),
        QuadratureDirection::Negative => positive_position_delta(start_position, current_position),
        QuadratureDirection::Invalid => 0,
    }
}
