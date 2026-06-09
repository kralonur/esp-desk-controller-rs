use alloc::{format, string::String};

use rust_mqtt::client::event::Event;

use crate::{
    config::{ConfigError, RuntimeConfigState},
    controller::{DeskControllerState, StopSubmission},
    persistent_config::RuntimeConfigPersistence,
    units::{PositionCounts, RelativeCounts},
};

use super::{
    config_api::{
        config_command_error_response, config_command_response, config_reset_response,
        config_set_response, erase_persisted_config, handle_config_set, save_or_defer_config,
    },
    parsing::{parse_i32, parse_override_steps},
    responses::{
        bool_name, config_error_name, controller_mode_name, fault_name, leg_side_name,
        override_direction_name, override_response_body, override_submission_response,
        response_body, submission_response,
    },
    settings::{MQTT_MAX_SUBSCRIPTION_IDENTIFIERS, MqttSettings},
    topics::{CommandTopic, IncomingTopic, OverrideTopic},
};

pub(super) struct IncomingResponse {
    pub(super) response: String,
    pub(super) publish_status: bool,
    pub(super) publish_config: bool,
}

pub(super) fn decode_command_event(
    state: &DeskControllerState,
    runtime_config_state: &RuntimeConfigState,
    runtime_config_persistence: Option<&mut RuntimeConfigPersistence>,
    settings: &MqttSettings,
    event: Event<'_, MQTT_MAX_SUBSCRIPTION_IDENTIFIERS>,
) -> Option<IncomingResponse> {
    let publish = match event {
        Event::Publish(publish) => publish,
        _ => return None,
    };

    let topic = publish.topic.as_ref().as_str();
    let payload = core::str::from_utf8(publish.message.as_ref()).ok()?.trim();
    let incoming = settings.parse_incoming_topic(topic)?;
    Some(handle_incoming_topic(
        state,
        runtime_config_state,
        runtime_config_persistence,
        incoming,
        payload,
    ))
}

pub(super) fn handle_incoming_topic(
    state: &DeskControllerState,
    runtime_config_state: &RuntimeConfigState,
    runtime_config_persistence: Option<&mut RuntimeConfigPersistence>,
    topic: IncomingTopic<'_>,
    payload: &str,
) -> IncomingResponse {
    match topic {
        IncomingTopic::Command(command) => IncomingResponse {
            response: handle_motion_command(state, command, payload),
            publish_status: true,
            publish_config: false,
        },
        IncomingTopic::Override(command) => IncomingResponse {
            response: handle_override_command(state, command, payload),
            publish_status: true,
            publish_config: false,
        },
        IncomingTopic::ConfigGet => IncomingResponse {
            response: config_command_response("get", "published"),
            publish_status: false,
            publish_config: true,
        },
        IncomingTopic::ConfigReset => {
            if !state.can_update_hardware_config() {
                return IncomingResponse {
                    response: config_command_error_response("reset", "rejected_not_idle_unhomed"),
                    publish_status: false,
                    publish_config: false,
                };
            }

            let persist_result = erase_persisted_config(runtime_config_persistence);
            let result = if persist_result.is_ok() {
                runtime_config_state.replace(Default::default())
            } else {
                Err(ConfigError::InvalidDeskConfig)
            };
            IncomingResponse {
                response: match result {
                    Ok(_) => config_reset_response("updated", persist_result.ok()),
                    Err(error) => {
                        let result = if persist_result.is_err() {
                            "persist_failed"
                        } else {
                            config_error_name(error)
                        };
                        config_command_error_response("reset", result)
                    }
                },
                publish_status: true,
                publish_config: result.is_ok(),
            }
        }
        IncomingTopic::ConfigSet(field_path) => {
            let result = handle_config_set(state, runtime_config_state, field_path, payload);
            let persist_result = if result.is_ok() {
                Some(save_or_defer_config(
                    state,
                    runtime_config_state,
                    runtime_config_persistence,
                ))
            } else {
                None
            };
            IncomingResponse {
                response: config_set_response(field_path, result, persist_result),
                publish_status: result.is_ok(),
                publish_config: result.is_ok(),
            }
        }
    }
}

fn handle_motion_command(
    state: &DeskControllerState,
    topic: CommandTopic,
    payload: &str,
) -> String {
    match topic {
        CommandTopic::Home => {
            submission_response(topic.response_name(), state.submit_home(), state.snapshot())
        }
        CommandTopic::ForceHome => submission_response(
            topic.response_name(),
            state.submit_force_home(),
            state.snapshot(),
        ),
        CommandTopic::Stop => match state.submit_stop() {
            StopSubmission::Accepted => {
                response_body(topic.response_name(), "accepted", state.snapshot())
            }
            StopSubmission::IgnoredIdle => {
                response_body(topic.response_name(), "ignored_idle", state.snapshot())
            }
        },
        CommandTopic::Up => match parse_i32(payload) {
            Some(steps) => match steps.checked_abs() {
                Some(steps) => submission_response(
                    topic.response_name(),
                    state.submit_move_by(RelativeCounts::new(steps)),
                    state.snapshot(),
                ),
                None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
            },
            None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
        },
        CommandTopic::Down => match parse_i32(payload) {
            Some(steps) => match steps.checked_abs() {
                Some(steps) => submission_response(
                    topic.response_name(),
                    state.submit_move_by(RelativeCounts::new(-steps)),
                    state.snapshot(),
                ),
                None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
            },
            None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
        },
        CommandTopic::MoveTo => match parse_i32(payload) {
            Some(position) => submission_response(
                topic.response_name(),
                state.submit_move_to(PositionCounts::new(position)),
                state.snapshot(),
            ),
            None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
        },
        CommandTopic::MoveBy => match parse_i32(payload) {
            Some(delta) => submission_response(
                topic.response_name(),
                state.submit_move_by(RelativeCounts::new(delta)),
                state.snapshot(),
            ),
            None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
        },
    }
}

fn handle_override_command(
    state: &DeskControllerState,
    topic: OverrideTopic,
    payload: &str,
) -> String {
    match topic {
        OverrideTopic::Unlock => {
            if payload != "UNLOCK" {
                return override_response_body(
                    "override_unlock",
                    "invalid_payload",
                    state.snapshot(),
                    state.override_unlocked(),
                );
            }

            let timeout = state.unlock_override();
            format!(
                "command=override_unlock\nresult=accepted\nmode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\noverride_unlocked=true\noverride_unlock_timeout_ms={}\n",
                controller_mode_name(state.snapshot().mode),
                bool_name(state.snapshot().command_pending),
                bool_name(state.snapshot().stop_requested),
                fault_name(state.snapshot().last_fault),
                timeout.as_millis(),
            )
        }
        OverrideTopic::Lock => {
            state.lock_override();
            override_response_body("override_lock", "accepted", state.snapshot(), false)
        }
        OverrideTopic::LegHome(side) => override_submission_response(
            "override_leg_home",
            state.submit_override_home(side),
            state.snapshot(),
            state.override_unlocked(),
            Some(side),
            None,
        ),
        OverrideTopic::LegMove(side, direction) => match parse_override_steps(payload) {
            Ok(steps) => override_submission_response(
                "override_leg_move",
                state.submit_override_move(side, direction, steps),
                state.snapshot(),
                state.override_unlocked(),
                Some(side),
                Some(direction),
            ),
            Err(error) => {
                let mut body = override_response_body(
                    "override_leg_move",
                    error,
                    state.snapshot(),
                    state.override_unlocked(),
                );
                body.push_str("leg=");
                body.push_str(leg_side_name(side));
                body.push('\n');
                body.push_str("direction=");
                body.push_str(override_direction_name(direction));
                body.push('\n');
                body
            }
        },
    }
}
