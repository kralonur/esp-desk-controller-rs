use alloc::{format, string::String};

use crate::{
    config::{ConfigError, ObstructionProfileConfig, ObstructionSensitivity, RuntimeConfigState},
    controller::DeskControllerState,
    persistent_config::RuntimeConfigPersistence,
};
use defmt::warn;

use super::{
    commands::IncomingResponse,
    parsing::{
        parse_count_delta, parse_duration_ms, parse_duty_percent, parse_duty_trim,
        parse_obstruction_sensitivity, parse_percent, parse_position_counts, parse_windows,
    },
    responses::config_error_name,
};

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum ConfigPersistResult {
    Disabled,
    Saved,
    Deferred,
    Erased,
    Failed,
}

pub(super) fn config_command_response(command: &'static str, result: &'static str) -> String {
    format!("command=config_{}\nresult={}\n", command, result)
}

pub(super) fn config_command_error_response(command: &'static str, result: &'static str) -> String {
    config_command_response(command, result)
}

pub(super) fn config_reset_response(
    result: &'static str,
    persist: Option<ConfigPersistResult>,
) -> String {
    format!(
        "command=config_reset\nresult={}\npersist={}\n",
        result,
        persist_result_name(persist)
    )
}

pub(super) fn config_set_response(
    field_path: &str,
    result: Result<(), &'static str>,
    persist: Option<ConfigPersistResult>,
) -> String {
    let result_name = match result {
        Ok(()) => "updated",
        Err(error) => error,
    };

    format!(
        "command=config_set\nfield={}\nresult={}\npersist={}\napplies_on_next_command=true\n",
        field_path,
        result_name,
        persist_result_name(persist)
    )
}

pub(super) fn persist_result_name(result: Option<ConfigPersistResult>) -> &'static str {
    match result {
        Some(ConfigPersistResult::Disabled) => "disabled",
        Some(ConfigPersistResult::Saved) => "saved",
        Some(ConfigPersistResult::Deferred) => "deferred",
        Some(ConfigPersistResult::Erased) => "erased",
        Some(ConfigPersistResult::Failed) => "failed",
        None => "unchanged",
    }
}

pub(super) fn save_or_defer_config(
    state: &DeskControllerState,
    runtime_config_state: &RuntimeConfigState,
    persistence: Option<&mut RuntimeConfigPersistence>,
) -> ConfigPersistResult {
    let Some(persistence) = persistence else {
        return ConfigPersistResult::Disabled;
    };

    if !state.can_persist_config() {
        persistence.mark_dirty();
        return ConfigPersistResult::Deferred;
    }

    match persistence.save(runtime_config_state.current()) {
        Ok(()) => ConfigPersistResult::Saved,
        Err(error) => {
            warn!("runtime config save failed: {:?}", error);
            persistence.mark_dirty();
            ConfigPersistResult::Failed
        }
    }
}

pub(super) fn flush_deferred_config(
    state: &DeskControllerState,
    runtime_config_state: &RuntimeConfigState,
    persistence: Option<&mut RuntimeConfigPersistence>,
) -> Option<IncomingResponse> {
    let persistence = persistence?;
    if !persistence.is_dirty() || !state.can_persist_config() {
        return None;
    }

    let persist_result = save_or_defer_config(state, runtime_config_state, Some(persistence));
    Some(IncomingResponse {
        response: format!(
            "command=config_persist\nresult={}\npersist={}\n",
            if matches!(persist_result, ConfigPersistResult::Saved) {
                "updated"
            } else {
                "persist_failed"
            },
            persist_result_name(Some(persist_result))
        ),
        publish_status: false,
        publish_config: matches!(persist_result, ConfigPersistResult::Saved),
    })
}

pub(super) fn erase_persisted_config(
    persistence: Option<&mut RuntimeConfigPersistence>,
) -> Result<ConfigPersistResult, ()> {
    let Some(persistence) = persistence else {
        return Ok(ConfigPersistResult::Disabled);
    };

    match persistence.erase() {
        Ok(()) => Ok(ConfigPersistResult::Erased),
        Err(error) => {
            warn!("runtime config erase failed: {:?}", error);
            Err(())
        }
    }
}

pub(super) fn handle_config_set(
    runtime_config_state: &RuntimeConfigState,
    field_path: &str,
    payload: &str,
) -> Result<(), &'static str> {
    match field_path {
        "desk/target_tolerance" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_target_tolerance(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/target_slow_zone" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_target_slow_zone(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/move_timeout_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_move_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/obstruction_sample_window_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_obstruction_sample_window(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/obstruction_warmup_duration_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_obstruction_warmup_duration(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/obstruction_warmup_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_obstruction_warmup_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/min_move_duty" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_min_move_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "desk/move_run_duty" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_move_run_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "desk/move_slow_duty" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_move_slow_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "desk/move_sync_duty_step" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_move_sync_duty_step(parse_duty_trim(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_poll_interval_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_poll_interval(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_start_timeout_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_start_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_stall_timeout_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_stall_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_backoff_steps" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_backoff_steps(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_run_duty" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_run_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_sync_duty_step" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_sync_duty_step(parse_duty_trim(payload)?)
                .map_err(config_error_name)
        }),
        "desk/sync_speedup_enter_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_sync_speedup_enter_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/sync_speedup_exit_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_sync_speedup_exit_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/catch_up_enter_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_catch_up_enter_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/catch_up_exit_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_catch_up_exit_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/fault_skew_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_fault_skew_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/homing_fault_skew_counts" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_homing_fault_skew_counts(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "desk/obstruction_sensitivity" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_obstruction_sensitivity(parse_obstruction_sensitivity(payload)?)
                .map_err(config_error_name)
        }),
        "desk/low_obstruction_min_percent" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::Low,
            payload,
            true,
        ),
        "desk/low_obstruction_windows" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::Low,
            payload,
            false,
        ),
        "desk/medium_obstruction_min_percent" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::Medium,
            payload,
            true,
        ),
        "desk/medium_obstruction_windows" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::Medium,
            payload,
            false,
        ),
        "desk/high_obstruction_min_percent" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::High,
            payload,
            true,
        ),
        "desk/high_obstruction_windows" => update_obstruction_profile(
            runtime_config_state,
            ObstructionSensitivity::High,
            payload,
            false,
        ),
        "desk/override_unlock_timeout_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_override_unlock_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "desk/mqtt_status_publish_interval_ms" => apply_desk_update(runtime_config_state, |desk| {
            desk.set_mqtt_status_publish_interval(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "leg/startup_duty" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_startup_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "leg/max_duty" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_max_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "leg/run_duty" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_run_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "leg/slow_duty" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_slow_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "leg/homing_duty" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_homing_duty(parse_duty_percent(payload)?)
                .map_err(config_error_name)
        }),
        "leg/startup_events" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_startup_events(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "leg/homing_start_timeout_ms" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_homing_start_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "leg/homing_stall_timeout_ms" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_homing_stall_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "leg/homing_poll_interval_ms" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_homing_poll_interval(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "leg/homing_backoff_steps" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_homing_backoff_steps(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "leg/default_max_position" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_default_max_position(parse_position_counts(payload)?)
                .map_err(config_error_name)
        }),
        "leg/move_stall_timeout_ms" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_move_stall_timeout(parse_duration_ms(payload)?)
                .map_err(config_error_name)
        }),
        "leg/target_slow_zone" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_target_slow_zone(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        "leg/target_tolerance" => apply_leg_update(runtime_config_state, |leg| {
            leg.set_target_tolerance(parse_count_delta(payload)?)
                .map_err(config_error_name)
        }),
        _ => Err("unknown_field"),
    }
}

fn update_obstruction_profile(
    runtime_config_state: &RuntimeConfigState,
    sensitivity: ObstructionSensitivity,
    payload: &str,
    update_percent: bool,
) -> Result<(), &'static str> {
    apply_desk_update(runtime_config_state, |desk| {
        let mut profile = desk
            .obstruction_profile(sensitivity)
            .ok_or(config_error_name(ConfigError::InvalidDeskConfig))?;
        if update_percent {
            profile = ObstructionProfileConfig::new(
                parse_percent(payload)?,
                profile.consecutive_windows(),
            );
        } else {
            profile = ObstructionProfileConfig::new(
                profile.minimum_baseline_percent(),
                parse_windows(payload)?,
            );
        }

        match sensitivity {
            ObstructionSensitivity::Low => desk
                .set_low_obstruction_profile(profile)
                .map_err(config_error_name),
            ObstructionSensitivity::Medium => desk
                .set_medium_obstruction_profile(profile)
                .map_err(config_error_name),
            ObstructionSensitivity::High => desk
                .set_high_obstruction_profile(profile)
                .map_err(config_error_name),
            ObstructionSensitivity::None => Err(config_error_name(ConfigError::InvalidDeskConfig)),
        }
    })
}

fn apply_desk_update(
    runtime_config_state: &RuntimeConfigState,
    update_fn: impl FnOnce(&mut crate::config::DeskConfig) -> Result<(), &'static str>,
) -> Result<(), &'static str> {
    apply_runtime_update(runtime_config_state, |config| config.update_desk(update_fn))
}

fn apply_leg_update(
    runtime_config_state: &RuntimeConfigState,
    update_fn: impl FnOnce(&mut crate::config::LegRuntimeConfig) -> Result<(), &'static str>,
) -> Result<(), &'static str> {
    apply_runtime_update(runtime_config_state, |config| config.update_leg(update_fn))
}

fn apply_runtime_update(
    runtime_config_state: &RuntimeConfigState,
    update_fn: impl FnOnce(&mut crate::config::RuntimeConfig) -> Result<(), &'static str>,
) -> Result<(), &'static str> {
    let mut inner_result = Ok(());
    let state_result = runtime_config_state.update(|config| {
        inner_result = update_fn(config);
    });

    match inner_result {
        Ok(()) => state_result.map(|_| ()).map_err(config_error_name),
        Err(error) => Err(error),
    }
}
