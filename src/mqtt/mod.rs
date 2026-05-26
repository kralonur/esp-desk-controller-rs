//! MQTT command and status bridge.
//!
//! This module owns the network-facing control surface for the desk. It connects
//! to the configured broker, subscribes to command/config/override topics,
//! translates MQTT payloads into `DeskControllerState` submissions, publishes
//! command responses, and republishes desk status/config snapshots.
//!
//! File layout:
//!
//! - `settings`: environment-derived broker/client/topic settings and MQTT
//!   string/topic conversion helpers.
//! - `session`: the MQTT task, reconnect loop, socket setup, subscriptions,
//!   and command/status/ping event loop.
//! - `publish`: the MQTT client alias and low-level status/config/response
//!   publication helpers.
//! - `topics`: command/config/override topic enums and incoming topic parsing.
//! - `commands`: publish-event decoding plus motion and override command
//!   handling.
//! - `config_api`: config get/reset/set handling and persistence save/defer
//!   behavior.
//! - `responses`: response/status/config body formatting and stable string
//!   names used on MQTT.
//! - `parsing`: payload parsers for command/config values.
//!
//! Keep the topic names, field paths, and response payload strings stable unless
//! the MQTT API is intentionally versioned.

mod commands;
mod config_api;
mod parsing;
mod publish;
mod responses;
mod session;
mod settings;
mod topics;

pub use session::{mqtt_task, spawn_mqtt};
pub use settings::MqttSetupError;
