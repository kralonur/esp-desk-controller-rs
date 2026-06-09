use alloc::{format, string::String};
use core::{num::NonZero, str::FromStr};

use embassy_net::Ipv4Address;
use embassy_time::Duration;
use rust_mqtt::{
    client::{MqttError, options::ConnectOptions},
    config::{KeepAlive, SessionExpiryInterval},
    types::{MqttBinary, MqttString, TopicFilter, TopicName},
};

use super::topics::{CommandTopic, IncomingTopic, parse_override_topic};

// Required compile-time IPv4 broker address, for example `192.168.1.10`.
const MQTT_BROKER_ADDR: Option<&str> = option_env!("MQTT_BROKER_ADDR");
// Optional compile-time broker TCP port; defaults to `DEFAULT_MQTT_PORT`.
const MQTT_BROKER_PORT: Option<&str> = option_env!("MQTT_BROKER_PORT");
// Optional compile-time MQTT username for brokers that require authentication.
const MQTT_USERNAME: Option<&str> = option_env!("MQTT_USERNAME");
// Optional compile-time MQTT password for brokers that require authentication.
const MQTT_PASSWORD: Option<&str> = option_env!("MQTT_PASSWORD");
// Required compile-time client ID base; `-cmd` and `-pub` are appended internally.
const MQTT_CLIENT_ID: Option<&str> = option_env!("MQTT_CLIENT_ID");
// Required compile-time topic root used to build every command/status/config topic.
const MQTT_TOPIC_PREFIX: Option<&str> = option_env!("MQTT_TOPIC_PREFIX");
// Optional compile-time MQTT keepalive in seconds; ping interval is half of this.
const MQTT_KEEP_ALIVE_SECS: Option<&str> = option_env!("MQTT_KEEP_ALIVE_SECS");
// Optional compile-time delay between reconnect attempts after MQTT failures.
const MQTT_RECONNECT_DELAY_SECS: Option<&str> = option_env!("MQTT_RECONNECT_DELAY_SECS");

// Bytes reserved for each MQTT TCP socket buffer.
pub(super) const MQTT_TCP_BUFFER_SIZE: usize = 2048;
// Maximum topic filters subscribed by the command client.
pub(super) const MQTT_MAX_SUBSCRIBES: usize = 12;
// MQTT receive window advertised to the broker.
pub(super) const MQTT_RECEIVE_MAXIMUM: usize = 8;
// MQTT send window used by the client.
pub(super) const MQTT_SEND_MAXIMUM: usize = 8;
// Maximum MQTT v5 subscription identifiers tracked by the client.
pub(super) const MQTT_MAX_SUBSCRIPTION_IDENTIFIERS: usize = 4;
// Default broker TCP port when `MQTT_BROKER_PORT` is not set.
const DEFAULT_MQTT_PORT: u16 = 1883;
// Default MQTT keepalive, in seconds, when `MQTT_KEEP_ALIVE_SECS` is not set.
const DEFAULT_MQTT_KEEP_ALIVE_SECS: u64 = 30;
// Default reconnect delay, in seconds, when `MQTT_RECONNECT_DELAY_SECS` is not set.
const DEFAULT_MQTT_RECONNECT_DELAY_SECS: u64 = 5;

#[derive(Clone)]
pub(super) struct MqttSettings {
    // Broker IPv4 address parsed from `MQTT_BROKER_ADDR`.
    pub(super) broker_addr: Ipv4Address,
    // Broker TCP port parsed from `MQTT_BROKER_PORT` or `DEFAULT_MQTT_PORT`.
    pub(super) broker_port: u16,
    // Optional broker username from `MQTT_USERNAME`.
    username: Option<&'static str>,
    // Optional broker password from `MQTT_PASSWORD`.
    password: Option<&'static str>,
    // MQTT keepalive interval from `MQTT_KEEP_ALIVE_SECS`.
    keep_alive: Duration,
    // Ping cadence, derived as half of `keep_alive`.
    pub(super) ping_interval: Duration,
    // Delay before reconnect attempts after connection failure.
    pub(super) reconnect_delay: Duration,
    // Command client ID derived from `MQTT_CLIENT_ID`.
    command_client_id: String,
    // Publisher client ID derived from `MQTT_CLIENT_ID`.
    publisher_client_id: String,
    // Publish topic for current desk status.
    topic_status: String,
    // Publish topic for command/config responses.
    topic_response: String,
    // Publish topic for full runtime config snapshots.
    topic_config: String,
    // Subscribe topic used to request a config snapshot.
    topic_config_get: String,
    // Subscribe topic used to reset runtime config to defaults.
    topic_config_reset: String,
    // Subscribe wildcard topic for runtime config field updates.
    topic_config_set_all: String,
    // Subscribe topic for homing the full desk.
    topic_home: String,
    // Subscribe topic for trusting the current position as home without movement.
    topic_force_home: String,
    // Subscribe topic for stopping active desk motion.
    topic_stop: String,
    // Subscribe topic for moving the desk upward.
    topic_up: String,
    // Subscribe topic for moving the desk downward.
    topic_down: String,
    // Subscribe topic for moving the desk to an absolute encoder position.
    topic_move_to: String,
    // Subscribe topic for moving the desk by a relative encoder delta.
    topic_move_by: String,
    // Subscribe wildcard topic for manual override commands.
    topic_override_all: String,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
/// MQTT setup failures while loading environment-derived settings.
pub enum MqttSetupError {
    MissingBrokerAddr,
    MissingClientId,
    MissingTopicPrefix,
    InvalidBrokerAddr,
    InvalidBrokerPort,
    InvalidKeepAlive,
    InvalidReconnectDelay,
}

impl MqttSettings {
    pub(super) fn load() -> Result<Self, MqttSetupError> {
        let broker_addr =
            parse_ipv4_address(MQTT_BROKER_ADDR.ok_or(MqttSetupError::MissingBrokerAddr)?)?;
        let broker_port = parse_u16_env(
            MQTT_BROKER_PORT,
            DEFAULT_MQTT_PORT,
            MqttSetupError::InvalidBrokerPort,
        )?;
        let keep_alive_secs = parse_u64_env(
            MQTT_KEEP_ALIVE_SECS,
            DEFAULT_MQTT_KEEP_ALIVE_SECS,
            MqttSetupError::InvalidKeepAlive,
        )?;
        let keep_alive = Duration::from_secs(keep_alive_secs);
        let ping_interval = Duration::from_secs(core::cmp::max(1, keep_alive_secs / 2));
        let reconnect_delay = Duration::from_secs(parse_u64_env(
            MQTT_RECONNECT_DELAY_SECS,
            DEFAULT_MQTT_RECONNECT_DELAY_SECS,
            MqttSetupError::InvalidReconnectDelay,
        )?);
        let client_id = MQTT_CLIENT_ID.ok_or(MqttSetupError::MissingClientId)?;
        let topic_prefix = MQTT_TOPIC_PREFIX.ok_or(MqttSetupError::MissingTopicPrefix)?;

        Ok(Self {
            broker_addr,
            broker_port,
            username: MQTT_USERNAME,
            password: MQTT_PASSWORD,
            keep_alive,
            ping_interval,
            reconnect_delay,
            command_client_id: format!("{client_id}-cmd"),
            publisher_client_id: format!("{client_id}-pub"),
            topic_status: format!("{topic_prefix}/status"),
            topic_response: format!("{topic_prefix}/response"),
            topic_config: format!("{topic_prefix}/config"),
            topic_config_get: format!("{topic_prefix}/config/get"),
            topic_config_reset: format!("{topic_prefix}/config/reset"),
            topic_config_set_all: format!("{topic_prefix}/config/set/#"),
            topic_home: format!("{topic_prefix}/{}", CommandTopic::Home.suffix()),
            topic_force_home: format!("{topic_prefix}/{}", CommandTopic::ForceHome.suffix()),
            topic_stop: format!("{topic_prefix}/{}", CommandTopic::Stop.suffix()),
            topic_up: format!("{topic_prefix}/{}", CommandTopic::Up.suffix()),
            topic_down: format!("{topic_prefix}/{}", CommandTopic::Down.suffix()),
            topic_move_to: format!("{topic_prefix}/{}", CommandTopic::MoveTo.suffix()),
            topic_move_by: format!("{topic_prefix}/{}", CommandTopic::MoveBy.suffix()),
            topic_override_all: format!("{topic_prefix}/cmd/override/#"),
        })
    }

    pub(super) fn connect_options(&self) -> ConnectOptions<'_> {
        let mut options = ConnectOptions::new()
            .clean_start()
            .session_expiry_interval(SessionExpiryInterval::NeverEnd)
            .keep_alive(duration_to_keep_alive(self.keep_alive));

        if let Some(username) = self.username {
            options =
                options.user_name(MqttString::from_str(username).expect("mqtt username too long"));
        }

        if let Some(password) = self.password {
            options = options.password(
                MqttBinary::from_slice(password.as_bytes()).expect("mqtt password too long"),
            );
        }

        options
    }

    pub(super) fn command_client_id(&self) -> Result<MqttString<'_>, MqttError<'static>> {
        mqtt_string(&self.command_client_id)
    }

    pub(super) fn publisher_client_id(&self) -> Result<MqttString<'_>, MqttError<'static>> {
        mqtt_string(&self.publisher_client_id)
    }

    pub(super) fn status_topic_name(&self) -> Result<TopicName<'_>, MqttError<'static>> {
        topic_name(&self.topic_status)
    }

    pub(super) fn response_topic_name(&self) -> Result<TopicName<'_>, MqttError<'static>> {
        topic_name(&self.topic_response)
    }

    pub(super) fn config_topic_name(&self) -> Result<TopicName<'_>, MqttError<'static>> {
        topic_name(&self.topic_config)
    }

    pub(super) fn command_topic_filter(
        &self,
        command: CommandTopic,
    ) -> Result<TopicFilter<'_>, MqttError<'static>> {
        let topic = match command {
            CommandTopic::Home => &self.topic_home,
            CommandTopic::ForceHome => &self.topic_force_home,
            CommandTopic::Stop => &self.topic_stop,
            CommandTopic::Up => &self.topic_up,
            CommandTopic::Down => &self.topic_down,
            CommandTopic::MoveTo => &self.topic_move_to,
            CommandTopic::MoveBy => &self.topic_move_by,
        };

        topic_name(topic).map(Into::into)
    }

    pub(super) fn config_get_topic_filter(&self) -> Result<TopicFilter<'_>, MqttError<'static>> {
        topic_name(&self.topic_config_get).map(Into::into)
    }

    pub(super) fn config_reset_topic_filter(&self) -> Result<TopicFilter<'_>, MqttError<'static>> {
        topic_name(&self.topic_config_reset).map(Into::into)
    }

    pub(super) fn config_set_topic_filter(&self) -> Result<TopicFilter<'_>, MqttError<'static>> {
        topic_filter(&self.topic_config_set_all)
    }

    pub(super) fn override_topic_filter(&self) -> Result<TopicFilter<'_>, MqttError<'static>> {
        topic_filter(&self.topic_override_all)
    }

    pub(super) fn parse_incoming_topic<'a>(&'a self, topic: &'a str) -> Option<IncomingTopic<'a>> {
        let command = match topic {
            topic if topic == self.topic_home => Some(CommandTopic::Home),
            topic if topic == self.topic_force_home => Some(CommandTopic::ForceHome),
            topic if topic == self.topic_stop => Some(CommandTopic::Stop),
            topic if topic == self.topic_up => Some(CommandTopic::Up),
            topic if topic == self.topic_down => Some(CommandTopic::Down),
            topic if topic == self.topic_move_to => Some(CommandTopic::MoveTo),
            topic if topic == self.topic_move_by => Some(CommandTopic::MoveBy),
            _ => None,
        };

        if let Some(command) = command {
            return Some(IncomingTopic::Command(command));
        }

        if topic == self.topic_config_get {
            return Some(IncomingTopic::ConfigGet);
        }

        if topic == self.topic_config_reset {
            return Some(IncomingTopic::ConfigReset);
        }

        if let Some(path) = topic.strip_prefix(self.topic_override_all.strip_suffix('#')?) {
            return parse_override_topic(path).map(IncomingTopic::Override);
        }

        topic
            .strip_prefix(self.topic_config_set_all.strip_suffix('#')?)
            .map(IncomingTopic::ConfigSet)
    }
}

fn parse_ipv4_address(value: &str) -> Result<Ipv4Address, MqttSetupError> {
    let mut parts = [0u8; 4];
    let mut count = 0;

    for segment in value.split('.') {
        if count >= 4 {
            return Err(MqttSetupError::InvalidBrokerAddr);
        }

        parts[count] = u8::from_str(segment).map_err(|_| MqttSetupError::InvalidBrokerAddr)?;
        count += 1;
    }

    if count != 4 {
        return Err(MqttSetupError::InvalidBrokerAddr);
    }

    Ok(Ipv4Address::new(parts[0], parts[1], parts[2], parts[3]))
}

fn parse_u16_env(
    value: Option<&str>,
    default: u16,
    error: MqttSetupError,
) -> Result<u16, MqttSetupError> {
    match value {
        Some(value) => u16::from_str(value).map_err(|_| error),
        None => Ok(default),
    }
}

fn parse_u64_env(
    value: Option<&str>,
    default: u64,
    error: MqttSetupError,
) -> Result<u64, MqttSetupError> {
    match value {
        Some(value) => u64::from_str(value).map_err(|_| error),
        None => Ok(default),
    }
}

fn mqtt_string(value: &str) -> Result<MqttString<'_>, MqttError<'static>> {
    MqttString::from_str(value).map_err(|_| MqttError::Alloc)
}

fn topic_name(value: &str) -> Result<TopicName<'_>, MqttError<'static>> {
    let string = mqtt_string(value)?;
    Ok(TopicName::new_unchecked(string))
}

fn topic_filter(value: &str) -> Result<TopicFilter<'_>, MqttError<'static>> {
    let string = mqtt_string(value)?;
    TopicFilter::new(string).ok_or(MqttError::Alloc)
}

fn duration_to_keep_alive(duration: Duration) -> KeepAlive {
    match u16::try_from(duration.as_secs()) {
        Ok(0) | Err(_) => KeepAlive::Infinite,
        Ok(seconds) => {
            KeepAlive::Seconds(NonZero::new(seconds).expect("keep alive must be non-zero"))
        }
    }
}
