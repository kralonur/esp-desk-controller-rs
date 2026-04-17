use alloc::{format, string::String};
use core::num::NonZero;
use core::str::FromStr;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::{Ipv4Address, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use rust_mqtt::{
    Bytes,
    buffer::AllocBuffer,
    client::{
        Client, MqttError,
        event::Event,
        options::{ConnectOptions, PublicationOptions, SubscriptionOptions, TopicReference},
    },
    config::{KeepAlive, SessionExpiryInterval},
    types::{MqttBinary, MqttString, TopicFilter, TopicName},
};

use crate::{
    config::ObstructionSensitivity,
    controller::{
        CommandSubmission, DeskControllerMode, DeskControllerSnapshot, DeskControllerState,
        DeskFault, StopSubmission,
    },
    desk::{DeskMotionState, DeskStatusWatcher, DeskStopReason},
    units::{PositionCounts, RelativeCounts},
};

const MQTT_BROKER_ADDR: Option<&str> = option_env!("MQTT_BROKER_ADDR");
const MQTT_BROKER_PORT: Option<&str> = option_env!("MQTT_BROKER_PORT");
const MQTT_USERNAME: Option<&str> = option_env!("MQTT_USERNAME");
const MQTT_PASSWORD: Option<&str> = option_env!("MQTT_PASSWORD");
const MQTT_CLIENT_ID: Option<&str> = option_env!("MQTT_CLIENT_ID");
const MQTT_TOPIC_PREFIX: Option<&str> = option_env!("MQTT_TOPIC_PREFIX");
const MQTT_KEEP_ALIVE_SECS: Option<&str> = option_env!("MQTT_KEEP_ALIVE_SECS");
const MQTT_RECONNECT_DELAY_SECS: Option<&str> = option_env!("MQTT_RECONNECT_DELAY_SECS");

const MQTT_TCP_BUFFER_SIZE: usize = 2048;
const MQTT_MAX_SUBSCRIBES: usize = 8;
const MQTT_RECEIVE_MAXIMUM: usize = 8;
const MQTT_SEND_MAXIMUM: usize = 8;
const MQTT_MAX_SUBSCRIPTION_IDENTIFIERS: usize = 4;
const DEFAULT_MQTT_PORT: u16 = 1883;
const DEFAULT_MQTT_KEEP_ALIVE_SECS: u64 = 30;
const DEFAULT_MQTT_RECONNECT_DELAY_SECS: u64 = 5;

#[derive(Clone)]
struct MqttSettings {
    broker_addr: Ipv4Address,
    broker_port: u16,
    username: Option<&'static str>,
    password: Option<&'static str>,
    keep_alive: Duration,
    ping_interval: Duration,
    reconnect_delay: Duration,
    command_client_id: String,
    publisher_client_id: String,
    topic_status: String,
    topic_response: String,
    topic_home: String,
    topic_stop: String,
    topic_up: String,
    topic_down: String,
    topic_move_to: String,
    topic_move_by: String,
}

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum MqttSetupError {
    MissingBrokerAddr,
    MissingClientId,
    MissingTopicPrefix,
    InvalidBrokerAddr,
    InvalidBrokerPort,
    InvalidKeepAlive,
    InvalidReconnectDelay,
}

#[derive(Clone, Copy)]
enum CommandTopic {
    Home,
    Stop,
    Up,
    Down,
    MoveTo,
    MoveBy,
}

impl CommandTopic {
    const ALL: [Self; 6] = [
        Self::Home,
        Self::Stop,
        Self::Up,
        Self::Down,
        Self::MoveTo,
        Self::MoveBy,
    ];

    const fn suffix(self) -> &'static str {
        match self {
            Self::Home => "cmd/home",
            Self::Stop => "cmd/stop",
            Self::Up => "cmd/up",
            Self::Down => "cmd/down",
            Self::MoveTo => "cmd/move_to",
            Self::MoveBy => "cmd/move_by",
        }
    }

    const fn response_name(self) -> &'static str {
        match self {
            Self::Home => "home",
            Self::Stop => "stop",
            Self::Up => "up",
            Self::Down => "down",
            Self::MoveTo => "move_to",
            Self::MoveBy => "move_by",
        }
    }

    fn parse(topic: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|command| topic.ends_with(command.suffix()))
    }
}

type MqttClient<'a> = Client<
    'a,
    TcpSocket<'a>,
    AllocBuffer,
    MQTT_MAX_SUBSCRIBES,
    MQTT_RECEIVE_MAXIMUM,
    MQTT_SEND_MAXIMUM,
    MQTT_MAX_SUBSCRIPTION_IDENTIFIERS,
>;

#[embassy_executor::task]
pub async fn mqtt_task(
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    mut status_watcher: DeskStatusWatcher,
) -> ! {
    let settings = match MqttSettings::load() {
        Ok(settings) => settings,
        Err(error) => {
            warn!("mqtt setup failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    loop {
        run_mqtt_session(stack, state, &mut status_watcher, &settings).await;
        Timer::after(settings.reconnect_delay).await;
    }
}

async fn run_mqtt_session(
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    status_watcher: &mut DeskStatusWatcher,
    settings: &MqttSettings,
) {
    let mut command_rx_buffer = [0; MQTT_TCP_BUFFER_SIZE];
    let mut command_tx_buffer = [0; MQTT_TCP_BUFFER_SIZE];
    let mut publisher_rx_buffer = [0; MQTT_TCP_BUFFER_SIZE];
    let mut publisher_tx_buffer = [0; MQTT_TCP_BUFFER_SIZE];

    let command_socket = match connect_socket(
        stack,
        settings,
        &mut command_rx_buffer,
        &mut command_tx_buffer,
    )
    .await
    {
        Ok(socket) => socket,
        Err(error) => {
            warn!("mqtt command connect failed: {:?}", error);
            return;
        }
    };
    let publisher_socket = match connect_socket(
        stack,
        settings,
        &mut publisher_rx_buffer,
        &mut publisher_tx_buffer,
    )
    .await
    {
        Ok(socket) => socket,
        Err(error) => {
            warn!("mqtt publisher connect failed: {:?}", error);
            return;
        }
    };

    let mut command_buffer = AllocBuffer;
    let mut publisher_buffer = AllocBuffer;
    let mut command_client = MqttClient::new(&mut command_buffer);
    let mut publisher_client = MqttClient::new(&mut publisher_buffer);

    let connect_options = settings.connect_options();
    let command_client_id = match settings.command_client_id() {
        Ok(client_id) => client_id,
        Err(error) => {
            warn!("mqtt invalid command client id: {:?}", error);
            return;
        }
    };
    let publisher_client_id = match settings.publisher_client_id() {
        Ok(client_id) => client_id,
        Err(error) => {
            warn!("mqtt invalid publisher client id: {:?}", error);
            return;
        }
    };

    if let Err(error) = command_client
        .connect(command_socket, &connect_options, Some(command_client_id))
        .await
    {
        warn!("mqtt command connect handshake failed: {:?}", error);
        return;
    }
    if let Err(error) = publisher_client
        .connect(
            publisher_socket,
            &connect_options,
            Some(publisher_client_id),
        )
        .await
    {
        warn!("mqtt publisher connect handshake failed: {:?}", error);
        return;
    }

    info!("mqtt connected");

    if let Err(error) = subscribe_commands(&mut command_client, settings).await {
        warn!("mqtt subscribe failed: {:?}", error);
        return;
    }
    if let Err(error) = publish_status(&mut publisher_client, settings, state).await {
        warn!("mqtt initial status publish failed: {:?}", error);
        return;
    }

    loop {
        match select3(
            command_client.poll_header(),
            status_watcher.wait_for_change(),
            Timer::after(settings.ping_interval),
        )
        .await
        {
            Either3::First(Ok(header)) => {
                let event = match command_client.poll_body(header).await {
                    Ok(event) => event,
                    Err(error) => {
                        warn!("mqtt command poll failed: {:?}", error);
                        return;
                    }
                };

                if let Some(response) = decode_command_event(state, event) {
                    if let Err(error) =
                        publish_response(&mut publisher_client, settings, &response).await
                    {
                        warn!("mqtt response publish failed: {:?}", error);
                        return;
                    }
                    if let Err(error) = publish_status(&mut publisher_client, settings, state).await
                    {
                        warn!("mqtt status publish failed: {:?}", error);
                        return;
                    }
                }
            }
            Either3::First(Err(error)) => {
                warn!("mqtt command header poll failed: {:?}", error);
                return;
            }
            Either3::Second(_status) => {
                if let Err(error) = publish_status(&mut publisher_client, settings, state).await {
                    warn!("mqtt status publish failed: {:?}", error);
                    return;
                }
            }
            Either3::Third(_) => {
                if let Err(error) = command_client.ping().await {
                    warn!("mqtt command ping failed: {:?}", error);
                    return;
                }
                if let Err(error) = publisher_client.ping().await {
                    warn!("mqtt publisher ping failed: {:?}", error);
                    return;
                }
            }
        }
    }
}

async fn connect_socket<'a>(
    stack: Stack<'static>,
    settings: &MqttSettings,
    rx_buffer: &'a mut [u8],
    tx_buffer: &'a mut [u8],
) -> Result<TcpSocket<'a>, embassy_net::tcp::ConnectError> {
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    socket
        .connect((settings.broker_addr, settings.broker_port))
        .await?;
    Ok(socket)
}

async fn subscribe_commands<'a>(
    client: &mut MqttClient<'a>,
    settings: &MqttSettings,
) -> Result<(), MqttError<'a>> {
    let subscription = SubscriptionOptions::new().at_least_once();
    for command in CommandTopic::ALL {
        client
            .subscribe(settings.command_topic_filter(command)?, subscription)
            .await?;
    }
    Ok(())
}

async fn publish_status<'a>(
    client: &mut MqttClient<'a>,
    settings: &MqttSettings,
    state: &DeskControllerState,
) -> Result<(), MqttError<'a>> {
    publish_message(
        client,
        settings.status_topic_name()?,
        &status_response(state),
    )
    .await
}

async fn publish_response<'a>(
    client: &mut MqttClient<'a>,
    settings: &MqttSettings,
    response: &str,
) -> Result<(), MqttError<'a>> {
    publish_message(client, settings.response_topic_name()?, response).await
}

async fn publish_message<'a>(
    client: &mut MqttClient<'a>,
    topic: TopicName<'_>,
    payload: &str,
) -> Result<(), MqttError<'a>> {
    let options = PublicationOptions::new(TopicReference::Name(topic));
    client.publish(&options, Bytes::from(payload)).await?;
    Ok(())
}

fn decode_command_event(
    state: &DeskControllerState,
    event: Event<'_, MQTT_MAX_SUBSCRIPTION_IDENTIFIERS>,
) -> Option<String> {
    let publish = match event {
        Event::Publish(publish) => publish,
        _ => return None,
    };

    let topic = publish.topic.as_ref().as_str();
    let payload = core::str::from_utf8(publish.message.as_ref()).ok()?.trim();
    let command = CommandTopic::parse(topic)?;
    Some(handle_command(state, command, payload))
}

fn handle_command(state: &DeskControllerState, topic: CommandTopic, payload: &str) -> String {
    match topic {
        CommandTopic::Home => {
            submission_response(topic.response_name(), state.submit_home(), state.snapshot())
        }
        CommandTopic::Stop => match state.submit_stop() {
            StopSubmission::Accepted => {
                response_body(topic.response_name(), "accepted", state.snapshot())
            }
            StopSubmission::IgnoredIdle => {
                response_body(topic.response_name(), "ignored_idle", state.snapshot())
            }
        },
        CommandTopic::Up => match parse_i32(payload) {
            Some(steps) => submission_response(
                topic.response_name(),
                state.submit_move_by(RelativeCounts::new(steps.abs())),
                state.snapshot(),
            ),
            None => response_body(topic.response_name(), "invalid_payload", state.snapshot()),
        },
        CommandTopic::Down => match parse_i32(payload) {
            Some(steps) => submission_response(
                topic.response_name(),
                state.submit_move_by(RelativeCounts::new(-steps.abs())),
                state.snapshot(),
            ),
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

fn parse_i32(value: &str) -> Option<i32> {
    i32::from_str(value).ok()
}

fn status_response(state: &DeskControllerState) -> String {
    let snapshot = state.snapshot();
    let status = state.status();
    format!(
        "mode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\nhomed={}\nneeds_rehome={}\nmotion={}\nlast_stop_reason={}\nobstruction_sensitivity={}\ntarget_active={}\ntarget_position={}\naverage_position={}\nleft_position={}\nright_position={}\nmin_position={}\nmax_position={}\nskew_counts={}\n",
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
        bool_name(status.homed),
        bool_name(status.needs_rehome),
        motion_name(status.motion),
        stop_reason_name(status.last_stop_reason),
        obstruction_sensitivity_name(state.obstruction_sensitivity()),
        bool_name(status.target_active),
        status.target_position,
        status.average_position,
        status.left_position,
        status.right_position,
        status.min_position,
        status.max_position,
        status.skew_counts,
    )
}

fn submission_response(
    command: &'static str,
    submission: CommandSubmission,
    snapshot: DeskControllerSnapshot,
) -> String {
    let result = match submission {
        CommandSubmission::Accepted => "accepted",
        CommandSubmission::RejectedBusy => "rejected_busy",
        CommandSubmission::RejectedUnhomed => "rejected_unhomed",
        CommandSubmission::RejectedFaulted => "rejected_faulted",
    };

    response_body(command, result, snapshot)
}

fn response_body(
    command: &'static str,
    result: &'static str,
    snapshot: DeskControllerSnapshot,
) -> String {
    format!(
        "command={}\nresult={}\nmode={}\ncommand_pending={}\nstop_requested={}\nlast_fault={}\n",
        command,
        result,
        controller_mode_name(snapshot.mode),
        bool_name(snapshot.command_pending),
        bool_name(snapshot.stop_requested),
        fault_name(snapshot.last_fault),
    )
}

fn controller_mode_name(mode: DeskControllerMode) -> &'static str {
    match mode {
        DeskControllerMode::Unhomed => "unhomed",
        DeskControllerMode::Ready => "ready",
        DeskControllerMode::Homing => "homing",
        DeskControllerMode::Moving => "moving",
        DeskControllerMode::Faulted => "faulted",
    }
}

fn fault_name(fault: Option<DeskFault>) -> &'static str {
    match fault {
        None => "none",
        Some(DeskFault::LeftLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "left_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "left_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "left_leg_move_timeout",
        },
        Some(DeskFault::RightLeg(error)) => match error {
            crate::leg::LegError::HomingStartTimeout => "right_leg_homing_start_timeout",
            crate::leg::LegError::PolarityMismatch => "right_leg_polarity_mismatch",
            crate::leg::LegError::MoveTimeout => "right_leg_move_timeout",
        },
        Some(DeskFault::MoveTimeout) => "move_timeout",
        Some(DeskFault::SkewFault) => "skew_fault",
        Some(DeskFault::RehomeRequired) => "rehome_required",
    }
}

fn motion_name(motion: DeskMotionState) -> &'static str {
    match motion {
        DeskMotionState::Idle => "idle",
        DeskMotionState::Homing => "homing",
        DeskMotionState::MovingUp => "moving_up",
        DeskMotionState::MovingDown => "moving_down",
    }
}

fn stop_reason_name(reason: DeskStopReason) -> &'static str {
    match reason {
        DeskStopReason::None => "none",
        DeskStopReason::TargetReached => "target_reached",
        DeskStopReason::UserStop => "user_stop",
        DeskStopReason::Obstruction => "obstruction",
    }
}

fn obstruction_sensitivity_name(sensitivity: ObstructionSensitivity) -> &'static str {
    match sensitivity {
        ObstructionSensitivity::None => "none",
        ObstructionSensitivity::Low => "low",
        ObstructionSensitivity::Medium => "medium",
        ObstructionSensitivity::High => "high",
    }
}

fn bool_name(value: bool) -> &'static str {
    if value { "true" } else { "false" }
}

impl MqttSettings {
    fn load() -> Result<Self, MqttSetupError> {
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
            topic_home: format!("{topic_prefix}/{}", CommandTopic::Home.suffix()),
            topic_stop: format!("{topic_prefix}/{}", CommandTopic::Stop.suffix()),
            topic_up: format!("{topic_prefix}/{}", CommandTopic::Up.suffix()),
            topic_down: format!("{topic_prefix}/{}", CommandTopic::Down.suffix()),
            topic_move_to: format!("{topic_prefix}/{}", CommandTopic::MoveTo.suffix()),
            topic_move_by: format!("{topic_prefix}/{}", CommandTopic::MoveBy.suffix()),
        })
    }

    fn connect_options(&self) -> ConnectOptions<'_> {
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

    fn command_client_id(&self) -> Result<MqttString<'_>, MqttError<'static>> {
        mqtt_string(&self.command_client_id)
    }

    fn publisher_client_id(&self) -> Result<MqttString<'_>, MqttError<'static>> {
        mqtt_string(&self.publisher_client_id)
    }

    fn status_topic_name(&self) -> Result<TopicName<'_>, MqttError<'static>> {
        topic_name(&self.topic_status)
    }

    fn response_topic_name(&self) -> Result<TopicName<'_>, MqttError<'static>> {
        topic_name(&self.topic_response)
    }

    fn command_topic_filter(
        &self,
        command: CommandTopic,
    ) -> Result<TopicFilter<'_>, MqttError<'static>> {
        let topic = match command {
            CommandTopic::Home => &self.topic_home,
            CommandTopic::Stop => &self.topic_stop,
            CommandTopic::Up => &self.topic_up,
            CommandTopic::Down => &self.topic_down,
            CommandTopic::MoveTo => &self.topic_move_to,
            CommandTopic::MoveBy => &self.topic_move_by,
        };

        topic_name(topic).map(Into::into)
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

fn duration_to_keep_alive(duration: Duration) -> KeepAlive {
    match u16::try_from(duration.as_secs()) {
        Ok(0) | Err(_) => KeepAlive::Infinite,
        Ok(seconds) => {
            KeepAlive::Seconds(NonZero::new(seconds).expect("keep alive must be non-zero"))
        }
    }
}

pub fn spawn_mqtt(
    spawner: &Spawner,
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    status_watcher: DeskStatusWatcher,
) {
    spawner.must_spawn(mqtt_task(stack, state, status_watcher));
}
