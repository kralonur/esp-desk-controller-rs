use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either4, select4};
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Instant, Timer};
use rust_mqtt::{
    buffer::AllocBuffer,
    client::{MqttError, options::SubscriptionOptions},
};

use crate::{
    config::RuntimeConfigState, controller::DeskControllerState, desk::DeskStatusWatcher,
    persistent_config::RuntimeConfigPersistence,
};

use super::{
    commands::decode_command_event,
    config_api::flush_deferred_config,
    publish::{MqttClient, publish_config, publish_response, publish_status},
    settings::{MQTT_TCP_BUFFER_SIZE, MqttSettings},
    topics::CommandTopic,
};

#[embassy_executor::task]
/// Run one MQTT command/status session.
///
/// Most callers should use `spawn_mqtt`; this entrypoint exists for the spawned
/// Embassy task and owns reconnect/session behavior.
pub async fn mqtt_task(
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    runtime_config_state: &'static RuntimeConfigState,
    mut runtime_config_persistence: Option<&'static mut RuntimeConfigPersistence>,
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
        run_mqtt_session(
            stack,
            state,
            runtime_config_state,
            runtime_config_persistence.as_deref_mut(),
            &mut status_watcher,
            &settings,
        )
        .await;
        Timer::after(settings.reconnect_delay).await;
    }
}

async fn run_mqtt_session(
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    runtime_config_state: &'static RuntimeConfigState,
    mut runtime_config_persistence: Option<&mut RuntimeConfigPersistence>,
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
    if let Err(error) = publish_config(&mut publisher_client, settings, runtime_config_state).await
    {
        warn!("mqtt initial config publish failed: {:?}", error);
        return;
    }

    let mut last_status_published_at = Instant::now();
    let mut pending_status = false;
    let mut next_ping_at = last_status_published_at + settings.ping_interval;

    loop {
        let status_publish_interval = runtime_config_state
            .current()
            .desk()
            .mqtt_status_publish_interval();
        match select4(
            command_client.poll_header(),
            status_watcher.wait_for_change(),
            wait_for_status_publish(
                pending_status,
                last_status_published_at,
                status_publish_interval,
            ),
            Timer::at(next_ping_at),
        )
        .await
        {
            Either4::First(Ok(header)) => {
                let event = match command_client.poll_body(header).await {
                    Ok(event) => event,
                    Err(error) => {
                        warn!("mqtt command poll failed: {:?}", error);
                        return;
                    }
                };

                if let Some(response) = decode_command_event(
                    state,
                    runtime_config_state,
                    runtime_config_persistence.as_deref_mut(),
                    settings,
                    event,
                ) {
                    if let Err(error) =
                        publish_response(&mut publisher_client, settings, &response.response).await
                    {
                        warn!("mqtt response publish failed: {:?}", error);
                        return;
                    }
                    if response.publish_status
                        && let Err(error) =
                            publish_status(&mut publisher_client, settings, state).await
                    {
                        warn!("mqtt status publish failed: {:?}", error);
                        return;
                    }
                    if response.publish_status {
                        last_status_published_at = Instant::now();
                        pending_status = false;
                    }
                    if response.publish_config
                        && let Err(error) =
                            publish_config(&mut publisher_client, settings, runtime_config_state)
                                .await
                    {
                        warn!("mqtt config publish failed: {:?}", error);
                        return;
                    }
                }
            }
            Either4::First(Err(error)) => {
                warn!("mqtt command header poll failed: {:?}", error);
                return;
            }
            Either4::Second(_status) => {
                pending_status = true;
                if let Some(response) = flush_deferred_config(
                    state,
                    runtime_config_state,
                    runtime_config_persistence.as_deref_mut(),
                ) {
                    if let Err(error) =
                        publish_response(&mut publisher_client, settings, &response.response).await
                    {
                        warn!("mqtt config persist response publish failed: {:?}", error);
                        return;
                    }
                    if response.publish_config
                        && let Err(error) =
                            publish_config(&mut publisher_client, settings, runtime_config_state)
                                .await
                    {
                        warn!("mqtt config publish failed: {:?}", error);
                        return;
                    }
                }
                if status_publish_due(
                    pending_status,
                    last_status_published_at,
                    status_publish_interval,
                ) {
                    if let Err(error) = publish_status(&mut publisher_client, settings, state).await
                    {
                        warn!("mqtt status publish failed: {:?}", error);
                        return;
                    }
                    last_status_published_at = Instant::now();
                    pending_status = false;
                }
            }
            Either4::Third(_) => {
                if let Err(error) = publish_status(&mut publisher_client, settings, state).await {
                    warn!("mqtt status publish failed: {:?}", error);
                    return;
                }
                last_status_published_at = Instant::now();
                pending_status = false;
            }
            Either4::Fourth(_) => {
                if let Err(error) = command_client.ping().await {
                    warn!("mqtt command ping failed: {:?}", error);
                    return;
                }
                if let Err(error) = publisher_client.ping().await {
                    warn!("mqtt publisher ping failed: {:?}", error);
                    return;
                }
                next_ping_at = Instant::now() + settings.ping_interval;
            }
        }
    }
}

async fn wait_for_status_publish(
    pending_status: bool,
    last_status_published_at: Instant,
    interval: Duration,
) {
    if pending_status {
        Timer::at(last_status_published_at + interval).await;
    } else {
        core::future::pending::<()>().await;
    }
}

fn status_publish_due(
    pending_status: bool,
    last_status_published_at: Instant,
    interval: Duration,
) -> bool {
    pending_status && Instant::now().saturating_duration_since(last_status_published_at) >= interval
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
    client
        .subscribe(settings.config_get_topic_filter()?, subscription)
        .await?;
    client
        .subscribe(settings.config_reset_topic_filter()?, subscription)
        .await?;
    client
        .subscribe(settings.config_set_topic_filter()?, subscription)
        .await?;
    client
        .subscribe(settings.override_topic_filter()?, subscription)
        .await?;
    Ok(())
}

/// Spawn the MQTT task with the command controller and status/config sources.
pub fn spawn_mqtt(
    spawner: &Spawner,
    stack: Stack<'static>,
    state: &'static DeskControllerState,
    runtime_config_state: &'static RuntimeConfigState,
    runtime_config_persistence: Option<&'static mut RuntimeConfigPersistence>,
    status_watcher: DeskStatusWatcher,
) {
    spawner.spawn(
        mqtt_task(
            stack,
            state,
            runtime_config_state,
            runtime_config_persistence,
            status_watcher,
        )
        .expect("spawn mqtt task"),
    );
}
