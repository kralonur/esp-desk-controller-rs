use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_net::{Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
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
            Either3::First(Err(error)) => {
                warn!("mqtt command header poll failed: {:?}", error);
                return;
            }
            Either3::Second(_status) => {
                if let Err(error) = publish_status(&mut publisher_client, settings, state).await {
                    warn!("mqtt status publish failed: {:?}", error);
                    return;
                }
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
