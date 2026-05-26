use embassy_net::tcp::TcpSocket;
use rust_mqtt::{
    Bytes,
    buffer::AllocBuffer,
    client::{
        Client, MqttError,
        options::{PublicationOptions, TopicReference},
    },
    types::TopicName,
};

use crate::{config::RuntimeConfigState, controller::DeskControllerState};

use super::{
    responses::{config_response, status_response},
    settings::{
        MQTT_MAX_SUBSCRIBES, MQTT_MAX_SUBSCRIPTION_IDENTIFIERS, MQTT_RECEIVE_MAXIMUM,
        MQTT_SEND_MAXIMUM, MqttSettings,
    },
};

pub(super) type MqttClient<'a> = Client<
    'a,
    TcpSocket<'a>,
    AllocBuffer,
    MQTT_MAX_SUBSCRIBES,
    MQTT_RECEIVE_MAXIMUM,
    MQTT_SEND_MAXIMUM,
    MQTT_MAX_SUBSCRIPTION_IDENTIFIERS,
>;

pub(super) async fn publish_status<'a>(
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

pub(super) async fn publish_response<'a>(
    client: &mut MqttClient<'a>,
    settings: &MqttSettings,
    response: &str,
) -> Result<(), MqttError<'a>> {
    publish_message(client, settings.response_topic_name()?, response).await
}

pub(super) async fn publish_config<'a>(
    client: &mut MqttClient<'a>,
    settings: &MqttSettings,
    runtime_config_state: &RuntimeConfigState,
) -> Result<(), MqttError<'a>> {
    publish_message(
        client,
        settings.config_topic_name()?,
        &config_response(runtime_config_state),
    )
    .await
}

pub(super) async fn publish_message<'a>(
    client: &mut MqttClient<'a>,
    topic: TopicName<'_>,
    payload: &str,
) -> Result<(), MqttError<'a>> {
    let options = PublicationOptions::new(TopicReference::Name(topic));
    client.publish(&options, Bytes::from(payload)).await?;
    Ok(())
}
