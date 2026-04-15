use alloc::string::String;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Config, DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_radio::{
    Controller,
    wifi::{
        ClientConfig, Config as WifiConfig, ModeConfig, PowerSaveMode, WifiController, WifiDevice,
        WifiEvent,
    },
};
use static_cell::StaticCell;

const WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");
const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");

#[derive(Clone, Copy, Debug, defmt::Format)]
pub enum WifiSetupError {
    MissingSsid,
    InitFailed,
    ConfigureFailed,
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) -> ! {
    loop {
        if esp_radio::wifi::sta_state() == esp_radio::wifi::WifiStaState::Connected {
            controller.wait_for_event(WifiEvent::StaDisconnected).await;
            Timer::after(Duration::from_millis(5000)).await
        }

        if !matches!(controller.is_started(), Ok(true)) {
            match controller.start_async().await {
                Ok(()) => info!("wifi controller started"),
                Err(error) => {
                    warn!("wifi start failed: {:?}", error);
                    Timer::after(Duration::from_secs(2)).await;
                    continue;
                }
            }
        }

        match controller.is_connected() {
            Ok(true) => {
                info!("wifi connected");
            }
            Ok(false) => match controller.connect_async().await {
                Ok(()) => {
                    info!("wifi connect requested");
                }
                Err(error) => {
                    warn!("wifi connect failed: {:?}", error);
                }
            },
            Err(error) => {
                warn!("wifi disconnected: {:?}", error);
            }
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) -> ! {
    runner.run().await
}

async fn wait_for_connection(stack: Stack<'_>) {
    info!("Waiting for link to be up");
    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    info!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            info!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

pub async fn start_wifi(
    radio_init: &'static Controller<'static>,
    wifi: WIFI<'static>,
    rng: Rng,
    spawner: &Spawner,
) -> Result<Stack<'static>, WifiSetupError> {
    let ssid = WIFI_SSID.ok_or(WifiSetupError::MissingSsid)?;
    let password = WIFI_PASSWORD.unwrap_or("");

    let wifi_config = WifiConfig::default().with_power_save_mode(PowerSaveMode::Minimum);
    let (mut controller, interfaces) = esp_radio::wifi::new(radio_init, wifi, wifi_config)
        .map_err(|_| WifiSetupError::InitFailed)?;

    let client_config = ClientConfig::default()
        .with_ssid(String::from(ssid))
        .with_password(String::from(password));

    controller
        .set_config(&ModeConfig::Client(client_config))
        .map_err(|_| WifiSetupError::ConfigureFailed)?;
    info!("wifi configured with minimum power save");

    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let config = Config::dhcpv4(DhcpConfig::default());
    static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        interfaces.sta,
        config,
        STACK_RESOURCES.uninit().write(StackResources::<3>::new()),
        net_seed,
    );

    spawner.must_spawn(connection_task(controller));
    spawner.must_spawn(net_task(runner));

    wait_for_connection(stack).await;

    Ok(stack)
}
