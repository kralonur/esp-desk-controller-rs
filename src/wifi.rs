//! WiFi startup and connection supervision.
//!
//! This module reads compile-time WiFi credentials, starts the ESP radio stack,
//! spawns background network tasks, and returns an Embassy network stack once
//! DHCP has completed.

use alloc::string::String;

use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Config, DhcpConfig, Runner, Stack, StackResources};
use embassy_time::{Duration, Timer};
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_radio::wifi::{
    Config as WifiConfig, ControllerConfig, Interface, PowerSaveMode, WifiController,
    sta::StationConfig,
};
use static_cell::StaticCell;

// Required compile-time WiFi network name. Set with `WIFI_SSID=...` when building.
const WIFI_SSID: Option<&str> = option_env!("WIFI_SSID");
// Optional compile-time WiFi password. Omit or set empty for open networks.
const WIFI_PASSWORD: Option<&str> = option_env!("WIFI_PASSWORD");

#[derive(Clone, Copy, Debug, defmt::Format)]
/// WiFi startup failures before the network stack is ready.
pub enum WifiSetupError {
    MissingSsid,
    InitFailed,
    ConfigureFailed,
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) -> ! {
    loop {
        if controller.is_connected() {
            info!("wifi connected");
            if let Err(error) = controller.wait_for_disconnect_async().await {
                warn!("wifi disconnect wait failed: {:?}", error);
            }
            Timer::after(Duration::from_secs(5)).await;
            continue;
        }

        match controller.connect_async().await {
            Ok(_) => {
                info!("wifi connected");
            }
            Err(error) => {
                warn!("wifi connect failed: {:?}", error);
            }
        }
        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface<'static>>) -> ! {
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

/// Start WiFi station mode and return a DHCP-ready network stack.
///
/// This spawns the connection supervisor and network runner tasks. Credentials
/// are read from `WIFI_SSID` and optional `WIFI_PASSWORD` at compile time.
pub async fn start_wifi(
    wifi: WIFI<'static>,
    rng: Rng,
    spawner: &Spawner,
) -> Result<Stack<'static>, WifiSetupError> {
    let ssid = WIFI_SSID.ok_or(WifiSetupError::MissingSsid)?;
    let password = WIFI_PASSWORD.unwrap_or("");

    let wifi_config = ControllerConfig::default();
    let (mut controller, interfaces) =
        esp_radio::wifi::new(wifi, wifi_config).map_err(|_| WifiSetupError::InitFailed)?;

    let client_config = StationConfig::default()
        .with_ssid(String::from(ssid))
        .with_password(String::from(password));

    controller
        .set_power_saving(PowerSaveMode::Minimum)
        .map_err(|_| WifiSetupError::ConfigureFailed)?;
    controller
        .set_config(&WifiConfig::Station(client_config))
        .map_err(|_| WifiSetupError::ConfigureFailed)?;
    info!("wifi configured with minimum power save");

    let net_seed = rng.random() as u64 | ((rng.random() as u64) << 32);
    let config = Config::dhcpv4(DhcpConfig::default());
    static STACK_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        interfaces.station,
        config,
        STACK_RESOURCES.uninit().write(StackResources::<3>::new()),
        net_seed,
    );

    spawner.spawn(connection_task(controller).expect("spawn wifi connection task"));
    spawner.spawn(net_task(runner).expect("spawn wifi net task"));

    wait_for_connection(stack).await;

    Ok(stack)
}
