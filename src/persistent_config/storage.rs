use alloc::vec::Vec;

use esp_nvs::{Key, Nvs};
use esp_storage::FlashStorage;

use crate::config::RuntimeConfig;
use crate::persistent_config::{
    PersistError,
    codec::{decode_runtime_config, encode_runtime_config},
};

// ESP-IDF's built-in "Single factory app, no OTA" partition table places the NVS partition at
// 0x9000 with size 0x6000:
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/partition-tables.html#built-in-partition-tables
const NVS_PARTITION_OFFSET: usize = 0x9000;
const NVS_PARTITION_SIZE: usize = 0x6000;

// ESP-IDF NVS keys are limited to 15 characters. Keep these short and ASCII-compatible.
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/storage/nvs_flash.html#keys-and-values
const CONFIG_NAMESPACE: Key = Key::from_array(b"deskcfg");
const CONFIG_KEY: Key = Key::from_array(b"runtime");

type ConfigNvs = Nvs<FlashStorage<'static>>;

/// ESP NVS persistence handle for runtime configuration.
///
/// The dirty flag lets MQTT defer writes while motion is active and save once
/// the controller reports persistence is safe.
pub struct RuntimeConfigPersistence {
    nvs: ConfigNvs,
    dirty: bool,
}

impl RuntimeConfigPersistence {
    /// Open the configured NVS partition for runtime config persistence.
    pub fn new(flash: esp_hal::peripherals::FLASH<'static>) -> Result<Self, PersistError> {
        let storage = FlashStorage::new(flash);
        let nvs = Nvs::new(NVS_PARTITION_OFFSET, NVS_PARTITION_SIZE, storage)
            .map_err(|_| PersistError::Storage)?;

        Ok(Self { nvs, dirty: false })
    }

    /// Load and decode the saved runtime config from NVS.
    pub fn load(&mut self) -> Result<RuntimeConfig, PersistError> {
        let bytes =
            self.nvs
                .get::<Vec<u8>>(&CONFIG_NAMESPACE, &CONFIG_KEY)
                .map_err(|error| match error {
                    esp_nvs::error::Error::KeyNotFound
                    | esp_nvs::error::Error::NamespaceNotFound => PersistError::Missing,
                    _ => PersistError::Storage,
                })?;
        decode_runtime_config(&bytes)
    }

    /// Encode and save runtime config, clearing the dirty flag on success.
    pub fn save(&mut self, config: RuntimeConfig) -> Result<(), PersistError> {
        let bytes = encode_runtime_config(config);
        self.nvs
            .set(&CONFIG_NAMESPACE, &CONFIG_KEY, bytes.as_slice())
            .map_err(|_| PersistError::Storage)?;
        self.clear_dirty();
        Ok(())
    }

    pub fn erase(&mut self) -> Result<(), PersistError> {
        self.nvs
            .delete(&CONFIG_NAMESPACE, &CONFIG_KEY)
            .map_err(|_| PersistError::Storage)?;
        self.clear_dirty();
        Ok(())
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    fn clear_dirty(&mut self) {
        self.dirty = false;
    }
}
