//! Persistent runtime configuration stored in ESP NVS.
//!
//! File layout:
//!
//! - `storage`: flash/NVS integration, including partition/key details,
//!   load/save/erase behavior, and the dirty flag used by MQTT config updates.
//! - `codec`: the private binary blob format used inside NVS. It encodes and
//!   decodes `RuntimeConfig`, validates blob magic/version/length, and keeps the
//!   byte-level reader/writer helpers with the codec tests.

mod codec;
mod storage;

pub use codec::{decode_runtime_config, encode_runtime_config};
pub use storage::RuntimeConfigPersistence;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
/// Errors from loading, decoding, encoding, or storing runtime config.
pub enum PersistError {
    Storage,
    Missing,
    InvalidFormat,
    InvalidConfig,
}
