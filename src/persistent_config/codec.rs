use alloc::vec::Vec;

use embassy_time::Duration;

use crate::config::{
    ConfigError, DeskConfig, LegRuntimeConfig, ObstructionProfileConfig, ObstructionSensitivity,
    RuntimeConfig,
};
use crate::persistent_config::PersistError;
use crate::units::{CountDelta, DutyPercent, DutyPercentTrim, Percent, PositionCounts};

// Private blob format marker for this firmware. This lets us reject unrelated/corrupt NVS blobs
// before decoding fields as runtime config.
const CONFIG_MAGIC: [u8; 4] = *b"DPWM";
const CONFIG_VERSION: u16 = 2;

const U8_LEN: usize = core::mem::size_of::<u8>();
const I32_LEN: usize = core::mem::size_of::<i32>();
const U16_LEN: usize = core::mem::size_of::<u16>();
const U32_LEN: usize = core::mem::size_of::<u32>();
const U64_LEN: usize = core::mem::size_of::<u64>();
const COUNT_DELTA_LEN: usize = U32_LEN;
const DUTY_PERCENT_LEN: usize = U8_LEN;
const DUTY_TRIM_LEN: usize = U8_LEN;
const DURATION_MS_LEN: usize = U64_LEN;
const OBSTRUCTION_SENSITIVITY_LEN: usize = U8_LEN;
const OBSTRUCTION_PROFILE_LEN: usize = U8_LEN + U8_LEN;
const CONFIG_MAGIC_LEN: usize = CONFIG_MAGIC.len();
const CONFIG_VERSION_LEN: usize = U16_LEN;
const CONFIG_HEADER_LEN: usize = CONFIG_MAGIC_LEN + CONFIG_VERSION_LEN;
const DESK_CONFIG_LEN: usize = COUNT_DELTA_LEN * 10
    + DUTY_PERCENT_LEN * 4
    + DUTY_TRIM_LEN * 2
    + DURATION_MS_LEN * 7
    + OBSTRUCTION_SENSITIVITY_LEN
    + OBSTRUCTION_PROFILE_LEN * 3
    + DURATION_MS_LEN;
const LEG_CONFIG_LEN: usize =
    DUTY_PERCENT_LEN * 5 + COUNT_DELTA_LEN * 4 + DURATION_MS_LEN * 4 + I32_LEN;
const CONFIG_BLOB_LEN: usize = CONFIG_HEADER_LEN + DESK_CONFIG_LEN + LEG_CONFIG_LEN;

pub fn encode_runtime_config(config: RuntimeConfig) -> Vec<u8> {
    let mut out = Vec::with_capacity(CONFIG_BLOB_LEN);
    out.extend_from_slice(&CONFIG_MAGIC);
    push_u16(&mut out, CONFIG_VERSION);

    let desk = config.desk();
    push_count_delta(&mut out, desk.target_tolerance());
    push_count_delta(&mut out, desk.target_slow_zone());
    push_duration(&mut out, desk.move_timeout());
    push_duration(&mut out, desk.obstruction_sample_window());
    push_duration(&mut out, desk.obstruction_warmup_duration());
    push_count_delta(&mut out, desk.obstruction_warmup_counts());
    push_duty(&mut out, desk.min_move_duty());
    push_duty(&mut out, desk.move_run_duty());
    push_duty(&mut out, desk.move_slow_duty());
    push_trim(&mut out, desk.move_sync_duty_step());
    push_duration(&mut out, desk.homing_poll_interval());
    push_duration(&mut out, desk.homing_start_timeout());
    push_duration(&mut out, desk.homing_stall_timeout());
    push_count_delta(&mut out, desk.homing_backoff_steps());
    push_duty(&mut out, desk.homing_run_duty());
    push_trim(&mut out, desk.homing_sync_duty_step());
    push_count_delta(&mut out, desk.sync_speedup_enter_counts());
    push_count_delta(&mut out, desk.sync_speedup_exit_counts());
    push_count_delta(&mut out, desk.catch_up_enter_counts());
    push_count_delta(&mut out, desk.catch_up_exit_counts());
    push_count_delta(&mut out, desk.fault_skew_counts());
    push_count_delta(&mut out, desk.homing_fault_skew_counts());
    out.push(obstruction_sensitivity_byte(desk.obstruction_sensitivity()));
    push_profile(&mut out, desk, ObstructionSensitivity::Low);
    push_profile(&mut out, desk, ObstructionSensitivity::Medium);
    push_profile(&mut out, desk, ObstructionSensitivity::High);
    push_duration(&mut out, desk.override_unlock_timeout());
    push_duration(&mut out, desk.mqtt_status_publish_interval());

    let leg = config.leg();
    push_duty(&mut out, leg.startup_duty());
    push_duty(&mut out, leg.max_duty());
    push_duty(&mut out, leg.run_duty());
    push_duty(&mut out, leg.slow_duty());
    push_duty(&mut out, leg.homing_duty());
    push_count_delta(&mut out, leg.startup_events());
    push_duration(&mut out, leg.homing_start_timeout());
    push_duration(&mut out, leg.homing_stall_timeout());
    push_duration(&mut out, leg.homing_poll_interval());
    push_count_delta(&mut out, leg.homing_backoff_steps());
    push_i32(&mut out, leg.default_max_position().get());
    push_duration(&mut out, leg.move_stall_timeout());
    push_count_delta(&mut out, leg.target_slow_zone());
    push_count_delta(&mut out, leg.target_tolerance());

    debug_assert_eq!(out.len(), CONFIG_BLOB_LEN);
    out
}

pub fn decode_runtime_config(bytes: &[u8]) -> Result<RuntimeConfig, PersistError> {
    if bytes.len() != CONFIG_BLOB_LEN || bytes[0..CONFIG_MAGIC_LEN] != CONFIG_MAGIC[..] {
        return Err(PersistError::InvalidFormat);
    }

    let mut reader = ConfigReader::new(bytes);
    reader.expect_magic()?;
    if reader.read_u16()? != CONFIG_VERSION {
        return Err(PersistError::InvalidFormat);
    }

    let mut desk = DeskConfig::new();
    desk.set_target_tolerance(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_target_slow_zone(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_move_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_obstruction_sample_window(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_obstruction_warmup_duration(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_obstruction_warmup_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_min_move_duty(reader.read_duty()?)
        .map_err(config_error)?;
    desk.set_move_run_duty(reader.read_duty()?)
        .map_err(config_error)?;
    desk.set_move_slow_duty(reader.read_duty()?)
        .map_err(config_error)?;
    desk.set_move_sync_duty_step(reader.read_trim()?)
        .map_err(config_error)?;
    desk.set_homing_poll_interval(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_homing_start_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_homing_stall_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_homing_backoff_steps(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_homing_run_duty(reader.read_duty()?)
        .map_err(config_error)?;
    desk.set_homing_sync_duty_step(reader.read_trim()?)
        .map_err(config_error)?;
    desk.set_sync_speedup_enter_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_sync_speedup_exit_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_catch_up_enter_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_catch_up_exit_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_fault_skew_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_homing_fault_skew_counts(reader.read_count_delta()?)
        .map_err(config_error)?;
    desk.set_obstruction_sensitivity(reader.read_obstruction_sensitivity()?)
        .map_err(config_error)?;
    desk.set_low_obstruction_profile(reader.read_profile()?)
        .map_err(config_error)?;
    desk.set_medium_obstruction_profile(reader.read_profile()?)
        .map_err(config_error)?;
    desk.set_high_obstruction_profile(reader.read_profile()?)
        .map_err(config_error)?;
    desk.set_override_unlock_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    desk.set_mqtt_status_publish_interval(reader.read_duration()?)
        .map_err(config_error)?;

    let mut leg = LegRuntimeConfig::new();
    leg.set_startup_duty(reader.read_duty()?)
        .map_err(config_error)?;
    leg.set_max_duty(reader.read_duty()?)
        .map_err(config_error)?;
    leg.set_run_duty(reader.read_duty()?)
        .map_err(config_error)?;
    leg.set_slow_duty(reader.read_duty()?)
        .map_err(config_error)?;
    leg.set_homing_duty(reader.read_duty()?)
        .map_err(config_error)?;
    leg.set_startup_events(reader.read_count_delta()?)
        .map_err(config_error)?;
    leg.set_homing_start_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    leg.set_homing_stall_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    leg.set_homing_poll_interval(reader.read_duration()?)
        .map_err(config_error)?;
    leg.set_homing_backoff_steps(reader.read_count_delta()?)
        .map_err(config_error)?;
    leg.set_default_max_position(PositionCounts::new(reader.read_i32()?))
        .map_err(config_error)?;
    leg.set_move_stall_timeout(reader.read_duration()?)
        .map_err(config_error)?;
    leg.set_target_slow_zone(reader.read_count_delta()?)
        .map_err(config_error)?;
    leg.set_target_tolerance(reader.read_count_delta()?)
        .map_err(config_error)?;

    if !reader.is_done() {
        return Err(PersistError::InvalidFormat);
    }

    RuntimeConfig::from_parts(desk, leg).map_err(config_error)
}

fn config_error(_error: ConfigError) -> PersistError {
    PersistError::InvalidConfig
}

fn push_profile(out: &mut Vec<u8>, desk: DeskConfig, sensitivity: ObstructionSensitivity) {
    let profile = desk
        .obstruction_profile(sensitivity)
        .expect("profile must exist");
    out.push(profile.minimum_baseline_percent().get());
    out.push(profile.consecutive_windows());
}

fn push_count_delta(out: &mut Vec<u8>, value: CountDelta) {
    push_u32(out, value.get());
}

fn push_duty(out: &mut Vec<u8>, value: DutyPercent) {
    out.push(value.get());
}

fn push_trim(out: &mut Vec<u8>, value: DutyPercentTrim) {
    out.push(value.get() as u8);
}

fn push_duration(out: &mut Vec<u8>, value: Duration) {
    push_u64(out, value.as_millis());
}

fn push_u16(out: &mut Vec<u8>, value: u16) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn push_u32(out: &mut Vec<u8>, value: u32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn push_u64(out: &mut Vec<u8>, value: u64) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn push_i32(out: &mut Vec<u8>, value: i32) {
    out.extend_from_slice(&value.to_le_bytes());
}

fn obstruction_sensitivity_byte(value: ObstructionSensitivity) -> u8 {
    match value {
        ObstructionSensitivity::None => 0,
        ObstructionSensitivity::Low => 1,
        ObstructionSensitivity::Medium => 2,
        ObstructionSensitivity::High => 3,
    }
}

struct ConfigReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> ConfigReader<'a> {
    fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn expect_magic(&mut self) -> Result<(), PersistError> {
        let magic = self.read_exact::<4>()?;
        if magic == CONFIG_MAGIC {
            Ok(())
        } else {
            Err(PersistError::InvalidFormat)
        }
    }

    fn read_count_delta(&mut self) -> Result<CountDelta, PersistError> {
        CountDelta::try_new(self.read_u32()?).ok_or(PersistError::InvalidConfig)
    }

    fn read_duty(&mut self) -> Result<DutyPercent, PersistError> {
        let value = self.read_u8()?;
        if value <= DutyPercent::MAX {
            Ok(DutyPercent::new(value))
        } else {
            Err(PersistError::InvalidConfig)
        }
    }

    fn read_trim(&mut self) -> Result<DutyPercentTrim, PersistError> {
        Ok(DutyPercentTrim::new(self.read_u8()? as i8))
    }

    fn read_duration(&mut self) -> Result<Duration, PersistError> {
        Ok(Duration::from_millis(self.read_u64()?))
    }

    fn read_profile(&mut self) -> Result<ObstructionProfileConfig, PersistError> {
        let percent = self.read_u8()?;
        let windows = self.read_u8()?;
        if percent <= 100 && windows > 0 {
            Ok(ObstructionProfileConfig::new(
                Percent::new(percent),
                windows,
            ))
        } else {
            Err(PersistError::InvalidConfig)
        }
    }

    fn read_obstruction_sensitivity(&mut self) -> Result<ObstructionSensitivity, PersistError> {
        match self.read_u8()? {
            0 => Ok(ObstructionSensitivity::None),
            1 => Ok(ObstructionSensitivity::Low),
            2 => Ok(ObstructionSensitivity::Medium),
            3 => Ok(ObstructionSensitivity::High),
            _ => Err(PersistError::InvalidConfig),
        }
    }

    fn read_u8(&mut self) -> Result<u8, PersistError> {
        Ok(self.read_exact::<1>()?[0])
    }

    fn read_u16(&mut self) -> Result<u16, PersistError> {
        Ok(u16::from_le_bytes(self.read_exact::<2>()?))
    }

    fn read_u32(&mut self) -> Result<u32, PersistError> {
        Ok(u32::from_le_bytes(self.read_exact::<4>()?))
    }

    fn read_u64(&mut self) -> Result<u64, PersistError> {
        Ok(u64::from_le_bytes(self.read_exact::<8>()?))
    }

    fn read_i32(&mut self) -> Result<i32, PersistError> {
        Ok(i32::from_le_bytes(self.read_exact::<4>()?))
    }

    fn read_exact<const N: usize>(&mut self) -> Result<[u8; N], PersistError> {
        let end = self
            .offset
            .checked_add(N)
            .ok_or(PersistError::InvalidFormat)?;
        if end > self.bytes.len() {
            return Err(PersistError::InvalidFormat);
        }

        let mut out = [0; N];
        out.copy_from_slice(&self.bytes[self.offset..end]);
        self.offset = end;
        Ok(out)
    }

    fn is_done(&self) -> bool {
        self.offset == self.bytes.len()
    }
}

#[cfg(all(test, target_arch = "x86_64"))]
mod tests {
    use super::*;

    #[test]
    fn default_config_round_trips() {
        let config = RuntimeConfig::default();
        let bytes = encode_runtime_config(config);
        assert_eq!(decode_runtime_config(&bytes), Ok(config));
    }

    #[test]
    fn modified_config_round_trips() {
        let mut config = RuntimeConfig::default();
        config
            .update_desk(|desk| {
                desk.set_min_move_duty(DutyPercent::new(23))
                    .map_err(|_| "invalid_config")?;
                desk.set_fault_skew_counts(CountDelta::new(100_091))
                    .map_err(|_| "invalid_config")
            })
            .unwrap();
        config
            .update_leg(|leg| {
                leg.set_default_max_position(PositionCounts::new(3456))
                    .map_err(|_| "invalid_config")?;
                leg.set_slow_duty(DutyPercent::new(19))
                    .map_err(|_| "invalid_config")
            })
            .unwrap();

        let bytes = encode_runtime_config(config);
        assert_eq!(decode_runtime_config(&bytes), Ok(config));
    }

    #[test]
    fn rejects_bad_magic() {
        let mut bytes = encode_runtime_config(RuntimeConfig::default());
        bytes[0] = b'X';
        assert_eq!(
            decode_runtime_config(&bytes),
            Err(PersistError::InvalidFormat)
        );
    }

    #[test]
    fn rejects_truncated_payload() {
        let bytes = encode_runtime_config(RuntimeConfig::default());
        assert_eq!(
            decode_runtime_config(&bytes[..bytes.len() - 1]),
            Err(PersistError::InvalidFormat)
        );
    }

    #[test]
    fn rejects_invalid_duty() {
        let mut bytes = encode_runtime_config(RuntimeConfig::default());
        bytes[42] = 101;
        assert_eq!(
            decode_runtime_config(&bytes),
            Err(PersistError::InvalidConfig)
        );
    }
}
