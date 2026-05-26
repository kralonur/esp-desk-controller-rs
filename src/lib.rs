//! Firmware library for the ESP PWM desk controller.
//!
//! The crate is organized around hardware-facing drivers (`motor`,
//! `quadrature`, `wifi`), domain modules (`leg`, `desk`, `controller`), runtime
//! configuration (`config`, `persistent_config`), MQTT integration, and the
//! top-level `app` startup wiring used by the binary entrypoint.

#![no_std]
#![feature(impl_trait_in_assoc_type)]

extern crate alloc;

pub mod app;
pub mod config;
pub mod controller;
pub mod desk;
pub mod leg;
pub mod motor;
pub mod mqtt;
pub mod persistent_config;
pub mod quadrature;
pub mod units;
pub mod wifi;
