use core::cell::RefCell;

use defmt::Format;
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use static_cell::StaticCell;

use crate::{
    config::{HardwareLegSide, RuntimeConfigReader},
    quadrature::QuadratureWatcher,
    units::PositionSign,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Current motion intent reported for one leg.
pub enum MotionState {
    Idle,
    MovingUp,
    MovingDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Latest observer-facing state for one leg.
pub struct LegStatus {
    pub position: i32,
    pub min_position: i32,
    pub max_position: i32,
    pub duty: u8,
    pub motion: MotionState,
    pub homed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
/// Lightweight progress event used by desk movement loops.
pub struct LegProgress {
    pub position: i32,
}

/// Static storage used to initialize one leg status channel.
pub struct LegStatusStorage {
    pub(super) state: StaticCell<LegStatusState>,
}

/// Async watcher for leg status changes.
pub struct LegStatusWatcher {
    pub(super) receiver: Receiver<'static, CriticalSectionRawMutex, LegStatus, 4>,
}

/// Async watcher for logical encoder progress.
pub struct LegProgressWatcher {
    pub(super) quadrature_watcher: QuadratureWatcher,
    pub(super) runtime_config_reader: RuntimeConfigReader,
    pub(super) side: HardwareLegSide,
}

pub(super) struct LegStatusState {
    status: Mutex<CriticalSectionRawMutex, RefCell<LegStatus>>,
    pub(super) watch: Watch<CriticalSectionRawMutex, LegStatus, 4>,
}

impl LegStatusState {
    pub(super) fn new(initial_status: LegStatus) -> Self {
        Self {
            status: Mutex::new(RefCell::new(initial_status)),
            watch: Watch::new(),
        }
    }

    pub(super) fn current(&self) -> LegStatus {
        self.status.lock(|status| *status.borrow())
    }

    pub(super) fn publish(&self, status: LegStatus) {
        let changed = self.status.lock(|cached_status| {
            let mut cached_status = cached_status.borrow_mut();
            if *cached_status == status {
                false
            } else {
                *cached_status = status;
                true
            }
        });
        if changed {
            self.watch.sender().send(status);
        }
    }

    pub(super) fn publish_position(&self, raw_position: i32, position_sign: PositionSign) {
        let position = position_sign.apply(raw_position);
        let (status, changed) = self.status.lock(|cached_status| {
            let mut cached_status = cached_status.borrow_mut();
            if cached_status.position == position {
                (*cached_status, false)
            } else {
                cached_status.position = position;
                (*cached_status, true)
            }
        });
        if changed {
            self.watch.sender().send(status);
        }
    }
}

#[embassy_executor::task(pool_size = 4)]
pub(super) async fn mirror_quadrature_to_leg_status(
    mut quadrature_watcher: QuadratureWatcher,
    status_state: &'static LegStatusState,
    runtime_config_reader: RuntimeConfigReader,
    side: HardwareLegSide,
) {
    loop {
        let event = quadrature_watcher.wait_for_change().await;
        let position_sign = runtime_config_reader
            .current()
            .hardware()
            .leg_config(side)
            .position_sign();
        status_state.publish_position(event.snapshot.position, position_sign);
    }
}

impl Default for LegStatusStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl LegStatusStorage {
    pub const fn new() -> Self {
        Self {
            state: StaticCell::new(),
        }
    }
}

impl LegStatusWatcher {
    pub async fn wait_for_change(&mut self) -> LegStatus {
        self.receiver.changed().await
    }
}

impl LegProgressWatcher {
    pub async fn wait_for_change(&mut self) -> LegProgress {
        let event = self.quadrature_watcher.wait_for_change().await;
        let position_sign = self
            .runtime_config_reader
            .current()
            .hardware()
            .leg_config(self.side)
            .position_sign();
        LegProgress {
            position: position_sign.apply(event.snapshot.position),
        }
    }
}
