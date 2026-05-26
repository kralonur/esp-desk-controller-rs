use core::cell::RefCell;

use defmt::Format;
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use static_cell::StaticCell;

use crate::{quadrature::QuadratureWatcher, units::PositionSign};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum MotionState {
    Idle,
    MovingUp,
    MovingDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct LegStatus {
    pub position: i32,
    pub min_position: i32,
    pub max_position: i32,
    pub duty: u8,
    pub motion: MotionState,
    pub homed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct LegProgress {
    pub position: i32,
}

pub struct LegStatusStorage {
    pub(super) state: StaticCell<LegStatusState>,
}

pub struct LegStatusWatcher {
    pub(super) receiver: Receiver<'static, CriticalSectionRawMutex, LegStatus, 4>,
}

pub struct LegProgressWatcher {
    pub(super) quadrature_watcher: QuadratureWatcher,
    pub(super) position_sign: PositionSign,
}

pub(super) struct LegStatusState {
    pub(super) position_sign: PositionSign,
    status: Mutex<CriticalSectionRawMutex, RefCell<LegStatus>>,
    pub(super) watch: Watch<CriticalSectionRawMutex, LegStatus, 4>,
}

impl LegStatusState {
    pub(super) fn new(initial_status: LegStatus, position_sign: PositionSign) -> Self {
        Self {
            position_sign,
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

    pub(super) fn logical_position(&self, raw_position: i32) -> i32 {
        self.position_sign.apply(raw_position)
    }

    pub(super) fn publish_position(&self, raw_position: i32) {
        let position = self.logical_position(raw_position);
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
) {
    loop {
        let event = quadrature_watcher.wait_for_change().await;
        status_state.publish_position(event.snapshot.position);
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
        LegProgress {
            position: self.position_sign.apply(event.snapshot.position),
        }
    }
}
