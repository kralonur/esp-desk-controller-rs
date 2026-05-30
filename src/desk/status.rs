use core::cell::RefCell;

use defmt::Format;
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use static_cell::StaticCell;

use crate::{
    leg::LegStatus,
    units::{abs_position_delta, average_position},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskMotionState {
    Idle,
    Homing,
    MovingUp,
    MovingDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DeskStopReason {
    None,
    TargetReached,
    UserStop,
    Obstruction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct DeskStatus {
    pub homed: bool,
    pub needs_rehome: bool,
    pub motion: DeskMotionState,
    pub last_stop_reason: DeskStopReason,
    pub target_active: bool,
    pub target_position: i32,
    pub left_position: i32,
    pub right_position: i32,
    pub left_duty: u8,
    pub right_duty: u8,
    pub average_position: i32,
    pub skew_counts: i32,
    pub min_position: i32,
    pub max_position: i32,
    pub(super) left_min_position: i32,
    pub(super) left_max_position: i32,
    pub(super) right_min_position: i32,
    pub(super) right_max_position: i32,
}

pub struct DeskStatusStorage {
    pub(super) state: StaticCell<DeskStatusState>,
}

pub struct DeskStatusWatcher {
    pub(super) receiver: Receiver<'static, CriticalSectionRawMutex, DeskStatus, 4>,
}

#[derive(Clone, Copy)]
pub struct DeskStatusReader {
    pub(super) status_state: &'static DeskStatusState,
}

pub(super) struct DeskStatusState {
    status: Mutex<CriticalSectionRawMutex, RefCell<DeskStatus>>,
    pub(super) watch: Watch<CriticalSectionRawMutex, DeskStatus, 4>,
}

#[derive(Clone, Copy)]
pub(super) enum DeskSide {
    Left,
    Right,
}

impl Default for DeskStatusStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl DeskStatusStorage {
    pub const fn new() -> Self {
        Self {
            state: StaticCell::new(),
        }
    }
}

impl DeskStatusWatcher {
    pub async fn wait_for_change(&mut self) -> DeskStatus {
        self.receiver.changed().await
    }
}

impl DeskStatusReader {
    pub fn current(&self) -> DeskStatus {
        self.status_state.current()
    }
}

impl DeskStatusState {
    pub(super) fn new(initial_status: DeskStatus) -> Self {
        Self {
            status: Mutex::new(RefCell::new(initial_status)),
            watch: Watch::new(),
        }
    }

    pub(super) fn current(&self) -> DeskStatus {
        self.status.lock(|status| *status.borrow())
    }

    pub(super) fn publish(&self, status: DeskStatus) {
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

    pub(super) fn update(&self, update_fn: impl FnOnce(&mut DeskStatus)) -> DeskStatus {
        let (status, changed) = self.status.lock(|cached_status| {
            let mut cached_status = cached_status.borrow_mut();
            let previous = *cached_status;
            update_fn(&mut cached_status);
            cached_status.average_position =
                average_position(cached_status.left_position, cached_status.right_position);
            cached_status.skew_counts =
                abs_position_delta(cached_status.left_position, cached_status.right_position);
            cached_status.min_position = cached_status
                .left_min_position
                .max(cached_status.right_min_position);
            cached_status.max_position = cached_status
                .left_max_position
                .min(cached_status.right_max_position);
            (*cached_status, *cached_status != previous)
        });
        if changed {
            self.watch.sender().send(status);
        }
        status
    }

    pub(super) fn update_from_leg(&self, side: DeskSide, leg_status: LegStatus) {
        self.update(|status| match side {
            DeskSide::Left => {
                status.left_position = leg_status.position;
                status.left_duty = leg_status.duty;
                status.left_min_position = leg_status.min_position;
                status.left_max_position = leg_status.max_position;
            }
            DeskSide::Right => {
                status.right_position = leg_status.position;
                status.right_duty = leg_status.duty;
                status.right_min_position = leg_status.min_position;
                status.right_max_position = leg_status.max_position;
            }
        });
    }
}
