use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::gpio::{Input, InputConfig, InputPin, Level, Pull};
use static_cell::StaticCell;

const HALL1_BIT: u8 = 0b10;
const HALL2_BIT: u8 = 0b01;

static QUADRATURE_STATE: StaticCell<QuadratureState> = StaticCell::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum QuadratureDirection {
    Positive,
    Negative,
    Invalid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QuadratureSnapshot {
    pub state: u8,
    pub position: i32,
    pub invalid_transitions: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct QuadratureEvent {
    pub channel: u8,
    pub level: Level,
    pub direction: QuadratureDirection,
    pub snapshot: QuadratureSnapshot,
}

struct QuadratureState {
    state: AtomicU8,
    position: AtomicI32,
    invalid_transitions: AtomicU32,
    events: Signal<CriticalSectionRawMutex, QuadratureEvent>,
}

pub struct Quadrature<'a> {
    hall1: Input<'a>,
    hall2: Input<'a>,
    state: &'static QuadratureState,
}

#[derive(Clone, Copy)]
pub struct QuadratureWatcher {
    state: &'static QuadratureState,
}

impl QuadratureSnapshot {
    pub fn hall1_high(self) -> bool {
        (self.state & HALL1_BIT) != 0
    }

    pub fn hall2_high(self) -> bool {
        (self.state & HALL2_BIT) != 0
    }
}

impl QuadratureState {
    const fn new() -> Self {
        Self {
            state: AtomicU8::new(0),
            position: AtomicI32::new(0),
            invalid_transitions: AtomicU32::new(0),
            events: Signal::new(),
        }
    }

    fn initialize(&self, initial_state: u8) {
        self.state.store(initial_state, Ordering::Release);
        self.position.store(0, Ordering::Release);
        self.invalid_transitions.store(0, Ordering::Release);
    }

    fn snapshot(&self) -> QuadratureSnapshot {
        QuadratureSnapshot {
            state: self.state.load(Ordering::Acquire),
            position: self.position.load(Ordering::Acquire),
            invalid_transitions: self.invalid_transitions.load(Ordering::Acquire),
        }
    }
}

impl Quadrature<'static> {
    pub fn new(
        hall1_pin: impl InputPin + 'static,
        hall2_pin: impl InputPin + 'static,
    ) -> (Self, QuadratureWatcher, QuadratureSnapshot) {
        let hall1 = Input::new(hall1_pin, hall_input_config());
        let hall2 = Input::new(hall2_pin, hall_input_config());
        let initial_state = state_from_levels(hall1.level(), hall2.level());
        let state = QUADRATURE_STATE.init(QuadratureState::new());

        state.initialize(initial_state);

        let snapshot = state.snapshot();
        let watcher = QuadratureWatcher { state };

        (
            Self {
                hall1,
                hall2,
                state,
            },
            watcher,
            snapshot,
        )
    }

    pub fn spawn(self, spawner: &Spawner) {
        spawner.must_spawn(monitor_hall(1, HALL1_BIT, self.hall1, self.state));
        spawner.must_spawn(monitor_hall(2, HALL2_BIT, self.hall2, self.state));
    }
}

impl QuadratureWatcher {
    pub fn snapshot(self) -> QuadratureSnapshot {
        self.state.snapshot()
    }

    pub async fn wait_for_change(self) -> QuadratureEvent {
        self.state.events.wait().await
    }
}

fn hall_input_config() -> InputConfig {
    InputConfig::default().with_pull(Pull::Up)
}

fn state_from_levels(hall1: Level, hall2: Level) -> u8 {
    let mut state = 0;

    if hall1 == Level::High {
        state |= HALL1_BIT;
    }

    if hall2 == Level::High {
        state |= HALL2_BIT;
    }

    state
}

fn quadrature_delta(old: u8, new: u8) -> i32 {
    match (old, new) {
        (0b00, 0b01) | (0b01, 0b11) | (0b11, 0b10) | (0b10, 0b00) => 1,
        (0b00, 0b10) | (0b10, 0b11) | (0b11, 0b01) | (0b01, 0b00) => -1,
        _ => 0,
    }
}

fn update_quadrature_state(state: &'static QuadratureState, channel: u8, bit: u8, level: Level) {
    loop {
        let old_state = state.state.load(Ordering::Acquire);
        let new_state = match level {
            Level::Low => old_state & !bit,
            Level::High => old_state | bit,
        };

        if new_state == old_state {
            return;
        }

        if state
            .state
            .compare_exchange(old_state, new_state, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            let delta = quadrature_delta(old_state, new_state);

            let snapshot = if delta == 0 {
                state.invalid_transitions.fetch_add(1, Ordering::AcqRel);
                QuadratureSnapshot {
                    state: new_state,
                    position: state.position.load(Ordering::Acquire),
                    invalid_transitions: state.invalid_transitions.load(Ordering::Acquire),
                }
            } else {
                let position = state.position.fetch_add(delta, Ordering::AcqRel) + delta;
                QuadratureSnapshot {
                    state: new_state,
                    position,
                    invalid_transitions: state.invalid_transitions.load(Ordering::Acquire),
                }
            };

            let direction = match delta {
                1 => QuadratureDirection::Positive,
                -1 => QuadratureDirection::Negative,
                _ => QuadratureDirection::Invalid,
            };

            state.events.signal(QuadratureEvent {
                channel,
                level,
                direction,
                snapshot,
            });

            return;
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn monitor_hall(
    channel: u8,
    bit: u8,
    mut pin: Input<'static>,
    state: &'static QuadratureState,
) {
    let mut level = pin.level();

    loop {
        pin.wait_for_any_edge().await;

        let next_level = pin.level();
        if next_level != level {
            level = next_level;
            update_quadrature_state(state, channel, bit, level);
        }
    }
}
