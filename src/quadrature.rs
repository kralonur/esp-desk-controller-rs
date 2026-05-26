//! PCNT-backed quadrature encoder driver.
//!
//! This module configures ESP PCNT units for hall sensor inputs, maintains a
//! software position accumulator across interrupts, and publishes position
//! events through watchers for leg status and movement logic.

use core::sync::atomic::{AtomicI32, AtomicU8, AtomicU32, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
    watch::{Receiver, Watch},
};
use esp_hal::{
    gpio::{Input, InputConfig, InputPin, Level, Pull},
    handler,
    interrupt::Priority,
    pcnt::{
        channel,
        unit::{Counter, Unit},
    },
    peripherals::PCNT,
};
use static_cell::StaticCell;

const HALL1_BIT: u8 = 0b10;
const HALL2_BIT: u8 = 0b01;

// PCNT references:
// - ESP-IDF PCNT docs, watch points/events and glitch filter:
//   https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/pcnt.html
// - ESP32Encoder PCNT accumulator pattern:
//   https://github.com/madhephaestus/ESP32Encoder
//
// `PCNT_GLITCH_FILTER_CYCLES` is the hardware input filter window in APB clock
// cycles. At the usual 80 MHz APB clock, 80 cycles is about 1 us; pulses shorter
// than that are treated as noise and ignored by the PCNT hardware.
const PCNT_GLITCH_FILTER_CYCLES: u16 = 80;

// `PCNT_EVENT_STEP` is the signed count threshold that wakes firmware. PCNT
// still counts every quadrature edge in hardware; the interrupt fires when the
// hardware count reaches +N or -N. The ISR adds that raw count into the software
// accumulator, clears the hardware counter back to zero, and wakes the async
// task to publish the latest position.
const PCNT_EVENT_STEP: i16 = 2;

static PCNT_UNIT0_BASE: AtomicI32 = AtomicI32::new(0);
static PCNT_UNIT1_BASE: AtomicI32 = AtomicI32::new(0);
static PCNT_UNIT0_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static PCNT_UNIT1_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Direction inferred from a quadrature state transition.
pub enum QuadratureDirection {
    Positive,
    Negative,
    Invalid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Current encoder state and accumulated position.
pub struct QuadratureSnapshot {
    pub state: u8,
    pub position: i32,
    pub invalid_transitions: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
/// Published quadrature edge event.
pub struct QuadratureEvent {
    pub channel: u8,
    pub level: Level,
    pub direction: QuadratureDirection,
    pub snapshot: QuadratureSnapshot,
}

#[derive(Clone)]
enum QuadratureCounter {
    Unit0(Counter<'static, 0>),
    Unit1(Counter<'static, 1>),
}

struct QuadratureState {
    state: AtomicU8,
    position: AtomicI32,
    invalid_transitions: AtomicU32,
    counter: QuadratureCounter,
    events: Watch<CriticalSectionRawMutex, QuadratureEvent, 4>,
}

/// Static storage used to initialize one quadrature channel.
pub struct QuadratureStorage {
    state: StaticCell<QuadratureState>,
}

/// PCNT-backed quadrature channel for one hardware PCNT unit.
///
/// `UNIT` must match the ESP PCNT unit used during construction.
pub struct Quadrature<const UNIT: usize> {
    hall1: Input<'static>,
    hall2: Input<'static>,
    unit: Unit<'static, UNIT>,
    state: &'static QuadratureState,
}

/// Async watcher for quadrature events and position snapshots.
pub struct QuadratureWatcher {
    receiver: Receiver<'static, CriticalSectionRawMutex, QuadratureEvent, 4>,
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

impl QuadratureCounter {
    fn get(&self) -> i32 {
        match self {
            Self::Unit0(counter) => counter.get() as i32,
            Self::Unit1(counter) => counter.get() as i32,
        }
    }

    fn base(&self) -> &'static AtomicI32 {
        match self {
            Self::Unit0(_) => &PCNT_UNIT0_BASE,
            Self::Unit1(_) => &PCNT_UNIT1_BASE,
        }
    }

    fn position(&self) -> i32 {
        self.base().load(Ordering::Acquire) + self.get()
    }

    fn reset(&self) {
        match self {
            Self::Unit0(_) => clear_pcnt_unit::<0>(),
            Self::Unit1(_) => clear_pcnt_unit::<1>(),
        }
        self.base().store(0, Ordering::Release);
    }
}

impl QuadratureState {
    fn new(counter: QuadratureCounter) -> Self {
        Self {
            state: AtomicU8::new(0),
            position: AtomicI32::new(0),
            invalid_transitions: AtomicU32::new(0),
            counter,
            events: Watch::new(),
        }
    }

    fn initialize(&self, initial_state: u8) {
        self.state.store(initial_state, Ordering::Release);
        self.position.store(0, Ordering::Release);
        self.counter.reset();
        self.invalid_transitions.store(0, Ordering::Release);
    }

    fn snapshot(&self) -> QuadratureSnapshot {
        QuadratureSnapshot {
            state: self.state.load(Ordering::Acquire),
            position: self.counter.position(),
            invalid_transitions: self.invalid_transitions.load(Ordering::Acquire),
        }
    }
}

impl Default for QuadratureStorage {
    fn default() -> Self {
        Self::new()
    }
}

impl QuadratureStorage {
    pub const fn new() -> Self {
        Self {
            state: StaticCell::new(),
        }
    }
}

impl Quadrature<0> {
    pub fn new(
        storage: &'static QuadratureStorage,
        unit: Unit<'static, 0>,
        hall1_pin: impl InputPin + 'static,
        hall2_pin: impl InputPin + 'static,
    ) -> (Self, QuadratureSnapshot) {
        Self::new_with_counter(
            storage,
            unit,
            QuadratureCounter::Unit0,
            hall1_pin,
            hall2_pin,
        )
    }

    pub fn spawn(self, spawner: &Spawner) {
        spawner.spawn(
            monitor_pcnt_unit0(self.hall1, self.hall2, self.unit, self.state)
                .expect("spawn pcnt unit0 monitor task"),
        );
    }
}

impl Quadrature<1> {
    pub fn new(
        storage: &'static QuadratureStorage,
        unit: Unit<'static, 1>,
        hall1_pin: impl InputPin + 'static,
        hall2_pin: impl InputPin + 'static,
    ) -> (Self, QuadratureSnapshot) {
        Self::new_with_counter(
            storage,
            unit,
            QuadratureCounter::Unit1,
            hall1_pin,
            hall2_pin,
        )
    }

    pub fn spawn(self, spawner: &Spawner) {
        spawner.spawn(
            monitor_pcnt_unit1(self.hall1, self.hall2, self.unit, self.state)
                .expect("spawn pcnt unit1 monitor task"),
        );
    }
}

impl<const UNIT: usize> Quadrature<UNIT> {
    fn new_with_counter(
        storage: &'static QuadratureStorage,
        unit: Unit<'static, UNIT>,
        counter: impl FnOnce(Counter<'static, UNIT>) -> QuadratureCounter,
        hall1_pin: impl InputPin + 'static,
        hall2_pin: impl InputPin + 'static,
    ) -> (Self, QuadratureSnapshot) {
        unit.pause();
        unit.set_filter(Some(PCNT_GLITCH_FILTER_CYCLES))
            .expect("valid PCNT glitch filter threshold");
        unit.set_threshold0(Some(PCNT_EVENT_STEP));
        unit.set_threshold1(Some(-PCNT_EVENT_STEP));
        unit.clear();

        let hall1 = Input::new(hall1_pin, hall_input_config());
        let hall2 = Input::new(hall2_pin, hall_input_config());
        let input1 = hall1.peripheral_input();
        let input2 = hall2.peripheral_input();

        let ch0 = &unit.channel0;
        ch0.set_ctrl_signal(input1.clone());
        ch0.set_edge_signal(input2.clone());
        ch0.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
        ch0.set_input_mode(channel::EdgeMode::Increment, channel::EdgeMode::Decrement);

        let ch1 = &unit.channel1;
        ch1.set_ctrl_signal(input2);
        ch1.set_edge_signal(input1);
        ch1.set_ctrl_mode(channel::CtrlMode::Reverse, channel::CtrlMode::Keep);
        ch1.set_input_mode(channel::EdgeMode::Decrement, channel::EdgeMode::Increment);

        let initial_state = state_from_levels(hall1.level(), hall2.level());
        let state = storage
            .state
            .init(QuadratureState::new(counter(unit.counter.clone())));

        state.initialize(initial_state);
        unit.listen();
        unit.resume();

        let snapshot = state.snapshot();

        (
            Self {
                hall1,
                hall2,
                unit,
                state,
            },
            snapshot,
        )
    }

    pub fn watcher(&self) -> QuadratureWatcher {
        let receiver = self
            .state
            .events
            .receiver()
            .expect("quadrature watch receiver limit reached");

        QuadratureWatcher {
            receiver,
            state: self.state,
        }
    }
}

impl QuadratureWatcher {
    pub fn resubscribe(&self) -> Self {
        let mut receiver = self
            .state
            .events
            .receiver()
            .expect("quadrature watch receiver limit reached");
        let _ = receiver.try_get();

        Self {
            receiver,
            state: self.state,
        }
    }

    pub fn snapshot(&self) -> QuadratureSnapshot {
        self.state.snapshot()
    }

    pub fn reset_position(&self) {
        self.state.counter.reset();
        self.state.position.store(0, Ordering::Release);
    }

    pub async fn wait_for_change(&mut self) -> QuadratureEvent {
        self.receiver.changed().await
    }

    pub async fn wait_for_direction(&mut self, direction: QuadratureDirection) -> QuadratureEvent {
        loop {
            let event = self.wait_for_change().await;
            if event.direction == direction {
                return event;
            }
        }
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

fn publish_position_if_changed(state: &'static QuadratureState, hall1: Level, hall2: Level) {
    state
        .state
        .store(state_from_levels(hall1, hall2), Ordering::Release);
    let snapshot = state.snapshot();
    let previous = state.position.swap(snapshot.position, Ordering::AcqRel);
    if snapshot.position == previous {
        return;
    }

    let direction = if snapshot.position > previous {
        QuadratureDirection::Positive
    } else {
        QuadratureDirection::Negative
    };

    state.events.sender().send(QuadratureEvent {
        channel: 0,
        level: hall1,
        direction,
        snapshot,
    });
}

fn clear_pcnt_unit<const UNIT: usize>() {
    let pcnt = PCNT::regs();
    pcnt.ctrl().modify(|_, w| w.cnt_rst_u(UNIT as u8).set_bit());
    pcnt.ctrl()
        .modify(|_, w| w.cnt_rst_u(UNIT as u8).clear_bit());
}

fn handle_pcnt_unit_interrupt<const UNIT: usize>(
    base: &'static AtomicI32,
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    let pcnt = PCNT::regs();
    if !pcnt.int_raw().read().cnt_thr_event_u(UNIT as u8).bit() {
        return;
    }

    let raw_position = pcnt.u_cnt(UNIT).read().cnt().bits() as i16 as i32;
    if raw_position != 0 {
        base.fetch_add(raw_position, Ordering::AcqRel);
        clear_pcnt_unit::<UNIT>();
    }

    pcnt.int_clr()
        .write(|w| w.cnt_thr_event_u(UNIT as u8).set_bit());
    signal.signal(());
}

#[handler(priority = Priority::Priority2)]
/// Interrupt handler for all configured PCNT quadrature units.
///
/// Register this once with `Pcnt::set_interrupt_handler` before spawning
/// quadrature monitor tasks.
pub fn pcnt_interrupt_handler() {
    handle_pcnt_unit_interrupt::<0>(&PCNT_UNIT0_BASE, &PCNT_UNIT0_SIGNAL);
    handle_pcnt_unit_interrupt::<1>(&PCNT_UNIT1_BASE, &PCNT_UNIT1_SIGNAL);
}

async fn monitor_pcnt(
    hall1: Input<'static>,
    hall2: Input<'static>,
    state: &'static QuadratureState,
    signal: &'static Signal<CriticalSectionRawMutex, ()>,
) {
    loop {
        signal.wait().await;
        publish_position_if_changed(state, hall1.level(), hall2.level());
    }
}

#[embassy_executor::task]
async fn monitor_pcnt_unit0(
    hall1: Input<'static>,
    hall2: Input<'static>,
    unit: Unit<'static, 0>,
    state: &'static QuadratureState,
) {
    monitor_pcnt(hall1, hall2, state, &PCNT_UNIT0_SIGNAL).await;
    unit.pause();
}

#[embassy_executor::task]
async fn monitor_pcnt_unit1(
    hall1: Input<'static>,
    hall2: Input<'static>,
    unit: Unit<'static, 1>,
    state: &'static QuadratureState,
) {
    monitor_pcnt(hall1, hall2, state, &PCNT_UNIT1_SIGNAL).await;
    unit.pause();
}
