use core::cell::RefCell;

use defmt::Format;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::Mutex,
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;

use crate::config::{LegRuntimeConfig, RuntimeConfigReader};
use crate::motor::Motor;
use crate::quadrature::{QuadratureDirection, QuadratureWatcher};
use crate::units::{DutyPercent, PWM_TIMER_MAX_TICKS, PositionSign};
use static_cell::StaticCell;

pub struct Unhomed;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum DriveSide {
    Left,
    Right,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LegConfig {
    pub up_drive: DriveSide,
    pub up_direction: QuadratureDirection,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum MotionState {
    Idle,
    MovingUp,
    MovingDown,
}

pub struct Ready {
    min_position: i32,
    max_position: i32,
    position_sign: PositionSign,
    up_direction: QuadratureDirection,
    down_direction: QuadratureDirection,
    motion: MotionState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct LegStatus {
    pub position: i32,
    pub min_position: i32,
    pub max_position: i32,
    pub motion: MotionState,
    pub homed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub struct LegProgress {
    pub position: i32,
}

pub struct LegStatusStorage {
    state: StaticCell<LegStatusState>,
}

pub struct LegStatusWatcher {
    receiver: Receiver<'static, CriticalSectionRawMutex, LegStatus, 4>,
}

pub struct LegProgressWatcher {
    quadrature_watcher: QuadratureWatcher,
    position_sign: PositionSign,
}

struct LegStatusState {
    position_sign: PositionSign,
    status: Mutex<CriticalSectionRawMutex, RefCell<LegStatus>>,
    watch: Watch<CriticalSectionRawMutex, LegStatus, 4>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum LegError {
    HomingStartTimeout,
    PolarityMismatch,
    MoveTimeout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveMode {
    Stop,
    UpBoost,
    UpRun,
    UpSlow,
    DownBoost,
    DownSlow,
    HomeDown,
}

pub struct Leg<'a, State, const OP: u8, PWM: PwmPeripheral> {
    config: LegConfig,
    runtime_config_reader: RuntimeConfigReader,
    motor: Motor<'a, OP, PWM>,
    quadrature_watcher: QuadratureWatcher,
    status_state: &'static LegStatusState,
    state: State,
}

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    fn configured_up_direction(&self) -> QuadratureDirection {
        self.config.up_direction
    }

    fn configured_down_direction(&self) -> QuadratureDirection {
        opposite_direction(self.config.up_direction)
    }

    fn position_sign(&self) -> PositionSign {
        match self.config.up_direction {
            QuadratureDirection::Positive | QuadratureDirection::Invalid => PositionSign::Positive,
            QuadratureDirection::Negative => PositionSign::Negative,
        }
    }

    pub fn logical_position(&self) -> i32 {
        self.current_status().position
    }

    pub fn encoder_position(&self) -> i32 {
        self.quadrature_watcher.snapshot().position
    }

    pub fn reset_position(&self) {
        self.quadrature_watcher.reset_position();
    }

    pub fn up_direction(&self) -> QuadratureDirection {
        self.configured_up_direction()
    }

    pub fn down_direction(&self) -> QuadratureDirection {
        self.configured_down_direction()
    }

    fn current_status(&self) -> LegStatus {
        self.status_state.current()
    }

    fn coast(&mut self) {
        self.motor.coast();
    }

    pub fn apply_drive_mode(&mut self, mode: DriveMode, runtime_config: LegRuntimeConfig) {
        match mode {
            DriveMode::Stop => self.coast(),
            DriveMode::UpBoost => self.drive_up_boost(runtime_config),
            DriveMode::UpRun => self.drive_up_run(runtime_config),
            DriveMode::UpSlow => self.drive_up_slow(runtime_config),
            DriveMode::DownBoost => self.drive_down_boost(runtime_config),
            DriveMode::DownSlow => self.drive_down_slow(runtime_config),
            DriveMode::HomeDown => self.drive_home_down(runtime_config),
        }

        let motion = match mode {
            DriveMode::Stop => MotionState::Idle,
            DriveMode::UpBoost | DriveMode::UpRun | DriveMode::UpSlow => MotionState::MovingUp,
            DriveMode::DownBoost | DriveMode::DownSlow | DriveMode::HomeDown => {
                MotionState::MovingDown
            }
        };

        self.send_status(LegStatus {
            motion,
            ..self.current_status()
        });
    }

    fn send_status(&self, status: LegStatus) {
        self.status_state.publish(status);
    }

    pub fn drive_up_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingUp {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.drive_side(self.config.up_drive, duty, runtime_config);
        self.send_status(LegStatus {
            motion: MotionState::MovingUp,
            ..self.current_status()
        });
    }

    pub fn drive_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingDown {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            duty,
            runtime_config,
        );
        self.send_status(LegStatus {
            motion: MotionState::MovingDown,
            ..self.current_status()
        });
    }

    pub fn drive_home_down_duty(&mut self, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = if self.current_status().motion != MotionState::MovingDown {
            runtime_config.startup_duty()
        } else {
            duty
        };
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            duty,
            runtime_config,
        );
        self.send_status(LegStatus {
            motion: MotionState::MovingDown,
            ..self.current_status()
        });
    }

    fn drive_up_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            self.config.up_drive,
            runtime_config.startup_duty(),
            runtime_config,
        );
    }

    fn drive_down_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.startup_duty(),
            runtime_config,
        );
    }

    fn drive_up_run(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            self.config.up_drive,
            runtime_config.run_duty(),
            runtime_config,
        );
    }

    fn drive_down_run(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.run_duty(),
            runtime_config,
        );
    }

    fn drive_up_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            self.config.up_drive,
            runtime_config.slow_duty(),
            runtime_config,
        );
    }

    fn drive_down_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.slow_duty(),
            runtime_config,
        );
    }

    fn drive_home_down(&mut self, runtime_config: LegRuntimeConfig) {
        self.drive_side(
            opposite_drive_side(self.config.up_drive),
            runtime_config.homing_duty(),
            runtime_config,
        );
    }

    fn drive_side(&mut self, side: DriveSide, duty: DutyPercent, runtime_config: LegRuntimeConfig) {
        let duty = duty
            .min(runtime_config.max_duty())
            .to_pwm_timestamp(PWM_TIMER_MAX_TICKS);
        match side {
            DriveSide::Left => self.motor.drive_left(duty),
            DriveSide::Right => self.motor.drive_right(duty),
        }
    }

    async fn move_up_steps(
        &mut self,
        expected_direction: QuadratureDirection,
        steps: u16,
        runtime_config: LegRuntimeConfig,
    ) {
        if steps == 0 {
            return;
        }

        if expected_direction == QuadratureDirection::Invalid {
            return;
        }

        self.apply_drive_mode(DriveMode::UpBoost, runtime_config);

        let startup_steps = steps.min(runtime_config.startup_events().get_u16());
        let mut steps_taken = 0;

        while steps_taken < startup_steps {
            self.quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            steps_taken += 1;
        }

        self.apply_drive_mode(DriveMode::UpRun, runtime_config);

        while steps_taken < steps {
            self.quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            steps_taken += 1;
        }

        self.apply_drive_mode(DriveMode::Stop, runtime_config);
    }

    pub fn status_watcher(&self) -> LegStatusWatcher {
        let receiver = self
            .status_state
            .watch
            .receiver()
            .expect("leg status watch receiver limit reached");

        LegStatusWatcher { receiver }
    }

    pub fn progress_watcher(&self) -> LegProgressWatcher {
        LegProgressWatcher {
            quadrature_watcher: self.quadrature_watcher.resubscribe(),
            position_sign: self.position_sign(),
        }
    }
}

impl LegStatusState {
    fn new(initial_status: LegStatus, position_sign: PositionSign) -> Self {
        Self {
            position_sign,
            status: Mutex::new(RefCell::new(initial_status)),
            watch: Watch::new(),
        }
    }

    fn current(&self) -> LegStatus {
        self.status.lock(|status| *status.borrow())
    }

    fn publish(&self, status: LegStatus) {
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

    fn logical_position(&self, raw_position: i32) -> i32 {
        self.position_sign.apply(raw_position)
    }

    fn publish_position(&self, raw_position: i32) {
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
async fn mirror_quadrature_to_leg_status(
    mut quadrature_watcher: QuadratureWatcher,
    status_state: &'static LegStatusState,
) {
    loop {
        let event = quadrature_watcher.wait_for_change().await;
        status_state.publish_position(event.snapshot.position);
    }
}

fn opposite_direction(direction: QuadratureDirection) -> QuadratureDirection {
    match direction {
        QuadratureDirection::Positive => QuadratureDirection::Negative,
        QuadratureDirection::Negative => QuadratureDirection::Positive,
        QuadratureDirection::Invalid => QuadratureDirection::Invalid,
    }
}

fn opposite_drive_side(side: DriveSide) -> DriveSide {
    match side {
        DriveSide::Left => DriveSide::Right,
        DriveSide::Right => DriveSide::Left,
    }
}

fn progressed_in_direction(
    last_position: i32,
    current_position: i32,
    direction: QuadratureDirection,
) -> bool {
    match direction {
        QuadratureDirection::Positive => current_position > last_position,
        QuadratureDirection::Negative => current_position < last_position,
        QuadratureDirection::Invalid => false,
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

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Unhomed, OP, PWM> {
    fn status(&self) -> LegStatus {
        LegStatus {
            position: self.logical_position(),
            min_position: 0,
            max_position: 0,
            motion: MotionState::Idle,
            homed: false,
        }
    }

    pub fn new(
        storage: &'static LegStatusStorage,
        config: LegConfig,
        runtime_config_reader: RuntimeConfigReader,
        motor: Motor<'a, OP, PWM>,
        quadrature_watcher: QuadratureWatcher,
        status_quadrature_watcher: QuadratureWatcher,
        spawner: &Spawner,
    ) -> Self {
        let initial_status = LegStatus {
            position: quadrature_watcher.snapshot().position,
            min_position: 0,
            max_position: 0,
            motion: MotionState::Idle,
            homed: false,
        };
        let status_state = storage.state.init(LegStatusState::new(
            initial_status,
            match config.up_direction {
                QuadratureDirection::Positive | QuadratureDirection::Invalid => {
                    PositionSign::Positive
                }
                QuadratureDirection::Negative => PositionSign::Negative,
            },
        ));
        spawner.must_spawn(mirror_quadrature_to_leg_status(
            status_quadrature_watcher,
            status_state,
        ));
        let leg = Self {
            config,
            runtime_config_reader,
            motor,
            quadrature_watcher,
            status_state,
            state: Unhomed,
        };
        leg.send_status(leg.status());
        leg
    }

    pub async fn home_down(mut self) -> Result<Leg<'a, Ready, OP, PWM>, (Self, LegError)> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let start_position = self.encoder_position();
        let start_deadline = Instant::now() + runtime_config.homing_start_timeout();
        let down_direction = self.configured_down_direction();
        let up_direction = self.configured_up_direction();

        self.apply_drive_mode(DriveMode::DownBoost, runtime_config);

        loop {
            let current_position = self.encoder_position();

            if progressed_in_direction(start_position, current_position, down_direction) {
                break;
            }

            if progressed_in_direction(start_position, current_position, up_direction) {
                self.coast();
                return Err((self, LegError::PolarityMismatch));
            }

            if Instant::now() >= start_deadline {
                self.coast();
                return Err((self, LegError::HomingStartTimeout));
            }

            Timer::after(runtime_config.homing_poll_interval()).await;
        }

        let mut last_position = self.encoder_position();
        let mut last_progress_at = Instant::now();
        self.apply_drive_mode(DriveMode::HomeDown, runtime_config);

        loop {
            Timer::after(runtime_config.homing_poll_interval()).await;

            let current_position = self.encoder_position();
            let progressed =
                progressed_in_direction(last_position, current_position, down_direction);

            if progressed {
                last_position = current_position;
                last_progress_at = Instant::now();
                continue;
            }

            if Instant::now().saturating_duration_since(last_progress_at)
                >= runtime_config.homing_stall_timeout()
            {
                break;
            }
        }

        self.apply_drive_mode(DriveMode::Stop, runtime_config);
        self.move_up_steps(
            up_direction,
            runtime_config.homing_backoff_steps().get_u16(),
            runtime_config,
        )
        .await;
        self.quadrature_watcher.reset_position();
        let leg = self.into_ready_with_config(runtime_config);
        leg.send_status(leg.status());

        Ok(leg)
    }

    pub fn into_ready_with_config(
        self,
        runtime_config: LegRuntimeConfig,
    ) -> Leg<'a, Ready, OP, PWM> {
        let position_sign = self.position_sign();
        let up_direction = self.configured_up_direction();
        let down_direction = self.configured_down_direction();

        self.status_state
            .publish_position(self.quadrature_watcher.snapshot().position);

        Leg {
            config: self.config,
            runtime_config_reader: self.runtime_config_reader,
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            status_state: self.status_state,
            state: Ready {
                min_position: 0,
                max_position: runtime_config.default_max_position().get(),
                position_sign,
                up_direction,
                down_direction,
                motion: MotionState::Idle,
            },
        }
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Ready, OP, PWM> {
    fn logical_position_from_raw(&self, raw_position: i32) -> i32 {
        self.state.position_sign.apply(raw_position)
    }

    pub fn status(&self) -> LegStatus {
        let mut status = self.current_status();
        status.min_position = self.state.min_position;
        status.max_position = self.state.max_position;
        status.homed = true;
        status
    }

    pub fn motion(&self) -> MotionState {
        self.current_status().motion
    }

    pub fn publish_status(&self) {
        self.send_status(self.status());
    }

    pub fn into_unhomed(mut self) -> Leg<'a, Unhomed, OP, PWM> {
        self.coast();
        self.send_status(LegStatus {
            position: self.logical_position(),
            min_position: 0,
            max_position: 0,
            motion: MotionState::Idle,
            homed: false,
        });

        Leg {
            config: self.config,
            runtime_config_reader: self.runtime_config_reader,
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            status_state: self.status_state,
            state: Unhomed,
        }
    }

    pub fn start_up_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingUp;
        self.apply_drive_mode(DriveMode::UpBoost, runtime_config);
    }

    pub fn start_up_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingUp;
        self.apply_drive_mode(DriveMode::UpSlow, runtime_config);
    }

    pub fn start_down_boost(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingDown;
        self.apply_drive_mode(DriveMode::DownBoost, runtime_config);
    }

    pub fn start_down_slow(&mut self, runtime_config: LegRuntimeConfig) {
        self.state.motion = MotionState::MovingDown;
        self.apply_drive_mode(DriveMode::DownSlow, runtime_config);
    }

    pub fn stop_for_desk(&mut self) {
        self.stop();
    }

    pub fn min_position(&self) -> i32 {
        self.state.min_position
    }

    pub fn max_position(&self) -> i32 {
        self.state.max_position
    }

    pub fn set_max_position(&mut self, max_position: i32) {
        self.state.max_position = max_position.max(self.state.min_position);
        self.send_status(self.status());
    }

    pub fn clamp_target(&self, target_position: i32) -> i32 {
        target_position.clamp(self.state.min_position, self.state.max_position)
    }

    pub fn move_up(&mut self) {
        let runtime_config = self.runtime_config_reader.current().leg();
        if self.logical_position() >= self.state.max_position {
            self.stop();
            return;
        }

        self.drive_up_run(runtime_config);
        self.state.motion = MotionState::MovingUp;
        self.send_status(self.status());
    }

    pub fn move_down(&mut self) {
        let runtime_config = self.runtime_config_reader.current().leg();
        if self.logical_position() <= self.state.min_position {
            self.stop();
            return;
        }

        self.drive_down_run(runtime_config);
        self.state.motion = MotionState::MovingDown;
        self.send_status(self.status());
    }

    pub fn stop(&mut self) {
        self.coast();
        self.state.motion = MotionState::Idle;
        self.send_status(self.status());
    }

    async fn wait_for_progress(
        &mut self,
        direction: QuadratureDirection,
        timeout: Duration,
    ) -> Result<crate::quadrature::QuadratureEvent, LegError> {
        let deadline = Instant::now() + timeout;

        loop {
            match with_deadline(deadline, self.quadrature_watcher.wait_for_change()).await {
                Ok(event) if event.direction == direction => return Ok(event),
                Ok(_) => {}
                Err(_) => {
                    self.stop();
                    return Err(LegError::MoveTimeout);
                }
            }
        }
    }

    async fn move_steps(
        &mut self,
        direction: QuadratureDirection,
        steps: u16,
    ) -> Result<(), LegError> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let allowed_steps = if direction == self.state.up_direction {
            (self.state.max_position - self.logical_position()).max(0) as u16
        } else if direction == self.state.down_direction {
            (self.logical_position() - self.state.min_position).max(0) as u16
        } else {
            0
        };
        let steps = steps.min(allowed_steps);

        if steps == 0 {
            self.stop();
            return Ok(());
        }

        let mut steps_taken = 0;
        let startup_steps = steps.min(runtime_config.startup_events().get_u16());

        if direction == self.state.up_direction {
            self.start_up_boost(runtime_config);
        } else if direction == self.state.down_direction {
            self.start_down_boost(runtime_config);
        } else {
            self.stop();
            return Ok(());
        }

        while steps_taken < startup_steps {
            self.wait_for_progress(direction, runtime_config.move_stall_timeout())
                .await?;
            steps_taken += 1;
        }

        if direction == self.state.up_direction {
            self.drive_up_run(runtime_config);
            self.state.motion = MotionState::MovingUp;
            self.send_status(self.status());
        } else if direction == self.state.down_direction {
            self.drive_down_run(runtime_config);
            self.state.motion = MotionState::MovingDown;
            self.send_status(self.status());
        }

        while steps_taken < steps {
            self.wait_for_progress(direction, runtime_config.move_stall_timeout())
                .await?;
            steps_taken += 1;
        }

        self.stop();
        Ok(())
    }

    pub async fn move_up_step(&mut self, steps: u16) -> Result<(), LegError> {
        self.move_steps(self.state.up_direction, steps).await
    }

    pub async fn move_down_step(&mut self, steps: u16) -> Result<(), LegError> {
        self.move_steps(self.state.down_direction, steps).await
    }

    pub async fn move_to(&mut self, target_position: i32) -> Result<(), LegError> {
        let runtime_config = self.runtime_config_reader.current().leg();
        let target_position = self.clamp_target(target_position);
        let current_position = self.logical_position();

        if (target_position - current_position).abs() <= runtime_config.target_tolerance().get() {
            self.stop();
            return Ok(());
        }

        if target_position > current_position {
            self.start_up_boost(runtime_config);

            loop {
                let event = self
                    .wait_for_progress(self.state.up_direction, runtime_config.move_stall_timeout())
                    .await?;
                let error =
                    target_position - self.logical_position_from_raw(event.snapshot.position);

                if error <= runtime_config.target_tolerance().get() {
                    self.stop();
                    break;
                }

                if error <= runtime_config.target_slow_zone().get() {
                    self.start_up_slow(runtime_config);
                } else {
                    self.drive_up_run(runtime_config);
                    self.state.motion = MotionState::MovingUp;
                    self.send_status(self.status());
                }
            }
        } else if target_position < current_position {
            self.start_down_boost(runtime_config);

            loop {
                let event = self
                    .wait_for_progress(
                        self.state.down_direction,
                        runtime_config.move_stall_timeout(),
                    )
                    .await?;
                let error =
                    self.logical_position_from_raw(event.snapshot.position) - target_position;

                if error <= runtime_config.target_tolerance().get() {
                    self.stop();
                    break;
                }

                if error <= runtime_config.target_slow_zone().get() {
                    self.start_down_slow(runtime_config);
                } else {
                    self.drive_down_run(runtime_config);
                    self.state.motion = MotionState::MovingDown;
                    self.send_status(self.status());
                }
            }
        } else {
            self.stop();
        }

        Ok(())
    }
}
