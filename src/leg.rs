use defmt::Format;
use embassy_time::{Duration, Instant, Timer, with_deadline};
use esp_hal::mcpwm::PwmPeripheral;

use crate::motor::Motor;
use crate::quadrature::{QuadratureDirection, QuadratureWatcher};

const STARTUP_DUTY: u16 = 99;
const RUN_DUTY: u16 = 30;
const SLOW_DUTY: u16 = 15;
const HOMING_DUTY: u16 = 20;
const STARTUP_EVENTS: u16 = 8;
const HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
const HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);
const HOMING_BACKOFF_STEPS: u16 = 20;
const DEFAULT_MAX_POSITION: i32 = 2_000;
const MOVE_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const TARGET_SLOW_ZONE: i32 = 10;
const TARGET_TOLERANCE: i32 = 5;

pub struct Unhomed;

enum MotionState {
    Idle,
    MovingUp,
    MovingDown,
}

pub struct Ready {
    min_position: i32,
    max_position: i32,
    up_direction: QuadratureDirection,
    down_direction: QuadratureDirection,
    motion: MotionState,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Format)]
pub enum LegError {
    HomingStartTimeout,
    MoveTimeout,
}

pub struct Leg<'a, State, const OP: u8, PWM: PwmPeripheral> {
    motor: Motor<'a, OP, PWM>,
    quadrature_watcher: QuadratureWatcher,
    state: State,
}

impl<'a, State, const OP: u8, PWM: PwmPeripheral> Leg<'a, State, OP, PWM> {
    pub fn position(&self) -> i32 {
        self.quadrature_watcher.snapshot().position
    }

    fn coast(&mut self) {
        self.motor.coast();
    }

    fn drive_up_boost(&mut self) {
        self.motor.drive_left(STARTUP_DUTY);
    }

    fn drive_down_boost(&mut self) {
        self.motor.drive_right(STARTUP_DUTY);
    }

    fn drive_up_run(&mut self) {
        self.motor.drive_left(RUN_DUTY);
    }

    fn drive_down_run(&mut self) {
        self.motor.drive_right(RUN_DUTY);
    }

    fn drive_up_slow(&mut self) {
        self.motor.drive_left(SLOW_DUTY);
    }

    fn drive_down_slow(&mut self) {
        self.motor.drive_right(SLOW_DUTY);
    }

    fn drive_home_down(&mut self) {
        self.motor.drive_right(HOMING_DUTY);
    }

    async fn move_up_steps(&mut self, expected_direction: QuadratureDirection, steps: u16) {
        if steps == 0 {
            return;
        }

        if expected_direction == QuadratureDirection::Invalid {
            return;
        }

        self.drive_up_boost();

        let startup_steps = steps.min(STARTUP_EVENTS);
        let mut steps_taken = 0;

        while steps_taken < startup_steps {
            self.quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            steps_taken += 1;
        }

        self.drive_up_run();

        while steps_taken < steps {
            self.quadrature_watcher
                .wait_for_direction(expected_direction)
                .await;
            steps_taken += 1;
        }

        self.coast();
    }
}

fn opposite_direction(direction: QuadratureDirection) -> QuadratureDirection {
    match direction {
        QuadratureDirection::Positive => QuadratureDirection::Negative,
        QuadratureDirection::Negative => QuadratureDirection::Positive,
        QuadratureDirection::Invalid => QuadratureDirection::Invalid,
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Unhomed, OP, PWM> {
    pub fn new(motor: Motor<'a, OP, PWM>, quadrature_watcher: QuadratureWatcher) -> Self {
        Self {
            motor,
            quadrature_watcher,
            state: Unhomed,
        }
    }

    pub async fn home_down(mut self) -> Result<Leg<'a, Ready, OP, PWM>, (Self, LegError)> {
        let start_position = self.position();
        let start_deadline = Instant::now() + HOMING_START_TIMEOUT;

        self.drive_down_boost();

        let homing_direction = loop {
            let current_position = self.position();

            if current_position > start_position {
                break QuadratureDirection::Positive;
            }

            if current_position < start_position {
                break QuadratureDirection::Negative;
            }

            if Instant::now() >= start_deadline {
                self.coast();
                return Err((self, LegError::HomingStartTimeout));
            }

            Timer::after(HOMING_POLL_INTERVAL).await;
        };

        let mut last_position = self.position();
        let mut last_progress_at = Instant::now();
        self.drive_home_down();

        loop {
            Timer::after(HOMING_POLL_INTERVAL).await;

            let current_position = self.position();
            let progressed = match homing_direction {
                QuadratureDirection::Positive => current_position > last_position,
                QuadratureDirection::Negative => current_position < last_position,
                QuadratureDirection::Invalid => false,
            };

            if progressed {
                last_position = current_position;
                last_progress_at = Instant::now();
                continue;
            }

            if Instant::now().saturating_duration_since(last_progress_at) >= HOMING_STALL_TIMEOUT {
                break;
            }
        }

        self.coast();
        let backoff_direction = opposite_direction(homing_direction);

        self.move_up_steps(backoff_direction, HOMING_BACKOFF_STEPS)
            .await;
        self.quadrature_watcher.reset_position();

        Ok(Leg {
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            state: Ready {
                min_position: 0,
                max_position: DEFAULT_MAX_POSITION,
                up_direction: opposite_direction(homing_direction),
                down_direction: homing_direction,
                motion: MotionState::Idle,
            },
        })
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Ready, OP, PWM> {
    pub fn min_position(&self) -> i32 {
        self.state.min_position
    }

    pub fn max_position(&self) -> i32 {
        self.state.max_position
    }

    pub fn set_max_position(&mut self, max_position: i32) {
        self.state.max_position = max_position.max(self.state.min_position);
    }

    pub fn clamp_target(&self, target_position: i32) -> i32 {
        target_position.clamp(self.state.min_position, self.state.max_position)
    }

    pub fn move_up(&mut self) {
        if self.position() >= self.state.max_position {
            self.stop();
            return;
        }

        self.drive_up_run();
        self.state.motion = MotionState::MovingUp;
    }

    pub fn move_down(&mut self) {
        if self.position() <= self.state.min_position {
            self.stop();
            return;
        }

        self.drive_down_run();
        self.state.motion = MotionState::MovingDown;
    }

    pub fn stop(&mut self) {
        self.coast();
        self.state.motion = MotionState::Idle;
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
        let allowed_steps = match direction {
            QuadratureDirection::Positive => {
                (self.state.max_position - self.position()).max(0) as u16
            }
            QuadratureDirection::Negative => {
                (self.position() - self.state.min_position).max(0) as u16
            }
            QuadratureDirection::Invalid => 0,
        };
        let steps = steps.min(allowed_steps);

        if steps == 0 {
            self.stop();
            return Ok(());
        }

        let mut steps_taken = 0;
        let startup_steps = steps.min(STARTUP_EVENTS);

        match direction {
            QuadratureDirection::Positive => {
                self.drive_up_boost();
                self.state.motion = MotionState::MovingUp;
            }
            QuadratureDirection::Negative => {
                self.drive_down_boost();
                self.state.motion = MotionState::MovingDown;
            }
            QuadratureDirection::Invalid => {
                self.stop();
                return Ok(());
            }
        }

        while steps_taken < startup_steps {
            self.wait_for_progress(direction, MOVE_STALL_TIMEOUT).await?;
            steps_taken += 1;
        }

        match direction {
            QuadratureDirection::Positive => self.move_up(),
            QuadratureDirection::Negative => self.move_down(),
            QuadratureDirection::Invalid => {}
        }

        while steps_taken < steps {
            self.wait_for_progress(direction, MOVE_STALL_TIMEOUT).await?;
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
        let target_position = self.clamp_target(target_position);
        let current_position = self.position();

        if (target_position - current_position).abs() <= TARGET_TOLERANCE {
            self.stop();
            return Ok(());
        }

        if target_position > current_position {
            self.drive_up_boost();
            self.state.motion = MotionState::MovingUp;

            loop {
                let event = self
                    .wait_for_progress(self.state.up_direction, MOVE_STALL_TIMEOUT)
                    .await?;
                let error = target_position - event.snapshot.position;

                if error <= TARGET_TOLERANCE {
                    self.stop();
                    break;
                }

                if error <= TARGET_SLOW_ZONE {
                    self.drive_up_slow();
                    self.state.motion = MotionState::MovingUp;
                } else {
                    self.move_up();
                }
            }
        } else if target_position < current_position {
            self.drive_down_boost();
            self.state.motion = MotionState::MovingDown;

            loop {
                let event = self
                    .wait_for_progress(self.state.down_direction, MOVE_STALL_TIMEOUT)
                    .await?;
                let error = event.snapshot.position - target_position;

                if error <= TARGET_TOLERANCE {
                    self.stop();
                    break;
                }

                if error <= TARGET_SLOW_ZONE {
                    self.drive_down_slow();
                    self.state.motion = MotionState::MovingDown;
                } else {
                    self.move_down();
                }
            }
        } else {
            self.stop();
        }

        Ok(())
    }
}
