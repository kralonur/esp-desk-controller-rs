use embassy_time::{Duration, Instant, Timer};
use esp_hal::mcpwm::PwmPeripheral;

use crate::motor::Motor;
use crate::quadrature::{QuadratureDirection, QuadratureWatcher};

const STARTUP_DUTY: u16 = 99;
const RUN_DUTY: u16 = 30;
const HOMING_DUTY: u16 = 20;
const STARTUP_EVENTS: u16 = 8;
const HOMING_START_TIMEOUT: Duration = Duration::from_millis(800);
const HOMING_STALL_TIMEOUT: Duration = Duration::from_millis(600);
const HOMING_POLL_INTERVAL: Duration = Duration::from_millis(20);

pub struct Unhomed;

enum MotionState {
    Idle,
    MovingUp,
    MovingDown,
}

pub struct Ready {
    motion: MotionState,
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

    fn drive_home_down(&mut self) {
        self.motor.drive_right(HOMING_DUTY);
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

    pub async fn home_down(mut self) -> Result<Leg<'a, Ready, OP, PWM>, Self> {
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
                return Err(self);
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
        self.quadrature_watcher.reset_position();

        Ok(Leg {
            motor: self.motor,
            quadrature_watcher: self.quadrature_watcher,
            state: Ready {
                motion: MotionState::Idle,
            },
        })
    }
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, Ready, OP, PWM> {
    pub fn move_up(&mut self) {
        self.drive_up_run();
        self.state.motion = MotionState::MovingUp;
    }

    pub fn move_down(&mut self) {
        self.drive_down_run();
        self.state.motion = MotionState::MovingDown;
    }

    pub fn stop(&mut self) {
        self.coast();
        self.state.motion = MotionState::Idle;
    }

    pub async fn move_up_step(&mut self, steps: u16) {
        let mut steps_taken = 0;
        let startup_steps = steps.min(STARTUP_EVENTS);

        self.drive_up_boost();
        self.state.motion = MotionState::MovingUp;

        while steps_taken < startup_steps {
            self.quadrature_watcher
                .wait_for_direction(QuadratureDirection::Positive)
                .await;
            steps_taken += 1;
        }

        self.move_up();

        while steps_taken < steps {
            self.quadrature_watcher
                .wait_for_direction(QuadratureDirection::Positive)
                .await;
            steps_taken += 1;
        }

        self.stop();
    }

    pub async fn move_down_step(&mut self, steps: u16) {
        let mut steps_taken = 0;
        let startup_steps = steps.min(STARTUP_EVENTS);

        self.drive_down_boost();
        self.state.motion = MotionState::MovingDown;

        while steps_taken < startup_steps {
            self.quadrature_watcher
                .wait_for_direction(QuadratureDirection::Negative)
                .await;
            steps_taken += 1;
        }

        self.move_down();

        while steps_taken < steps {
            self.quadrature_watcher
                .wait_for_direction(QuadratureDirection::Negative)
                .await;
            steps_taken += 1;
        }

        self.stop();
    }
}
