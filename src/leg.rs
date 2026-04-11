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

pub struct Leg<'a, const OP: u8, PWM: PwmPeripheral> {
    motor: Motor<'a, OP, PWM>,
    quadrature_watcher: QuadratureWatcher,
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Leg<'a, OP, PWM> {
    pub fn new(motor: Motor<'a, OP, PWM>, quadrature_watcher: QuadratureWatcher) -> Self {
        Self {
            motor,
            quadrature_watcher,
        }
    }

    pub fn move_up(&mut self) {
        self.motor.drive_left(RUN_DUTY);
    }

    pub fn move_down(&mut self) {
        self.motor.drive_right(RUN_DUTY);
    }

    pub fn stop(&mut self) {
        self.motor.coast();
    }

    fn move_up_boost(&mut self) {
        self.motor.drive_left(STARTUP_DUTY);
    }

    fn move_down_boost(&mut self) {
        self.motor.drive_right(STARTUP_DUTY);
    }

    fn move_home_down(&mut self) {
        self.motor.drive_right(HOMING_DUTY);
    }

    pub async fn home_down(&mut self) -> bool {
        let start_position = self.quadrature_watcher.snapshot().position;
        let start_deadline = Instant::now() + HOMING_START_TIMEOUT;

        // Break static friction first, then switch to slow homing once motion is confirmed.
        self.move_down_boost();

        let homing_direction = loop {
            let current_position = self.quadrature_watcher.snapshot().position;

            if current_position > start_position {
                break QuadratureDirection::Positive;
            }

            if current_position < start_position {
                break QuadratureDirection::Negative;
            }

            if Instant::now() >= start_deadline {
                self.stop();
                return false;
            }

            Timer::after(HOMING_POLL_INTERVAL).await;
        };

        let mut last_position = self.quadrature_watcher.snapshot().position;
        let mut last_progress_at = Instant::now();
        self.move_home_down();

        loop {
            Timer::after(HOMING_POLL_INTERVAL).await;

            let current_position = self.quadrature_watcher.snapshot().position;
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

        self.stop();
        self.quadrature_watcher.reset_position();
        true
    }

    pub async fn move_up_step(&mut self, steps: u16) {
        let mut steps_taken = 0;
        let startup_steps = steps.min(STARTUP_EVENTS);

        self.move_up_boost();
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

        self.move_down_boost();
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
