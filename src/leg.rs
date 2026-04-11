use esp_hal::mcpwm::PwmPeripheral;

use crate::motor::Motor;
use crate::quadrature::QuadratureWatcher;

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
        self.motor.drive_left(30);
    }

    pub fn move_down(&mut self) {
        self.motor.drive_right(30);
    }

    pub fn stop(&mut self) {
        self.motor.coast();
    }

    pub async fn move_up_step(&mut self, steps: u16) {
        let mut steps_taken = 0;
        while steps_taken < steps {
            self.move_up();
            self.quadrature_watcher.wait_for_change().await;
            steps_taken += 1;
        }
        self.stop();
    }

    pub async fn move_down_step(&mut self, steps: u16) {
        let mut steps_taken = 0;
        while steps_taken < steps {
            self.move_down();
            self.quadrature_watcher.wait_for_change().await;
            steps_taken += 1;
        }
        self.stop();
    }
}
