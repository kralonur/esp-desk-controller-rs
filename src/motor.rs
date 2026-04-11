use esp_hal::{
    gpio::{Level, Output, OutputConfig, OutputPin, interconnect::PeripheralOutput},
    mcpwm::{
        PwmPeripheral,
        operator::{Operator, PwmPin},
    },
};

pub struct Motor<'a, const OP: u8, PWM: PwmPeripheral> {
    left_enable: Output<'a>,
    right_enable: Output<'a>,
    left_pwm: PwmPin<'a, PWM, OP, true>,
    right_pwm: PwmPin<'a, PWM, OP, false>,
}

impl<'a, const OP: u8, PWM: PwmPeripheral> Motor<'a, OP, PWM> {
    pub fn new(
        left_enable_pin: impl OutputPin + 'a,
        right_enable_pin: impl OutputPin + 'a,
        left_pwm_pin: impl PeripheralOutput<'a>,
        right_pwm_pin: impl PeripheralOutput<'a>,
        operator: Operator<'a, OP, PWM>,
    ) -> Self {
        let left_enable = Output::new(left_enable_pin, Level::Low, OutputConfig::default());
        let right_enable = Output::new(right_enable_pin, Level::Low, OutputConfig::default());

        let (mut left_pwm, mut right_pwm) = operator.with_pins(
            left_pwm_pin,
            esp_hal::mcpwm::operator::PwmPinConfig::UP_ACTIVE_HIGH,
            right_pwm_pin,
            esp_hal::mcpwm::operator::PwmPinConfig::UP_ACTIVE_HIGH,
        );

        left_pwm.set_timestamp(0);
        right_pwm.set_timestamp(0);

        Self {
            left_enable,
            right_enable,
            left_pwm,
            right_pwm,
        }
    }

    pub fn enable(&mut self) {
        self.left_enable.set_high();
        self.right_enable.set_high();
    }

    pub fn disable(&mut self) {
        self.left_enable.set_low();
        self.right_enable.set_low();
    }

    pub fn coast(&mut self) {
        self.left_pwm.set_timestamp(0);
        self.right_pwm.set_timestamp(0);
    }

    pub fn drive_left(&mut self, duty: u16) {
        self.left_pwm.set_timestamp(duty);
        self.right_pwm.set_timestamp(0);
    }

    pub fn drive_right(&mut self, duty: u16) {
        self.left_pwm.set_timestamp(0);
        self.right_pwm.set_timestamp(duty);
    }
}
