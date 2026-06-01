//! Board-level resource mapping for the ESP32-S3 SuperMini wiring used here.
//!
//! GPIO choices are compile-time HAL resources, so adapting this firmware to a
//! different board or wiring layout should happen in this module.

use esp_hal::peripherals::{
    FLASH, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO8, GPIO9, GPIO10, GPIO11, GPIO12, GPIO13,
    MCPWM0, PCNT, Peripherals, SW_INTERRUPT, TIMG0, WIFI,
};

/// Hardware resources consumed by the firmware application.
pub struct AppResources {
    pub flash: FLASH<'static>,
    pub wifi: WIFI<'static>,
    pub timer_group0: TIMG0<'static>,
    pub software_interrupt: SW_INTERRUPT<'static>,
    pub mcpwm0: MCPWM0<'static>,
    pub pcnt: PCNT<'static>,
    pub desk_pins: DeskPins,
}

/// GPIO resources for both physical desk legs.
pub struct DeskPins {
    pub left_leg: LeftLegPins,
    pub right_leg: RightLegPins,
}

/// GPIO resources for one desk leg.
pub struct LegPins<DriveLeftEnable, DriveRightEnable, DriveLeftPwm, DriveRightPwm, HallA, HallB> {
    pub drive_left_enable: DriveLeftEnable,
    pub drive_right_enable: DriveRightEnable,
    pub drive_left_pwm: DriveLeftPwm,
    pub drive_right_pwm: DriveRightPwm,
    pub hall_a: HallA,
    pub hall_b: HallB,
}

type LeftLegPins = LegPins<
    GPIO4<'static>,
    GPIO3<'static>,
    GPIO2<'static>,
    GPIO1<'static>,
    GPIO5<'static>,
    GPIO6<'static>,
>;

type RightLegPins = LegPins<
    GPIO10<'static>,
    GPIO11<'static>,
    GPIO12<'static>,
    GPIO13<'static>,
    GPIO9<'static>,
    GPIO8<'static>,
>;

impl AppResources {
    pub fn split(peripherals: Peripherals) -> Self {
        Self {
            flash: peripherals.FLASH,
            wifi: peripherals.WIFI,
            timer_group0: peripherals.TIMG0,
            software_interrupt: peripherals.SW_INTERRUPT,
            mcpwm0: peripherals.MCPWM0,
            pcnt: peripherals.PCNT,
            desk_pins: DeskPins {
                left_leg: LegPins {
                    drive_left_enable: peripherals.GPIO4,
                    drive_right_enable: peripherals.GPIO3,
                    drive_left_pwm: peripherals.GPIO2,
                    drive_right_pwm: peripherals.GPIO1,
                    hall_a: peripherals.GPIO5,
                    hall_b: peripherals.GPIO6,
                },
                right_leg: LegPins {
                    drive_left_enable: peripherals.GPIO10,
                    drive_right_enable: peripherals.GPIO11,
                    drive_left_pwm: peripherals.GPIO12,
                    drive_right_pwm: peripherals.GPIO13,
                    hall_a: peripherals.GPIO9,
                    hall_b: peripherals.GPIO8,
                },
            },
        }
    }
}
