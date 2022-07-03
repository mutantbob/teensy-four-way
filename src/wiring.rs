use crate::{DynamicPin, MyLED};
use embedded_hal::digital::v2::OutputPin;

/// A switch configured with a pull-up resistor that will be overwhelmed by a connnection to ground.
pub struct SwitchPullUp<'a> {
    switch_pin: DynamicPin<'a>,
}

impl<'a> SwitchPullUp<'a> {
    pub fn new(switch_pin: DynamicPin<'a>) -> Self {
        SwitchPullUp { switch_pin }
    }

    /// When the swich is pressed, it connects the pin to ground, overriding the pull-up resistor.
    pub fn closed(&self) -> bool {
        !self.switch_pin.is_set()
    }
}

/// an LED with the anode connected to 3.3V and cathode connected to a signal pin
pub struct LED3VAndSignal {
    led: MyLED,
}

impl LED3VAndSignal {
    pub fn new(led: MyLED) -> Self {
        LED3VAndSignal { led }
    }

    pub fn shine(&mut self, on: bool) {
        if on {
            self.led.set_low() // create a voltage difference between signal and 3.3v
        } else {
            self.led.set_high() // signal is now same as 3.3v, so no voltage difference to drive current
        }
        .unwrap()
    }
}
