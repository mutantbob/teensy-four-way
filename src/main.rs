//! The starter code slowly blinks the LED, and sets up
//! USB logging.

#![no_std]
#![no_main]

use imxrt_hal::gpio::GPIO;
use imxrt_hal::iomuxc::b0::B0_10;
use imxrt_hal::iomuxc::{Hysteresis, PullKeep, PullKeepSelect, PullUpDown};
use teensy4_bsp as bsp;
use teensy4_bsp::hal::iomuxc;
use teensy4_bsp::SysTick;
use teensy4_bsp::LED;
use teensy4_panic as _;

//mod logging;
mod mission4;
mod support;

const LED_PERIOD_MS: u32 = 500;

#[cortex_m_rt::entry]
fn main() -> ! {
    if true {
        mission4::mission4()
    } else {
        let p = bsp::Peripherals::take().unwrap();
        let mut systick = bsp::SysTick::new(cortex_m::Peripherals::take().unwrap().SYST);
        let pins = bsp::t40::into_pins(p.iomuxc);
        let mut led = bsp::configure_led(pins.p13);

        match 3 {
            1 => mission1(pins.p6, &mut led, &mut systick),
            2 => mission2(pins.p6, &mut led, &mut systick),
            3 => mission3(pins.p6, &mut led, &mut systick),
            _ => mission0(&mut led, &mut systick),
        }
    }
}

fn mission0(led: &mut LED, systick: &mut SysTick) -> ! {
    // See the `logging` module docs for more info.
    //assert!(logging::init().is_ok());

    loop {
        led.toggle();
        systick.delay(LED_PERIOD_MS);
        log::info!("Hello world");
    }
}

// pull-down resistor.  Switch drags to 3.3v
fn mission1(mut switch_pin: B0_10, led: &mut LED, systick: &mut SysTick) -> ! {
    let cfg = teensy4_bsp::hal::iomuxc::Config::zero().set_pullupdown(PullUpDown::Pulldown100k);
    iomuxc::configure(&mut switch_pin, cfg);

    let bacon = GPIO::new(switch_pin);

    loop {
        if bacon.is_set() {
            led.toggle()
        }
        systick.delay(300);
    }
}

// pull-up resistor.  Switch drags to ground
fn mission2(mut switch_pin: B0_10, led: &mut LED, systick: &mut SysTick) -> ! {
    let pull_up = match 22 {
        100 => PullUpDown::Pullup100k, // unreliable
        47 => PullUpDown::Pullup47k,   // unreliable
        _ => PullUpDown::Pullup22k,    // unreliable
    };
    let cfg = teensy4_bsp::hal::iomuxc::Config::zero().set_pullupdown(pull_up);
    iomuxc::configure(&mut switch_pin, cfg);

    let bacon = GPIO::new(switch_pin);

    loop {
        if !bacon.is_set() {
            led.toggle()
        }
        systick.delay(300);
    }
}

// pull-up resistor.  Switch drags to ground
fn mission3(mut switch_pin: B0_10, led: &mut LED, systick: &mut SysTick) -> ! {
    let cfg = teensy4_bsp::hal::iomuxc::Config::zero()
        .set_hysteresis(Hysteresis::Enabled)
        .set_pull_keep(PullKeep::Enabled)
        .set_pull_keep_select(PullKeepSelect::Pull)
        .set_pullupdown(PullUpDown::Pulldown100k);
    iomuxc::configure(&mut switch_pin, cfg);

    let bacon = GPIO::new(switch_pin);

    loop {
        if bacon.is_set() {
            led.toggle()
        }
        systick.delay(300);
    }
}
