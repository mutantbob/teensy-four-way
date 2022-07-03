// I copied this from imxrt-usbd 's examples/teensy/src/ directory and trimmed it down.

//! Support library (qualified as `support`) for all examples.

use core::time::Duration;

pub use bsp::hal;
pub use hal::ral;
pub use teensy4_bsp as bsp;

use crate::support::bsp::common::{P14, P15};
use crate::support::bsp::hal::ccm::perclk::Configured;
use crate::support::bsp::hal::dcdc::DCDC;
use crate::support::bsp::hal::gpt::{Unclocked, GPT};
use crate::support::bsp::hal::iomuxc;
use crate::support::bsp::hal::iomuxc::{Hysteresis, PullKeep, PullKeepSelect, PullUpDown};

/// Allocates a `BusAdapter`
///
/// # Panics
///
/// Panics if any of the `imxrt-ral` USB instances are already
/// taken.
pub fn new_bus_adapter() -> imxrt_usbd::full_speed::BusAdapter {
    // If we're here, we have exclusive access to ENDPOINT_MEMORY
    static mut ENDPOINT_MEMORY: [u8; 4096] = [0; 4096];

    unsafe {
        // Safety: With proper scoping and checks for singleton access, we ensure the memory is
        // only available to a single caller.
        imxrt_usbd::full_speed::BusAdapter::new(UsbPeripherals::usb1(), &mut ENDPOINT_MEMORY)
    }
}

//
// Keep in sync with the imxrt_usbd::Peripherals example!
//

struct UsbPeripherals {
    _usb: ral::usb::Instance,
    _phy: ral::usbphy::Instance,
    _nc: ral::usbnc::Instance,
    _analog: ral::usb_analog::Instance,
}

impl UsbPeripherals {
    /// Panics if the instances are already taken
    fn usb1() -> UsbPeripherals {
        Self {
            _usb: ral::usb::USB1::take().unwrap(),
            _phy: ral::usbphy::USBPHY1::take().unwrap(),
            _nc: ral::usbnc::USBNC1::take().unwrap(),
            _analog: ral::usb_analog::USB_ANALOG::take().unwrap(),
        }
    }
}

unsafe impl imxrt_usbd::Peripherals for UsbPeripherals {
    fn instance(&self) -> imxrt_usbd::Instance {
        imxrt_usbd::Instance::USB1
    }
}

pub mod ccm {
    use super::ral;

    /// Initialize CCM clocks for USB1
    pub fn initialize(ccm: &ral::ccm::Instance, ccm_analog: &ral::ccm_analog::Instance) {
        // Enable the PLL...
        loop {
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB1, ENABLE == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB1_SET, ENABLE: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB1, POWER == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB1_SET, POWER: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB1, LOCK == 0) {
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB1, BYPASS == 1) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB1_CLR, BYPASS: 1);
                continue;
            }
            if ral::read_reg!(ral::ccm_analog, ccm_analog, PLL_USB1, EN_USB_CLKS == 0) {
                ral::write_reg!(ral::ccm_analog, ccm_analog, PLL_USB1_SET, EN_USB_CLKS: 1);
                continue;
            }
            break;
        }

        // Enable the clock gates...
        ral::modify_reg!(ral::ccm, ccm, CCGR6, CG0: 0b11);
    }
}

/// Drive the logging implementation.
pub fn poll_logger() {
    imxrt_uart_log::dma::poll();
}

/// Required for proper function of `time_elapse`.
const GPT_OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;

/// Once the GPT has elapsed, invoke `func`.
pub fn time_elapse(gpt: &mut hal::gpt::GPT, func: impl FnOnce()) {
    let mut status = gpt.output_compare_status(GPT_OCR);
    if status.is_set() {
        status.clear();
        func();
    }
}

fn rig_pull_up_switch<I: iomuxc::IOMUX>(switch_pin: &mut I) {
    let cfg = teensy4_bsp::hal::iomuxc::Config::zero()
        .set_hysteresis(Hysteresis::Enabled)
        .set_pull_keep(PullKeep::Enabled)
        .set_pull_keep_select(PullKeepSelect::Pull)
        .set_pullupdown(PullUpDown::Pullup22k);

    iomuxc::configure(switch_pin, cfg);
}

pub fn rigged_pull_up_switch<I: iomuxc::IOMUX>(mut switch_pin: I) -> I {
    rig_pull_up_switch(&mut switch_pin);
    switch_pin
}

pub fn initialize_uart(
    logging_baud: u32,
    dma: imxrt_hal::dma::Unclocked,
    uart: imxrt_hal::uart::Unclocked,
    ccm_handle: &mut imxrt_hal::ccm::Handle,
    pin14: P14,
    pin15: P15,
) {
    let mut dma_channels = dma.clock(ccm_handle);
    let mut channel = dma_channels[7].take().unwrap();
    channel.set_interrupt_on_completion(false);
    let uarts = uart.clock(
        ccm_handle,
        hal::ccm::uart::ClockSelect::OSC,
        hal::ccm::uart::PrescalarSelect::DIVIDE_1,
    );
    let uart = uarts.uart2.init(pin14, pin15, logging_baud).unwrap();
    let (tx, _) = uart.split();
    imxrt_uart_log::dma::init(tx, channel, Default::default()).unwrap();
}

pub fn rig_timer<'a>(
    duration: Duration,
    dcdc: &mut DCDC,
    gpt1: Unclocked,
    ccm_pll1: &mut imxrt_hal::ccm::PLL1,
    ccm_handle: &'a mut imxrt_hal::ccm::Handle,
    ccm_perclk: imxrt_hal::ccm::perclk::Multiplexer,
) -> (GPT, imxrt_hal::ccm::perclk::Configured<'a>) {
    let (_frequency, ipg_hz) = ccm_pll1.set_arm_clock(hal::ccm::PLL1::ARM_HZ, ccm_handle, dcdc);
    let mut cfg = ccm_perclk.configure(
        ccm_handle,
        hal::ccm::perclk::PODF::DIVIDE_3,
        hal::ccm::perclk::CLKSEL::IPG(ipg_hz),
    );
    let gpt1 = rig_timer_2(duration, gpt1, &mut cfg);

    (gpt1, cfg)
}

pub fn rig_timer_2(duration: Duration, gpt: Unclocked, cfg: &mut Configured<'_>) -> GPT {
    let mut gpt = gpt.clock(cfg);
    gpt.set_wait_mode_enable(true);
    gpt.set_mode(hal::gpt::Mode::Reset);

    let gpt_ocr: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;
    gpt.set_output_compare_duration(gpt_ocr, duration);
    gpt
}
