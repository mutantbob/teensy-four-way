use crate::support;
use imxrt_hal::gpio::GPIO;
//use imxrt_hal::iomuxc::b0::B0_10;
use teensy4_bsp as bsp;
//use teensy4_bsp::LED;
use teensy4_bsp::hal;

use core::time::Duration;
//use imxrt_hal::ccm::CCM;
use imxrt_hal::dcdc::DCDC;
use imxrt_hal::gpt::{Unclocked, GPT};
use teensy4_bsp::common::{P14, P15};
//use teensy4_bsp::t40::Pins;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::SerializedDescriptor;

pub fn mission4() -> ! {
    let (mut led, mut gpt1, mut ccm, switch_pin) = {
        let duration = core::time::Duration::from_millis(500);
        let logging_baud = 115_200;
        let hal::Peripherals {
            iomuxc,
            mut ccm,
            dma,
            uart,
            mut dcdc,
            gpt1,
            ..
        } = hal::Peripherals::take().unwrap();
        let pins = bsp::t40::into_pins(iomuxc);
        let led = bsp::configure_led(pins.p13);
        let switch_pin = pins.p6;

        let gpt1 = rig_timer(
            duration,
            &mut dcdc,
            gpt1,
            &mut ccm.pll1,
            &mut ccm.handle,
            ccm.perclk,
        );

        initialize_uart(logging_baud, dma, uart, &mut ccm.handle, pins.p14, pins.p15);
        (led, gpt1, ccm.handle, switch_pin)
    };

    let switch_pin = GPIO::new(switch_pin);

    //
    //

    let (ccm, ccm_analog) = ccm.raw();
    support::ccm::initialize(ccm, ccm_analog);

    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let mut hid =
        usbd_hid::hid_class::HIDClass::new(&bus, usbd_hid::descriptor::MouseReport::desc(), 10);
    let mut device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
        .product("imxrt-usbd")
        .build();

    //

    gpt1.set_enable(true);
    loop {
        support::poll_logger();
        if !device.poll(&mut [&mut hid]) {
            continue;
        }
        let state = device.state();
        if state == usb_device::device::UsbDeviceState::Configured {
            break;
        }
    }

    device.bus().configure();
    led.set();

    loop {
        switch_pin.is_set();

        support::time_elapse(&mut gpt1, || {
            led.toggle();

            let cmd = usbd_hid::descriptor::MouseReport {
                x: 4,
                y: 4,
                buttons: 0,
                pan: 0,
                wheel: 0,
            };
            hid.push_input(&cmd).unwrap();
        });
        if !device.poll(&mut [&mut hid]) {
            continue;
        }
        //systick.delay(5);
    }
}

fn initialize_uart(
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

fn rig_timer(
    duration: Duration,
    mut dcdc: &mut DCDC,
    gpt1: Unclocked,
    ccm_pll1: &mut imxrt_hal::ccm::PLL1,
    ccm_handle: &mut imxrt_hal::ccm::Handle,
    ccm_perclk: imxrt_hal::ccm::perclk::Multiplexer,
) -> GPT {
    let (_, ipg_hz) = ccm_pll1.set_arm_clock(hal::ccm::PLL1::ARM_HZ, ccm_handle, &mut dcdc);
    let mut cfg = ccm_perclk.configure(
        ccm_handle,
        hal::ccm::perclk::PODF::DIVIDE_3,
        hal::ccm::perclk::CLKSEL::IPG(ipg_hz),
    );
    let mut gpt1 = gpt1.clock(&mut cfg);
    gpt1.set_wait_mode_enable(true);
    gpt1.set_mode(hal::gpt::Mode::Reset);

    let gpt_ocr: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;
    gpt1.set_output_compare_duration(gpt_ocr, duration);
    gpt1
}
