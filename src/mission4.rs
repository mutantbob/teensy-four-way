use crate::support;
use imxrt_hal::gpio::GPIO;
//use imxrt_hal::iomuxc::b0::B0_10;
use teensy4_bsp as bsp;
//use teensy4_bsp::LED;
use teensy4_bsp::hal;

use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::SerializedDescriptor;

pub fn mission4() -> ! {
    let BLINK_PERIOD = core::time::Duration::from_millis(500);
    let UART_BAUD = 115_200;
    let (mut led, mut gpt1, mut ccm, switch_pin) = {
        let duration = BLINK_PERIOD;
        let logging_baud = UART_BAUD;
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

        let (_, ipg_hz) =
            ccm.pll1
                .set_arm_clock(hal::ccm::PLL1::ARM_HZ, &mut ccm.handle, &mut dcdc);
        let mut cfg = ccm.perclk.configure(
            &mut ccm.handle,
            hal::ccm::perclk::PODF::DIVIDE_3,
            hal::ccm::perclk::CLKSEL::IPG(ipg_hz),
        );
        let mut gpt1 = gpt1.clock(&mut cfg);
        gpt1.set_wait_mode_enable(true);
        gpt1.set_mode(hal::gpt::Mode::Reset);
        let GPT_OCR: hal::gpt::OutputCompareRegister = hal::gpt::OutputCompareRegister::One;

        gpt1.set_output_compare_duration(GPT_OCR, duration);
        let mut dma_channels = dma.clock(&mut ccm.handle);
        let mut channel = dma_channels[7].take().unwrap();
        channel.set_interrupt_on_completion(false);
        let uarts = uart.clock(
            &mut ccm.handle,
            hal::ccm::uart::ClockSelect::OSC,
            hal::ccm::uart::PrescalarSelect::DIVIDE_1,
        );
        let uart = uarts.uart2.init(pins.p14, pins.p15, logging_baud).unwrap();
        let (tx, _) = uart.split();
        imxrt_uart_log::dma::init(tx, channel, Default::default()).unwrap();
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
