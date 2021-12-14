#![no_std]
#![no_main]

use core::time::Duration;

use imxrt_hal::dcdc::DCDC;
use imxrt_hal::gpio::GPIO;
use imxrt_hal::gpt::{Unclocked, GPT};
use imxrt_hal::iomuxc::gpio::Pin;
use imxrt_hal::iomuxc::{Hysteresis, PullKeep, PullKeepSelect, PullUpDown};
use imxrt_usbd::full_speed::BusAdapter;
use teensy4_bsp as bsp;
use teensy4_bsp::common::{P14, P15};
use teensy4_bsp::hal::gpio::Input;
use teensy4_bsp::hal::iomuxc;
use teensy4_bsp::{hal, LED};
use teensy4_panic as _;
use usb_device::device::UsbDevice;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

use core::iter::Map;
use core::slice::Iter;
use keycode_translation::{CodeSequence, PushBackIterator};
use usb_device::bus::UsbBusAllocator;

mod support;

struct HardwareParts {
    led: LED,
    gpt1: GPT,
    switch_pin: teensy4_bsp::common::P8,
}

impl HardwareParts {
    pub fn start_up(peripherals: hal::Peripherals) -> HardwareParts {
        let duration = core::time::Duration::from_millis(300);
        let logging_baud = 115_200;
        let hal::Peripherals {
            iomuxc,
            mut ccm,
            dma,
            uart,
            mut dcdc,
            gpt1,
            ..
        } = peripherals;
        let pins = bsp::t40::into_pins(iomuxc);
        let led = bsp::configure_led(pins.p13);
        let mut switch_pin = pins.p8;

        let gpt1 = rig_timer(
            duration,
            &mut dcdc,
            gpt1,
            &mut ccm.pll1,
            &mut ccm.handle,
            ccm.perclk,
        );

        rig_pull_down_switch(&mut switch_pin);

        initialize_uart(logging_baud, dma, uart, &mut ccm.handle, pins.p14, pins.p15);

        let mut ccm = ccm.handle;

        let (ccm_i, ccm_analog) = ccm.raw();
        support::ccm::initialize(ccm_i, ccm_analog);

        HardwareParts {
            led,
            gpt1,
            switch_pin,
        }
    }

    pub fn stage_2<'a>(self, bus: &'a UsbBusAllocator<BusAdapter>) -> HardwareParts2<'a> {
        HardwareParts2::new(self, bus)
    }
}

//

struct HardwareParts2<'a> {
    led: LED,
    gpt1: GPT,
    switch_pin: teensy4_bsp::common::P8,
    hid: HIDClass<'a, BusAdapter>,
    device: UsbDevice<'a, BusAdapter>,
}

impl<'a> HardwareParts2<'a> {
    pub fn new(part1: HardwareParts, bus: &'a UsbBusAllocator<BusAdapter>) -> HardwareParts2
    {
        let hid = usbd_hid::hid_class::HIDClass::new(
            &bus,
            //usbd_hid::descriptor::MouseReport::desc(),
            usbd_hid::descriptor::KeyboardReport::desc(),
            10,
        );
        let device = UsbDeviceBuilder::new(&bus, UsbVidPid(0x5824, 0x27dd))
            .product("imxrt-usbd")
            .build();

        HardwareParts2 {
            led: part1.led,
            gpt1: part1.gpt1,
            switch_pin: part1.switch_pin,
            hid,
            device,
        }
    }
}

//

pub fn spam_keyboard<I>(
    not_idle: bool,
    generator: &mut PushBackIterator<KeyboardReport, I>,
    led: &mut LED,
    hid: &mut HIDClass<BusAdapter>,
) where
    I: Iterator<Item = KeyboardReport>,
{
    if not_idle {
        let cmd = generator.next().unwrap();

        let would_block = match hid.push_input(&cmd) {
            Ok(_x) => false,
            Err(_usb_error) => {
                // probably buffer full, try again later
                generator.push_back(cmd);
                true
            }
        };

        if would_block {
            led.set();
        } else {
            led.clear();
        }
    } else {
        // if we don't keep sending commands, something gets stuck
        let cmd = usbd_hid::descriptor::KeyboardReport {
            modifier: 0,
            reserved: 0,
            leds: 0,
            keycodes: [0, 0, 0, 0, 0, 0],
        };
        let _ = hid.push_input(&cmd);
    }
}

//

trait MissionMode<P: Pin> {
    fn reboot(&mut self);

    fn one_usb_pass(
        &mut self,
        led: &mut LED,
        switch: &GPIO<P, Input>,
        gpt1: &mut GPT,
        hid: &mut HIDClass<BusAdapter>,
    );

    fn maybe_deactivate(&mut self) -> bool;
}

//

/// base iterator for the Ia MissionMode
type IaBI = core::str::Chars<'static>;
/// CodeSequence for the Ia MissionMode
type IaCS = CodeSequence<fn() -> IaBI, IaBI>;

struct Ia {
    generator: PushBackIterator<KeyboardReport, IaCS>,
}

impl Ia {
    fn ia() -> core::str::Chars<'static> {
        "Ia! Ia! Cthulhu fhtagn.  ".chars()
    }

    pub fn standard_generator() -> PushBackIterator<KeyboardReport, IaCS> {
        let msg: IaCS = CodeSequence::from_chars(Self::ia);
        PushBackIterator::from(msg)
    }
}

impl Default for Ia {
    fn default() -> Self {
        Ia {
            generator: Self::standard_generator(),
        }
    }
}

impl<P: Pin> MissionMode<P> for Ia {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        led: &mut LED,
        switch: &GPIO<P, Input>,
        _gpt1: &mut GPT,
        hid: &mut HIDClass<BusAdapter>,
    ) {
        spam_keyboard(!switch.is_set(), &mut self.generator, led, hid);
    }

    fn maybe_deactivate(&mut self) -> bool {
        todo!()
    }
}

//

/// base iterator for the CallOfCthulhu MissionMode
type CoCBI = Map<Iter<'static, u8>, fn(&'static u8) -> char>;
/// CodeSequence for the CallOfCthulhu MissionMode
type CoCCS = CodeSequence<fn() -> CoCBI, CoCBI>;

struct CallOfCthulhu {
    generator: PushBackIterator<KeyboardReport, CoCCS>,
}

impl CallOfCthulhu {
    fn story_text() -> CoCBI {
        let orig = include_bytes!("../keycode_translation/src/call-of-cthulhu.txt");
        orig.iter().map(|&b| b as char)
    }

    fn standard_generator() -> PushBackIterator<KeyboardReport, CoCCS> {
        let msg: CoCCS = CodeSequence::from_chars(Self::story_text);
        PushBackIterator::from(msg)
    }
}

impl Default for CallOfCthulhu {
    fn default() -> Self {
        CallOfCthulhu {
            generator: Self::standard_generator(),
        }
    }
}

impl<P: Pin> MissionMode<P> for CallOfCthulhu {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        led: &mut LED,
        switch: &GPIO<P, Input>,
        _gpt1: &mut GPT,
        hid: &mut HIDClass<BusAdapter>,
    ) {
        spam_keyboard(!switch.is_set(), &mut self.generator, led, hid)
    }

    fn maybe_deactivate(&mut self) -> bool {
        todo!()
    }
}

//

struct ApplicationState {
    mode1: Ia,
    mode2: CallOfCthulhu,
}

impl ApplicationState {
    pub fn new() -> ApplicationState {
        ApplicationState {
            mode1: Ia::default(),
            mode2: CallOfCthulhu::default(),
        }
    }

    pub fn curr<P: Pin>(&mut self) -> &mut dyn MissionMode<P> {
        &mut self.mode2
    }
}

//

#[cortex_m_rt::entry]
fn main() -> ! {
    let env = HardwareParts::start_up(hal::Peripherals::take().unwrap());

    //
    //

    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let env = env.stage_2(&bus);

    let HardwareParts2 {
        mut led,
        mut gpt1,
        switch_pin,
        mut hid,
        mut device,
    } = env;

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

    led.clear();

    let switch_pin = GPIO::new(switch_pin);

    keyboard_mission3(led, switch_pin, &mut gpt1, &mut hid, &mut device)
}

fn rig_pull_down_switch<I: iomuxc::IOMUX>(switch_pin: &mut I) {
    let cfg = teensy4_bsp::hal::iomuxc::Config::zero()
        .set_hysteresis(Hysteresis::Enabled)
        .set_pull_keep(PullKeep::Enabled)
        .set_pull_keep_select(PullKeepSelect::Pull)
        .set_pullupdown(PullUpDown::Pullup22k);

    iomuxc::configure(switch_pin, cfg);
}

//

//

// https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2
fn keyboard_mission3<P: Pin>(
    mut led: LED,
    switch: GPIO<P, Input>,
    gpt1: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    let mut app_state = ApplicationState::new();

    loop {
        if device.poll(&mut [hid]) {
            app_state.curr().one_usb_pass(&mut led, &switch, gpt1, hid);
        }

        support::time_elapse(gpt1, || {
            led.toggle();
        });
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
