#![no_std]
#![no_main]

use core::iter::Map;
use core::slice::Iter;

use imxrt_hal::gpio::GPIO;
use imxrt_hal::gpt::GPT;
use imxrt_hal::iomuxc::gpio::Pin;
use imxrt_usbd::full_speed::BusAdapter;
use teensy4_bsp as bsp;
use teensy4_bsp::hal::gpio::Input;
use teensy4_bsp::{hal, LED};
use teensy4_panic as _;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

use keycode_translation::{simple_kr1, CodeSequence, PushBackIterator};

mod support;

struct MyPins {
    led: LED,
    switch_pin: teensy4_bsp::common::P8,
    rotary_pin_1: teensy4_bsp::common::P9,
    rotary_pin_2: teensy4_bsp::common::P10,
    rotary_pin_3: teensy4_bsp::common::P11,
    rotary_pin_4: teensy4_bsp::common::P12,
}

//

struct HardwareParts {
    pins: MyPins,
    gpt1: GPT,
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
        let switch_pin = support::rigged_pull_down_switch(pins.p8);
        let rotary_pin_1 = support::rigged_pull_down_switch(pins.p9);
        let rotary_pin_2 = support::rigged_pull_down_switch(pins.p10);
        let rotary_pin_3 = support::rigged_pull_down_switch(pins.p11);
        let rotary_pin_4 = support::rigged_pull_down_switch(pins.p12);

        support::initialize_uart(logging_baud, dma, uart, &mut ccm.handle, pins.p14, pins.p15);

        let gpt1 = support::rig_timer(
            duration,
            &mut dcdc,
            gpt1,
            &mut ccm.pll1,
            &mut ccm.handle,
            ccm.perclk,
        );

        let mut ccm = ccm.handle;

        let (ccm_i, ccm_analog) = ccm.raw();
        support::ccm::initialize(ccm_i, ccm_analog);

        HardwareParts {
            pins: MyPins {
                led,
                switch_pin,
                rotary_pin_1,
                rotary_pin_2,
                rotary_pin_3,
                rotary_pin_4,
            },
            gpt1,
        }
    }

    pub fn stage_2(self, bus: &UsbBusAllocator<BusAdapter>) -> HardwareParts2 {
        HardwareParts2::new(self, bus)
    }
}

//

struct HardwareParts2<'a> {
    pins: MyPins,
    gpt1: GPT,
    hid: HIDClass<'a, BusAdapter>,
    device: UsbDevice<'a, BusAdapter>,
}

impl<'a> HardwareParts2<'a> {
    pub fn new(part1: HardwareParts, bus: &'a UsbBusAllocator<BusAdapter>) -> HardwareParts2 {
        let hid = usbd_hid::hid_class::HIDClass::new(
            bus,
            //usbd_hid::descriptor::MouseReport::desc(),
            usbd_hid::descriptor::KeyboardReport::desc(),
            10,
        );
        let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x5824, 0x27dd))
            .product("imxrt-usbd")
            .build();

        HardwareParts2 {
            pins: part1.pins,
            gpt1: part1.gpt1,
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

    fn maybe_deactivate(&mut self, hid: &mut HIDClass<BusAdapter>) -> bool;
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

    fn maybe_deactivate(&mut self, hid: &mut HIDClass<BusAdapter>) -> bool {
        hid.push_input(&simple_kr1(0, 0)).is_ok()
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

    fn maybe_deactivate(&mut self, hid: &mut HIDClass<BusAdapter>) -> bool {
        hid.push_input(&simple_kr1(0, 0)).is_ok()
    }
}

//

struct ApplicationState {
    mode: RotaryMode,
    mode1: Ia,
    mode2: CallOfCthulhu,
}

impl ApplicationState {
    pub fn new() -> ApplicationState {
        ApplicationState {
            mode: RotaryMode::Unknown,
            mode1: Ia::default(),
            mode2: CallOfCthulhu::default(),
        }
    }

    pub fn curr<P: Pin>(&mut self) -> Option<&mut dyn MissionMode<P>> {
        match self.mode {
            RotaryMode::Position1 => Some(&mut self.mode1),
            RotaryMode::Position2 => Some(&mut self.mode2),
            RotaryMode::Unknown => None,
        }
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
        pins:
            MyPins {
                mut led,
                switch_pin,
                rotary_pin_1,
                rotary_pin_2,
                rotary_pin_3,
                rotary_pin_4,
            },
        mut gpt1,
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

    let switch_pin: GPIO<teensy4_bsp::common::P8, Input> = GPIO::new(switch_pin);

    keyboard_mission3(
        led,
        switch_pin,
        GPIO::new(rotary_pin_1),
        &mut gpt1,
        &mut hid,
        &mut device,
    )
}

//

#[derive(Copy, Clone, PartialEq)]
enum RotaryMode {
    Position1,
    Position2,
    Unknown,
}

impl RotaryMode {
    pub fn get<R1: Pin>(rotary_pin_1: &GPIO<R1, Input>) -> RotaryMode {
        if !rotary_pin_1.is_set() {
            RotaryMode::Position1
        } else {
            RotaryMode::Position2
        }
    }
}

//

// https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2
fn keyboard_mission3<P: Pin, R1: Pin>(
    mut led: LED,
    switch: GPIO<P, Input>,
    rotary_pin_1: GPIO<R1, Input>,
    gpt1: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    let mut app_state = ApplicationState::new();

    loop {
        if device.poll(&mut [hid]) {
            let new_mode = RotaryMode::get(&rotary_pin_1);

            if app_state.mode != new_mode {
                let should_reboot = match app_state.curr::<P>() {
                    Some(curr) => curr.maybe_deactivate(hid),
                    None => true,
                };
                if should_reboot {
                    app_state.mode = new_mode;
                    if let Some(curr) = app_state.curr::<P>() {
                        curr.reboot();
                    }
                }
            } else if let Some(curr) = app_state.curr::<P>() {
                curr.one_usb_pass(&mut led, &switch, gpt1, hid);
            }
        }

        support::time_elapse(gpt1, || {
            led.toggle();
        });
    }
}
