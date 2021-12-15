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

use keycode_translation::{simple_kr1, CodeSequence};

mod support;

type SwitchPin = teensy4_bsp::common::P8;

struct MyPins<SP: Pin> {
    led: LED,
    switch_pin: GPIO<SP, Input>,
    rotary_pin_1: GPIO<teensy4_bsp::common::P9, Input>,
    rotary_pin_2: GPIO<teensy4_bsp::common::P10, Input>,
    rotary_pin_3: GPIO<teensy4_bsp::common::P11, Input>,
    rotary_pin_4: GPIO<teensy4_bsp::common::P12, Input>,
}

//

struct HardwareParts {
    pins: MyPins<SwitchPin>,
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
        let switch_pin = GPIO::new(support::rigged_pull_down_switch(pins.p8));
        let rotary_pin_1 = GPIO::new(support::rigged_pull_down_switch(pins.p9));
        let rotary_pin_2 = GPIO::new(support::rigged_pull_down_switch(pins.p10));
        let rotary_pin_3 = GPIO::new(support::rigged_pull_down_switch(pins.p11));
        let rotary_pin_4 = GPIO::new(support::rigged_pull_down_switch(pins.p12));

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

///
///  HardwareParts is split into two initialization phases, because allocating a bus at the beginning
/// appears to fail.  I am not sure why
///
struct HardwareParts2<'a> {
    pins: MyPins<SwitchPin>,
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

trait MissionMode<P: Pin> {
    fn reboot(&mut self);

    fn one_usb_pass(
        &mut self,
        led: &mut LED,
        switch: &GPIO<P, Input>,
        gpt1: &mut GPT,
        hid: &mut HIDClass<BusAdapter>,
    ) -> KeyboardReport;

    fn maybe_deactivate(&mut self, hid: &mut HIDClass<BusAdapter>) -> Option<KeyboardReport>;
}

//

/// base iterator for the Ia MissionMode
type IaBI = core::str::Chars<'static>;
/// CodeSequence for the Ia MissionMode
type IaCS = CodeSequence<fn() -> IaBI, IaBI>;

struct Ia {
    generator: IaCS,
    deactivated: bool,
}

impl Ia {
    fn ia() -> core::str::Chars<'static> {
        "Ia! Ia! Cthulhu fhtagn.  ".chars()
    }

    pub fn standard_generator() -> IaCS {
        CodeSequence::from_chars(Self::ia)
    }
}

impl Default for Ia {
    fn default() -> Self {
        Ia {
            generator: Self::standard_generator(),
            deactivated: false,
        }
    }
}

impl<P: Pin> MissionMode<P> for Ia {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut LED,
        switch: &GPIO<P, Input>,
        _gpt1: &mut GPT,
        _hid: &mut HIDClass<BusAdapter>,
    ) -> KeyboardReport {
        let not_idle = !switch.is_set();
        if not_idle {
            self.generator.next().unwrap()
        } else {
            simple_kr1(0, 0)
        }
    }

    fn maybe_deactivate(&mut self, _hid: &mut HIDClass<BusAdapter>) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}

//

/// base iterator for the CallOfCthulhu MissionMode
type CoCBI = Map<Iter<'static, u8>, fn(&'static u8) -> char>;
/// CodeSequence for the CallOfCthulhu MissionMode
type CoCCS = CodeSequence<fn() -> CoCBI, CoCBI>;

struct CallOfCthulhu {
    generator: CoCCS,
    deactivated: bool,
}

impl CallOfCthulhu {
    fn story_text() -> CoCBI {
        let orig = include_bytes!("../keycode_translation/src/call-of-cthulhu.txt");
        orig.iter().map(|&b| b as char)
    }

    fn standard_generator() -> CoCCS {
        CodeSequence::from_chars(Self::story_text)
    }
}

impl Default for CallOfCthulhu {
    fn default() -> Self {
        CallOfCthulhu {
            generator: Self::standard_generator(),
            deactivated: false,
        }
    }
}

impl<P: Pin> MissionMode<P> for CallOfCthulhu {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut LED,
        switch: &GPIO<P, Input>,
        _gpt1: &mut GPT,
        _hid: &mut HIDClass<BusAdapter>,
    ) -> KeyboardReport {
        let not_idle = !switch.is_set();
        if not_idle {
            self.generator.next().unwrap()
        } else {
            simple_kr1(0, 0)
        }
        //spam_keyboard(not_idle, &mut self.generator, led, hid)
    }

    fn maybe_deactivate(&mut self, _hid: &mut HIDClass<BusAdapter>) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}

//

struct Eeeeee {
    deactivated: bool,
}

impl<P: Pin> MissionMode<P> for Eeeeee {
    fn reboot(&mut self) {}

    fn one_usb_pass(
        &mut self,
        _led: &mut LED,
        switch: &GPIO<P, Input>,
        _gpt1: &mut GPT,
        _hid: &mut HIDClass<BusAdapter>,
    ) -> KeyboardReport {
        let code = if !switch.is_set() { b'e' - b'a' + 4 } else { 0 };
        simple_kr1(0, code)
    }

    fn maybe_deactivate(&mut self, _hid: &mut HIDClass<BusAdapter>) -> Option<KeyboardReport> {
        if self.deactivated {
            None
        } else {
            self.deactivated = true;
            Some(simple_kr1(0, 0))
        }
    }
}

impl Default for Eeeeee {
    fn default() -> Self {
        Eeeeee { deactivated: false }
    }
}

//

struct ApplicationState {
    mode: RotaryMode,
    mode1: Ia,
    mode2: CallOfCthulhu,
    mode3: Eeeeee,
}

impl ApplicationState {
    pub fn new() -> ApplicationState {
        ApplicationState {
            mode: RotaryMode::Unknown,
            mode1: Ia::default(),
            mode2: CallOfCthulhu::default(),
            mode3: Eeeeee::default(),
        }
    }

    pub fn curr<P: Pin>(&mut self) -> Option<&mut dyn MissionMode<P>> {
        match self.mode {
            RotaryMode::Position1 => Some(&mut self.mode1),
            RotaryMode::Position2 => Some(&mut self.mode2),
            RotaryMode::Position3 => Some(&mut self.mode3),
            _ => None,
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
        mut pins,
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

    pins.led.clear();

    keyboard_mission3::<teensy4_bsp::common::P8>(&mut pins, &mut gpt1, &mut hid, &mut device)
}

//

#[derive(Copy, Clone, PartialEq)]
enum RotaryMode {
    Position1,
    Position2,
    Position3,
    Position4,
    Unknown,
}

impl RotaryMode {
    pub fn get<P: Pin>(pins: &MyPins<P>) -> RotaryMode {
        if !pins.rotary_pin_1.is_set() {
            RotaryMode::Position1
        } else if !pins.rotary_pin_2.is_set() {
            RotaryMode::Position2
        } else if !pins.rotary_pin_3.is_set() {
            RotaryMode::Position3
        } else if !pins.rotary_pin_4.is_set() {
            RotaryMode::Position4
        } else {
            RotaryMode::Unknown
        }
    }
}

//

fn keyboard_mission3<P: Pin>(
    pins: &mut MyPins<P>,
    gpt1: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    let mut app_state = ApplicationState::new();

    let mut pushed_back = None;

    loop {
        if device.poll(&mut [hid]) {
            let new_mode = RotaryMode::get(pins);

            let keyboard_report = if app_state.mode != new_mode {
                pushed_back.take();
                let delay_reboot = app_state
                    .curr::<P>()
                    .and_then(|curr| curr.maybe_deactivate(hid));
                match delay_reboot {
                    None => {
                        app_state.mode = new_mode;
                        if let Some(curr) = app_state.curr::<P>() {
                            curr.reboot();
                            curr.one_usb_pass(&mut pins.led, &pins.switch_pin, gpt1, hid)
                        } else {
                            simple_kr1(0, 0)
                        }
                    }
                    Some(kr) => kr,
                }
            } else {
                match app_state.curr::<P>() {
                    Some(curr) => match pushed_back.take() {
                        None => curr.one_usb_pass(&mut pins.led, &pins.switch_pin, gpt1, hid),
                        Some(kr) => kr,
                    },
                    None => simple_kr1(0, 0),
                }
            };

            match hid.push_input(&keyboard_report) {
                Ok(_) => {}
                Err(_) => pushed_back = Some(keyboard_report),
            }
        }

        support::time_elapse(gpt1, || {
            pins.led.toggle();
        });
    }
}
