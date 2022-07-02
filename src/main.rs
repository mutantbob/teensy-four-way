#![no_std]
#![no_main]
#![feature(default_alloc_error_handler)]
#![feature(generic_const_exprs)]

extern crate alloc;

use crate::dynamic_pin::DynamicPin;
use crate::mission_modes::{CallOfCthulhu, Eeeeee, Ia, IcarusJog, MissionMode};
use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use embedded_hal::digital::v2::{OutputPin, PinState};
use imxrt_hal::gpio::{Output, GPIO};
use imxrt_hal::gpt::GPT;
use imxrt_hal::iomuxc::gpio::Pin;
use imxrt_usbd::full_speed::BusAdapter;
use keycode_translation::simple_kr1;
use mission_modes::TestKey;
use teensy4_bsp as bsp;
use teensy4_bsp::hal;
use teensy4_panic as _;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usb_device::prelude::{UsbDeviceBuilder, UsbVidPid};
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};
use usbd_hid::hid_class::HIDClass;

mod dynamic_pin;
mod mission_modes;
mod support;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

//

pub struct ConstCheck<const CHECK: bool>;

pub trait True {}
impl True for ConstCheck<true> {}

//

pub type MyLED = GPIO<teensy4_bsp::common::P16, Output>;

//

struct MyPins<'a, const N: usize> {
    led: MyLED,
    switch_pin: DynamicPin<'a>,
    rotary_pins: [DynamicPin<'a>; N],
}

impl<'a, const N: usize> MyPins<'a, N> {
    /// LED is connected between 3.3V and an output pin (with a current-limiting resistor), so high means off, and low means on.
    pub(crate) fn shine_led(&mut self, on: bool) {
        self.led
            .set_state(if on { PinState::Low } else { PinState::High })
            .unwrap()
    }
}

//

struct HardwareParts<'a> {
    gpt1: GPT,
    gpt2: GPT,
    pins: MyPins<'a, 12>,
}

fn dynamic_pull_down<'a, P: Pin + 'a>(pin: P) -> DynamicPin<'a> {
    DynamicPin::new(GPIO::new(support::rigged_pull_down_switch(pin)))
}

impl<'a> HardwareParts<'a> {
    /// pin 3 is the switch.
    /// pin 16 is the LED.
    pub fn start_up(peripherals: hal::Peripherals) -> HardwareParts<'a> {
        let duration = core::time::Duration::from_millis(100);
        let logging_baud = 115_200;
        let hal::Peripherals {
            iomuxc,
            mut ccm,
            dma,
            uart,
            mut dcdc,
            gpt1,
            gpt2,
            ..
        } = peripherals;

        let pins = bsp::t40::into_pins(iomuxc);
        let led = {
            let mut led = GPIO::new(pins.p16);
            led.set_fast(true);
            led.output()
        };
        let switch_pin = GPIO::new(support::rigged_pull_down_switch(pins.p3));

        support::initialize_uart(logging_baud, dma, uart, &mut ccm.handle, pins.p14, pins.p15);

        let (mut gpt1, mut clock_config) = support::rig_timer(
            duration,
            &mut dcdc,
            gpt1,
            &mut ccm.pll1,
            &mut ccm.handle,
            ccm.perclk,
        );
        gpt1.set_enable(true);

        let mut gpt2 = support::rig_timer_2(
            core::time::Duration::from_millis(500),
            gpt2,
            &mut clock_config,
        );
        gpt2.set_enable(true);

        let mut ccm = ccm.handle;

        let (ccm_i, ccm_analog) = ccm.raw();
        support::ccm::initialize(ccm_i, ccm_analog);

        HardwareParts {
            pins: MyPins {
                led,
                switch_pin: DynamicPin::new(switch_pin),
                rotary_pins: [
                    dynamic_pull_down(pins.p4),
                    dynamic_pull_down(pins.p5),
                    dynamic_pull_down(pins.p6),
                    dynamic_pull_down(pins.p7),
                    dynamic_pull_down(pins.p8),
                    dynamic_pull_down(pins.p9),
                    dynamic_pull_down(pins.p17),
                    dynamic_pull_down(pins.p18),
                    dynamic_pull_down(pins.p19),
                    dynamic_pull_down(pins.p20),
                    dynamic_pull_down(pins.p21),
                    dynamic_pull_down(pins.p22),
                ],
            },
            gpt1,
            gpt2,
        }
    }

    pub fn stage_2(self, bus: &'a UsbBusAllocator<BusAdapter>) -> HardwareParts2<12> {
        HardwareParts2::new(self, bus)
    }
}

//

///
///  HardwareParts is split into two initialization phases, because allocating a bus at the beginning
/// appears to fail.  I am not sure why
///
struct HardwareParts2<'a, const N: usize> {
    pins: MyPins<'a, N>,
    gpt1: GPT,
    gpt2: GPT,
    hid: HIDClass<'a, BusAdapter>,
    device: UsbDevice<'a, BusAdapter>,
}

impl<'a> HardwareParts2<'a, 12> {
    pub fn new(
        part1: HardwareParts<'a>,
        bus: &'a UsbBusAllocator<BusAdapter>,
    ) -> HardwareParts2<'a, 12> {
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
            gpt2: part1.gpt2,
            hid,
            device,
        }
    }
}

//

trait ActivePredicate<const N: usize> {
    fn is_active(&mut self, pins: &MyPins<N>) -> bool;
}

//

struct ApplicationState<'a, const N: usize, const M: usize> {
    mode: Option<usize>,
    modes: [Box<dyn MissionMode + 'a>; N],
    on_off_switch: Box<dyn ActivePredicate<M> + 'a>,
}

impl<'a, const N: usize, const M: usize> ApplicationState<'a, N, M> {
    pub fn icarus_jog<AP: ActivePredicate<M> + 'a>(
        on_off_switch: AP,
    ) -> ApplicationState<'a, 4, M> {
        ApplicationState {
            mode: None,
            modes: [
                Box::new(IcarusJog::new(0.0)), //Ia::default(),
                Box::new(IcarusJog::new(0.6)), //CallOfCthulhu::default(),
                Box::new(IcarusJog::new(0.7)), //Eeeeee::default(),
                Box::new(IcarusJog::new(1.0)),
            ],
            on_off_switch: Box::new(on_off_switch),
        }
    }

    pub fn madness<AP: ActivePredicate<M> + 'a>(on_off_switch: AP) -> ApplicationState<'a, 3, M> {
        ApplicationState {
            mode: None,
            modes: [
                Box::new(Ia::default()),
                Box::new(CallOfCthulhu::default()),
                Box::new(Eeeeee::default()),
                //Box::new(IcarusJog::new(0.7)),
            ],
            on_off_switch: Box::new(on_off_switch),
        }
    }

    pub fn test_twelve<AP: ActivePredicate<M> + 'a>(
        on_off_switch: AP,
    ) -> ApplicationState<'a, 12, M> {
        ApplicationState {
            mode: None,
            modes: [
                TestKey::box_new('1'),
                TestKey::box_new('2'),
                TestKey::box_new('3'),
                TestKey::box_new('4'),
                TestKey::box_new('5'),
                TestKey::box_new('6'),
                TestKey::box_new('7'),
                TestKey::box_new('8'),
                TestKey::box_new('9'),
                TestKey::box_new('a'),
                TestKey::box_new('b'),
                TestKey::box_new('c'),
            ],
            on_off_switch: Box::new(on_off_switch),
        }
    }
}

impl<'a, const N: usize, const M: usize> ApplicationState<'a, N, M>
where
    ConstCheck<{ N <= M }>: True,
{
    pub fn curr(&mut self) -> Option<&mut dyn MissionMode> {
        match self.mode {
            Some(idx) => {
                if idx < self.modes.len() {
                    Some(self.modes[idx].as_mut())
                } else {
                    None
                }
            }
            None => None,
        }
    }
}

impl<'a, const N: usize, const M: usize> ApplicationState<'a, N, M> {
    fn usb_keyboard_response(
        &mut self,
        pins: &mut MyPins<M>,
        hid: &mut HIDClass<BusAdapter>,
        pushed_back: &mut Option<KeyboardReport>,
        millis_elapsed: u32,
        new_mode: Option<usize>,
    ) -> KeyboardReport
    where
        ConstCheck<{ N <= M }>: True,
    {
        let idle = self.on_off_switch.is_active(pins);

        if idle {
            return simple_kr1(0, 0);
        }

        if self.mode != new_mode {
            pushed_back.take();
            let delay_reboot = self.curr().and_then(|curr| curr.maybe_deactivate(hid));
            match delay_reboot {
                None => {
                    self.mode = new_mode;
                    if let Some(curr) = self.curr() {
                        curr.reboot();
                        curr.one_usb_pass(&mut pins.led, hid, millis_elapsed)
                    } else {
                        simple_kr1(0, 0)
                    }
                }
                Some(kr) => kr,
            }
        } else {
            match self.curr() {
                Some(curr) => match pushed_back.take() {
                    None => curr.one_usb_pass(&mut pins.led, hid, millis_elapsed),
                    Some(kr) => kr,
                },
                None => simple_kr1(0, 0),
            }
        }
    }
}
//

#[cortex_m_rt::entry]
fn main() -> ! {
    {
        // Initialize the allocator BEFORE you use it
        let start = cortex_m_rt::heap_start() as usize;
        let size = 1024; // in bytes
        unsafe { ALLOCATOR.init(start, size) }
    }

    let env = HardwareParts::start_up(hal::Peripherals::take().unwrap());

    //
    //

    let bus_adapter = support::new_bus_adapter();
    let bus = usb_device::bus::UsbBusAllocator::new(bus_adapter);

    let env = env.stage_2(&bus);

    let HardwareParts2 {
        mut pins,
        mut gpt1,
        mut gpt2,
        mut hid,
        mut device,
    } = env;

    //

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

    match 5 {
        3 => keyboard_mission3(&mut pins, &mut gpt1, &mut gpt2, &mut hid, &mut device),
        4 => keyboard_mission4(&mut pins, &mut gpt1, &mut gpt2, &mut hid, &mut device),
        _ => keyboard_mission5(&mut pins, &mut gpt1, &mut gpt2, &mut hid, &mut device),
    }
}

//

//

#[derive(Copy, Clone)]
struct ToggleSwitchActive {}

impl<const N: usize> ActivePredicate<N> for ToggleSwitchActive {
    fn is_active(&mut self, pins: &MyPins<N>) -> bool {
        pins.switch_pin.is_set()
    }
}

//

#[derive(Copy, Clone)]
struct MomentarySwitchActive {
    momentary_state: bool,
    state: bool,
}

impl MomentarySwitchActive {
    pub fn new() -> Self {
        MomentarySwitchActive {
            momentary_state: false,
            state: false,
        }
    }
}

impl<const N: usize> ActivePredicate<N> for MomentarySwitchActive {
    fn is_active(&mut self, pins: &MyPins<N>) -> bool {
        let is_set = pins.switch_pin.is_set();
        if is_set != self.momentary_state {
            if is_set {
                self.state = !self.state;
            }
            self.momentary_state = is_set;
        }

        self.state
    }
}

//

fn keyboard_mission3(
    pins: &mut MyPins<12>,
    gpt1: &mut GPT,
    gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    let switch_active = switch_active();

    core_application_loop(
        pins,
        gpt1,
        gpt2,
        hid,
        device,
        ApplicationState::<0, 12>::madness(switch_active),
    )
}

fn switch_active() -> impl ActivePredicate<12> {
    // let mut switch_active = ToggleSwitchActive{};
    let switch_active = MomentarySwitchActive::new();
    switch_active
}

fn keyboard_mission4(
    pins: &mut MyPins<12>,
    gpt1: &mut GPT,
    gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    // let mut switch_active = ToggleSwitchActive{};
    let switch_active = MomentarySwitchActive::new();

    core_application_loop(
        pins,
        gpt1,
        gpt2,
        hid,
        device,
        ApplicationState::<0, 12>::icarus_jog(switch_active),
    )
}

fn keyboard_mission5(
    pins: &mut MyPins<12>,
    gpt1: &mut GPT,
    gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    // let mut switch_active = ToggleSwitchActive{};
    let switch_active = MomentarySwitchActive::new();

    core_application_loop(
        pins,
        gpt1,
        gpt2,
        hid,
        device,
        ApplicationState::<0, 12>::test_twelve(switch_active),
    )
}

fn core_application_loop<const N: usize, const M: usize>(
    pins: &mut MyPins<M>,
    gpt1: &mut GPT,
    _gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
    mut app_state: ApplicationState<N, M>,
) -> !
where
    ConstCheck<{ N <= M }>: True,
{
    let mut pushed_back = None;
    let mut millis_elapsed = 0;

    loop {
        device.poll(&mut [hid]);

        {
            let new_mode = pins
                .rotary_pins
                .iter()
                .enumerate()
                .filter_map(|(idx, pin)| if !pin.is_set() { Some(idx) } else { None })
                .next();

            let keyboard_report = app_state.usb_keyboard_response(
                pins,
                hid,
                &mut pushed_back,
                millis_elapsed,
                new_mode,
            );

            if keyboard_report.modifier == 0 && keyboard_report.keycodes[0] == 0 {
                pins.shine_led(false)
            } else {
                pins.shine_led(true)
            }

            match hid.push_input(&keyboard_report) {
                Ok(_) => {}
                Err(_) => pushed_back = Some(keyboard_report),
            }
        }

        support::time_elapse(gpt1, || {
            millis_elapsed += 100;
        });

        /*support::time_elapse(gpt2, || {
            pins.led.toggle();
        });*/
    }
}
