#![no_std]
#![no_main]
#![feature(default_alloc_error_handler)]

extern crate alloc;

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

use crate::dynamic_pin::DynamicPin;
use crate::mission_modes::{CallOfCthulhu, Eeeeee, Ia, IcarusJog, MissionMode};
use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use keycode_translation::simple_kr1;

mod dynamic_pin;
mod mission_modes;
mod support;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

type SwitchPin = teensy4_bsp::common::P8;

struct MyPins<'a, SP: Pin, const N: usize> {
    led: LED,
    switch_pin: GPIO<SP, Input>,
    rotary_pins: [DynamicPin<'a>; N],
}

//

struct HardwareParts<'a> {
    pins: MyPins<'a, SwitchPin, 4>,
    gpt1: GPT,
    gpt2: GPT,
}

impl<'a> HardwareParts<'a> {
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
        let led = bsp::configure_led(pins.p13);
        let switch_pin = GPIO::new(support::rigged_pull_down_switch(pins.p8));
        let rotary_pin_1 = GPIO::new(support::rigged_pull_down_switch(pins.p9));
        let rotary_pin_2 = GPIO::new(support::rigged_pull_down_switch(pins.p10));
        let rotary_pin_3 = GPIO::new(support::rigged_pull_down_switch(pins.p11));
        let rotary_pin_4 = GPIO::new(support::rigged_pull_down_switch(pins.p12));

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
                switch_pin,
                rotary_pins: [
                    DynamicPin::new(rotary_pin_1),
                    DynamicPin::new(rotary_pin_2),
                    DynamicPin::new(rotary_pin_3),
                    DynamicPin::new(rotary_pin_4),
                ],
            },
            gpt1,
            gpt2,
        }
    }

    pub fn stage_2(self, bus: &'a UsbBusAllocator<BusAdapter>) -> HardwareParts2<4> {
        HardwareParts2::new(self, bus)
    }
}

//

///
///  HardwareParts is split into two initialization phases, because allocating a bus at the beginning
/// appears to fail.  I am not sure why
///
struct HardwareParts2<'a, const N: usize> {
    pins: MyPins<'a, SwitchPin, N>,
    gpt1: GPT,
    gpt2: GPT,
    hid: HIDClass<'a, BusAdapter>,
    device: UsbDevice<'a, BusAdapter>,
}

impl<'a> HardwareParts2<'a, 4> {
    pub fn new(
        part1: HardwareParts<'a>,
        bus: &'a UsbBusAllocator<BusAdapter>,
    ) -> HardwareParts2<'a, 4> {
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

trait ActivePredicate<P: Pin, const N: usize> {
    fn is_active(&mut self, pins: &MyPins<P, N>) -> bool;
}

//

struct ApplicationState<'a, P, const N: usize> {
    mode: Option<usize>,
    modes: [Box<dyn MissionMode + 'a>; N],
    on_off_switch: Box<dyn ActivePredicate<P, N> + 'a>,
}

impl<'a, P: Pin, const N: usize> ApplicationState<'a, P, N> {
    pub fn new<AP: ActivePredicate<P, 4> + 'a>(on_off_switch: AP) -> ApplicationState<'a, P, 4> {
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

    pub fn new2<AP: ActivePredicate<P, 4> + 'a>(on_off_switch: AP) -> ApplicationState<'a, P, 4> {
        ApplicationState {
            mode: None,
            modes: [
                Box::new(Ia::default()),
                Box::new(CallOfCthulhu::default()),
                Box::new(Eeeeee::default()),
                Box::new(IcarusJog::new(0.7)),
            ],
            on_off_switch: Box::new(on_off_switch),
        }
    }
}
impl<'a, P: Pin, const N: usize> ApplicationState<'a, P, N> {
    pub fn curr(&mut self) -> Option<&mut dyn MissionMode> {
        match self.mode {
            Some(idx) => Some(self.modes[idx].as_mut()),
            None => None,
        }
    }

    fn usb_keyboard_response(
        &mut self,
        pins: &mut MyPins<P, N>,
        hid: &mut HIDClass<BusAdapter>,
        pushed_back: &mut Option<KeyboardReport>,
        millis_elapsed: u32,
        new_mode: Option<usize>,
    ) -> KeyboardReport {
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

    keyboard_mission3::<teensy4_bsp::common::P8>(
        &mut pins,
        &mut gpt1,
        &mut gpt2,
        &mut hid,
        &mut device,
    )
}

//

//

#[derive(Copy, Clone)]
struct ToggleSwitchActive {}

impl<P: Pin, const N: usize> ActivePredicate<P, N> for ToggleSwitchActive {
    fn is_active(&mut self, pins: &MyPins<P, N>) -> bool {
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

impl<P: Pin, const N: usize> ActivePredicate<P, N> for MomentarySwitchActive {
    fn is_active(&mut self, pins: &MyPins<P, N>) -> bool {
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

fn keyboard_mission3<P: Pin>(
    pins: &mut MyPins<P, 4>,
    gpt1: &mut GPT,
    gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    // let mut switch_active = ToggleSwitchActive{};
    let switch_active = MomentarySwitchActive::new();
    let mut app_state = if false {
        ApplicationState::<P, 0>::new(switch_active)
    } else {
        ApplicationState::<P, 0>::new2(switch_active)
    };

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

            match hid.push_input(&keyboard_report) {
                Ok(_) => {}
                Err(_) => pushed_back = Some(keyboard_report),
            }
        }

        support::time_elapse(gpt1, || {
            millis_elapsed += 100;
        });

        support::time_elapse(gpt2, || {
            pins.led.toggle();
        });
    }
}
