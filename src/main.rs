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

use crate::mission_modes::{CallOfCthulhu, Eeeeee, Ia, IcarusJog, MissionMode};
use alloc::boxed::Box;
use alloc_cortex_m::CortexMHeap;
use keycode_translation::simple_kr1;

mod mission_modes;
mod support;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

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
    gpt2: GPT,
}

impl HardwareParts {
    pub fn start_up(peripherals: hal::Peripherals) -> HardwareParts {
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
                rotary_pin_1,
                rotary_pin_2,
                rotary_pin_3,
                rotary_pin_4,
            },
            gpt1,
            gpt2,
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
    gpt2: GPT,
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
            gpt2: part1.gpt2,
            hid,
            device,
        }
    }
}

//

//

struct ApplicationState {
    mode: RotaryMode,
    modes: [Box<dyn MissionMode>; 4],
}

impl ApplicationState {
    pub fn new() -> ApplicationState {
        ApplicationState {
            mode: RotaryMode::Unknown,
            modes: [
                Box::new(IcarusJog::new(0.0)), //Ia::default(),
                Box::new(IcarusJog::new(0.6)), //CallOfCthulhu::default(),
                Box::new(IcarusJog::new(0.7)), //Eeeeee::default(),
                Box::new(IcarusJog::new(1.0)),
            ],
        }
    }

    pub fn new2() -> ApplicationState {
        ApplicationState {
            mode: RotaryMode::Unknown,
            modes: [
                Box::new(Ia::default()),
                Box::new(CallOfCthulhu::default()),
                Box::new(Eeeeee::default()),
                Box::new(IcarusJog::new(0.7)),
            ],
        }
    }

    pub fn curr(&mut self) -> Option<&mut dyn MissionMode> {
        let idx = match self.mode {
            RotaryMode::Position1 => Some(0),
            RotaryMode::Position2 => Some(1),
            RotaryMode::Position3 => Some(2),
            RotaryMode::Position4 => Some(3),
            _ => None,
        };
        match idx {
            None => None,
            Some(i) => Some(self.modes[i].as_mut()),
        };
        idx.map(move |i| {
            let tmp: &mut dyn MissionMode = self.modes[i].as_mut();
            tmp
        })
    }

    fn usb_keyboard_response<P: Pin>(
        &mut self,
        pins: &mut MyPins<P>,
        hid: &mut HIDClass<BusAdapter>,
        pushed_back: &mut Option<KeyboardReport>,
        millis_elapsed: u32,
        new_mode: RotaryMode,
    ) -> KeyboardReport {
        let idle = pins.switch_pin.is_set();
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
    gpt2: &mut GPT,
    hid: &mut HIDClass<BusAdapter>,
    device: &mut UsbDevice<BusAdapter>,
) -> ! {
    let mut app_state = if true {
        ApplicationState::new()
    } else {
        ApplicationState::new2()
    };

    let mut pushed_back = None;
    let mut millis_elapsed = 0;

    loop {
        device.poll(&mut [hid]);

        {
            let new_mode = RotaryMode::get(pins);

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
