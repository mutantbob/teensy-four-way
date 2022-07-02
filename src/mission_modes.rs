use crate::MyLED;
use alloc::boxed::Box;
use imxrt_usbd::full_speed::BusAdapter;
use keycode_translation::{simple_kr1, CodeSequence};
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;

pub trait MissionMode {
    fn reboot(&mut self);

    fn one_usb_pass(
        &mut self,
        led: &mut MyLED,
        hid: &mut HIDClass<BusAdapter>,
        millis_elapsed: u32,
    ) -> KeyboardReport;

    fn maybe_deactivate(&mut self, hid: &mut HIDClass<BusAdapter>) -> Option<KeyboardReport>;
}

//

pub struct Ia {
    generator: CodeSequence<'static>,
    deactivated: bool,
}

impl Ia {
    pub fn standard_generator() -> CodeSequence<'static> {
        let chars = "Ia! Ia! Cthulhu fhtagn.  ".chars();
        CodeSequence::new(Box::new(chars.cycle()))
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

impl MissionMode for Ia {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut MyLED,
        _hid: &mut HIDClass<BusAdapter>,
        _millis_elapsed: u32,
    ) -> KeyboardReport {
        self.generator.next().unwrap()
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

pub struct CallOfCthulhu {
    generator: CodeSequence<'static>,
    deactivated: bool,
}

impl CallOfCthulhu {
    fn story_text() -> impl Iterator<Item = char> {
        let orig = include_bytes!("../keycode_translation/src/call-of-cthulhu.txt");
        orig.iter().cycle().map(|&b| b as char)
    }

    fn standard_generator() -> CodeSequence<'static> {
        CodeSequence::new(Box::new(Self::story_text()))
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

impl MissionMode for CallOfCthulhu {
    fn reboot(&mut self) {
        self.generator = Self::standard_generator();
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut MyLED,
        _hid: &mut HIDClass<BusAdapter>,
        _millis_elapsed: u32,
    ) -> KeyboardReport {
        self.generator.next().unwrap()
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

#[derive(Default)]
pub struct Eeeeee {
    deactivated: bool,
}

impl MissionMode for Eeeeee {
    fn reboot(&mut self) {}

    fn one_usb_pass(
        &mut self,
        _led: &mut MyLED,
        _hid: &mut HIDClass<BusAdapter>,
        _millis_elapsed: u32,
    ) -> KeyboardReport {
        const CODE: u8 = b'e' - b'a' + 4;
        simple_kr1(0, CODE)
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

pub struct IcarusJog {
    sprint: bool,
    duty_cycle: f32,
    deactivated: bool,
    phase_millis: Option<u32>,
}

impl IcarusJog {
    pub fn new(duty_cycle: f32) -> IcarusJog {
        IcarusJog {
            sprint: false,
            duty_cycle,
            deactivated: false,
            phase_millis: None,
        }
    }
}

impl MissionMode for IcarusJog {
    fn reboot(&mut self) {
        self.sprint = false;
        self.phase_millis = None;
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut MyLED,
        _hid: &mut HIDClass<BusAdapter>,
        millis_elapsed: u32,
    ) -> KeyboardReport {
        let phase_millis = match self.phase_millis {
            None => {
                self.phase_millis = Some(millis_elapsed);
                millis_elapsed
            }
            Some(val) => val,
        };
        let period = 6000;
        self.sprint =
            ((millis_elapsed - phase_millis) % period) < (period as f32 * self.duty_cycle) as u32;

        //support::time_elapse(gpt1, || self.sprint = !self.sprint);
        /*let mut status = gpt2.output_compare_status(hal::gpt::OutputCompareRegister::One);
        if status.is_set() {
            status.clear();
            self.sprint = !self.sprint
        }*/

        simple_kr1(if self.sprint { 2 } else { 0 }, b'w' - b'a' + 4)
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

pub struct TestKey {
    key_code: KeyboardReport,
    deactivated: bool,
}

impl TestKey {
    pub fn box_new(ch: char) -> Box<TestKey> {
        Box::new(Self::new(ch))
    }

    pub fn new(ch: char) -> TestKey {
        TestKey {
            key_code: keycode_translation::translate_char(ch).unwrap(),
            deactivated: false,
        }
    }
}

impl MissionMode for TestKey {
    fn reboot(&mut self) {
        // nothing to do
    }

    fn one_usb_pass(
        &mut self,
        _led: &mut MyLED,
        _hid: &mut HIDClass<BusAdapter>,
        _millis_elapsed: u32,
    ) -> KeyboardReport {
        self.key_code.clone()
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
