#![no_std]

extern crate alloc;
use alloc::boxed::Box;
use usbd_hid::descriptor::KeyboardReport;

pub fn simple_kr(modifier: u8, keycodes: [u8; 6]) -> KeyboardReport {
    KeyboardReport {
        modifier,
        reserved: 0,
        leds: 0,
        keycodes,
    }
}

pub fn simple_kr1(modifier: u8, key_code_1: u8) -> KeyboardReport {
    simple_kr(modifier, [key_code_1, 0, 0, 0, 0, 0])
}

pub fn translate_char(ch: char) -> Option<KeyboardReport> {
    match ch {
        'a'..='z' => {
            let code = (ch as u8) - b'a' + 4;
            Some(simple_kr1(0, code))
        }
        'A'..='Z' => {
            let code = (ch as u8) - b'A' + 4;
            Some(simple_kr1(2, code))
        }
        '!'..=')' => {
            let code = (ch as u8) - b'!' + 0x1e;
            Some(simple_kr1(2, code))
        }
        '\n' => Some(simple_kr1(0, 0x28)),
        '.' => Some(simple_kr1(0, 0x37)),
        ',' => Some(simple_kr1(0, 0x36)),
        '/' => Some(simple_kr1(0, 0x38)),
        '?' => Some(simple_kr1(2, 0x38)),
        ';' => Some(simple_kr1(0, 0x33)),
        ':' => Some(simple_kr1(2, 0x33)),
        '1'..='9' => {
            let code = (ch as u8) - b'1' + 0x1e;
            Some(simple_kr1(0, code))
        }
        '0' => {
            let code = 0x27;
            Some(simple_kr1(0, code))
        }
        '-' => Some(simple_kr1(0, 0x2d)),
        'é' | 'ë' => {
            Some(simple_kr1(0, 8)) // XXX sloppy
        }
        'Å' => {
            Some(simple_kr1(0, 4)) // XXX sloppy
        }
        '°' => {
            Some(simple_kr1(0, 0x2c)) // XXX wrong
        }
        ' ' => Some(simple_kr1(0, 0x2c)),
        // lots of stuff is missing, and I'm sure there are keyboard layouts that this is incorrect for.
        _ => None,
    }
}

pub struct CodeSequence<'a> {
    pub orig: Box<dyn 'a + Fn() -> Box<dyn 'a + Iterator<Item = char>>>,
    pub iter: Box<dyn 'a + Iterator<Item = char>>,
    pub prev_char: Option<char>,
    pub next_val: Option<KeyboardReport>,
}

impl<'a> CodeSequence<'a> {
    pub fn new<F, I: 'a + Iterator<Item = char>>(orig1: F) -> CodeSequence<'a>
    where
        F: 'a + Fn() -> I,
    {
        let orig: Box<dyn 'a + Fn() -> Box<dyn 'a + Iterator<Item = char>>> =
            Box::new(move || Box::new(orig1()));
        let iter = orig();
        CodeSequence {
            orig,
            iter,
            prev_char: None,
            next_val: None,
        }
    }

    pub fn new_from_str(orig: &'a str) -> CodeSequence<'a> {
        Self::new(|| orig.chars())
    }

    pub fn generate(&mut self) -> KeyboardReport {
        loop {
            if let Some(kr) = self.next_val.take() {
                return kr;
            }

            let ch = self.iter.next();
            match ch {
                None => {
                    self.iter = (self.orig)();
                }
                Some(ch) => {
                    match (self.prev_char.take(), translate_char(ch)) {
                        (Some(prev_ch), Some(code)) => {
                            self.prev_char = Some(ch);
                            if prev_ch == ch {
                                self.next_val = Some(code);
                                return simple_kr1(0, 0);
                            } else {
                                return code;
                            }
                        }
                        (None, Some(code)) => {
                            self.prev_char = Some(ch);
                            return code;
                        }
                        (_, None) => {}
                    };
                }
            }
        }
    }
}

impl<'a> Iterator for CodeSequence<'a> {
    type Item = KeyboardReport;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.generate())
    }
}

//
//
//

#[cfg(test)]
mod tests {
    use crate::{translate_char, CodeSequence};

    #[test]
    pub fn test_translate() {
        let text = include_bytes!("call-of-cthulhu.txt");
        let text = text.iter().map(|&b| b as char);

        let mut line = 1;
        let mut col = 1;
        for ch in text {
            let code = translate_char(ch);

            match code {
                None => {
                    panic!("no translation for '{}' L{}c{}", ch, line, col)
                }
                Some(_) => {} // good
            }

            if ch == '\n' {
                line += 1;
                col = 1;
            } else {
                col += 1;
            }
        }
    }

    #[test]
    pub fn test2() {
        let mut src = CodeSequence::new(|| "horror".chars());

        let _h = src.generate();
        let _o = src.generate();
        let _r = src.generate();
        let z = src.generate();
        assert_eq!(0, z.keycodes[0]);
        let r = src.generate();
        assert_eq!(21, r.keycodes[0]);
    }
}
