#![no_std]

use core::str::Chars;
use usbd_hid::descriptor::KeyboardReport;

fn simple_kr(modifier: u8, keycodes: [u8; 6]) -> KeyboardReport {
    KeyboardReport {
        modifier,
        reserved: 0,
        leds: 0,
        keycodes,
    }
}

fn translate_char(ch: char) -> Option<KeyboardReport> {
    match ch {
        'a'..='z' => {
            let code = (ch as u8) - b'a' + 4;
            Some(simple_kr(0, [code, 0, 0, 0, 0, 0]))
        }
        'A'..='Z' => {
            let code = (ch as u8) - b'A' + 4;
            Some(simple_kr(2, [code, 0, 0, 0, 0, 0]))
        }
        '!'..=')' => {
            let code = (ch as u8) - b'!' + 0x1e;
            Some(simple_kr(2, [code, 0, 0, 0, 0, 0]))
        }
        '\n' => Some(simple_kr(0, [0x28, 0, 0, 0, 0, 0])),
        '.' => Some(simple_kr(0, [0x37, 0, 0, 0, 0, 0])),
        ',' => Some(simple_kr(0, [0x36, 0, 0, 0, 0, 0])),
        '/' => Some(simple_kr(0, [0x38, 0, 0, 0, 0, 0])),
        '?' => Some(simple_kr(2, [0x38, 0, 0, 0, 0, 0])),
        ';' => Some(simple_kr(0, [0x33, 0, 0, 0, 0, 0])),
        ':' => Some(simple_kr(2, [0x33, 0, 0, 0, 0, 0])),
        '1'..='9' => {
            let code = (ch as u8) - b'1' + 0x1e;
            Some(simple_kr(0, [code, 0, 0, 0, 0, 0]))
        }
        '0' => {
            let code = 0x27;
            Some(simple_kr(0, [code, 0, 0, 0, 0, 0]))
        }
        '-' => Some(simple_kr(0, [0x2d, 0, 0, 0, 0, 0])),
        'é' | 'ë' => {
            Some(simple_kr(0, [8, 0, 0, 0, 0, 0])) // XXX sloppy
        }
        'Å' => {
            Some(simple_kr(0, [4, 0, 0, 0, 0, 0])) // XXX sloppy
        }
        '°' => {
            Some(simple_kr(0, [0x2c, 0, 0, 0, 0, 0])) // XXX wrong
        }
        ' ' => Some(simple_kr(0, [0x2c, 0, 0, 0, 0, 0])),
        // lots of stuff is missing, and I'm sure there are keyboard layouts that this is incorrect for.
        _ => None,
    }
}

pub struct CodeSequence<F, I>
where
    F: Fn() -> I,
    I: Iterator<Item = char>,
{
    orig: F,
    iter: I,
}

impl<'a, F, I> CodeSequence<F, I>
where
    F: Fn() -> I,
    I: Iterator<Item = char>,
{
    pub fn new_from_str(orig: &'a str) -> CodeSequence<impl Fn() -> Chars<'a>, Chars<'a>> {
        CodeSequence::from_chars(|| orig.chars())
    }

    pub fn from_chars(orig: F) -> CodeSequence<F, I> {
        let iter = orig();
        CodeSequence { orig, iter }
    }

    pub fn generate(&mut self) -> KeyboardReport {
        loop {
            let ch = self.iter.next();
            match ch {
                None => {
                    self.iter = (self.orig)();
                }
                Some(ch) => {
                    if let Some(code) = translate_char(ch) {
                        return code;
                    }
                }
            }
        }
    }
}

impl<F, I> Iterator for CodeSequence<F, I>
where
    F: Fn() -> I,
    I: Iterator<Item = char>,
{
    type Item = KeyboardReport;

    fn next(&mut self) -> Option<Self::Item> {
        Some(self.generate())
    }
}

pub struct PushBackIterator<T, I: Iterator<Item = T>> {
    base: I,
    buffer: Option<T>,
}

impl<T, I: Iterator<Item = T>> PushBackIterator<T, I> {
    pub fn from(base: I) -> PushBackIterator<T, I> {
        Self { base, buffer: None }
    }

    pub fn push_back(&mut self, val: T) {
        self.buffer = Some(val);
    }
}

impl<T, I: Iterator<Item = T>> Iterator for PushBackIterator<T, I> {
    type Item = T;

    fn next(&mut self) -> Option<Self::Item> {
        match self.buffer.take() {
            None => self.base.next(),
            Some(val) => Some(val),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::translate_char;

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
}
