This project is for a Teensy 4.0 that is connected to an activation switch and a rotary switch.

The activation switch can close pin 8 to ground.
The rotary switch can close one of pins 9-12 to ground (for example https://www.digikey.com/en/products/detail/e-switch/KC14A10-001NPS/1804511).
We use the Teensy's internal pull-up resistor on these pins.

The rotary switch controls what keystrokes the Teensy will send, and the activation switch enables the sending of keystrokes.

This was written in a time when the APIs were quite unpolished.  There are many secret handshakes that if you do not get them right the entire subsystem gets wedged.

For instance, it seems that if you fail to `hid.push_input(kr)` for too long of an interval, it stops being able to send keystrokes altogether.  I am not sure if this is a Linux weirdness, or part of the USB keyboard protocol.

The primary loop is in `keyboard_mission3()`.  It is a bit of a mess because of the dance to switch between modes (and it would be more dangerous without Rust's expression blocks).

The modes controlled by the rotary switch are wrapped up in `ApplicationState`.

The `keycode_translation` nested workspace contains logic that (badly) translates a string into a sequence of USB keycodes.  Since there is no uniformity between applications in how to enter arbitrary unicode, I just mangle things to a corresponding US keycode.

## Building

I build the code using the following BASH spell
```
~/.cargo/bin/cargo +nightly build && ~/.cargo/bin/cargo +nightly objcopy --release -- -O ihex /tmp/kbd.hex
````

If you get `Failed to execute tool: objcopy` you probably need to
```
~/.cargo/bin/rustup +nightly  component add llvm-tools-previewinfo
```

for the unit tests:
```
(cd keycode_translation/; ~/.cargo/bin/cargo +nightly  test --target x86_64-unknown-linux-gnu)
```