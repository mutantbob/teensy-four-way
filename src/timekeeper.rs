static mut TIMESTAMP_MILLIS: u32 = 0;

pub fn millis() -> u32 {
    unsafe { TIMESTAMP_MILLIS }
}

pub fn incr_timestamp(plus: u32) {
    unsafe { TIMESTAMP_MILLIS += plus }
}
