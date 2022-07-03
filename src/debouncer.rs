pub struct Debouncer {
    stabilized_state: bool,
    curr_state: bool,
    timestamp_last_transition: u32,
    ticks_for_stability: u32,
}

impl Debouncer {
    pub fn new(initial_state: bool, ticks_for_stability: u32) -> Self {
        Debouncer {
            stabilized_state: initial_state,
            curr_state: initial_state,
            timestamp_last_transition: 0,
            ticks_for_stability,
        }
    }
    pub fn debounced_value(&mut self, raw_value: bool, timestamp: u32) -> bool {
        if raw_value != self.curr_state {
            self.curr_state = raw_value;
            self.timestamp_last_transition = timestamp;
        }
        let n = timestamp - self.timestamp_last_transition;
        if n >= self.ticks_for_stability {
            self.stabilized_state = self.curr_state;
        }

        self.stabilized_state
    }
}
