// $ cargo test
#![no_std]
#![no_main]

use f411_rtic as _; // memory layout + panic handler
use stm32f4xx_hal::{
    gpio::{
        gpiob::{PB8, PB9},
        Floating, Input,
    },
    pac,
    prelude::*,
};

struct State {
    pb8: PB8<Input<Floating>>,
    pb9: PB9<Input<Floating>>,
}

#[defmt_test::tests]
mod tests {
    use super::*;
    use defmt::assert;

    #[init]
    fn init() -> State {
        let dp = stm32::Peripherals::take().unwrap();
        let gpiob = dp.GPIOB.split();
        let pb8 = gpiob.pb8.into_floating_input();
        let pb9 = gpiob.pb9.into_floating_input();
        State { pb8, pb9 }
    }

    // Connect pin PB8 to GND
    #[test]
    fn gnd_is_low(state: &mut State) {
        assert!(state.pb8.is_low().unwrap());
    }

    // Connect pin PB9 to Vdd
    #[test]
    fn vdd_is_high(state: &mut State) {
        assert!(state.pb9.is_high().unwrap());
    }
}
