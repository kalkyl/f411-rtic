// RTIC Monotonic impl for the 32-bit timers
#![no_main]
#![no_std]
use rtic_monotonic::{embedded_time, Clock, Fraction, Instant, Monotonic};
use stm32f4xx_hal::{
    pac::{RCC, TIM2, TIM5},
    rcc::Clocks,
};

pub struct MonoTimer<T, const FREQ: u32>(T);

impl<const FREQ: u32> MonoTimer<TIM2, FREQ> {
    pub fn new(timer: TIM2, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let prescaler = clocks.hclk().0 / FREQ - 1;
        timer.psc.write(|w| w.psc().bits(prescaler as u16));
        timer.arr.write(|w| unsafe { w.bits(u32::MAX) });
        timer.egr.write(|w| w.ug().set_bit());
        timer.sr.modify(|_, w| w.uif().clear_bit());
        timer.cr1.modify(|_, w| w.cen().set_bit().udis().set_bit());
        MonoTimer(timer)
    }
}

impl<const FREQ: u32> Clock for MonoTimer<TIM2, FREQ> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, FREQ);
    type T = u32;

    #[inline(always)]
    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.0.cnt.read().bits()))
    }
}

impl<const FREQ: u32> Monotonic for MonoTimer<TIM2, FREQ> {
    unsafe fn reset(&mut self) {
        self.0.dier.modify(|_, w| w.cc1ie().set_bit());
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        self.0
            .ccr1
            .write(|w| w.ccr().bits(instant.duration_since_epoch().integer()));
    }

    fn clear_compare_flag(&mut self) {
        self.0.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}

impl<const FREQ: u32> MonoTimer<TIM5, FREQ> {
    pub fn new(timer: TIM5, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim5en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().clear_bit());
        let prescaler = clocks.hclk().0 / FREQ - 1;
        timer.psc.write(|w| w.psc().bits(prescaler as u16));
        timer.arr.write(|w| unsafe { w.bits(u32::MAX) });
        timer.egr.write(|w| w.ug().set_bit());
        timer.sr.modify(|_, w| w.uif().clear_bit());
        timer.cr1.modify(|_, w| w.cen().set_bit().udis().set_bit());
        MonoTimer(timer)
    }
}

impl<const FREQ: u32> Clock for MonoTimer<TIM5, FREQ> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, FREQ);
    type T = u32;

    #[inline(always)]
    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.0.cnt.read().bits()))
    }
}

impl<const FREQ: u32> Monotonic for MonoTimer<TIM5, FREQ> {
    unsafe fn reset(&mut self) {
        self.0.dier.modify(|_, w| w.cc1ie().set_bit());
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        self.0
            .ccr1
            .write(|w| w.ccr().bits(instant.duration_since_epoch().integer()));
    }

    fn clear_compare_flag(&mut self) {
        self.0.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}
