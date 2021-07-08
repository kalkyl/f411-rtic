// RTIC Monotonic impl for the 32-bit timers
use rtic_monotonic::{embedded_time, Clock, Fraction, Instant, Monotonic};
use stm32f4xx_hal::{
    pac::{tim2, tim5, RCC, TIM2, TIM5},
    rcc::Clocks,
};

pub struct MonoTimer<T: Instance32, const FREQ: u32>(T);

impl<T: Instance32, const FREQ: u32> MonoTimer<T, FREQ> {
    pub fn new(mut timer: T, clocks: &Clocks) -> Self {
        let psc = clocks.hclk().0 / FREQ - 1;
        timer.init(psc as u16);
        MonoTimer(timer)
    }
}

impl<T: Instance32, const FREQ: u32> Clock for MonoTimer<T, FREQ> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, FREQ);
    type T = u32;

    #[inline(always)]
    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.0.count()))
    }
}

impl<T: Instance32, const FREQ: u32> Monotonic for MonoTimer<T, FREQ> {
    unsafe fn reset(&mut self) {
        self.0.enable_interrupt();
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        self.0
            .set_compare(*instant.duration_since_epoch().integer());
    }

    fn clear_compare_flag(&mut self) {
        self.0.clear_compare_flag();
    }
}

pub trait Instance32 {
    type T;
    fn init(&mut self, psc: u16);
    fn count(&self) -> u32;
    fn enable_interrupt(&self);
    fn set_compare(&mut self, duration: u32);
    fn clear_compare_flag(&mut self);
}

impl Instance32 for TIM2 {
    type T = tim2::RegisterBlock;
    fn init(&mut self, psc: u16) {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        self.psc.write(|w| w.psc().bits(psc));
        self.arr.write(|w| unsafe { w.bits(u32::MAX) });
        self.egr.write(|w| w.ug().set_bit());
        self.sr.modify(|_, w| w.uif().clear_bit());
        self.cr1.modify(|_, w| w.cen().set_bit().udis().set_bit());
    }
    fn count(&self) -> u32 {
        self.cnt.read().bits()
    }
    fn enable_interrupt(&self) {
        self.dier.modify(|_, w| w.cc1ie().set_bit());
    }
    fn set_compare(&mut self, duration: u32) {
        self.ccr1.write(|w| w.ccr().bits(duration));
    }
    fn clear_compare_flag(&mut self) {
        self.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}

impl Instance32 for TIM5 {
    type T = tim5::RegisterBlock;
    fn init(&mut self, psc: u16) {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim5en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim5rst().clear_bit());
        self.psc.write(|w| w.psc().bits(psc));
        self.arr.write(|w| unsafe { w.bits(u32::MAX) });
        self.egr.write(|w| w.ug().set_bit());
        self.sr.modify(|_, w| w.uif().clear_bit());
        self.cr1.modify(|_, w| w.cen().set_bit().udis().set_bit());
    }
    fn count(&self) -> u32 {
        self.cnt.read().bits()
    }
    fn enable_interrupt(&self) {
        self.dier.modify(|_, w| w.cc1ie().set_bit());
    }
    fn set_compare(&mut self, duration: u32) {
        self.ccr1.write(|w| w.ccr().bits(duration));
    }
    fn clear_compare_flag(&mut self) {
        self.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}
