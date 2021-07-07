// $ cargo rb rtic-mono
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::MonoTimer;
    use rtic::time::duration::Seconds;
    use stm32f4xx_hal::{
        gpio::{gpioa::PA5, Output, PushPull},
        prelude::*,
    };

    #[monotonic(binds = TIM2, default = true)]
    type MyMono = MonoTimer<48_000_000>;

    #[resources]
    struct Resources {
        led: PA5<Output<PushPull>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Set up the monotonic timer
        let mono = MyMono::new(ctx.device.TIM2, &clocks);

        // Set up the LED. On the Nucleo-F411RE it's connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

        blink::spawn().ok();
        (init::LateResources { led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(resources = [led])]
    fn blink(mut ctx: blink::Context) {
        ctx.resources.led.lock(|l| l.toggle().ok());
        defmt::info!("Blink!");
        blink::spawn_after(Seconds(1_u32)).ok();
    }
}

use rtic_monotonic::{embedded_time, Clock, Fraction, Instant, Monotonic};
use stm32f4xx_hal::{
    pac::{RCC, TIM2},
    rcc::Clocks,
};

pub struct MonoTimer<const FREQ: u32>(TIM2);

impl<const FREQ: u32> MonoTimer<FREQ> {
    pub fn new(timer: TIM2, clocks: &Clocks) -> Self {
        let rcc = unsafe { &(*RCC::ptr()) };
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().set_bit());
        rcc.apb1rstr.modify(|_, w| w.tim2rst().clear_bit());
        let psc = clocks.hclk().0 / FREQ - 1;
        timer.psc.write(|w| w.psc().bits(psc as u16));
        timer.arr.write(|w| unsafe { w.bits(u32::MAX) });
        timer.egr.write(|w| w.ug().set_bit());
        timer.sr.modify(|_, w| w.uif().clear_bit());
        timer.cr1.modify(|_, w| w.cen().set_bit().udis().set_bit());
        MonoTimer(timer)
    }
}

impl<const FREQ: u32> Clock for MonoTimer<FREQ> {
    const SCALING_FACTOR: Fraction = Fraction::new(1, FREQ);
    type T = u32;

    #[inline(always)]
    fn try_now(&self) -> Result<Instant<Self>, embedded_time::clock::Error> {
        Ok(Instant::new(self.0.cnt.read().bits()))
    }
}

impl<const FREQ: u32> Monotonic for MonoTimer<FREQ> {
    unsafe fn reset(&mut self) {
        self.0.dier.modify(|_, w| w.cc1ie().set_bit());
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        self.0
            .ccr1
            .write(|w| w.ccr().bits(*instant.duration_since_epoch().integer()));
    }

    fn clear_compare_flag(&mut self) {
        self.0.sr.modify(|_, w| w.cc1if().clear_bit());
    }
}
