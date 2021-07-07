// $ cargo rb rtic-tick
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout
mod mono; // monotonic timer impl for TIM2

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use super::mono::MonoTimer;
    use rtic::time::duration::Seconds;
    use stm32f4xx_hal::prelude::*;

    #[monotonic(binds = TIM2, default = true)]
    type MyMono = MonoTimer<48_000_000>;

    #[init]
    fn init(ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        let mono = MyMono::new(ctx.device.TIM2, &clocks);
        tick::spawn().ok();
        (init::LateResources {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task]
    fn tick(_: tick::Context) {
        defmt::info!("Tick!");
        tick::spawn_after(Seconds(1_u32)).ok();
    }
}
