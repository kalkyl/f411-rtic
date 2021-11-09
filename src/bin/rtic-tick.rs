// $ cargo rb rtic-tick
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout
mod mono; // monotonic timer impl for TIM2

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use super::mono::MonoTimer;
    use rtic::time::duration::Seconds;
    use stm32f4xx_hal::{pac, prelude::*};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[monotonic(binds = TIM2, default = true)]
    type MyMono = MonoTimer<pac::TIM2, 48_000_000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        let mono = MyMono::new(ctx.device.TIM2, &clocks);
        tick::spawn().ok();
        (Shared {}, Local {}, init::Monotonics(mono))
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
