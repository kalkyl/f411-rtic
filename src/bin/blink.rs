// $ cargo rb blink
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::Seconds;
    use stm32f4xx_hal::{
        gpio::{gpioa::PA5, Output, PushPull},
        prelude::*,
    };

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<48_000_000>; // 48 MHz

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Set up the LED. On the Nucleo-F411RE it's connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        defmt::info!("Hello world!");
        blink::spawn_after(Seconds(1_u32)).ok();
        (Shared {}, Local { led }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(local = [led])]
    fn blink(ctx: blink::Context) {
        ctx.local.led.toggle();
        defmt::info!("Blink!");
        blink::spawn_after(Seconds(1_u32)).ok();
    }
}
