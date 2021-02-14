#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT, dispatchers = [USART1])]
mod app {
    use rtic::cyccnt::U32Ext;
    use stm32f4xx_hal::{
        gpio::{gpioa::PA5, Output, PushPull},
        prelude::*,
    };

    #[resources]
    struct Resources {
        led: PA5<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let _clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Set up the LED. On the Nucleo-F411RE it's connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

        defmt::info!("Hello world!");
        blink::spawn().ok();

        init::LateResources { led }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(resources = [led])]
    fn blink(mut ctx: blink::Context) {
        ctx.resources.led.lock(|l| l.toggle().ok());
        defmt::info!("Blink!");
        blink::schedule(ctx.scheduled + 48_000_000.cycles()).ok();
    }
}
