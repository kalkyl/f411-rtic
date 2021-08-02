// $ cargo rb button-double
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::Milliseconds;
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        prelude::*,
    };

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<48_000_000>; // 48 MHz

    #[shared]
    struct Shared {
        btn: PC13<Input<PullUp>>,
        count: u8,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::RisingFalling);

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        defmt::info!("Press the button!");
        (Shared { btn, count: 0 }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI15_10, shared = [btn])]
    fn on_exti(mut ctx: on_exti::Context) {
        ctx.shared.btn.lock(|b| b.clear_interrupt_pending_bit());
        debounce::spawn_after(Milliseconds(30_u32)).ok();
    }

    #[task(shared = [btn, count], local = [clear: Option<clear::SpawnHandle> = None])]
    fn debounce(ctx: debounce::Context) {
        if let Some(handle) = ctx.local.clear.take() {
            handle.cancel().ok();
        }
        *ctx.local.clear = clear::spawn_after(Milliseconds(200_u32)).ok();
        (ctx.shared.btn, ctx.shared.count).lock(|btn, count| {
            if btn.is_low() {
                match *count > 0 {
                    true => {
                        *count = 0;
                        defmt::info!("Double press");
                    }
                    _ => defmt::info!("Single press"),
                }
                *count += 1;
            }
        });
    }

    #[task(shared = [count])]
    fn clear(mut ctx: clear::Context) {
        ctx.shared.count.lock(|c| *c = 0);
    }
}
