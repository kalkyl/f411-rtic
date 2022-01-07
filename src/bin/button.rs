// $ DEFMT_LOG=info cargo rb button
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use dwt_systick_monotonic::{DwtSystick, ExtU32};
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        prelude::*,
    };
    const FREQ: u32 = 48_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;
    #[shared]
    struct Shared {
        btn: PC13<Input<PullUp>>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(FREQ.hz()).freeze();

        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        defmt::info!("Press the button!");
        (Shared { btn }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(binds = EXTI15_10, shared = [btn])]
    fn on_exti(mut ctx: on_exti::Context) {
        ctx.shared.btn.lock(ExtiPin::clear_interrupt_pending_bit);
        debounce::spawn_after(30.millis()).ok();
    }

    #[task(shared = [btn], local = [hold: Option<hold::SpawnHandle> = None])]
    fn debounce(mut ctx: debounce::Context) {
        if let Some(handle) = ctx.local.hold.take() {
            handle.cancel().ok();
        }
        if ctx.shared.btn.lock(|b| b.is_low()) {
            defmt::info!("Button was pressed!");
            *ctx.local.hold = hold::spawn_after(1.secs()).ok();
        }
    }

    #[task(shared = [btn])]
    fn hold(mut ctx: hold::Context) {
        if ctx.shared.btn.lock(|b| b.is_low()) {
            defmt::info!("Long press...");
        }
    }
}
