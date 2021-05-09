// $ cargo rb button
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::{Milliseconds, Seconds};
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        prelude::*,
    };

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<48_000_000>; // 48 MHz

    #[resources]
    struct Resources {
        btn: PC13<Input<PullUp>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        let mut sys_cfg = ctx.device.SYSCFG.constrain();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::FALLING);

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        defmt::info!("Press the button!");
        (init::LateResources { btn }, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = EXTI15_10, resources = [btn])]
    fn on_exti(mut ctx: on_exti::Context) {
        ctx.resources.btn.lock(|b| b.clear_interrupt_pending_bit());
        debounce::spawn_after(Milliseconds(30_u32)).ok();
    }

    #[task(resources = [btn])]
    fn debounce(mut ctx: debounce::Context) {
        static mut HOLD: Option<hold::SpawnHandle> = None;
        ctx.resources.btn.lock(|btn| {
            if btn.is_low().unwrap() {
                defmt::info!("Button was pressed!");
                *HOLD = match HOLD.take() {
                    Some(handle) => handle.reschedule_after(Seconds(1_u32)).ok(),
                    None => hold::spawn_after(Seconds(1_u32)).ok(),
                };
            }
        });
    }

    #[task(resources = [btn])]
    fn hold(mut ctx: hold::Context) {
        ctx.resources.btn.lock(|btn| {
            if btn.is_low().unwrap() {
                defmt::info!("Long press...");
            }
        });
    }
}
