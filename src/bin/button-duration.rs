// $ cargo rb button-duration
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use core::convert::TryInto;
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::{duration::Milliseconds, Instant};
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
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::RISING_FALLING);

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        defmt::info!("Press or hold the button!");
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
        static mut PRESSED_AT: Option<Instant<MyMono>> = None;
        if let Some(handle) = HOLD.take() {
            handle.cancel().ok();
        }
        if ctx.resources.btn.lock(|b| b.is_low().unwrap()) {
            PRESSED_AT.replace(monotonics::MyMono::now());
            *HOLD = hold::spawn_after(Milliseconds(1000_u32)).ok();
        } else {
            if PRESSED_AT
                .take()
                .and_then(|i| monotonics::MyMono::now().checked_duration_since(&i))
                .and_then(|d| d.try_into().ok())
                .map(|t: Milliseconds<u32>| t < Milliseconds(1000_u32))
                .unwrap_or(false)
            {
                defmt::info!("Short press")
            }
        }
    }

    #[task(resources = [btn])]
    fn hold(mut ctx: hold::Context) {
        if ctx.resources.btn.lock(|b| b.is_low().unwrap()) {
            defmt::info!("Long press");
        }
    }
}
