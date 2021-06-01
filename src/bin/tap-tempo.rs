// $ cargo rb tap-tempo
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use core::convert::TryInto;
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::{duration::Milliseconds, Instant};
    use stm32f4xx_hal::{
        gpio::{gpioa::PA5, gpioc::PC13, Edge, ExtiPin, Input, Output, PullUp, PushPull},
        prelude::*,
    };

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<48_000_000>; // 48 MHz

    #[resources]
    struct Resources {
        btn: PC13<Input<PullUp>>,
        led: PA5<Output<PushPull>>,

        #[init(None)]
        pressed: Option<Instant<MyMono>>,
        #[init(500)]
        ms: u32,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

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

        defmt::info!("Tap the button!");
        blink::spawn().ok();
        (init::LateResources { btn, led }, init::Monotonics(mono))
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

    #[task(resources = [btn, pressed, ms])]
    fn debounce(ctx: debounce::Context) {
        static mut CLEAR: Option<clear::SpawnHandle> = None;
        let debounce::Resources { btn, pressed, ms } = ctx.resources;
        if let Some(handle) = CLEAR.take() {
            handle.cancel().ok();
        }
        (btn, pressed, ms).lock(|btn, pressed, ms| {
            if btn.is_low().unwrap() {
                if let Some(instant) = pressed.take() {
                    let diff: Option<Milliseconds> = monotonics::MyMono::now()
                        .checked_duration_since(&instant)
                        .and_then(|d| d.try_into().ok());
                    if let Some(t) = diff {
                        *ms = t.0;
                        defmt::info!("{} bpm", 60_000 / *ms);
                    }
                }
                pressed.replace(monotonics::MyMono::now());
            }
        });
        *CLEAR = clear::spawn_after(Milliseconds(2000_u32)).ok();
    }

    #[task(resources = [pressed])]
    fn clear(mut ctx: clear::Context) {
        ctx.resources.pressed.lock(|i| i.take());
    }

    #[task(resources = [led, ms])]
    fn blink(mut ctx: blink::Context) {
        blink::spawn_after(Milliseconds(ctx.resources.ms.lock(|t| *t))).ok();
        ctx.resources.led.lock(|l| l.set_high().ok());
        cortex_m::asm::delay(480_000);
        ctx.resources.led.lock(|l| l.set_low().ok());
    }
}
