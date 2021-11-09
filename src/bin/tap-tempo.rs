// $ cargo rb tap-tempo
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
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

    #[shared]
    struct Shared {
        btn: PC13<Input<PullUp>>,
        pressed: Option<Instant<MyMono>>,
        ms: u32,
    }

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_push_pull_output();

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

        defmt::info!("Tap the button!");
        blink::spawn().ok();
        (
            Shared {
                btn,
                pressed: None,
                ms: 500,
            },
            Local { led },
            init::Monotonics(mono),
        )
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

    #[task(shared = [btn, ms, pressed], local = [clear: Option<clear::SpawnHandle> = None])]
    fn debounce(ctx: debounce::Context) {
        if let Some(handle) = ctx.local.clear.take() {
            handle.cancel().ok();
        }
        (ctx.shared.btn, ctx.shared.pressed, ctx.shared.ms).lock(|btn, pressed, ms| {
            if btn.is_low() {
                if let Some(instant) = pressed.take() {
                    let diff: Option<Milliseconds> = monotonics::MyMono::now()
                        .checked_duration_since(&instant)
                        .and_then(|d| d.try_into().ok());
                    if let Some(Milliseconds(t)) = diff {
                        *ms = t;
                        defmt::info!("{} bpm", 60_000 / *ms);
                    }
                }
                pressed.replace(monotonics::MyMono::now());
            }
        });
        *ctx.local.clear = clear::spawn_after(Milliseconds(2000_u32)).ok();
    }

    #[task(shared = [pressed])]
    fn clear(mut ctx: clear::Context) {
        ctx.shared.pressed.lock(|i| i.take());
    }

    #[task(shared = [ms], local = [led])]
    fn blink(mut ctx: blink::Context) {
        blink::spawn_after(Milliseconds(ctx.shared.ms.lock(|t| *t))).ok();
        ctx.local.led.set_high();
        cortex_m::asm::delay(480_000);
        ctx.local.led.set_low();
    }
}
