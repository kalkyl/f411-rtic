// $ cargo rb tap-tempo
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers = [USART1])]
mod app {
    use dwt_systick_monotonic::{
        fugit::{MillisDurationU32, TimerInstantU32},
        DwtSystick, ExtU32,
    };
    use stm32f4xx_hal::{
        gpio::{gpioa::PA5, gpioc::PC13, Edge, ExtiPin, Input, Output, PullUp, PushPull},
        prelude::*,
    };
    const FREQ: u32 = 48_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        btn: PC13<Input<PullUp>>,
        pressed: Option<TimerInstantU32<FREQ>>,
        ms: MillisDurationU32,
    }

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(FREQ.hz()).freeze();

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
                ms: 500.millis(),
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
        debounce::spawn_after(30.millis()).ok();
    }

    #[task(shared = [btn, ms, pressed], local = [clear: Option<clear::SpawnHandle> = None])]
    fn debounce(ctx: debounce::Context) {
        if let Some(handle) = ctx.local.clear.take() {
            handle.cancel().ok();
        }
        (ctx.shared.btn, ctx.shared.pressed, ctx.shared.ms).lock(|btn, pressed, ms| {
            if btn.is_low() {
                if let Some(instant) = pressed.take() {
                    *ms = (monotonics::now() - instant).convert();
                    defmt::info!("{:?} bpm", 60_000 / (*ms).ticks());
                }
                pressed.replace(monotonics::now());
            }
        });
        *ctx.local.clear = clear::spawn_after(2_000.millis()).ok();
    }

    #[task(shared = [pressed])]
    fn clear(mut ctx: clear::Context) {
        ctx.shared.pressed.lock(|i| i.take());
    }

    #[task(shared = [ms], local = [led])]
    fn blink(mut ctx: blink::Context) {
        blink::spawn_after(ctx.shared.ms.lock(|t| *t).convert()).ok();
        ctx.local.led.set_high();
        cortex_m::asm::delay(FREQ / 100);
        ctx.local.led.set_low();
    }
}
