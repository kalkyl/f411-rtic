// $ DEFMT_LOG=info cargo rb pwm
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use pwm::C1;
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        pac::TIM2,
        prelude::*,
        pwm::{self, PwmChannel},
        timer::Timer,
    };

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        btn: PC13<Input<PullUp>>,
        pwm: PwmChannel<TIM2, C1>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Enable SYSCFG.
        let mut sys_cfg = ctx.device.SYSCFG.constrain();

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Set up the LED and PWM. On the Nucleo-F411RE it's connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_alternate();
        let pwm = Timer::new(ctx.device.TIM2, &clocks).pwm(led, 20.khz());

        // Set up the button. On the Nucleo-F411RE it's connected to pin PC13.
        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::Falling);

        defmt::info!("Press button!");
        (Shared {}, Local { btn, pwm }, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds = EXTI15_10, local = [btn, pwm, level: u16 = 1])]
    fn on_exti(ctx: on_exti::Context) {
        ctx.local.btn.clear_interrupt_pending_bit();
        let (level, pwm) = (ctx.local.level, ctx.local.pwm);
        defmt::info!("Duty = {:?}/{:?}", pwm.get_max_duty(), *level);
        pwm.set_duty(pwm.get_max_duty() / *level);
        *level = if *level < 2048 { (*level) * 2 } else { 1 };
    }
}
