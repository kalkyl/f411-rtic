// $ cargo rb pwm
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use pwm::C1;
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Edge, ExtiPin, Input, PullUp},
        prelude::*,
        pwm::{self, PwmChannels},
        stm32::TIM2,
    };

    #[resources]
    struct Resources {
        btn: PC13<Input<PullUp>>,
        #[init(1)]
        level: u16,
        pwm: PwmChannels<TIM2, C1>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        // Enable SYSCFG.
        let mut sys_cfg = ctx.device.SYSCFG.constrain();

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // Set up the LED and PWM. On the Nucleo-F411RE it's connected to pin PA5.
        let gpioa = ctx.device.GPIOA.split();
        let led = gpioa.pa5.into_alternate_af1();
        let mut pwm = pwm::tim2(ctx.device.TIM2, led, clocks, 20.khz());
        pwm.enable();

        // Set up the button. On the Nucleo-F411RE it's connected to pin PC13.
        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        btn.make_interrupt_source(&mut sys_cfg);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::FALLING);

        defmt::info!("Press button!");
        (init::LateResources { btn, pwm }, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = EXTI15_10, resources = [btn, level, pwm])]
    fn on_exti(mut ctx: on_exti::Context) {
        ctx.resources.btn.lock(|b| b.clear_interrupt_pending_bit());
        (ctx.resources.level, ctx.resources.pwm).lock(|level, pwm| {
            defmt::info!("Duty = {:?}/{:?}", pwm.get_max_duty(), *level);
            pwm.set_duty(pwm.get_max_duty() / *level);
            *level = if *level < 2048 { (*level) * 2 } else { 1 };
        });
    }
}
