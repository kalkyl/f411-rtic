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
    fn init(mut ctx: init::Context) -> init::LateResources {
        // Enable SYSCFG.
        ctx.device.RCC.apb2enr.write(|w| w.syscfgen().enabled());

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
        btn.make_interrupt_source(&mut ctx.device.SYSCFG);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::FALLING);

        defmt::info!("Press button!");
        init::LateResources { btn, pwm }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = EXTI15_10, resources = [btn, level, pwm])]
    fn on_exti(ctx: on_exti::Context) {
        let on_exti::Resources {
            mut btn,
            mut level,
            mut pwm,
        } = ctx.resources;
        btn.lock(|b| b.clear_interrupt_pending_bit());
        pwm.lock(|p| {
            let max_duty = p.get_max_duty();
            level.lock(|l| {
                p.set_duty(max_duty / *l);
                defmt::info!("Duty = {:?}/{:?}", max_duty, *l);
                *l = if *l < 2048 { (*l) * 2 } else { 1 };
            })
        });
    }
}
