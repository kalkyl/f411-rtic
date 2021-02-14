#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use core::fmt::Write;

    use ssd1306::{mode::TerminalMode, prelude::*, Builder, I2CDIBuilder};
    use stm32f4xx_hal::{
        gpio::{
            gpiob::{PB8, PB9},
            gpioc::PC13,
            AlternateOD, Edge, ExtiPin, Input, PullUp, AF4,
        },
        i2c::I2c,
        prelude::*,
        stm32::I2C1,
    };

    #[resources]
    struct Resources {
        btn: PC13<Input<PullUp>>,
        #[init(65)]
        count: u8,
        disp: TerminalMode<I2CInterface<I2c<I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>>>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> init::LateResources {
        // Enable SYSCFG.
        ctx.device.RCC.apb2enr.write(|w| w.syscfgen().enabled());

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpiob = ctx.device.GPIOB.split();
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        let i2c = I2c::i2c1(ctx.device.I2C1, (scl, sda), 400.khz(), clocks);

        // Configure the OLED display
        let interface = I2CDIBuilder::new().init(i2c);
        let mut disp: TerminalMode<_> = Builder::new().connect(interface).into();
        disp.init().ok();
        disp.flush().ok();
        disp.clear().ok();
        disp.write_str("Hello world!\n").ok();

        // Set up the button. On the Nucleo-F411RE it's connected to pin PC13.
        let gpioc = ctx.device.GPIOC.split();
        let mut btn = gpioc.pc13.into_pull_up_input();
        btn.make_interrupt_source(&mut ctx.device.SYSCFG);
        btn.enable_interrupt(&mut ctx.device.EXTI);
        btn.trigger_on_edge(&mut ctx.device.EXTI, Edge::FALLING);

        defmt::info!("Press button!");
        init::LateResources { btn, disp }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = EXTI15_10, resources = [btn, count, disp])]
    fn on_exti(ctx: on_exti::Context) {
        let on_exti::Resources {
            mut btn,
            mut count,
            mut disp,
        } = ctx.resources;
        btn.lock(|b| b.clear_interrupt_pending_bit());
        count.lock(|c| {
            // Print letter on display
            disp.lock(|disp| disp.write_str(core::str::from_utf8(&[*c]).unwrap()).ok());
            // Wrap around
            *c = if *c < 90 { *c + 1 } else { 65 };
        });
    }
}
