// $ DEFMT_LOG=info cargo rb usb-mouse
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Input, PullUp},
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };
    use usb_device::{bus::UsbBusAllocator, prelude::*};
    use usbd_hid::{
        descriptor::{generator_prelude::SerializedDescriptor, MouseReport},
        hid_class::HIDClass,
    };

    #[shared]
    struct Shared {
        hid: HIDClass<'static, UsbBusType>,
    }

    #[local]
    struct Local {
        btn: PC13<Input<PullUp>>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
    }

    #[init(local = [ep_memory: [u32; 1024] = [0; 1024], usb_bus: Option<UsbBusAllocator<UsbBusType>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).require_pll48clk().freeze();

        let gpioc = ctx.device.GPIOC.split();
        let btn = gpioc.pc13.into_pull_up_input();

        let gpioa = ctx.device.GPIOA.split();
        let usb = USB {
            usb_global: ctx.device.OTG_FS_GLOBAL,
            usb_device: ctx.device.OTG_FS_DEVICE,
            usb_pwrclk: ctx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        let usb_bus = ctx.local.usb_bus;
        usb_bus.replace(UsbBus::new(usb, ctx.local.ep_memory));

        let hid = HIDClass::new(usb_bus.as_ref().unwrap(), MouseReport::desc(), 60);
        let usb_dev = UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0xc410, 0x0000))
            .manufacturer("Fake company")
            .product("Mouse")
            .serial_number("TEST")
            .device_class(0)
            .build();

        defmt::info!("Mouse example");
        (Shared { hid }, Local { btn, usb_dev }, init::Monotonics())
    }

    #[idle(shared = [hid], local=[btn, counter: u8 = 0])]
    fn idle(mut ctx: idle::Context) -> ! {
        let counter = ctx.local.counter;
        loop {
            let buttons = if ctx.local.btn.is_low() { 1 } else { 0 };
            let report = MouseReport {
                x: if *counter < 64 { 3 } else { -3 },
                y: 0,
                buttons,
                wheel: 0,
                pan: 0,
            };
            ctx.shared.hid.lock(|hid| hid.push_input(&report).ok());
            *counter = (*counter + 1) % 128;
            cortex_m::asm::delay(500_000);
        }
    }

    #[task(binds=OTG_FS, shared = [hid], local=[usb_dev])]
    fn on_usb(mut ctx: on_usb::Context) {
        let usb_dev = ctx.local.usb_dev;
        ctx.shared.hid.lock(|hid| if !usb_dev.poll(&mut [hid]) {});
    }
}
