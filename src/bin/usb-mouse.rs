// $ cargo rb usb-mouse
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use stm32f4xx_hal::{
        gpio::{gpioc::PC13, Input, PullUp},
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };
    use usb_device::{bus::UsbBusAllocator, prelude::*};
    use usbd_hid::{
        descriptor::{generator_prelude::*, MouseReport},
        hid_class::HIDClass,
    };

    #[resources]
    struct Resources {
        btn: PC13<Input<PullUp>>,
        hid: HIDClass<'static, UsbBusType>,
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (init::LateResources, init::Monotonics) {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let _clocks = rcc.cfgr.sysclk(48.mhz()).require_pll48clk().freeze();

        let gpioc = ctx.device.GPIOC.split();
        let btn = gpioc.pc13.into_pull_up_input();

        let gpioa = ctx.device.GPIOA.split();
        let usb = USB {
            usb_global: ctx.device.OTG_FS_GLOBAL,
            usb_device: ctx.device.OTG_FS_DEVICE,
            usb_pwrclk: ctx.device.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate_af10(),
            pin_dp: gpioa.pa12.into_alternate_af10(),
        };
        USB_BUS.replace(UsbBus::new(usb, EP_MEMORY));

        let hid = HIDClass::new(USB_BUS.as_ref().unwrap(), MouseReport::desc(), 60);
        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0xc410, 0x0000))
            .manufacturer("Fake company")
            .product("Mouse")
            .serial_number("TEST")
            .device_class(0)
            .build();

        defmt::info!("Mouse example");
        (
            init::LateResources { btn, hid, usb_dev },
            init::Monotonics(),
        )
    }

    #[idle(resources=[btn, hid])]
    fn idle(mut ctx: idle::Context) -> ! {
        static mut COUNTER: u8 = 0;
        loop {
            let buttons = match ctx.resources.btn.lock(|b| b.is_low().unwrap()) {
                true => 1,
                false => 0,
            };
            let report = MouseReport {
                x: if *COUNTER < 64 { 3 } else { -3 },
                y: 0,
                buttons,
                wheel: 0,
            };
            ctx.resources.hid.lock(|hid| hid.push_input(&report).ok());
            *COUNTER = (*COUNTER + 1) % 128;
            cortex_m::asm::delay(500_000);
        }
    }

    #[task(binds=OTG_FS, resources = [hid, usb_dev])]
    fn on_usb(mut ctx: on_usb::Context) {
        let mut usb_dev = ctx.resources.usb_dev;
        ctx.resources.hid.lock(|hid| {
            if !usb_dev.lock(|u| u.poll(&mut [hid])) {
                return;
            }
        });
    }
}
