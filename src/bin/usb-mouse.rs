// $ cargo rb usb-serial
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use stm32f4xx_hal::{
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
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        hid: HIDClass<'static, UsbBusType>,
    }

    #[init]
    fn init(ctx: init::Context) -> init::LateResources {
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_BUS: Option<UsbBusAllocator<UsbBusType>> = None;

        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let _clocks = rcc.cfgr.sysclk(48.mhz()).require_pll48clk().freeze();

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
            .product("mouse")
            .serial_number("TEST")
            .device_class(0)
            .build();

        defmt::info!("Mouse example");
        init::LateResources { hid, usb_dev }
    }

    #[idle(resources=[hid])]
    fn idle(mut ctx: idle::Context) -> ! {
        static mut COUNTER: u8 = 0;
        const P: u8 = 128;
        loop {
            ctx.resources.hid.lock(|hid| {
                // Move mouse cursor
                if *COUNTER < P / 2 {
                    hid.push_input(&MouseReport {
                        x: 3,
                        y: 0,
                        buttons: 0,
                        wheel: 0,
                    })
                    .ok();
                } else {
                    hid.push_input(&MouseReport {
                        x: -3,
                        y: 0,
                        buttons: 0,
                        wheel: -0,
                    })
                    .ok();
                }
            });
            *COUNTER = (*COUNTER + 1) % P;
            cortex_m::asm::delay(500_000);
        }
    }

    #[task(binds=OTG_FS, resources = [hid, usb_dev])]
    fn on_usb(mut ctx: on_usb::Context) {
        let mut hid = ctx.resources.hid;
        ctx.resources.usb_dev.lock(|usb_dev| {
            hid.lock(|hid| {
                if !usb_dev.poll(&mut [hid]) {
                    return;
                }
            });
        });
    }
}
