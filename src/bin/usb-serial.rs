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
    use usbd_serial::SerialPort;

    #[resources]
    struct Resources {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        serial: SerialPort<'static, UsbBus<USB>>,
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

        let serial = SerialPort::new(USB_BUS.as_ref().unwrap());
        let usb_dev = UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Fake company")
            .product("Serial port")
            .serial_number("TEST")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();

        defmt::info!("Send me a string!");
        init::LateResources { serial, usb_dev }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds=OTG_FS, resources = [serial, usb_dev])]
    fn on_usb(mut ctx: on_usb::Context) {
        let mut serial = ctx.resources.serial;
        ctx.resources.usb_dev.lock(|usb_dev| {
            serial.lock(|serial| {
                if !usb_dev.poll(&mut [serial]) {
                    return;
                }
                let mut buf = [0u8; 64];
                match serial.read(&mut buf) {
                    Ok(count) if count > 0 => {
                        defmt::info!("Received: {:?}", core::str::from_utf8(&buf[..]).unwrap());
                        // Echo back in upper case
                        for c in buf[0..count].iter_mut() {
                            if 0x61 <= *c && *c <= 0x7a {
                                *c &= !0x20;
                            }
                        }
                        let mut write_offset = 0;
                        while write_offset < count {
                            match serial.write(&buf[write_offset..count]) {
                                Ok(len) if len > 0 => {
                                    write_offset += len;
                                }
                                _ => {}
                            }
                        }
                    }
                    _ => {}
                }
            });
        });
    }
}
