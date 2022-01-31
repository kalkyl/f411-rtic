// $ DEFMT_LOG=info cargo rb usb-audio
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32f4xx_hal::{
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_audio: usbd_audio::AudioClass<'static, UsbBus<USB>>,
    }

    #[init(local = [ep_memory: [u32; 512] = [0; 512], usb_bus: Option<UsbBusAllocator<UsbBusType>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).require_pll48clk().freeze();

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

        let usb_audio = AudioClassBuilder::new()
            .input(
                StreamConfig::new_discrete(Format::S16le, 1, &[48000], TerminalType::InMicrophone)
                    .unwrap(),
            )
            .output(
                StreamConfig::new_discrete(Format::S24le, 2, &[48000], TerminalType::OutSpeaker)
                    .unwrap(),
            )
            .build(usb_bus.as_ref().unwrap())
            .unwrap();

        let usb_dev = UsbDeviceBuilder::new(usb_bus.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
            .max_packet_size_0(64)
            .manufacturer("Kiffie Labs")
            .product("Audio port")
            .serial_number("42")
            .build();

        (Shared {}, Local { usb_dev, usb_audio }, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    #[task(binds = OTG_FS, local = [usb_dev, usb_audio, cnt: u32 = 0, in_setting: u8 = 0, out_setting: u8 = 0])]
    fn on_usb(ctx: on_usb::Context) {
        let SINE: [i16; 48] = [
            0i16, 4276, 8480, 12539, 16383, 19947, 23169, 25995, 28377, 30272, 31650, 32486, 32767,
            32486, 31650, 30272, 28377, 25995, 23169, 19947, 16383, 12539, 8480, 4276, 0, -4276,
            -8480, -12539, -16383, -19947, -23169, -25995, -28377, -30272, -31650, -32486, -32767,
            -32486, -31650, -30272, -28377, -25995, -23169, -19947, -16383, -12539, -8480, -4276,
        ];
        let sinetab_le = unsafe { &*(&SINE as *const _ as *const [u8; 96]) };

        let usb_dev = ctx.local.usb_dev;
        let usb_audio = ctx.local.usb_audio;
        let ctr = ctx.local.cnt;
        let input_alt_setting = ctx.local.in_setting;
        let output_alt_setting = ctx.local.out_setting;

        if usb_dev.poll(&mut [usb_audio]) {
            let mut buf = [0u8; 1024];
            if let Ok(len) = usb_audio.read(&mut buf) {
                *ctr += 1;
                if *ctr >= 1000 {
                    *ctr = 0;
                    defmt::info!("RX len = {}", len);
                }
            }
        }
        if *input_alt_setting != usb_audio.input_alt_setting().unwrap()
            || *output_alt_setting != usb_audio.output_alt_setting().unwrap()
        {
            *input_alt_setting = usb_audio.input_alt_setting().unwrap();
            *output_alt_setting = usb_audio.output_alt_setting().unwrap();
            defmt::info!("Alt. set. {} {}", input_alt_setting, output_alt_setting);
        }
        usb_audio.write(sinetab_le).ok();
    }
}
