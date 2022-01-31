// $ DEFMT_LOG=info cargo rb usb-audio-i2s
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac)]
mod app {
    use stm32_i2s_v12x::format::{Data16Frame16, FrameFormat};
    use stm32_i2s_v12x::{MasterClock, MasterConfig, Polarity, TransmitMode};
    use stm32f4xx_hal::dma::config::DmaConfig;
    use stm32f4xx_hal::dma::MemoryToPeripheral;
    use stm32f4xx_hal::dma::{Stream5, StreamsTuple, Transfer};
    use stm32f4xx_hal::gpio::{
        gpioa::PA4,
        gpioc::{PC10, PC12, PC7},
        Alternate, PushPull,
    };
    use stm32f4xx_hal::pac::{interrupt, Interrupt};
    use stm32f4xx_hal::pac::{CorePeripherals, Peripherals};
    use stm32f4xx_hal::pac::{DMA1, SPI3};
    use stm32f4xx_hal::{
        i2s::I2s,
        otg_fs::{UsbBus, UsbBusType, USB},
        prelude::*,
    };
    use usb_device::{class_prelude::*, prelude::*};
    use usbd_audio::{AudioClassBuilder, Format, StreamConfig, TerminalType};
    const SINE_SAMPLES: usize = 64;

    /// A sine wave spanning 64 samples
    ///
    /// With a sample rate of 48 kHz, this produces a 750 Hz tone.
    const SINE_750: [i16; SINE_SAMPLES] = [
        0, 3211, 6392, 9511, 12539, 15446, 18204, 20787, 23169, 25329, 27244, 28897, 30272, 31356,
        32137, 32609, 32767, 32609, 32137, 31356, 30272, 28897, 27244, 25329, 23169, 20787, 18204,
        15446, 12539, 9511, 6392, 3211, 0, -3211, -6392, -9511, -12539, -15446, -18204, -20787,
        -23169, -25329, -27244, -28897, -30272, -31356, -32137, -32609, -32767, -32609, -32137,
        -31356, -30272, -28897, -27244, -25329, -23169, -20787, -18204, -15446, -12539, -9511,
        -6392, -3211,
    ];

    type I2sDmaTransfer = Transfer<
        Stream5<DMA1>,
        stm32_i2s_v12x::I2s<
            I2s<
                SPI3,
                (
                    PA4<Alternate<6>>,
                    PC10<Alternate<6>>,
                    PC7<Alternate<6>>,
                    PC12<Alternate<6>>,
                ),
            >,
            TransmitMode<Data16Frame16>,
        >,
        MemoryToPeripheral,
        &'static mut [u16; SINE_SAMPLES * 2],
        0,
    >;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        usb_dev: UsbDevice<'static, UsbBus<USB>>,
        usb_audio: usbd_audio::AudioClass<'static, UsbBus<USB>>,
        transfer: Option<I2sDmaTransfer>,
    }

    #[init(local = [ep_memory: [u32; 512] = [0; 512], usb_bus: Option<UsbBusAllocator<UsbBusType>> = None, SINE_750_MUT: [u16; SINE_SAMPLES * 2] = [0; SINE_SAMPLES * 2]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Set up the system clock.
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            .sysclk(86.mhz())
            .require_pll48clk()
            .i2s_clk(86.mhz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpioc = ctx.device.GPIOC.split();

        // I2S pins: (WS, CK, MCLK, SD) for I2S3
        let i2s_pins = (
            gpioa.pa4.into_alternate(),
            gpioc.pc10.into_alternate(),
            gpioc.pc7.into_alternate(),
            gpioc.pc12.into_alternate(),
        );
        let hal_i2s = I2s::new(ctx.device.SPI3, i2s_pins, clocks);
        let i2s_clock = hal_i2s.input_clock();

        // Audio timing configuration:
        // Sample rate 48 kHz
        // 16 bits per sample -> SCK rate 1.536 MHz
        // MCK frequency = 256 * sample rate -> MCK rate 12.228 MHz (also equal to 8 * SCK rate)
        let sample_rate = 48000;

        let i2s = stm32_i2s_v12x::I2s::new(hal_i2s);
        let mut i2s = i2s.configure_master_transmit(MasterConfig::with_sample_rate(
            i2s_clock.0,
            sample_rate,
            Data16Frame16,
            FrameFormat::PhilipsI2s,
            Polarity::IdleHigh,
            MasterClock::Enable,
        ));
        i2s.set_dma_enabled(true);

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

        {
            // Copy samples from flash into the buffer that DMA will use
            let mut dest_iter = ctx.local.SINE_750_MUT.iter_mut();
            for sample in SINE_750.iter() {
                // Duplicate sample for the left and right channels
                let left = dest_iter.next().unwrap();
                let right = dest_iter.next().unwrap();
                *left = *sample as u16;
                *right = *sample as u16;
            }
        }

        // Set up DMA: DMA 1 stream 5 channel 0 memory -> peripheral
        let dma1_streams = StreamsTuple::new(ctx.device.DMA1);
        let dma_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true);
        let mut transfer: I2sDmaTransfer = Transfer::init_memory_to_peripheral(
            dma1_streams.5,
            i2s,
            ctx.local.SINE_750_MUT,
            None,
            dma_config,
        );

        transfer.start(|i2s| i2s.enable());

        (
            Shared {},
            Local {
                usb_dev,
                usb_audio,
                transfer: Some(transfer),
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    /// This interrupt handler runs when DMA 1 finishes a transfer to the I2S peripheral
    #[task(binds = DMA1_STREAM5, local = [transfer])]
    fn on_dma(ctx: on_dma::Context) {
        static mut TRANSFER: Option<I2sDmaTransfer> = None;

        let transfer = ctx.local.transfer.as_mut().unwrap();

        transfer.clear_transfer_complete_interrupt();
        unsafe {
            transfer
                .next_transfer_with(|buffer, _active_buffer| {
                    // Transfer again with the same buffer
                    (buffer, ())
                })
                .unwrap();
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
