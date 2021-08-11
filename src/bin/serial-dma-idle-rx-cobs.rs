// $ cargo rb serial-dma-idle-rx-cobs
// Receive serial data of arbitrary length using DMA
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[SPI2])]
mod app {
    use stm32f4xx_hal::dma::traits::Stream;
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, PeripheralToMemory, Stream5, StreamX, StreamsTuple, Transfer},
        pac::{DMA2, USART1},
        prelude::*,
        serial::{config::*, Rx, Serial},
    };
    const BUF_SIZE: usize = 8;
    use defmt::Format;
    use heapless::Vec;
    use postcard::{CobsAccumulator, FeedResult};
    use serde::Deserialize;

    #[derive(Deserialize, Format)]
    struct MyData {
        a: u32,
        b: [u32; 4],
    }

    #[shared]
    struct Shared {
        #[lock_free]
        rx: Transfer<Stream5<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[local]
    struct Local {}

    #[init(local = [rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let rx_pin = gpioa.pa10.into_alternate();
        let serial_config = Config {
            baudrate: 115_200.bps(),
            dma: stm32f4xx_hal::serial::config::DmaConfig::Rx,
            ..Config::default()
        };
        let mut serial = Serial::rx(ctx.device.USART1, rx_pin, serial_config, clocks).unwrap();
        serial.listen_idle();

        let stream = StreamsTuple::new(ctx.device.DMA2).5;
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);
        let mut rx =
            Transfer::init_peripheral_to_memory(stream, serial, ctx.local.rx_buf, None, dma_config);
        rx.start(|_| ());

        defmt::info!("Send me: 02 7b 01 01 02 04 01 01 02 05 01 01 02 06 01 01 02 07 01 01 01 00");
        (Shared { rx }, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    // Triggers on DMA transfer complete
    #[task(binds=DMA2_STREAM5, shared = [rx], priority = 2)]
    fn on_dma(ctx: on_dma::Context) {
        let rx = ctx.shared.rx;
        let data = unsafe {
            rx.next_transfer_with(|buf, _| {
                let data = *buf;
                (buf, data)
            })
            .unwrap()
        };
        accumulate::spawn(Vec::from_slice(&data).unwrap()).ok();
    }

    // Triggers on serial line Idle
    #[task(binds = USART1, shared = [rx], priority = 2)]
    fn on_idle(ctx: on_idle::Context) {
        let rx = ctx.shared.rx;
        rx.pause(|serial| serial.clear_idle_interrupt());
        let end = BUF_SIZE - StreamX::<DMA2, 5>::get_number_of_transfers() as usize;
        let data = &mut [0u8; BUF_SIZE][..end];
        unsafe {
            let _ = rx.next_transfer_with(|buf, _| {
                data.copy_from_slice(&buf[..end]);
                (buf, ())
            });
        };
        accumulate::spawn(Vec::from_slice(data).unwrap()).ok();
    }

    #[task(local = [cobs_buf: CobsAccumulator<64> = CobsAccumulator::new()], priority = 1, capacity = 2)]
    fn accumulate(ctx: accumulate::Context, data: Vec<u8, BUF_SIZE>) {
        match ctx.local.cobs_buf.feed::<MyData>(data.as_slice()) {
            FeedResult::Success { data, .. } => {
                defmt::info!("{:?}", data);
            }
            _ => (),
        };
    }
}
