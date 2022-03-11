// $ DEFMT_LOG=info cargo rb serial-dma-dbl-rx
// Receive serial data using DMA with double buffer
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers=[SPI2])]
mod app {
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, PeripheralToMemory, Stream5, StreamsTuple, Transfer},
        pac::{DMA2, USART1},
        prelude::*,
        serial::{config::Config, Rx, Serial},
    };
    const BUF_SIZE: usize = 8;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        rx: Transfer<Stream5<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[init(local = [buf1: [u8; BUF_SIZE] = [0; BUF_SIZE], buf2: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma2en().enabled());
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let rx_pin = gpioa.pa10.into_alternate();
        let serial_config = Config {
            baudrate: 115_200.bps(),
            dma: stm32f4xx_hal::serial::config::DmaConfig::Rx,
            ..Config::default()
        };
        let serial = Serial::rx(ctx.device.USART1, rx_pin, serial_config, clocks).unwrap();
        let stream = StreamsTuple::new(ctx.device.DMA2).5;
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .double_buffer(true);

        let mut rx = Transfer::init_peripheral_to_memory(
            stream,
            serial,
            ctx.local.buf1,
            Some(ctx.local.buf2),
            dma_config,
        );
        rx.start(|_| ());

        defmt::info!("Send me {} byte frames", BUF_SIZE);
        (Shared {}, Local { rx }, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            continue;
        }
    }

    // Triggers on DMA transfer complete
    #[task(binds=DMA2_STREAM5, local = [rx], priority = 2)]
    fn on_dma(ctx: on_dma::Context) {
        let rx = ctx.local.rx;
        let data = unsafe {
            rx.next_transfer_with(|buf, _| {
                let data = *buf;
                (buf, data)
            })
            .unwrap()
        };
        print::spawn(data).ok();
    }

    #[task(priority = 1, capacity = 4)]
    fn print(_: print::Context, data: [u8; BUF_SIZE]) {
        match core::str::from_utf8(&data) {
            Ok(str) => defmt::info!("RX: {}", str),
            _ => defmt::info!("RX: {:x}", data),
        }
    }
}
