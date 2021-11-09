// $ cargo rb serial-dma-tx
// Transmit serial data using DMA
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, dispatchers=[SPI2])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic_monotonic::Seconds;
    use stm32f4xx_hal::dma::traits::StreamISR;
    use stm32f4xx_hal::pac::DMA2;
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, MemoryToPeripheral, Stream7, StreamX, StreamsTuple, Transfer},
        pac::USART1,
        prelude::*,
        serial::{config::*, Serial, Tx},
    };
    const BUF_SIZE: usize = 8;
    const FREQ: u32 = 48_000_000;

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        #[lock_free]
        tx: Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[local]
    struct Local {}

    #[init(local = [tx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(FREQ.hz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let serial_config = Config {
            baudrate: 115_200.bps(),
            dma: stm32f4xx_hal::serial::config::DmaConfig::Tx,
            ..Config::default()
        };
        let serial = Serial::tx(ctx.device.USART1, tx_pin, serial_config, clocks).unwrap();
        let stream = StreamsTuple::new(ctx.device.DMA2).7;
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);
        let tx =
            Transfer::init_memory_to_peripheral(stream, serial, ctx.local.tx_buf, None, dma_config);

        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);

        emit_message::spawn().ok();
        (Shared { tx }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(shared = [tx])]
    fn emit_message(ctx: emit_message::Context) {
        let msg = "Hello!!!";
        defmt::info!("TX: {}", msg);
        let tx = ctx.shared.tx;
        unsafe {
            tx.next_transfer_with(|buf, _| {
                buf.copy_from_slice(msg.as_bytes());
                (buf, ())
            })
            .ok();
        }
    }

    // Triggers on DMA transfer complete
    #[task(binds=DMA2_STREAM7, shared = [tx])]
    fn on_dma(ctx: on_dma::Context) {
        if StreamX::<DMA2, 7>::get_transfer_complete_flag() {
            ctx.shared.tx.clear_transfer_complete_interrupt();
            emit_message::spawn_after(Seconds(1_u32)).ok();
        }
    }
}
