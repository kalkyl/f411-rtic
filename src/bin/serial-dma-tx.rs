// $ cargo rb serial-dma-tx
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers=[SPI2])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic_monotonic::Seconds;
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, MemoryToPeripheral, Stream7, StreamsTuple, Transfer},
        prelude::*,
        serial::{config::*, Serial, Tx},
        stm32::{DMA2, USART1},
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
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma1en().enabled());
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let serial_config = Config {
            baudrate: 9_600.bps(),
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            dma: stm32f4xx_hal::serial::config::DmaConfig::Tx,
        };
        let serial = Serial::tx(ctx.device.USART1, tx_pin, serial_config, clocks).unwrap();
        let stream = StreamsTuple::new(ctx.device.DMA2).7;
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);

        let tx =
            Transfer::init_memory_to_peripheral(stream, serial, ctx.local.tx_buf, None, dma_config);

        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, FREQ);

        emit_status::spawn().ok();
        (Shared { tx }, Local {}, init::Monotonics(mono))
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(binds=DMA2_STREAM7, shared = [tx])]
    fn on_dma(ctx: on_dma::Context) {
        ctx.shared.tx.clear_transfer_complete_interrupt();
        emit_status::spawn_after(Seconds(1_u32)).ok();
    }

    #[task(shared = [tx])]
    fn emit_status(ctx: emit_status::Context) {
        let data = "Hello!!!";
        defmt::info!("TX: {}", data);
        let tx = ctx.shared.tx;
        unsafe {
            tx.next_transfer_with(|buf, _| {
                buf.copy_from_slice(data.as_bytes());
                (buf, ())
            })
            .ok();
        }
    }
}
