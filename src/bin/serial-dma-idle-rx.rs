// $ cargo rb serial-dma-idle-rx
// Receive serial data of arbitrary length using DMA
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[SPI2])]
mod app {
    use stm32f4xx_hal::{
        dma::{config::DmaConfig, PeripheralToMemory, Stream5, StreamsTuple, Transfer},
        pac::{DMA2, USART1},
        prelude::*,
        serial::{config::*, Rx, Serial},
    };
    const BUF_SIZE: usize = 8;
    use heapless::Vec;

    #[shared]
    struct Shared {
        #[lock_free]
        rx: Transfer<Stream5<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[local]
    struct Local {}

    #[init(local = [rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.device.RCC.ahb1enr.modify(|_, w| w.dma2en().enabled());
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let rx_pin = gpioa.pa10.into_alternate();
        let serial_config = Config {
            baudrate: 115_200.bps(),
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
            dma: stm32f4xx_hal::serial::config::DmaConfig::Rx,
        };
        let serial = Serial::rx(ctx.device.USART1, rx_pin, serial_config, clocks).unwrap();
        // Enable idle line interrupt
        unsafe { (*USART1::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) }

        let stream = StreamsTuple::new(ctx.device.DMA2).5;
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);
        let mut rx =
            Transfer::init_peripheral_to_memory(stream, serial, ctx.local.rx_buf, None, dma_config);
        rx.start(|_| ());

        defmt::info!("Send me data of arbitrary length (<= 256 bytes)");
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
        print::spawn(Vec::from_slice(&data).unwrap()).ok();
    }

    // Triggers on serial line Idle
    #[task(binds = USART1, shared = [rx], priority = 2)]
    fn on_idle(ctx: on_idle::Context) {
        let rx = ctx.shared.rx;
        rx.pause(|_| clear_idle_interrupt());
        let pending = unsafe { (*DMA2::ptr()).st[5].ndtr.read().ndt().bits() as usize };
        let mut data = [0u8; BUF_SIZE];
        unsafe {
            rx.next_transfer_with(|buf, _| {
                data.copy_from_slice(buf);
                (buf, ())
            })
            .ok()
        };
        print::spawn(Vec::from_slice(&data[..BUF_SIZE - pending]).unwrap()).ok();
    }

    #[task(local = [msg: Vec<u8, 256> = Vec::new()], priority = 1)]
    fn print(ctx: print::Context, data: Vec<u8, BUF_SIZE>) {
        let is_completed = data.len() != BUF_SIZE;
        ctx.local.msg.extend(data);
        if is_completed {
            match core::str::from_utf8(ctx.local.msg.as_slice()) {
                Ok(str) => defmt::info!("{}", str),
                _ => defmt::info!("{:x}", ctx.local.msg.as_slice()),
            }
            ctx.local.msg.clear();
        }
    }

    #[inline]
    fn clear_idle_interrupt() {
        unsafe {
            let _ = (*USART1::ptr()).sr.read().idle();
            let _ = (*USART1::ptr()).dr.read().bits();
        }
    }
}
