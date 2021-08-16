// $ cargo rb serial-dma-loopback
// Send/receive commands using serial dma
// Connect pin PA9 to PA10
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers=[SPI2])]
mod app {
    use dwt_systick_monotonic::DwtSystick;
    use rtic_monotonic::Milliseconds;
    use stm32f4xx_hal::dma::traits::Stream;
    use stm32f4xx_hal::serial::Event;
    use stm32f4xx_hal::{
        dma::{
            config::DmaConfig, MemoryToPeripheral, PeripheralToMemory, Stream5, Stream7, StreamX,
            StreamsTuple, Transfer,
        },
        gpio::{gpioa::PA5, Output, PushPull},
        pac::{DMA2, USART1},
        prelude::*,
        serial::{config::*, Rx, Serial, Tx},
    };
    const BUF_SIZE: usize = 8;
    use defmt::Format;
    use heapless::Vec;
    use postcard::{CobsAccumulator, FeedResult};
    use serde::{Deserialize, Serialize};
    const FREQ: u32 = 48_000_000;

    const COMMANDS: [Command; 4] = [
        Command::SetLed(LedStatus::On),
        Command::SetLed(LedStatus::Off),
        Command::SetLed(LedStatus::Blinking(300)),
        Command::SetLed(LedStatus::Blinking(100)),
    ];

    #[monotonic(binds = SysTick, default = true)]
    type SysMono = DwtSystick<FREQ>;

    #[derive(Clone, Copy, Serialize, Deserialize, Format)]
    pub enum Command {
        SetLed(LedStatus),
    }

    #[derive(Clone, Copy, Serialize, Deserialize, Format)]
    pub enum LedStatus {
        On,
        Off,
        Blinking(u16),
    }

    #[shared]
    struct Shared {
        #[lock_free]
        handle: Option<update_led::SpawnHandle>,
        #[lock_free]
        status: LedStatus,
        #[lock_free]
        rx: Transfer<Stream5<DMA2>, Rx<USART1>, PeripheralToMemory, &'static mut [u8; BUF_SIZE], 4>,
        #[lock_free]
        tx: Transfer<Stream7<DMA2>, Tx<USART1>, MemoryToPeripheral, &'static mut [u8; BUF_SIZE], 4>,
    }

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
    }

    #[init(local = [rx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE], tx_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();
        let mono = DwtSystick::new(&mut ctx.core.DCB, ctx.core.DWT, ctx.core.SYST, 48_000_000);

        let gpioa = ctx.device.GPIOA.split();
        let tx_pin = gpioa.pa9.into_alternate();
        let rx_pin = gpioa.pa10.into_alternate();
        let serial_config = Config {
            baudrate: 115_200.bps(),
            dma: stm32f4xx_hal::serial::config::DmaConfig::TxRx,
            ..Config::default()
        };
        let mut serial =
            Serial::new(ctx.device.USART1, (tx_pin, rx_pin), serial_config, clocks).unwrap();
        serial.listen(Event::Idle);

        let (serial_tx, serial_rx) = serial.split();
        let streams = StreamsTuple::new(ctx.device.DMA2);
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true);
        let mut rx = Transfer::init_peripheral_to_memory(
            streams.5,
            serial_rx,
            ctx.local.rx_buf,
            None,
            dma_config,
        );
        let tx = Transfer::init_memory_to_peripheral(
            streams.7,
            serial_tx,
            ctx.local.tx_buf,
            None,
            dma_config,
        );
        rx.start(|_| ());
        send_command::spawn(COMMANDS[0]).ok();
        (
            Shared {
                handle: None,
                rx,
                tx,
                status: LedStatus::Off,
            },
            Local {
                led: gpioa.pa5.into_push_pull_output(),
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    // Triggers on RX DMA transfer complete
    #[task(binds=DMA2_STREAM5, shared = [rx], priority = 2)]
    fn on_rx_dma(ctx: on_rx_dma::Context) {
        let rx = ctx.shared.rx;
        let data = unsafe {
            rx.next_transfer_with(|buf, _| {
                let data = *buf;
                (buf, data)
            })
            .unwrap()
        };
        parser::spawn(Vec::from_slice(&data).unwrap()).ok();
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
        parser::spawn(Vec::from_slice(data).unwrap()).ok();
    }

    #[task(local = [cobs_buf: CobsAccumulator<16> = CobsAccumulator::new()], shared = [status, handle], priority = 1, capacity = 2)]
    fn parser(ctx: parser::Context, data: Vec<u8, BUF_SIZE>) {
        match ctx.local.cobs_buf.feed::<Command>(data.as_slice()) {
            FeedResult::Success { data: command, .. } => {
                defmt::info!("RX: {:?}", command);
                match command {
                    Command::SetLed(status) => {
                        *ctx.shared.status = status;
                        if let Some(handle) = ctx.shared.handle.take() {
                            handle.cancel().ok();
                        }
                        update_led::spawn().ok();
                    }
                }
            }
            _ => (),
        };
    }

    #[task(local = [led], shared = [status, handle], priority = 1)]
    fn update_led(ctx: update_led::Context) {
        let led = ctx.local.led;
        match ctx.shared.status {
            LedStatus::On => led.set_high(),
            LedStatus::Off => led.set_low(),
            LedStatus::Blinking(ms) => {
                led.toggle();
                *ctx.shared.handle = update_led::spawn_after(Milliseconds(*ms as u32)).ok();
            }
        }
    }

    #[task(shared = [tx])]
    fn send_command(ctx: send_command::Context, command: Command) {
        defmt::info!("TX: {:?}", command);
        let tx = ctx.shared.tx;
        unsafe {
            let _ = tx.next_transfer_with(|buf, _| {
                postcard::to_slice_cobs(&command, buf).ok();
                (buf, ())
            });
        }
    }

    // Triggers on TX DMA transfer complete
    #[task(binds=DMA2_STREAM7, local = [cmd: usize = 0], shared = [tx])]
    fn on_tx_dma(ctx: on_tx_dma::Context) {
        ctx.shared.tx.clear_transfer_complete_interrupt();
        let cmd = ctx.local.cmd;
        *cmd = (*cmd + 1) % COMMANDS.len();
        send_command::spawn_after(Milliseconds(2_000_u32), COMMANDS[*cmd]).ok();
    }
}
