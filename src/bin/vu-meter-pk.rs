// $ cargo rb vu-meter-pk
#![no_main]
#![no_std]

use f411_rtic as _; // global logger + panicking-behavior + memory layout

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, dispatchers = [USART1])]
mod app {
    use core::convert::TryInto;
    use dwt_systick_monotonic::DwtSystick;
    use rtic::time::duration::{Microseconds, Milliseconds};
    use rtic_monotonic::Instant;
    use smart_leds::{brightness, colors, SmartLedsWrite, RGB8};
    use stm32f4xx_hal::{
        gpio::{Alternate, NoPin, Pin},
        pac::SPI1,
        prelude::*,
        spi::{Spi, TransferModeNormal},
    };
    use ws2812_spi::{self, devices, Ws2812};
    const FREQ: u32 = 48_000_000;
    const DECAY: f32 = 0.005;
    const PEAK_HOLD: u32 = 800;
    const NUM_LEDS: usize = 24;
    const THRESHOLDS: [(u16, RGB8); NUM_LEDS / 2] = [
        (16_u16, colors::GREEN),
        (32, colors::GREEN),
        (64, colors::GREEN),
        (128, colors::GREEN),
        (256, colors::GREEN),
        (512, colors::GREEN),
        (1024, colors::ORANGE),
        (1536, colors::ORANGE),
        (2048, colors::ORANGE),
        (2560, colors::RED),
        (3072, colors::RED),
        (4096, colors::RED),
    ];
    type MeterSPI = Spi<SPI1, (NoPin, NoPin, Pin<Alternate<5>, 'A', 7>), TransferModeNormal>;
    type Peak = (f32, Instant<MyMono>);

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = DwtSystick<FREQ>;

    #[shared]
    struct Shared {
        env: (f32, f32),
        peak: (Option<Peak>, Option<Peak>),
    }

    #[local]
    struct Local {
        meter: Ws2812<MeterSPI, devices::Ws2812>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(FREQ.hz()).freeze();

        let mono = DwtSystick::new(
            &mut ctx.core.DCB,
            ctx.core.DWT,
            ctx.core.SYST,
            clocks.hclk().0,
        );

        let gpioa = ctx.device.GPIOA.split();
        let mosi = gpioa.pa7.into_alternate();

        let spi = Spi::new(
            ctx.device.SPI1,
            (NoPin, NoPin, mosi),
            ws2812_spi::MODE,
            stm32f4xx_hal::time::KiloHertz(3_000).into(),
            clocks,
        );
        let meter = ws2812_spi::Ws2812::new(spi);

        mock_adc::spawn().ok();
        update_leds::spawn().ok();
        (
            Shared {
                env: (0.0, 0.0),
                peak: (None, None),
            },
            Local { meter },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {}
    }

    #[task(shared = [env, peak], local = [t: u16 = 0])]
    fn mock_adc(ctx: mock_adc::Context) {
        // Mock rectified input signals
        let t = ctx.local.t;
        const L: [(u16, f32); 2] = [(0, 4096.0), (700, 2048.0)];
        const R: [(u16, f32); 2] = [(0, 4096.0), (900, 600.0)];
        let in_l = L.iter().find(|s| s.0 == *t).map(|s| s.1).unwrap_or(0.0);
        let in_r = R.iter().find(|s| s.0 == *t).map(|s| s.1).unwrap_or(0.0);
        *t = (*t + 1) % 2048;

        // Calc and update signal envelopes
        (ctx.shared.env, ctx.shared.peak).lock(|(env_l, env_r), (pk_l, pk_r)| {
            *env_l = match in_l > *env_l {
                true => {
                    pk_l.replace((in_l, monotonics::MyMono::now()));
                    in_l
                }
                false => *env_l * (1.0 - DECAY),
            };
            *env_r = match in_r > *env_r {
                true => {
                    pk_r.replace((in_r, monotonics::MyMono::now()));
                    in_r
                }
                false => *env_r * (1.0 - DECAY),
            };
        });
        mock_adc::spawn_after(Microseconds(900u32)).ok();
    }

    #[task(shared = [env, peak], local = [meter])]
    fn update_leds(mut ctx: update_leds::Context) {
        let meter = ctx.local.meter;
        let (env_l, env_r) = ctx.shared.env.lock(|e| *e);
        let (mut pk_l, mut pk_r) = ctx.shared.peak.lock(|e| *e);
        clear_peak(&mut pk_l);
        clear_peak(&mut pk_r);
        let left = bargraph(&THRESHOLDS, env_l, pk_l);
        let right = bargraph(&THRESHOLDS, env_r, pk_r);
        let pixels = left.iter().chain(right.iter().rev()).cloned();
        meter.write(brightness(pixels, 10)).ok();
        update_leds::spawn_after(Milliseconds(15u32)).ok();
    }

    fn bargraph<const N: usize>(
        thresh: &[(u16, RGB8); N],
        x: f32,
        pk: Option<(f32, Instant<MyMono>)>,
    ) -> [RGB8; N] {
        let mut pixels = [RGB8::default(); N];
        for (i, led) in pixels.iter_mut().enumerate() {
            let scaling = match thresh.iter().rposition(|t| x as u16 >= t.0) {
                Some(t) if i <= t => 1.0,
                Some(t) if i == t + 1 => {
                    (x - thresh[t].0 as f32) / (thresh[t + 1].0 - thresh[t].0) as f32
                }
                None if i == 0 => x / thresh[0].0 as f32,
                _ => 0.0,
            };
            *led = match thresh
                .iter()
                .rposition(|t| pk.map(|(level, _)| level as u16 >= t.0).unwrap_or(false))
            {
                Some(t) if t + 1 == i || (t == N - 1 && i == t) => colors::RED,
                _ => RGB8 {
                    r: (thresh[i].1.r as f32 * scaling) as u8,
                    g: (thresh[i].1.g as f32 * scaling) as u8,
                    b: (thresh[i].1.b as f32 * scaling) as u8,
                },
            }
        }
        pixels
    }

    fn clear_peak(peak: &mut Option<(f32, Instant<MyMono>)>) {
        if let Some((level, instant)) = peak.take() {
            let duration: Option<Milliseconds> = monotonics::MyMono::now()
                .checked_duration_since(&instant)
                .and_then(|d| d.try_into().ok());
            match duration {
                Some(Milliseconds(t)) if t < PEAK_HOLD => *peak = Some((level, instant)),
                _ => (),
            }
        }
    }
}
