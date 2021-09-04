#![no_std]
#![no_main]

use core::mem::size_of;
use defmt_rtt as _; // global logger
use nucleo_wl55jc_bsp::{
    self as bsp,
    hal::{self, pac},
};
use panic_probe as _; // panic handler

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", pac::DWT::get_cycle_count() / 48);

// serialize message
fn msg_ser(
    buf: &mut [u8],
    vbat: u16,
    data: &str,
) -> Result<usize, minicbor::encode::Error<minicbor::encode::write::EndOfSlice>> {
    minicbor::Encoder::new(buf).u16(vbat)?.str(data)?;
    Ok(size_of::<u16>() + data.len() + 2)
}

#[rtic::app(device = stm32wl::stm32wle5)]
mod app {
    use super::{bsp, hal, msg_ser, pac, size_of};
    use bsp::{
        led::{Led, Red},
        pb::{Pb3, PushButton},
        RfSwitch,
    };
    use core::sync::atomic::{AtomicU16, Ordering};
    use defmt::unwrap;
    use hal::{
        adc::{self, Adc},
        aes::Aes,
        cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
        dma::{AllDma, Dma1Ch1, Dma1Ch2},
        embedded_hal::digital::v2::ToggleableOutputPin,
        gpio::{pins, Exti, ExtiTrg, Output, PortB, PortC},
        rcc,
        subghz::{
            wakeup, CfgIrq, FallbackMode, Irq, Ocp, RegMode, SleepCfg, StandbyClk, Startup,
            StatusMode, SubGhz,
        },
        util::reset_cycle_count,
    };
    use shared::{
        BASE_PACKET_PARAMS, IMG_CAL, MOD_PARAMS, PA_CONFIG, PRIV_KEY, RF_FREQ, SYNC_WORD,
        TCXO_MODE, TIMEOUT_100_MILLIS, TX_PARAMS,
    };

    const SLEEP_CFG: SleepCfg = SleepCfg::new()
        .set_startup(Startup::Cold)
        .set_rtc_wakeup_en(false);

    const IRQ_CFG: CfgIrq = CfgIrq::new()
        .irq_enable_all(Irq::TxDone)
        .irq_enable_all(Irq::RxDone)
        .irq_enable_all(Irq::Timeout)
        .irq_enable_all(Irq::Err);

    static VBAT: AtomicU16 = AtomicU16::new(0);

    #[shared]
    struct Shared {
        adc: Adc,
        delay: Delay,
        sg: SubGhz<Dma1Ch1, Dma1Ch2>,
        rfs: RfSwitch,
        state: bool,
        b3: Output<pins::B3>,
        led: Red,
    }

    #[local]
    struct Local {
        aes: Aes,
        iv: [u32; 3],
        buf: [u8; 255],
    }

    static mut IV: [u32; 3] = [0; 3];
    static mut BUF: [u8; 255] = [0; 255];

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: pac::Peripherals = ctx.device;
        unsafe {
            rcc::set_sysclk_msi(
                &mut dp.FLASH,
                &mut dp.PWR,
                &mut dp.RCC,
                rcc::MsiRange::Range48M,
                // symptom of a version mismatch when using the RTIC alpha
                // see: https://github.com/rust-embedded/cortex-m/pull/350
                // replace with `ctx.cs` when cortex-m gets updated
                &cortex_m::interrupt::CriticalSection::new(),
            )
        };
        let mut cp: pac::CorePeripherals = ctx.core;

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut b3: Output<pins::B3> = Output::default(gpiob.b3);
        b3.toggle().unwrap();
        let d5: Red = Red::new(gpiob.b11);
        let _ = Pb3::new(gpioc.c6);
        <Pb3 as PushButton>::Pin::setup_exti_c1(&mut dp.EXTI, &mut dp.SYSCFG, ExtiTrg::Falling);

        // enable the HSI16 source clock
        dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}

        let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccHsi, &mut dp.RCC);

        let delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
        let rfs: RfSwitch = RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5);

        let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
        let sg: SubGhz<Dma1Ch1, Dma1Ch2> =
            SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d1.c2, &mut dp.RCC);

        let aes: Aes = Aes::new(dp.AES, &mut dp.RCC);

        (
            Shared {
                adc,
                delay,
                state: false,
                sg,
                b3,
                rfs,
                led: d5,
            },
            Local {
                aes,
                // safety: RTIC will lock these buffers
                buf: unsafe { BUF },
                iv: unsafe { IV },
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI9_5, shared = [adc, delay, sg, rfs, state, led])]
    fn pb3(mut ctx: pb3::Context) {
        defmt::warn!("Pb3: No rate limiting or debounce, weird things could occur with the ADC");

        // clear IRQ
        <Pb3 as PushButton>::Pin::clear_exti();

        ctx.shared.led.lock(|led| led.toggle());

        // send out a request for nonce
        (ctx.shared.sg, ctx.shared.rfs, ctx.shared.state).lock(|sg, rfs, state| {
            *state = true;
            unsafe { wakeup() };
            unwrap!(sg.set_standby(StandbyClk::Rc));
            unwrap!(sg.set_tcxo_mode(&TCXO_MODE));
            unwrap!(sg.set_standby(StandbyClk::Hse));
            unwrap!(sg.set_tx_rx_fallback_mode(FallbackMode::StandbyHse));
            unwrap!(sg.set_regulator_mode(RegMode::Ldo));
            unwrap!(sg.set_buffer_base_address(0, 0));
            unwrap!(sg.set_pa_config(&PA_CONFIG));
            unwrap!(sg.set_pa_ocp(Ocp::Max60m));
            unwrap!(sg.set_tx_params(&TX_PARAMS));
            unwrap!(sg.set_sync_word(&SYNC_WORD));
            unwrap!(sg.set_fsk_mod_params(&MOD_PARAMS));
            unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS.set_payload_len(0)));
            unwrap!(sg.calibrate_image(IMG_CAL));
            unwrap!(sg.set_rf_frequency(&RF_FREQ));
            unwrap!(sg.set_irq_cfg(&IRQ_CFG));
            rfs.set_tx_lp();
            unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
        });

        // start enabling the ADC to sample vbat
        // this takes less than 1ms, which is why we start the radio first
        (ctx.shared.adc, ctx.shared.delay).lock(|adc, delay| {
            // should already be disabled
            adc.disable();
            // enable the voltage regulator as soon as possible to start the
            // countdown on the regulator setup time
            adc.enable_vreg();
            // enable ADC end-of-calibration IRQ
            adc.set_ier(adc::irq::EOCAL);

            // delay the required duration
            delay.delay_us(adc::T_ADCVREG_SETUP_MICROS.into());

            defmt::info!("Starting ADC calibration");
            adc.start_calibrate();
        });
    }

    #[task(binds = ADC, shared = [adc])]
    fn vbat(mut ctx: vbat::Context) {
        let isr = Adc::isr();

        if isr.eocal().bit_is_set() {
            defmt::info!("Calibration done");
            ctx.shared.adc.lock(|adc| {
                // start enable
                let should_poll: bool = adc.start_enable();
                defmt::assert!(should_poll);
                // clear IRQ
                adc.set_isr(adc::irq::EOCAL);
                // enable ready IRQ
                adc.set_ier(adc::irq::ADRDY);
            });
        } else if isr.adrdy().bit_is_set() {
            defmt::info!("Ready");
            ctx.shared.adc.lock(|adc| {
                // select VBAT channel
                adc.start_chsel(adc::Ch::Vbat.mask());
                // enable VBAT
                adc.enable_vbat();
                // this takes fewer cycles to poll than it does to switch context
                // appx 15 CPU cycles 5 ADC cycles at 48MHz/16Mhz
                while Adc::isr().ccrdy().is_not_complete() {}
                // clear IRQs
                adc.set_isr(adc::irq::ADRDY | adc::irq::CCRDY);
                // set the sample times
                adc.set_max_sample_time();
                // start a conversion
                adc.start_conversion();
                // enable the end-of-conversion IRQ
                adc.set_ier(adc::irq::EOC);
            });
        } else if isr.eoc().bit_is_set() {
            defmt::info!("Sample done");
            ctx.shared.adc.lock(|adc| {
                // clear IRQ
                adc.set_isr(adc::irq::EOC);
                // read sample
                VBAT.store(adc.data(), Ordering::Release);
                // start the ADC disable sequence
                adc.start_disable();
            });
        } else {
            defmt::error!("Unhandled IRQ: {:#X}", isr.bits());
        }
    }

    #[task(binds = RADIO_IRQ_BUSY, shared = [sg, rfs, state, b3], local=[iv, buf, aes])]
    fn radio(ctx: radio::Context) {
        let buf = ctx.local.buf;
        let mut iv = ctx.local.iv;
        let aes = ctx.local.aes;

        (
            ctx.shared.sg,
            ctx.shared.rfs,
            ctx.shared.state,
            ctx.shared.b3,
        )
            .lock(|sg, rfs, state, b3| {
                b3.toggle().unwrap();
                let (status, irq_status) = unwrap!(sg.irq_status());

                if irq_status & Irq::TxDone.mask() != 0 {
                    defmt::info!("TxDone {}", status);
                    defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyHse));
                    unwrap!(sg.clear_irq_status(Irq::TxDone.mask()));

                    if *state {
                        rfs.set_rx();
                        unwrap!(sg.set_rx(TIMEOUT_100_MILLIS));
                        // expecting 16B packet in response
                        unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS.set_payload_len(16)));
                        *state = false;
                    } else {
                        unwrap!(unsafe { sg.set_sleep(SLEEP_CFG) });
                    }
                } else if irq_status & Irq::RxDone.mask() != 0 {
                    defmt::info!("RxDone {}", status);
                    let (_status, len, ptr) = unwrap!(sg.rx_buffer_status());
                    unwrap!(sg.clear_irq_status(Irq::RxDone.mask()));

                    // 12-bytes -> 96-bits -> nonce reply
                    if len == 12 {
                        unwrap!(sg.read_buffer(ptr, bytemuck::bytes_of_mut::<[u32; 3]>(&mut iv)));

                        let end_of_buf: usize =
                            match msg_ser(buf, VBAT.load(Ordering::Acquire), "Hello, World!") {
                                Err(_) => {
                                    defmt::error!("failed to serialize message");
                                    return;
                                }
                                Ok(end_of_buf) => end_of_buf,
                            };

                        defmt::debug!("encrypting {} byte long buffer", end_of_buf);

                        let mut tag: [u32; 4] = [0; 4];
                        unwrap!(aes.encrypt_gcm_inplace(
                            &PRIV_KEY,
                            &iv,
                            &[],
                            &mut buf[..end_of_buf],
                            &mut tag
                        ));

                        let tag_bytes: &[u8] = bytemuck::bytes_of::<[u32; 4]>(&tag);

                        // append tag to message
                        for (idx, byte) in tag_bytes.iter().enumerate() {
                            buf[end_of_buf + idx] = *byte;
                        }
                        let end_of_buf: usize = end_of_buf + size_of::<[u32; 4]>();

                        defmt::debug!("TX {} byte packet", end_of_buf);

                        unwrap!(sg.set_packet_params(
                            &BASE_PACKET_PARAMS.set_payload_len(end_of_buf as u8)
                        ));
                        unwrap!(sg.write_buffer(0, &buf[..end_of_buf]));
                        rfs.set_tx_lp();
                        unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
                    } else {
                        defmt::warn!("Message with unknown length ignored: {}", len);
                    }
                } else if irq_status & Irq::Timeout.mask() != 0 {
                    defmt::error!("Why are we timing out? {}", status);
                    unwrap!(sg.clear_irq_status(Irq::Timeout.mask()));
                } else if irq_status & Irq::Err.mask() != 0 {
                    defmt::warn!("Packet error {}", sg.fsk_packet_status());
                    unwrap!(sg.clear_irq_status(Irq::Err.mask()));
                } else {
                    defmt::error!("Unhandled IRQ: {:#X} {}", irq_status, status);
                    unwrap!(sg.clear_irq_status(irq_status));
                }
            });
    }
}
