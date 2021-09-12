#![no_std]
#![no_main]

use bsp::{
    led::{Led, Red},
    pb::{Pb3, PushButton},
    RfSwitch,
};
use core::{
    convert::{TryFrom, TryInto},
    mem::size_of,
};
use defmt::unwrap;
use defmt_rtt as _; // global logger
use hal::{
    adc::{self, Adc},
    aes::Aes,
    chrono::{NaiveDate, NaiveDateTime},
    cortex_m::{delay::Delay, peripheral::syst::SystClkSource},
    dma::{AllDma, Dma1Ch1, Dma1Ch2},
    embedded_hal::digital::v2::ToggleableOutputPin,
    embedded_hal::timer::CountDown,
    gpio::{pins, Exti, ExtiTrg, Output, PortB, PortC},
    info,
    lptim::{self, LpTim, LpTim1},
    rcc,
    rng::{self, Rng},
    rtc::{self, Rtc},
    subghz::{Irq, SleepCfg, Startup, StatusMode, SubGhz},
    util::reset_cycle_count,
};
use nucleo_wl55jc_bsp::{
    self as bsp,
    hal::{self, pac},
};
use panic_probe as _; // panic handler
use shared::{
    setup_radio_with_payload_len, BASE_PACKET_PARAMS, IV_AND_TAG_LEN, PRIV_KEY, TIMEOUT_100_MILLIS,
};

const SLEEP_CFG: SleepCfg = SleepCfg::new()
    .set_startup(Startup::Cold)
    .set_rtc_wakeup_en(false);

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

// radio task written outside of RTIC to prevent needless indentation
fn locked_radio(
    sg: &mut SubGhz<Dma1Ch1, Dma1Ch2>,
    rfs: &mut RfSwitch,
    rtc: &mut Rtc,
    aes: &mut Aes,
    lptim1: &mut LpTim1,
    time_sync_nonce: &mut u32,
    time_sync_requested: &mut bool,
    buf: &mut [u32; 64],
) {
    let (status, irq_status) = unwrap!(sg.irq_status());

    if irq_status & Irq::TxDone.mask() != 0 {
        defmt::info!("TxDone {}", status);
        defmt::assert_eq!(status.mode(), Ok(StatusMode::StandbyHse));
        unwrap!(sg.clear_irq_status(Irq::TxDone.mask()));

        // TxDone indicates either our time synchronization request was sent
        // in which case we should enter RX mode to wait for a reply,
        // or the "Hello, World!" was sent and we should put the radio to sleep
        // while we wait for the next button IRQ
        if *time_sync_requested {
            rfs.set_rx();
            unwrap!(sg.set_rx(TIMEOUT_100_MILLIS));
            unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS.set_payload_len(IV_AND_TAG_LEN as u8)));
            *time_sync_requested = false;
        } else {
            // normally you would also enter a low-power mode here
            unwrap!(unsafe { sg.set_sleep(SLEEP_CFG) });
        }
    } else if irq_status & Irq::RxDone.mask() != 0 {
        let (_status, len, ptr) = unwrap!(sg.rx_buffer_status());
        defmt::info!("RxDone len={} ptr={} {}", len, ptr, status);
        unwrap!(sg.read_buffer(
            ptr,
            &mut bytemuck::bytes_of_mut::<[u32; 64]>(buf)[..usize::from(len)]
        ));
        unwrap!(sg.clear_irq_status(Irq::RxDone.mask()));

        // 28-bytes -> time sync reply
        if len == (IV_AND_TAG_LEN as u8) {
            let iv: [u32; 3] = [info::uid64_devnum(), *time_sync_nonce, buf[0]];

            let (aad, remainder): (&mut [u32], &mut [u32]) = buf.split_at_mut(1);
            let (data, remainder): (&mut [u32], &mut [u32]) = remainder.split_at_mut(2);
            let rx_tag: &mut [u32] = &mut remainder[..4];

            defmt::trace!("iv={:08X}", iv);
            defmt::trace!("aad={:08X}", aad);
            defmt::trace!("data={:08X}", data);

            let mut tag: [u32; 4] = [0; 4];
            unwrap!(aes.decrypt_gcm_inplace_u32(&PRIV_KEY, &iv, aad, data, &mut tag));

            defmt::trace!("data={:08X}", data);
            defmt::trace!("tag={:08X}", tag);

            if tag != rx_tag {
                defmt::warn!("time sync message is not authentic");

                // try again in another 2s
                lptim1.start(u16::MAX);
            } else {
                let timestamp_millis: i64 = i64::from(data[0]) << 32 | i64::from(data[1]);
                let secs: i64 = timestamp_millis / 1000;
                let nsec: u32 = unwrap!(u32::try_from(timestamp_millis % 1000).ok()) * 1_000_000;
                let date_time: NaiveDateTime =
                    unwrap!(NaiveDateTime::from_timestamp_opt(secs, nsec));
                rtc.set_date_time(date_time);
                defmt::info!("set date time to {}", defmt::Display2Format(&date_time));

                unwrap!(unsafe { sg.set_sleep(SLEEP_CFG) });
            }
        } else {
            defmt::warn!("Message with unknown length ignored: {}", len);
        }
    } else if irq_status & Irq::Timeout.mask() != 0 {
        unwrap!(sg.clear_irq_status(Irq::Timeout.mask()));
        defmt::error!(
            "server did not respond to time sync request in {}",
            TIMEOUT_100_MILLIS.as_duration()
        );
        // clear nonce
        *time_sync_nonce = 0;
        // restart timer
        lptim1.start(u16::MAX);
    } else if irq_status & Irq::Err.mask() != 0 {
        defmt::warn!("Packet error {}", sg.fsk_packet_status());
        unwrap!(sg.clear_irq_status(Irq::Err.mask()));
    } else {
        defmt::error!("Unhandled IRQ: {:#X} {}", irq_status, status);
        unwrap!(sg.clear_irq_status(irq_status));
    }
}

#[rtic::app(device = stm32wl::stm32wle5)]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        adc: Adc,
        delay: Delay,
        sg: SubGhz<Dma1Ch1, Dma1Ch2>,
        rfs: RfSwitch,
        led: Red,
        rng: Rng,
        aes: Aes,
        lptim1: LpTim1,
        rtc: Rtc,
        time_sync_nonce: u32,
        time_sync_requested: bool,
    }

    #[local]
    struct Local {}

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
                &hal::cortex_m::interrupt::CriticalSection::new(),
            )
        };
        let mut cp: pac::CorePeripherals = ctx.core;

        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();
        reset_cycle_count(&mut cp.DWT);

        // start enabling the LSE clock before we need it
        unsafe { rcc::pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
        dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
        dp.RCC
            .bdcr
            .modify(|_, w| w.lseon().on().lsesysen().enabled());

        let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut b3: Output<pins::B3> = Output::default(gpiob.b3);
        b3.toggle().unwrap();
        let d5: Red = Red::new(gpiob.b11);
        let _ = Pb3::new(gpioc.c6);
        <Pb3 as PushButton>::Pin::setup_exti_c1(&mut dp.EXTI, &mut dp.SYSCFG, ExtiTrg::Falling);

        let adc: Adc = Adc::new(dp.ADC, adc::Clk::RccSysClk, &mut dp.RCC);

        let aes: Aes = Aes::new(dp.AES, &mut dp.RCC);
        let rng: Rng = Rng::new(dp.RNG, rng::Clk::MSI, &mut dp.RCC);

        let delay: Delay = Delay::new(cp.SYST, rcc::cpu1_systick_hz(&dp.RCC, SystClkSource::Core));
        let rfs: RfSwitch = RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5);

        let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
        let sg: SubGhz<Dma1Ch1, Dma1Ch2> =
            SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d1.c2, &mut dp.RCC);

        // wait for LSE to be ready
        while dp.RCC.bdcr.read().lserdy().is_not_ready() {}

        let rtc: Rtc = Rtc::new(dp.RTC, rtc::Clk::Lse, &mut dp.PWR, &mut dp.RCC);

        // start 2s timer
        let mut lptim1: LpTim1 = LpTim1::new(
            dp.LPTIM1,
            lptim::Clk::Lse,
            lptim::Prescaler::Div1,
            &mut dp.RCC,
        );
        lptim1.set_ier(lptim::irq::ARRM);
        lptim1.start(u16::MAX);

        (
            Shared {
                adc,
                delay,
                sg,
                rfs,
                led: d5,
                lptim1,
                rng,
                rtc,
                aes,
                time_sync_nonce: 0,
                time_sync_requested: true,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = LPTIM1, shared = [lptim1, rng, sg, rfs, time_sync_nonce, time_sync_requested])]
    fn timer(ctx: timer::Context) {
        let isr: u32 = LpTim1::isr();
        // UE is the only interrupt enabled, sanity check
        defmt::debug_assert_ne!(isr & lptim::irq::UE, 0);
        defmt::debug!("LpTim1::isr()=0x{:08X}", isr);

        (
            ctx.shared.lptim1,
            ctx.shared.rng,
            ctx.shared.sg,
            ctx.shared.rfs,
            ctx.shared.time_sync_nonce,
            ctx.shared.time_sync_requested,
        )
            .lock(
                |lptim1, rng, sg, rfs, time_sync_nonce, time_sync_requested| {
                    // safety:
                    // we are following the erratum
                    // 1. interrupt is cleared in the associated ISR
                    // 2. only the interrupt that is set is cleared
                    unsafe { lptim1.set_icr(isr) };

                    // send request for time to set RTC
                    match rng.try_u32() {
                        Ok(rand_u32) => {
                            *time_sync_nonce = rand_u32;
                            let devnum: u32 = info::uid64_devnum();
                            defmt::info!(
                                "requesting time from server with devnum 0x{:08X} and nonce 0x{:08X}",
                                devnum, rand_u32
                            );
                            let pkt: [u32; 2] = [devnum, rand_u32];
                            unwrap!(setup_radio_with_payload_len(sg, 8));
                            unwrap!(sg.write_buffer(0, &bytemuck::bytes_of::<[u32; 2]>(&pkt)));
                            rfs.set_tx_lp();
                            unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
                            *time_sync_requested = true;
                        }
                        Err(e) => {
                            defmt::error!("failed to generate entropy: {}", e);

                            // try again in another 2s
                            lptim1.start(u16::MAX);
                        }
                    }
                },
            );
    }

    #[task(binds = EXTI9_5, shared = [adc, delay, led, rtc], local = [prev: i64 = 0])]
    fn pb3(mut ctx: pb3::Context) {
        let prev: &mut i64 = ctx.local.prev;

        // clear IRQ
        <Pb3 as PushButton>::Pin::clear_exti();

        // check RTC is setup and we have not exhausted the rate limit
        ctx.shared.rtc.lock(|rtc| match rtc.date_time() {
            Some(date_time) => {
                let min_date: NaiveDate = NaiveDate::from_ymd(2021, 09, 11);
                let min_date_time: NaiveDateTime = min_date.and_hms(0, 0, 0);
                if date_time < min_date_time {
                    defmt::warn!(
                        "ignoring PB3 IRQ, RTC date is in the past {}",
                        defmt::Display2Format(&date_time)
                    );
                    return;
                }

                let timestamp: i64 = date_time.timestamp_millis();
                if prev.saturating_add(300) > timestamp {
                    defmt::warn!("ignoring PB3 IRQ, rate limited to 300 ms");
                    return;
                } else {
                    *prev = timestamp;
                }
            }
            None => {
                defmt::warn!("ignoring PB3 IRQ, RTC is not setup");
                return;
            }
        });

        ctx.shared.led.lock(|led| led.toggle());

        // start enabling the ADC to sample vbat
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

    #[task(
        binds = ADC,
        shared = [adc, aes, sg, rfs, rng, rtc],
        local = [buf: [u32; 64] = [0; 64]]
    )]
    fn vbat(mut ctx: vbat::Context) {
        let buf: &mut [u32; 64] = ctx.local.buf;

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

            (
                ctx.shared.adc,
                ctx.shared.aes,
                ctx.shared.sg,
                ctx.shared.rfs,
                ctx.shared.rng,
                ctx.shared.rtc,
            ).lock(|adc, aes, sg, rfs, rng, rtc| {
                // clear IRQ
                adc.set_isr(adc::irq::EOC);
                // read sample
                let vbat: u16 = adc.data();
                // start the ADC disable sequence
                adc.start_disable();

                // split buffer into parts
                let (iv, remainder): (&mut [u32], &mut [u32]) = buf.split_at_mut(3);
                let iv: &mut [u32; 3] = unwrap!(iv.try_into().ok());
                let (tag, remainder): (&mut [u32], &mut [u32]) = remainder.split_at_mut(4);
                let tag: &mut [u32; 4] = unwrap!(tag.try_into().ok());
                let remainder: &mut [u32; 57] = unwrap!(remainder.try_into().ok());
                let remainder: &mut [u8] = unsafe { core::mem::transmute::<&mut [u32; 57], &mut [u8; 57 * 4]>(remainder) };

                // construct the message
                let end_of_buf: usize =
                    match msg_ser(remainder, vbat, "Hello, World!") {
                        Err(_) => {
                            defmt::error!("failed to serialize message");
                            return;
                        }
                        Ok(end_of_buf) => end_of_buf,
                    };

                // fill IV
                iv[0] = unwrap!(rng.try_u32());
                let millis: i64 = unwrap!(rtc.date_time()).timestamp_millis();
                iv[1] = (millis >> 32) as u32;
                iv[2] = millis as u32;

                unwrap!(aes.encrypt_gcm_inplace(
                    &PRIV_KEY,
                    &iv,
                    &[],
                    &mut remainder[..end_of_buf],
                    tag
                ));

                let end_of_buf: usize = end_of_buf +  size_of::<[u32; 3]>() + size_of::<[u32; 4]>();

                unwrap!(setup_radio_with_payload_len(sg, end_of_buf as u8));
                unwrap!(sg.write_buffer(0, &bytemuck::bytes_of::<[u32; 64]>(buf)[..end_of_buf]));
                rfs.set_tx_lp();
                unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
            });
        } else {
            defmt::error!("Unhandled IRQ: {:#X}", isr.bits());
        }
    }

    #[task(
        binds = RADIO_IRQ_BUSY,
        shared = [sg, rfs, rtc, lptim1, aes, time_sync_nonce, time_sync_requested],
        local = [buf: [u32; 64] = [0; 64]]
    )]
    fn radio(ctx: radio::Context) {
        let buf: &mut [u32; 64] = ctx.local.buf;

        (
            ctx.shared.sg,
            ctx.shared.rfs,
            ctx.shared.rtc,
            ctx.shared.aes,
            ctx.shared.lptim1,
            ctx.shared.time_sync_nonce,
            ctx.shared.time_sync_requested,
        )
            .lock(
                |sg, rfs, rtc, aes, lptim1, time_sync_nonce, time_sync_requested| {
                    locked_radio(
                        sg,
                        rfs,
                        rtc,
                        aes,
                        lptim1,
                        time_sync_nonce,
                        time_sync_requested,
                        buf,
                    )
                },
            );
    }
}
