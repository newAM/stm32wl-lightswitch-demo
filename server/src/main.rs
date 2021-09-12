#![no_std]
#![no_main]

use defmt_rtt as _; // global logger
use panic_probe as _; // panic handler

use nucleo_wl55jc_bsp::{
    self as bsp,
    hal::{self, pac},
};

// WARNING will wrap-around eventually, use this for relative timing only
defmt::timestamp!("{=u32:µs}", pac::DWT::get_cycle_count() / 48);

#[rtic::app(device = stm32wl::stm32wle5)]
mod app {
    use super::{bsp, hal, pac};
    use bsp::{
        led::{Led, Red},
        RfSwitch,
    };
    use core::{convert::TryInto, mem::size_of};
    #[allow(unused_imports)]
    use defmt::{assert, assert_eq, unwrap};
    use hal::{
        aes::Aes,
        chrono::naive::{NaiveDate, NaiveDateTime},
        dma::{AllDma, Dma1Ch1, Dma1Ch2},
        gpio::{PortB, PortC},
        rcc,
        rtc::{self, Rtc},
        subghz::{GenericPacketParams, Irq, SubGhz, Timeout},
        util::reset_cycle_count,
    };
    use shared::{setup_radio, BASE_PACKET_PARAMS, IV_AND_TAG_LEN, PRIV_KEY, TIMEOUT_100_MILLIS};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sg: SubGhz<Dma1Ch1, Dma1Ch2>,
        rfs: RfSwitch,
        aes: Aes,
        led: Red,
        rtc: Rtc,
    }

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

        let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let d5: Red = Red::new(gpiob.b11);

        let mut rfs: RfSwitch = RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5);

        let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
        let mut sg: SubGhz<Dma1Ch1, Dma1Ch2> =
            SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d1.c2, &mut dp.RCC);

        let aes: Aes = Aes::new(dp.AES, &mut dp.RCC);

        unsafe { rcc::pulse_reset_backup_domain(&mut dp.RCC, &mut dp.PWR) };
        dp.PWR.cr1.modify(|_, w| w.dbp().enabled());
        dp.RCC.bdcr.modify(|_, w| w.lseon().on());
        while dp.RCC.bdcr.read().lserdy().is_not_ready() {}

        let mut rtc: Rtc = Rtc::new(dp.RTC, rtc::Clk::Lse, &mut dp.PWR, &mut dp.RCC);

        // this is a security problem that needs to be fixed
        // the server should have the time set by an up-to-date source.
        let date: NaiveDate = NaiveDate::from_ymd(2021, 09, 11);
        let date_time: NaiveDateTime = date.and_hms(10, 47, 25);
        rtc.set_date_time(date_time);

        unwrap!(setup_radio(&mut sg));
        rfs.set_rx();
        unwrap!(sg.set_rx(Timeout::DISABLED));

        (
            Shared {},
            Local {
                sg,
                aes,
                rfs,
                led: d5,
                rtc,
            },
            init::Monotonics(),
        )
    }

    #[task(
        binds = RADIO_IRQ_BUSY,
        local = [
            sg, rfs, led, aes, rtc,
            buf: [u32; 64] = [0; 64],
            time_sync_cnt: u32 = 0,
        ]
    )]
    fn radio(ctx: radio::Context) {
        let sg = ctx.local.sg;
        let rfs = ctx.local.rfs;
        let buf = ctx.local.buf;
        let led = ctx.local.led;
        let aes = ctx.local.aes;
        let rtc = ctx.local.rtc;
        let time_sync_cnt = ctx.local.time_sync_cnt;

        let (status, irq_status) = unwrap!(sg.irq_status());

        if irq_status & Irq::TxDone.mask() != 0 {
            defmt::info!("TxDone {}", status);
            rfs.set_rx();
            unwrap!(sg.set_rx(Timeout::DISABLED));
            unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS));
            unwrap!(sg.clear_irq_status(Irq::TxDone.mask()));
        } else if irq_status & Irq::RxDone.mask() != 0 {
            let (_status, len, ptr) = unwrap!(sg.rx_buffer_status());
            defmt::info!("RxDone len={} ptr={} {}", len, ptr, status);
            unwrap!(sg.read_buffer(
                ptr,
                &mut bytemuck::bytes_of_mut::<[u32; 64]>(buf)[..usize::from(len)]
            ));
            unwrap!(sg.clear_irq_status(Irq::RxDone.mask()));

            // message type is determined by length
            if len == 8 {
                // 8 bytes => time synchronization request

                defmt::info!(
                    "time request from client with ID 0x{:08X} and nonce 0x{:08X}",
                    buf[0],
                    buf[1]
                );

                // this is a security problem that needs to be fixed
                // this needs to be non-volatile to prevent nonce reuse
                *time_sync_cnt = unwrap!(time_sync_cnt.checked_add(1));

                // provided by client:
                // * 4 bytes unique client id (assumes all devices are STM32WL)
                // * 4 bytes client nonce
                let iv: [u32; 3] = [buf[0], buf[1], *time_sync_cnt];

                // prepare packet parameters
                // done ahead of time to reduce duration of code between reading
                // time and sending packet
                const PKT_LEN: u8 = size_of::<[u32; 7]>() as u8;
                const PACKET_PARAMS: GenericPacketParams =
                    BASE_PACKET_PARAMS.set_payload_len(PKT_LEN);
                unwrap!(sg.set_packet_params(&PACKET_PARAMS));

                let millis: i64 = unwrap!(rtc.date_time()).timestamp_millis();
                buf[0] = *time_sync_cnt;
                buf[1] = (millis >> 32) as u32;
                buf[2] = millis as u32;

                const AAD_LEN: usize = 1;
                const DATA_LEN: usize = 2;
                let (aad, remainder): (&mut [u32], &mut [u32]) = buf.split_at_mut(AAD_LEN);
                let (data, tag): (&mut [u32], &mut [u32]) = remainder.split_at_mut(DATA_LEN);
                let tag: &mut [u32; 4] = unwrap!((&mut tag[..4]).try_into().ok());

                defmt::trace!("iv={:08X}", iv);
                defmt::trace!("aad={:08X}", aad);
                defmt::trace!("data={:08X}", data);

                unwrap!(aes.encrypt_gcm_inplace_u32(&PRIV_KEY, &iv, aad, data, tag));

                defmt::trace!("data={:08X}", data);
                defmt::trace!("tag={:08X}", tag);

                unwrap!(sg.write_buffer(
                    0,
                    &bytemuck::bytes_of::<[u32; 64]>(buf)[..(PKT_LEN as usize)]
                ));
                rfs.set_tx_lp();
                unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
            } else if len > (IV_AND_TAG_LEN as u8) {
                // assume packet is data if it is long enough to have a tag

                // setup for reading the next packet
                rfs.set_rx();
                unwrap!(sg.set_rx(Timeout::DISABLED));

                let (iv, remainder): (&mut [u32], &mut [u32]) = buf.split_at_mut(3);
                let iv: &mut [u32; 3] = unwrap!(iv.try_into().ok());
                let (rx_tag, remainder): (&mut [u32], &mut [u32]) = remainder.split_at_mut(4);
                let rx_tag: &mut [u32; 4] = unwrap!(rx_tag.try_into().ok());
                let remainder: &mut [u32; 57] = unwrap!(remainder.try_into().ok());
                let remainder: &mut [u8] = &mut unsafe {
                    core::mem::transmute::<&mut [u32; 57], &mut [u8; 57 * 4]>(remainder)
                }[..(len as usize) - IV_AND_TAG_LEN];

                // verify message was sent recently
                let now: i64 = unwrap!(rtc.date_time()).timestamp_millis();
                let msg_millis: i64 = i64::from(iv[1]) << 32 | i64::from(iv[2]);
                let elapsed: i64 = now - msg_millis;
                defmt::info!("client-server time Δ: {} ms", elapsed);
                if elapsed > 10_000 {
                    defmt::warn!("message is too old");
                }

                // decrypt message
                let mut tag: [u32; 4] = [0; 4];
                unwrap!(aes.decrypt_gcm_inplace(&PRIV_KEY, &iv, &[], remainder, &mut tag));

                if &tag != rx_tag {
                    defmt::warn!("data is not authentic");
                    return;
                }

                led.toggle();

                let mut decoder = minicbor::Decoder::new(remainder);

                match decoder.u16() {
                    Err(_) => {
                        defmt::warn!("failed to decode vbat");
                        return;
                    }
                    Ok(vbat) => defmt::info!("vbat: {}", vbat),
                };

                match decoder.str() {
                    Err(_) => {
                        defmt::warn!("failed to decode data");
                        return;
                    }
                    Ok(data) => defmt::info!("data: {}", data),
                };
            } else {
                defmt::warn!("Message with unknown length ignored: {}", len);
                rfs.set_rx();
                unwrap!(sg.set_rx(Timeout::DISABLED));
            }
        } else if irq_status & Irq::Err.mask() != 0 {
            defmt::warn!("Packet error {}", sg.fsk_packet_status());
            unwrap!(sg.clear_irq_status(Irq::Err.mask()));
        } else {
            defmt::error!("Unhandled IRQ: {:#X} {}", irq_status, status);
            unwrap!(sg.clear_irq_status(irq_status));
        }
    }
}
