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
    use core::mem::size_of;
    #[allow(unused_imports)]
    use defmt::{assert, assert_eq, unwrap};
    use hal::{
        aes::Aes,
        dma::{AllDma, Dma1Ch1, Dma1Ch2},
        embedded_hal::digital::v2::ToggleableOutputPin,
        gpio::{pins, Output, PortB, PortC},
        rcc,
        rng::{self, Rng},
        subghz::{
            CfgIrq, FallbackMode, GenericPacketParams, Irq, Ocp, RegMode, StandbyClk, SubGhz,
            Timeout,
        },
        util::reset_cycle_count,
    };
    use shared::{
        BASE_PACKET_PARAMS, IMG_CAL, MOD_PARAMS, PA_CONFIG, PRIV_KEY, RF_FREQ, SYNC_WORD,
        TCXO_MODE, TIMEOUT_100_MILLIS, TX_PARAMS,
    };

    const IRQ_CFG: CfgIrq = CfgIrq::new()
        .irq_enable_all(Irq::TxDone)
        .irq_enable_all(Irq::RxDone)
        .irq_enable_all(Irq::Timeout)
        .irq_enable_all(Irq::Err);

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        sg: SubGhz<Dma1Ch1, Dma1Ch2>,
        rfs: RfSwitch,
        rng: Rng,
        aes: Aes,
        led: Red,
        b3: Output<pins::B3>,
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

        // enable the HSI16 source clock
        dp.RCC.cr.modify(|_, w| w.hsion().set_bit());
        while dp.RCC.cr.read().hsirdy().is_not_ready() {}

        let gpiob: PortB = PortB::split(dp.GPIOB, &mut dp.RCC);
        let gpioc: PortC = PortC::split(dp.GPIOC, &mut dp.RCC);
        let mut b3: Output<pins::B3> = Output::default(gpiob.b3);
        b3.toggle().unwrap();
        let d5: Red = Red::new(gpiob.b11);

        let mut rfs: RfSwitch = RfSwitch::new(gpioc.c3, gpioc.c4, gpioc.c5);

        let dma: AllDma = AllDma::split(dp.DMAMUX, dp.DMA1, dp.DMA2, &mut dp.RCC);
        let mut sg: SubGhz<Dma1Ch1, Dma1Ch2> =
            SubGhz::new_with_dma(dp.SPI3, dma.d1.c1, dma.d1.c2, &mut dp.RCC);

        let rng: Rng = Rng::new(dp.RNG, rng::Clk::MSI, &mut dp.RCC);
        let aes: Aes = Aes::new(dp.AES, &mut dp.RCC);

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
        unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS));
        unwrap!(sg.calibrate_image(IMG_CAL));
        unwrap!(sg.set_rf_frequency(&RF_FREQ));
        unwrap!(sg.set_irq_cfg(&IRQ_CFG));
        rfs.set_rx();
        unwrap!(sg.set_rx(Timeout::DISABLED));

        (
            Shared {},
            Local {
                rng,
                sg,
                aes,
                rfs,
                b3,
                led: d5,
            },
            init::Monotonics(),
        )
    }

    #[task(
        binds = RADIO_IRQ_BUSY,
        local = [sg, rfs, rng, led, aes, b3, iv: [u32; 3] = [0; 3], buf: [u8; 255] = [0; 255]]
    )]
    fn radio(ctx: radio::Context) {
        let sg = ctx.local.sg;
        let rfs = ctx.local.rfs;
        let rng = ctx.local.rng;
        let buf = ctx.local.buf;
        let led = ctx.local.led;
        let aes = ctx.local.aes;
        let b3 = ctx.local.b3;
        let iv = ctx.local.iv;

        b3.toggle().unwrap();
        let (status, irq_status) = unwrap!(sg.irq_status());

        if irq_status & Irq::TxDone.mask() != 0 {
            defmt::info!("TxDone {}", status);
            rfs.set_rx();
            unwrap!(sg.set_rx(Timeout::DISABLED));
            unwrap!(sg.set_packet_params(&BASE_PACKET_PARAMS));
            unwrap!(sg.clear_irq_status(Irq::TxDone.mask()));
        } else if irq_status & Irq::RxDone.mask() != 0 {
            defmt::info!("RxDone {}", status);
            let (_status, len, ptr) = unwrap!(sg.rx_buffer_status());
            unwrap!(sg.clear_irq_status(Irq::RxDone.mask()));

            // message type is determined by length
            if len == 0 {
                // 0 bytes, nonce request
                // reply with 96-bit nonce
                unwrap!(rng.try_fill_u32(iv.as_mut()));

                const PACKET_PARAMS: GenericPacketParams =
                    BASE_PACKET_PARAMS.set_payload_len(size_of::<[u32; 3]>() as u8);

                unwrap!(sg.set_packet_params(&PACKET_PARAMS));
                unwrap!(sg.write_buffer(0, bytemuck::bytes_of::<[u32; 3]>(iv)));
                rfs.set_tx_lp();
                unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
            } else if len > 16 {
                // assume packet is data if it is long enough to have a tag
                let filled_buf: &mut [u8] = &mut buf[..(len as usize)];
                unwrap!(sg.read_buffer(ptr, filled_buf));

                // setup for reading the next packet
                rfs.set_rx();
                unwrap!(sg.set_rx(Timeout::DISABLED));

                let client_tag_idx: usize = (len - 16) as usize;
                let encrypted_buf: &mut [u8] = &mut filled_buf[..client_tag_idx];
                defmt::debug!("decrypting {} byte long buffer", encrypted_buf.len());

                let mut tag: [u32; 4] = [0; 4];
                unwrap!(aes.decrypt_gcm_inplace(&PRIV_KEY, &iv, &[], encrypted_buf, &mut tag));

                let client_tag: &[u8] = bytemuck::bytes_of::<[u32; 4]>(&tag);
                let derived_tag: &[u8] = &filled_buf[client_tag_idx..];
                if client_tag != derived_tag {
                    defmt::warn!("data is not authentic");
                    return;
                }

                led.toggle();

                let mut decoder = minicbor::Decoder::new(filled_buf);

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
