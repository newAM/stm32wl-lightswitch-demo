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
    use core::convert::TryInto;
    #[allow(unused_imports)]
    use defmt::{assert, assert_eq, unwrap};
    use hal::{
        aes::Aes,
        dma::{AllDma, Dma1Ch1, Dma1Ch2},
        embedded_hal::digital::v2::ToggleableOutputPin,
        gpio::{pins, Output, PortB, PortC},
        rcc,
        rng::{self, Rng},
        subghz::{CfgIrq, FallbackMode, Irq, Ocp, RegMode, StandbyClk, SubGhz, Timeout},
        util::reset_cycle_count,
    };
    use sha2::{Digest, Sha256};
    use shared::{
        p256_cortex_m4::PublicKey, Msg, MsgVariant, AES_KEY, IMG_CAL, MOD_PARAMS, PACKET_PARAMS,
        PA_CONFIG, PKT_LEN, PUB_KEY, RF_FREQ, SYNC_WORD, TCXO_MODE, TIMEOUT_100_MILLIS, TX_PARAMS,
    };

    const IRQ_CFG: CfgIrq = CfgIrq::new()
        .irq_enable_all(Irq::TxDone)
        .irq_enable_all(Irq::RxDone)
        .irq_enable_all(Irq::Timeout);

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
        hasher: Sha256,
        msg: &'static mut Msg,
    }

    static mut MSG: Msg = Msg::new();

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut dp: pac::Peripherals = ctx.device;
        rcc::set_sysclk_to_msi_48megahertz(&mut dp.FLASH, &mut dp.PWR, &mut dp.RCC);
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
            SubGhz::new_with_dma(dp.SPI3, dma.d1c1, dma.d1c2, &mut dp.RCC);

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
        unwrap!(sg.set_packet_params(&PACKET_PARAMS));
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
                hasher: Sha256::new(),
                // safety: RTIC will lock this buffer
                msg: unsafe { &mut MSG },
            },
            init::Monotonics(),
        )
    }

    #[task(binds = RADIO_IRQ_BUSY, local = [sg, rfs, rng, msg, hasher, led, aes, b3])]
    fn radio(ctx: radio::Context) {
        let sg = ctx.local.sg;
        let rfs = ctx.local.rfs;
        let rng = ctx.local.rng;
        let msg = ctx.local.msg;
        let hasher = ctx.local.hasher;
        let led = ctx.local.led;
        let aes = ctx.local.aes;
        let b3 = ctx.local.b3;

        b3.toggle().unwrap();
        let (status, irq_status) = unwrap!(sg.irq_status());

        if irq_status & Irq::TxDone.mask() != 0 {
            defmt::trace!("TxDone {}", status);
            rfs.set_rx();
            unwrap!(sg.set_rx(Timeout::DISABLED));
            unwrap!(sg.clear_irq_status(Irq::TxDone.mask()));
        } else if irq_status & Irq::RxDone.mask() != 0 {
            defmt::trace!("RxDone {}", status);
            let (_status, len, ptr) = unwrap!(sg.rx_buffer_status());
            assert_eq!(len, PKT_LEN);
            unwrap!(sg.clear_irq_status(Irq::RxDone.mask()));
            unwrap!(sg.read_buffer(ptr, msg.as_mut_buf()));

            match msg.variant() {
                x if x == MsgVariant::ReqNonce as u8 => {
                    msg.set_variant(MsgVariant::ServNonce);
                    unwrap!(rng.try_fill_u8(msg.as_serv_nonce().nonce_as_mut()));

                    hasher.update(msg.as_serv_nonce().nonce());
                    unwrap!(sg.write_buffer(0, msg.as_buf()));
                    rfs.set_tx_lp();
                    unwrap!(sg.set_tx(TIMEOUT_100_MILLIS));
                }
                x if x == MsgVariant::Data as u8 => {
                    rfs.set_rx();
                    unwrap!(sg.set_rx(Timeout::DISABLED));

                    for chunck in msg.as_encrypt().chunks_mut(4) {
                        unwrap!(aes.decrypt_ecb_inplace(&AES_KEY, chunck.try_into().unwrap()));
                    }

                    hasher.update(msg.as_data_hashable());

                    let pub_key: PublicKey = unwrap!(PublicKey::from_untagged_bytes(&PUB_KEY).ok());

                    if pub_key
                        .verify_prehashed(hasher.finalize_reset().as_ref(), &msg.as_data().sig())
                    {
                        led.toggle();
                        defmt::info!("data: {:?}", msg.as_data().data());
                        defmt::info!("vbat: {:#X}", msg.as_data().vbat());
                    } else {
                        defmt::error!("Verify failed");
                    }
                }
                x => defmt::panic!("Message not handled: {:#X}", x),
            }
        } else {
            defmt::error!("Unhandled IRQ: {:#X} {}", irq_status, status);
            unwrap!(sg.clear_irq_status(irq_status));
        }
    }
}
