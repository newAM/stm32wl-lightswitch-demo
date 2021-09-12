#![no_std]

use core::{mem::size_of, time::Duration};
use stm32wl_hal::{
    dma::{Dma1Ch1, Dma1Ch2},
    subghz::{
        self, AddrComp, CalibrateImage, CfgIrq, CrcType, FallbackMode, FskBandwidth, FskBitrate,
        FskFdev, FskModParams, FskPulseShape, GenericPacketParams, HeaderType, Irq, Ocp, PaConfig,
        PreambleDetection, RampTime, RegMode, RfFreq, StandbyClk, SubGhz, TcxoMode, TcxoTrim,
        Timeout, TxParams,
    },
};

extern crate static_assertions as sa;

// I think this goes without saying; this is a terrible key
// Also do as I say not as I do: do not commit your private key to a public repo
pub const PRIV_KEY: [u32; 4] = [0; 4];

pub const IV_AND_TAG_LEN: usize = size_of::<[u32; 3]>() + size_of::<[u32; 4]>();

pub const TIMEOUT_100_MILLIS: Timeout = Timeout::from_duration_sat(Duration::from_millis(100));

pub const PREAMBLE_LEN: u16 = 256;

pub const RF_FREQ: RfFreq = RfFreq::F433;
pub const IMG_CAL: CalibrateImage = CalibrateImage::ISM_430_440;

pub const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
pub const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
pub const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;

pub const BASE_PACKET_PARAMS: GenericPacketParams = GenericPacketParams::new()
    .set_preamble_len(PREAMBLE_LEN)
    .set_preamble_detection(PreambleDetection::Bit8)
    .set_sync_word_len(SYNC_WORD_LEN_BITS)
    .set_addr_comp(AddrComp::Disabled)
    .set_header_type(HeaderType::Variable)
    .set_payload_len(u8::MAX)
    .set_crc_type(CrcType::Byte2)
    .set_whitening_enable(true);

pub const MOD_PARAMS: FskModParams = FskModParams::new()
    .set_bitrate(FskBitrate::from_bps(300_000))
    .set_pulse_shape(FskPulseShape::Bt05)
    .set_bandwidth(FskBandwidth::Bw467)
    .set_fdev(FskFdev::from_hertz(50_000));

sa::const_assert!(MOD_PARAMS.is_valid_worst_case());

pub const PA_CONFIG: PaConfig = PaConfig::LP_10;
pub const TX_PARAMS: TxParams = TxParams::LP_10.set_ramp_time(RampTime::Micros40);

pub const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_duration_sat(Duration::from_millis(10)));

const IRQ_CFG: CfgIrq = CfgIrq::new()
    .irq_enable_all(Irq::TxDone)
    .irq_enable_all(Irq::RxDone)
    .irq_enable_all(Irq::Timeout)
    .irq_enable_all(Irq::Err);

#[inline]
pub fn setup_radio_with_payload_len(
    sg: &mut SubGhz<Dma1Ch1, Dma1Ch2>,
    len: u8,
) -> Result<(), subghz::Error> {
    unsafe { subghz::wakeup() };
    sg.set_standby(StandbyClk::Rc)?;
    sg.set_tcxo_mode(&TCXO_MODE)?;
    sg.set_standby(StandbyClk::Hse)?;
    sg.set_tx_rx_fallback_mode(FallbackMode::StandbyHse)?;
    sg.set_regulator_mode(RegMode::Ldo)?;
    sg.set_buffer_base_address(0, 0)?;
    sg.set_pa_config(&PA_CONFIG)?;
    sg.set_pa_ocp(Ocp::Max60m)?;
    sg.set_tx_params(&TX_PARAMS)?;
    sg.set_sync_word(&SYNC_WORD)?;
    sg.set_fsk_mod_params(&MOD_PARAMS)?;
    sg.set_packet_params(&BASE_PACKET_PARAMS.set_payload_len(len))?;
    sg.calibrate_image(IMG_CAL)?;
    sg.set_rf_frequency(&RF_FREQ)?;
    sg.set_irq_cfg(&IRQ_CFG)?;
    Ok(())
}

#[inline]
pub fn setup_radio(sg: &mut SubGhz<Dma1Ch1, Dma1Ch2>) -> Result<(), subghz::Error> {
    setup_radio_with_payload_len(sg, u8::MAX)
}
