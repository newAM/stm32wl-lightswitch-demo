#![no_std]

use core::time::Duration;
use stm32wl_hal::subghz::{
    AddrComp, CalibrateImage, CrcType, FskBandwidth, FskBitrate, FskFdev, FskModParams,
    FskPulseShape, GenericPacketParams, HeaderType, PaConfig, PreambleDetection, RampTime, RfFreq,
    TcxoMode, TcxoTrim, Timeout, TxParams,
};

extern crate static_assertions as sa;

// I think this goes without saying; this is a terrible key
// Also do as I say not as I do: do not commit your private key to a public repo
pub const PRIV_KEY: [u32; 4] = [0; 4];

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
    .set_payload_len(255)
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
