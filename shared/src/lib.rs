#![no_std]

use core::{convert::TryInto, str::from_utf8, time::Duration};
use stm32wl_hal::subghz::{
    AddrComp, CalibrateImage, CrcType, FskBandwidth, FskBitrate, FskFdev, FskModParams,
    FskPulseShape, GenericPacketParams, HeaderType, PaConfig, PaSel, PreambleDetection, RampTime,
    RfFreq, TcxoMode, TcxoTrim, Timeout, TxParams,
};

pub use p256_cortex_m4;

extern crate static_assertions as sa;

// I think this goes without saying; this is a terrible key
pub const AES_KEY: [u32; 4] = [0; 4];

// do as I say not as I do, do not commit your private key to a public repo
pub const PRIV_KEY: [u8; 32] = [
    0x51, 0x9b, 0x42, 0x3d, 0x71, 0x5f, 0x8b, 0x58, 0x1f, 0x4f, 0xa8, 0xee, 0x59, 0xf4, 0x77, 0x1a,
    0x5b, 0x44, 0xc8, 0x13, 0x0b, 0x4e, 0x3e, 0xac, 0xca, 0x54, 0xa5, 0x6d, 0xda, 0x72, 0xb4, 0x64,
];

pub const PUB_KEY: [u8; 64] = [
    0x1c, 0xcb, 0xe9, 0x1c, 0x07, 0x5f, 0xc7, 0xf4, 0xf0, 0x33, 0xbf, 0xa2, 0x48, 0xdb, 0x8f, 0xcc,
    0xd3, 0x56, 0x5d, 0xe9, 0x4b, 0xbf, 0xb1, 0x2f, 0x3c, 0x59, 0xff, 0x46, 0xc2, 0x71, 0xbf, 0x83,
    0xce, 0x40, 0x14, 0xc6, 0x88, 0x11, 0xf9, 0xa2, 0x1a, 0x1f, 0xdb, 0x2c, 0x0e, 0x61, 0x13, 0xe0,
    0x6d, 0xb7, 0xca, 0x93, 0xb7, 0x40, 0x4e, 0x78, 0xdc, 0x7c, 0xcd, 0x5c, 0xa8, 0x9a, 0x4c, 0xa9,
];

/// Fixed packet length
pub const PKT_LEN: u8 = 132;

// There is probably a better way to do this in rust.
// I still think in C a little too much.
#[repr(C, align(4))]
pub union Msg {
    buf: [u8; PKT_LEN as usize],
    bufu32: [u32; (PKT_LEN / 4) as usize],
    req_nonce: MsgReqNonce,
    serv_nonce: MsgServNonce,
    data: MsgData,
}

impl Msg {
    pub const fn new() -> Msg {
        Msg {
            buf: [0; (PKT_LEN as usize)],
        }
    }

    pub fn variant(&self) -> u8 {
        unsafe { self.buf[0] }
    }

    pub fn as_buf(&self) -> &[u8] {
        unsafe { &self.buf }
    }

    pub fn as_mut_buf(&mut self) -> &mut [u8] {
        unsafe { &mut self.buf }
    }

    pub fn set_variant(&mut self, variant: MsgVariant) {
        unsafe { self.buf[0] = variant as u8 }
    }

    pub fn as_serv_nonce(&mut self) -> &mut MsgServNonce {
        unsafe { &mut self.serv_nonce }
    }

    pub fn as_data(&mut self) -> &mut MsgData {
        unsafe { &mut self.data }
    }

    pub fn as_data_hashable(&self) -> &[u8; 68] {
        // hash everything except the signature
        unsafe { &self.buf[..((PKT_LEN as usize) - 64)] }
            .try_into()
            .unwrap()
    }

    pub fn as_encrypt(&mut self) -> &mut [u32; 32] {
        // encrypt everything except the message variant
        unsafe { &mut self.bufu32[1..] }.try_into().unwrap()
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum MsgVariant {
    /// Request server nonce
    ReqNonce = 0x00,
    /// Server noonce response
    ServNonce = 0x01,
    /// Message with useable data
    Data = 0x02,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct MsgReqNonce {
    variant: u8,
    rsvd: [u8; 3],
    undefined: [u8; 128],
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct MsgServNonce {
    variant: u8,
    rsvd: [u8; 3],
    nonce: [u8; 32],
    undefined: [u8; 96],
}

impl MsgServNonce {
    pub fn nonce_as_mut(&mut self) -> &mut [u8; 32] {
        &mut self.nonce
    }

    pub fn nonce(&self) -> &[u8; 32] {
        &self.nonce
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct MsgData {
    variant: u8,
    rsvd: [u8; 3],
    vbat: u16,
    data: [u8; 62],
    sig: [u8; 64],
}

impl MsgData {
    pub fn set_vbat(&mut self, vbat: u16) {
        self.vbat = vbat
    }

    pub fn set_data(&mut self, data: &str) {
        self.data.iter_mut().for_each(|x| *x = 0);
        self.data[0..data.len()].copy_from_slice(data.as_bytes());
    }

    pub fn sig(&self) -> p256_cortex_m4::Signature {
        p256_cortex_m4::Signature::from_untagged_bytes(&self.sig).unwrap()
    }

    pub fn set_sig(&mut self, signature: &p256_cortex_m4::Signature) {
        self.sig = signature.to_untagged_bytes()
    }

    pub fn vbat(&self) -> u16 {
        self.vbat
    }

    pub fn data(&self) -> Option<&str> {
        let end: usize = self
            .data
            .iter()
            .position(|&ch| ch == 0)
            .unwrap_or(self.data.len());
        from_utf8(self.data[..end].as_ref()).ok()
    }
}

sa::assert_eq_size!(
    [u8; PKT_LEN as usize],
    MsgReqNonce,
    MsgServNonce,
    MsgData,
    Msg
);

pub const TIMEOUT_100_MILLIS: Timeout = Timeout::from_duration_sat(Duration::from_millis(100));

pub const PREAMBLE_LEN: u16 = 256;

pub const RF_FREQ: RfFreq = RfFreq::F915;
pub const IMG_CAL: CalibrateImage = CalibrateImage::ISM_902_928;

pub const SYNC_WORD: [u8; 8] = [0x79, 0x80, 0x0C, 0xC0, 0x29, 0x95, 0xF8, 0x4A];
pub const SYNC_WORD_LEN: u8 = SYNC_WORD.len() as u8;
pub const SYNC_WORD_LEN_BITS: u8 = SYNC_WORD_LEN * 8;

pub const PACKET_PARAMS: GenericPacketParams = GenericPacketParams::new()
    .set_preamble_len(PREAMBLE_LEN)
    .set_preamble_detection(PreambleDetection::Bit8)
    .set_sync_word_len(SYNC_WORD_LEN_BITS)
    .set_addr_comp(AddrComp::Disabled)
    .set_header_type(HeaderType::Fixed)
    .set_payload_len(PKT_LEN)
    .set_crc_type(CrcType::Byte2)
    .set_whitening_enable(true);

pub const MOD_PARAMS: FskModParams = FskModParams::new()
    .set_bitrate(FskBitrate::from_bps(300_000))
    .set_pulse_shape(FskPulseShape::Bt05)
    .set_bandwidth(FskBandwidth::Bw467)
    .set_fdev(FskFdev::from_hertz(50_000));

sa::const_assert!(MOD_PARAMS.is_valid(30));

// configuration for +10 dBm output power
// see table 35 "PA optimal setting and operating modes"
pub const PA_CONFIG: PaConfig = PaConfig::new()
    .set_pa_duty_cycle(0x2)
    .set_hp_max(0x2)
    .set_pa(PaSel::Hp);

pub const TCXO_MODE: TcxoMode = TcxoMode::new()
    .set_txco_trim(TcxoTrim::Volts1pt7)
    .set_timeout(Timeout::from_duration_sat(Duration::from_millis(10)));

pub const TX_PARAMS: TxParams = TxParams::new()
    .set_power(0x16)
    .set_ramp_time(RampTime::Micros40);
