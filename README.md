# STM32WL Lightswitch Demo

This is a demo project for the [stm32wl-hal].

This runs on two [NUCLEO-WL55JC2] boards.

What does it do?  Well, it is a real fancy lightswitch.
Press B3 on the board flashed with the "client" firmware, it will toggle its own
LED, then the board with the "server" firmware will toggle its LED after
authenticating the request.

## Requirements

1. [rustup](https://rustup.rs/)
2. Two [NUCLEO-WL55JC2]
3. [probe-rs]

## Usage

Run the `server` package and `client` package on two separate targets at the
same time using `probe-rs`.

```console
$ probe-rs list
The following debug probes were found:
[0]: STLink V3 (VID: 0483, PID: 374e, Serial: 001600345553500A20393256, StLink)
[1]: STLink V3 (VID: 0483, PID: 374e, Serial: 001D00145553500A20393256, StLink)
$ DEFMT_LOG=trace cargo run -p server -- --probe 0483:374e:001600345553500A20393256
$ DEFMT_LOG=trace cargo run -p client -- --probe 0483:374e:001D00145553500A20393256
```

If you want to be fancy, I use tmux to launch these at the same time in one
terminal session.

```bash
tmux new-session "DEFMT_LOG=trace cargo run -p server -- --probe 0483:374e:001600345553500A20393256" \; split-window "DEFMT_LOG=trace cargo run -p client -- --probe 0483:374e:001D00145553500A20393256" \; setw remain-on-exit on \;
```

## Features demonstrated

* ADC sampling
* AES GCM encryption + decryption
* GFSK TX + RX
* GPIOs
* Random number generation
* RTC
* Timers

## Performance

End-to-end latency is about 31.3 ms.

## Security

The usual warnings apply, assume the code is full of security bugs.

The code is for demonstration purposes only and does not represent best
security practices.
Implementing a custom protocol is typically a bad idea when security is a
requirement.

The code **is not** secure as-is:

1. The server time is not accurate, leading to nonce reuse.
2. The time-synchronization nonce is reset when the server is reset,
   leading to nonce reuse.
3. The private key is comitted to a public repository.

A better nonce would be a non-volatile value stored in flash, but I do not want
to wear out your flash for a demonstration.

```rs
/// Non-volatile u64 stored in the last page of flash
///
/// * Service lifetime is 30 years (flash data detention)
/// * Page endurance is 10k cycles
/// * Page size is 2048 bytes
///
/// service_days = 30 * 365 = 10950
/// u64_writes = 10k * (2048 / 8) = 2_560_000
/// writes_per_day = u64_writes / service_days = 233.79
///
/// You can use more pages or an external EEPROM if you need more writes/day.
pub struct NonVolatileU64 {
    _priv: (),
}

impl NonVolatileU64 {
    const U64_PER_PAGE: usize = Page::SIZE / size_of::<u64>();
    const U64_PER_PAGE_ISIZE: isize = Self::U64_PER_PAGE as isize;
    const PAGE: Page = unsafe { Page::from_index_unchecked(127) };
    const FIRST: *const u64 = Self::PAGE.addr() as *const u64;
    const LAST: *const u64 = (Self::PAGE.addr() + Page::SIZE - size_of::<u64>()) as *const u64;

    #[inline]
    pub fn fetch_increment(flash: &mut pac::FLASH) -> Result<u64, flash::Error> {
        if let Some((val, offset)) = Self::fetch() {
            let mut flash: Flash = Flash::unlock(flash);

            let next: *mut u64 = if offset == 0 {
                unsafe { flash.page_erase(Self::PAGE)? };
                Self::FIRST as *mut u64
            } else {
                unsafe { Self::LAST.offset(-offset + 1) as *mut u64 }
            };

            // in theory this should be a checked add, panicing at overflow
            // but the flash page will die before u64 wraps
            let next_val: u64 = val.wrapping_add(1);

            unsafe { flash.standard_program(&next_val, next)? };

            Ok(next_val)
        } else {
            let mut flash: Flash = Flash::unlock(flash);
            unsafe {
                flash.page_erase(Self::PAGE)?;
                flash.standard_program(&0, Self::FIRST as *mut u64)?
            };
            Ok(0)
        }
    }

    fn fetch() -> Option<(u64, isize)> {
        for offset in 0..Self::U64_PER_PAGE_ISIZE {
            let ptr: *const u64 = unsafe { Self::LAST.offset(-offset) };
            let value: u64 = unsafe { ptr.read_volatile() };
            if value != u64::MAX {
                return Some((value, offset));
            }
        }
        None
    }
}
```

## Limitations

Notes on what needs to be improved to extend this for a real-world application:

1. Security, see above.
2. Error handling! Currently everything panics upon unexpected errors.
   Retry loops should be used to handle the unexpected.
3. Adding node addresses to serve multiple clients.
4. Entry/exit into a low power state when the client is idle.
5. Saving panic messages to non-volatile memory,
   and a mechanism to retrieve the panic messages.

[stm32wl-hal]: https://github.com/newAM/stm32wl-hal
[NUCLEO-WL55JC2]: https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html#sample-buy
[probe-rs]: https://github.com/probe-rs/probe-rs
[rustup]: https://rustup.rs/
