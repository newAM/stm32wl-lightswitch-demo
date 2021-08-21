# STM32WL Lightswitch Demo

This is a demo project for the [stm32wl-hal].

**Warning** This is going to be out of date with the latest HAL because it is
still in early development; please file an issue if you would like this demo
updated.

This runs on two [seeed LoRa-E5 development kits].

What does it do?  Well, it is a real fancy lightswitch.
Press D1 on the board flashed with the "client" firmware, it will toggle its own
LED, then the board with the "server" firmware will toggle its LED.

## Requirements

1. [rustup](https://rustup.rs/)
2. Two [seeed LoRa-E5 development kits] with probes compatible with [probe-run].
3. [probe-run] with patches for the STM32WL
   `cargo install --git https://github.com/newAM/probe-run.git`
4. `arm-none-eabi-gcc` to assemble the lightening-fast assembly
   P256 implementation.

## Usage

Run the `server` package and `client` package on two separate targets at the
same time using `probe-run`.

```console
$ probe-run --list-probes
The following devices were found:
[0]: STLink V2 (VID: 0483, PID: 3748, Serial: 152A061B2315303030303032, StLink)
[1]: STLink V2 (VID: 0483, PID: 3748, Serial: 0712041B2115303030303032, StLink)
$ cargo run -p server -- --probe 152A061B2315303030303032
$ cargo run -p server -- --probe 0712041B2115303030303032
```

If you want to be fancy, I use tmux to launch these at the same time in one
terminal session.

```bash
tmux new-session "cargo run -p server -- --probe 152A061B2315303030303032" \; split-window "cargo run -p client -- --probe 0712041B2115303030303032" \; setw remain-on-exit on \;
```

## Features demonstrated

* AES encryption/decryption
* ECDSA signing/verification (without PKA because the hardware PKA is SO SLOW)
* Random number generation
* GFSK transmission/reception
* GPIOs
* ADC sampling

## Latency

End-to-end latency is about 75 ms; majority of this is ECDSA processing times.

1. 29.6 ms client wakeup from sleep, TX nonce request
2. 5.2 ms server RX nonce request, TX nonce reply
3. 14.5 ms client RX nonce reply, SHA256 hashing, ECDSA signing, TX data
4. 25.3 ms server RX data, SHA256 hashing, ECDSA verification

## Security

The usual warnings apply, assume this is insecure and full of bugs.

I know this is not secure because there is no timeout on the server
nonce, and this code is therefore vulnerable to a delayed-replay attack.

Also the private key is committed into this repo, that is a bad idea.

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
[seeed LoRa-E5 development kits]: https://www.seeedstudio.com/LoRa-E5-Dev-Kit-p-4868.html
[probe-run]: https://github.com/knurling-rs/probe-run
[rustup]: https://rustup.rs/
