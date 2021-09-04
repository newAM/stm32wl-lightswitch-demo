# STM32WL Lightswitch Demo

This is a demo project for the [stm32wl-hal].

⚠️ This is likely out of date with the latest HAL because the HAL is still in
early development; please file an issue if you would like this demo updated.

This runs on two [NUCLEO-WL55JC2] boards.

What does it do?  Well, it is a real fancy lightswitch.
Press B3 on the board flashed with the "client" firmware, it will toggle its own
LED, then the board with the "server" firmware will toggle its LED.

## Requirements

1. [rustup](https://rustup.rs/)
2. Two [NUCLEO-WL55JC2]
3. [probe-run] with patches for the STM32WL
   `cargo install --git https://github.com/newAM/probe-run.git`

## Usage

Run the `server` package and `client` package on two separate targets at the
same time using `probe-run`.

```console
$ probe-run --list-probes
The following devices were found:
[0]: STLink V3 (VID: 0483, PID: 374e, Serial: 001600345553500A20393256, StLink)
[1]: STLink V3 (VID: 0483, PID: 374e, Serial: 001D00145553500A20393256, StLink)
$ cargo run -p server -- --probe 001600345553500A20393256
$ cargo run -p server -- --probe 001D00145553500A20393256
```

If you want to be fancy, I use tmux to launch these at the same time in one
terminal session.

```bash
tmux new-session "cargo run -p server -- --probe 001600345553500A20393256" \; split-window "cargo run -p client -- --probe 001D00145553500A20393256" \; setw remain-on-exit on \;
```

## Features demonstrated

* AES GCM encryption + decryption
* Random number generation
* GFSK TX + RX
* GPIOs
* ADC sampling

## Latency

End-to-end latency is about 34.1 ms; majority of this is radio wakeup.

1. 29.7 ms client wakeup from sleep, TX nonce request
2. 1.9 ms server RX nonce request, TX nonce reply
3. 2.5 ms client RX nonce reply, AES GCM encryption, TX data
4. 0.2 ms server RX data, AES GCM decryption

These numbers do not add up to 34.1ms because the client TX and server RX occur
concurrently.

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
[NUCLEO-WL55JC2]: https://www.st.com/en/evaluation-tools/nucleo-wl55jc.html#sample-buy
[probe-run]: https://github.com/knurling-rs/probe-run
[rustup]: https://rustup.rs/
