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
$ DEFMT_LOG=trace cargo run -p server -- --probe 001600345553500A20393256
$ DEFMT_LOG=trace cargo run -p server -- --probe 001D00145553500A20393256
```

If you want to be fancy, I use tmux to launch these at the same time in one
terminal session.

```bash
tmux new-session "DEFMT_LOG=trace cargo run -p server -- --probe 001600345553500A20393256" \; split-window "DEFMT_LOG=trace cargo run -p client -- --probe 001D00145553500A20393256" \; setw remain-on-exit on \;
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
