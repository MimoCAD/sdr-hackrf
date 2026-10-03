# sdr-hackrf

A pure-Rust HackRF driver, receive and transmit, over [nusb]. No libusb,
no libhackrf: this crate → nusb → the OS, or WebUSB in a browser.

- Open by spec: `HackRf::open("")` (first found) or
  `HackRf::open("serial=…")`; in a browser `HackRf::request()` (the
  device chooser).
- Tuning, sample rate (with the baseband filter tracking it), and gain as
  one number or per stage: the RF amp (off / +14 dB), the IF LNA (0–40 dB
  in 8 dB steps), the baseband VGA (0–62 dB in 2 dB steps); transmit VGA
  0–47 dB.
- **Receive** by becoming a `Stream` (`into_stream`): the wire bytes
  (signed 8-bit I then Q, as `hackrf_transfer` writes) or `Iq` samples;
  split into a `ReadHalf` and a `Control` for two threads or tasks. A
  change made while samples flow is placed in the stream by the
  firmware's own byte count (a `Break`), and `finish` reports the
  firmware's count of shortfalls.
- **Transmit** by becoming a `TxStream` (`into_tx_stream`): wire bytes or
  `Iq` samples written, keyed when the queue first fills; split into a
  `WriteHalf` and a `TxControl`. `finish` waits until the radio has sent
  every sample and reports its underruns.
- The firmware runs one transceiver mode at a time: a radio is receiving
  or transmitting, and each stream owns it until `finish` hands it back.
- Async at its core (WebUSB in a browser); natively `blocking` wraps every
  call, its streams `std::io::Read` and `std::io::Write`. `no_std` with
  `alloc` without the default `std` feature (the protocol's encodings and
  the gain steps).

```rust
use std::io::Read;
use sdr_core::TuneRequest;
use sdr_hackrf::blocking::HackRf;

let mut hackrf = HackRf::open("")?;
hackrf.set_rate(10e6)?;
hackrf.set_freq(&TuneRequest::auto(446.0e6))?;
hackrf.set_gain(40.0)?;
let (mut reader, control) = hackrf.into_stream(0, 0).map_err(|e| e.error)?.split();
let mut buf = [0u8; 65536];
let n = reader.read(&mut buf)?;
```

`bench/README.md` (in the repository) records the measurements the
streams rest on; `web/` is a page that runs the driver in a browser.

[nusb]: https://crates.io/crates/nusb
[`sdr-core`]: https://crates.io/crates/sdr-core

License: MIT OR Apache-2.0.
