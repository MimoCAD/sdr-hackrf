# sdr-hackrf over WebUSB

The proof page: sdr-hackrf compiled to WebAssembly, driving a HackRF from a
Chromium-family browser (Chrome, Edge, …; Firefox and Safari have no
WebUSB). Two buttons:

- **Receive check** — the browser's device chooser on first use, 4 s at
  8 Msps read as wire bytes on one task while another steps the gain three
  times, then the firmware's count of shortfalls.
- **Transmit check** — a tone for 2 s at the lowest transmit gain with the
  amp off, then the firmware's count of underruns and whether every sample
  was sent. Only with the box ticked: the antenna port cabled to a
  receiver through attenuation, nothing radiating.

Build (from this directory; `wasm-bindgen` the command-line tool at the
version in `Cargo.lock`):

```
cargo build --release
wasm-bindgen --target web --out-dir pkg target/wasm32-unknown-unknown/release/sdr_hackrf_web.wasm
```

Serve this directory over `http://localhost` (WebUSB needs a secure
context, which localhost is), for example `python3 -m http.server 8000`, and
open `http://localhost:8000/`.

Per platform: macOS works as is; Windows needs the WinUSB driver for the
HackRF (e.g. through Zadig); on Linux the user needs access to the device
node.
