# Bench records

What the streams rest on, measured on a HackRF Pro (firmware with the M0
state request) over USB 2.0, through the programs in `examples/`. Each
`data/` file is one program's output, a JSON line per reading.

"M0" and "M4" are the HackRF's two processor cores (its LPC4320's
Cortex-M0, which moves samples between the radio and the sample buffer,
and Cortex-M4, which moves the buffer to and from USB): `m0_count` and
`m4_count` are the bytes each has moved.

## The M0 counters while receiving — `data/m0-counters.jsonl`, `data/m0-stall.jsonl`

`examples/m0probe.rs`, 10 Msps, two 3 s runs each.

- A mode start zeroes `m0_count`, `m4_count` and the shortfalls, once the
  firmware has switched: a read straight after the request still shows the
  last run's counts with `active_mode` 0.
- While receiving, `m4_count` is the bytes the host has received, to the
  byte; `m0_count` leads it by the device's buffer fill (~15-20 kB).
- Read every 10 ms while samples flow: 0 shortfalls. Reading the counters
  costs nothing.
- Reading stopped for 200 ms with 4 transfers in flight: the host 4.0 MB
  short (200 ms at 10 Msps), the firmware 1 shortfall, its length
  (`longest_shortfall`) the bytes missing. Those bytes are not in
  `m0_count`, so `m0_count` stays the host stream's byte index; the
  firmware keeps the longest shortfall, not their sum.
- At the finish (transceiver off, the queue taken), up to one 16 KiB piece
  stays in the device: `m4_count` only ever moved in 16 KiB steps.

## Where a change lands — `data/break-placement.jsonl`

`examples/breakcheck.rs`, 10 Msps: the gain stepped 0 ↔ 70 dB ten times
from a second thread while the first read every sample, the step found in
the samples' power (64-sample windows). In 10 of 10 the step fell between
the `Break`'s `at_sample` (`m0_count` read before the request) and
`settled_at` (read after): 1,792-4,752 samples after the first, 704-3,344
before the second.

## Where a retune lands, and its settling — `data/retune-2M.jsonl`, `data/retune-4M.jsonl`, `data/retune-8M.jsonl`

`examples/retunecheck.rs`: a USRP B206mini's tone at 300.3 MHz (transmit
gain 60 dB, 20 dB below full scale, through a 20 dB pad: ~ −55 dBm at the
antenna port) while the HackRF retuned ten times between 300.0 and
300.1 MHz (the tone at +300 and +200 kHz), at 2, 4 and 8 Msps.

| rate | takeover − `at_sample` | takeover − `settled_at` | settled − `settled_at` |
|---|---|---|---|
| 2 Msps | 560 … 1,232 | −496 … 64 | −496 … 64 |
| 4 Msps | 912 … 1,616 | −1,408 … −416 | −128 … 352 |
| 8 Msps | 1,264 … 3,200 | −2,512 … −1,536 | −208 … 560 |

(samples; the takeover the first 32-sample window where the new offset
outweighs the old; settled the first of eight 256-sample windows in a row
within 500 Hz of it.) In 30 of 30 the new tuning appeared between the
`Break`'s `at_sample` and `settled_at`, and was within 500 Hz no later than
~90 µs past `settled_at`; at 8 Msps it swung by up to ±15 kHz for the first
~0.25 ms. Ten more at 4 Msps on a second computer (the tone keyed through
sdr-uhd): takeover 576 … 1,312 after `at_sample`, −656 … −288 before
`settled_at`, settled 224 … 416 past it (≤ 104 µs). With the tone 10-20 dB weaker the 8 Msps windows were too noisy
to read and the takeover could fire on noise before `at_sample`.

## Transmitting — `data/tx.jsonl`, `data/tx-stall.jsonl`

`examples/txcheck.rs`, 8 Msps, a tone at the lowest transmit gain, the
amp off, the antenna port cabled to a receiver through a 20 dB pad.

- While transmitting `m0_count` is the bytes sent and `m4_count` the bytes
  taken from USB; after `finish` both equal the bytes written: every
  sample sent before the radio was unkeyed (`sent_all`), the finish 18 ms.
- Writing stopped for 200 ms: 1 underrun of ~1.55 M samples (~195 ms). The
  end of the transmission is not counted (the count is read once the last
  transfer reached the radio).

## In a browser — `data/browser.jsonl`

`web/` in Chrome over WebUSB, 8 Msps, the page opened as `?auto=…`.

- Receive, 4 s, three gain steps from a second task: 63,963,136 bytes
  read, 16 MB/s in every full second, 0 shortfalls.
- Transmit, 2 s: 0 underruns, every sample sent, the finish 19 ms.
- At the receive finish 23,920 samples (47,840 bytes) were left over: more
  than one 16 KiB piece, because a browser cannot cancel a queued transfer,
  so the last partly filled 64 KiB transfer never reaches the host.
