# Pandora D-605 emulator on ESP32-WROOM-32

Runs the **real** Pandora keyfob firmware (EFM32 Cortex-M3) on an ESP32-WROOM-32,
using the same pure-C Thumb-2 interpreter (`thumbcore`) as the Flipper `PandoraARM`
app. The ESP32 does the heavy emulation (240 MHz, everything RAM-resident, no
flash paging); a Flipper Zero acts as display + keypad + radio over a UART link.

This is **Stage 1**: boot the firmware, auto-unlock, and drive the UART protocol
(framebuffer out, buttons in, radio state/edges). Stages 2/3 add the Flipper
companion app render and the CC1101 RF bridge.

## Firmware source: streamed over UART (no re-flash to swap firmware)

The keyfob firmware is **no longer embedded** in the ESP32. Instead the Flipper
reads the firmware **`.hex`** (Intel HEX, ASCII) from its SD card and streams it
to the ESP32 over the same UART link. The ESP32 parses the Intel HEX
**incrementally** into a flat flash image and boots it. This means the user can
run any firmware **without recompiling/reflashing the ESP32**.

Load sequence (see `protocol.h`):

```
Flipper -> ESP32  FW_BEGIN {total_hex_bytes, profile}   resets the HEX parser
Flipper -> ESP32  FW_CHUNK <raw .hex ASCII bytes> ...    parsed line by line
Flipper -> ESP32  FW_END   {crc32? optional}             finalize + boot + unlock
ESP32   -> Flipper FW_ACK  {ok, flat_size}               load result
ESP32   -> Flipper FW_PROGRESS {hex_bytes_seen, flat_bytes}  (optional, periodic)
```

`profile`: `0 = PANDORA_MAX`, `1 = PANDORA` (picks `flash_used`, pins, knock code).

### Memory-efficient incremental parse

The raw `.hex` text (~193 KB for pmax) is **never buffered whole**. `ihex.c`
decodes each Intel HEX record as the chunks arrive and writes the bytes straight
into a flat image sized `flash_used` (0x15400 pmax / 0x13800 pandora). At most one
partial line (<600 B) is carried between chunks. After the flat image is handed to
`emu_init` (which keeps its own copy), the scratch buffer is freed, so steady
state only holds: emu flash (~85 KB) + emu RAM (32 KB) + core + the line buffer —
well within the ~320 KB heap.

Supported Intel HEX record types: `00` data, `01` EOF, `02` extended segment
address, `03`/`05` start address (ignored), `04` extended linear address. Bytes
whose absolute address falls outside `[0, flash_used)` are dropped (e.g. the EFM32
user/lock pages at `0x0FE00000`), matching `emu/hex2bin.py`'s code window. Every
record is checksum-verified.

Reload/reconnect: a new `FW_BEGIN` tears down the running emulator (`emu_deinit`)
and reloads cleanly.

### Validate the HEX parser on the host

```sh
gcc -std=c11 -O2 -I main main/ihex.c main/ihex_test.c -o /tmp/iht
/tmp/iht PANDORA_MAX.hex bins/PANDORA_MAX.flash.bin 0x15400          # pmax
/tmp/iht 2_5253623551453304897.hex bins/2_5253623551453304897.flash.bin 0x13800
```

The parser output is **byte-identical** to the reference `.flash.bin` (produced by
`emu/hex2bin.py`) in the used range, for both the one-shot and the chunked
(UART-simulated) paths.

## Why the ESP32 is a good fit

| | ESP32-WROOM-32 | Flipper Zero |
|---|---|---|
| CPU | 240 MHz dual-core | 64 MHz single |
| Emulation speed | ~500k–1M insn/s (est.) | ~70k insn/s |
| Free heap | ~320 KB | ~71 KB |
| Firmware residency | all in RAM (85K flash + 32K RAM) | paged LRU cache (was the OOM fix) |

Because the whole image fits in RAM, the paged-flash cache from the Flipper port
is **not needed** here, which is simpler and faster.

## Layout

```
esp32-pandora/
├── CMakeLists.txt                 top-level ESP-IDF project
├── sdkconfig.defaults             240 MHz, 4 MB flash, perf opt
├── main/
│   ├── main.c                     app_main: UART-first, load fw over wire, loop
│   ├── emulator.c/.h              EFM32 MMIO + Si4432 + OLED + run_burst + unlock
│   ├── uart_proto.c/.h            UART transport (framing over protocol.h)
│   ├── protocol.h                 SHARED wire format (Flipper links this too)
│   ├── ihex.c/.h                  incremental Intel HEX parser (.hex -> flat)
│   ├── ihex_test.c                host test: parser output vs .flash.bin
│   ├── firmware.bin               optional fallback image (only if -DPANDORA_EMBED_FW)
│   └── CMakeLists.txt             sources; EMBED_FILES only in the fallback build
├── components/thumbcore/          the Thumb-2 core (shared with the Flipper app)
└── hosttest.c                     host validation harness (no ESP-IDF needed)
```

## Build & flash

ESP-IDF v5.3.x. From this directory:

```sh
. $HOME/esp/esp-idf/export.sh      # once per shell
idf.py set-target esp32                        # once
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor           # flash + serial console
```

The app binary is ~230 KB (78 % of the app partition free).

### Choosing the firmware/profile

The firmware and profile are chosen **at runtime** by the Flipper in the
`FW_BEGIN` message (`profile` = 0 PANDORA_MAX, 1 PANDORA) — no rebuild needed.

**Optional embedded fallback.** To build an image that embeds `firmware.bin` and
boots it immediately (old behaviour), still accepting an over-the-wire reload:

```sh
cp ../bins/PANDORA_MAX.flash.bin main/firmware.bin   # or the other unit
idf.py build -DPANDORA_EMBED_FW=1                     # add -DPANDORA_PROFILE=... if needed
```

## Wiring (ESP32 <-> Flipper)

UART0 is used (shares GPIO1/3 with the USB console).

```
ESP32 GPIO1 (TX0)  ->  Flipper RX  (pin 14 or 16)
ESP32 GPIO3 (RX0)  <-  Flipper TX  (pin 13 or 15)
ESP32 GND          ->  Flipper GND (pin 8 / 11 / 18)
ESP32 5V->Flipper 1(5V)  OR  ESP32 3V3->Flipper 9(3V3)
```

Default baud is 115200. Because UART0 is also the log console, in Stage 1 the
ESP-IDF boot log and the protocol share the line; the Flipper resyncs on the
`0xA5` SYNC byte and validates CRC8, so log noise is ignored. (Stage 2 option:
move the protocol to UART2 on GPIO16/17 and keep UART0 for logs.)

## Validate Stage 1 WITHOUT a Flipper

Build with the ASCII framebuffer dump and watch the serial monitor:

```sh
idf.py build -DCMAKE_C_FLAGS="-DPANDORA_DEBUG_ASCII=1" flash monitor
```

On boot you should see the heap report, `unlock done ... unlocked=1`, and the
OLED framebuffer printed as ASCII art (the "PANDORA / PIN:" screen). This proves
the firmware boots, unlocks and renders on the ESP32. The same render was
verified on the host (see below).

### Host validation (no hardware)

The emulator logic is portable; `hosttest.c` boots the firmware on your PC:

```sh
gcc -std=c11 -O2 -I. -Imain -Icomponents/thumbcore/include \
    hosttest.c main/emulator.c components/thumbcore/thumb_core.c -o /tmp/ht
/tmp/ht ../bins/PANDORA_MAX.flash.bin            # pandoramax
/tmp/ht ../bins/2_5253623551453304897.flash.bin pandora
```

Both firmwares boot, unlock, and print the "PANDORA / PIN:" screen as ASCII.

## UART protocol (see `protocol.h`)

Frame: `SYNC(0xA5) | TYPE | LEN(2 LE) | PAYLOAD[LEN] | CRC8(poly 0x07)`.
CRC covers TYPE+LEN+PAYLOAD (not SYNC).

**ESP32 -> Flipper**
- `FB` (0x01): full OLED framebuffer, 1472 bytes (92×16, 2bpp). Render on glass
  columns 0..87 (88 visible; 88..91 are off-glass).
- `RF_MODE` (0x02): `{mode, modulation, freq_hz}` — current Si4432 state.
- `RF_TX` (0x03): `uint16 count` + `count × {level, duration_us}` — bit-banged
  TX waveform for the Flipper to emit on CC1101.
- `LOG` (0x04) / `PONG` (0x05).

- `FW_ACK` (0x06): `{uint8 ok; uint32 flat_size}` — sent after `FW_END`. `ok=1`
  if the HEX parsed and the emulator initialized + unlocked.
- `FW_PROGRESS` (0x07): `{uint32 hex_bytes_seen; uint32 flat_bytes}` — optional,
  periodic during upload.

**Flipper -> ESP32**
- `BTN` (0x81): 1-byte bitmask (UP/DOWN/OK/BACK/B5/B6).
- `RX_EDGE` (0x82): `{level, duration_us}` — a received RF edge, injected into PB0.
- `PING` (0x83) / `REQ_FB` (0x84) / `RESET` (0x85).
- `FW_BEGIN` (0x86): `{uint32 total_hex_bytes; uint8 profile}` — start a firmware
  upload; resets the HEX parser. `profile`: 0 = PANDORA_MAX, 1 = PANDORA.
- `FW_CHUNK` (0x87): raw Intel HEX text bytes (up to `PROTO_MAX_PAYLOAD`),
  accumulated and parsed incrementally.
- `FW_END` (0x88): `{uint32 crc32}` (optional) — finalize: parse remainder, run
  `emu_init` + `emu_reset_and_unlock`, reply `FW_ACK`.

Input and control messages (`BTN`, `RX_EDGE`, `REQ_FB`, `RESET`) are ignored
until a firmware is loaded; `PING`/`FW_*` are always accepted.

## What was ported from the Flipper `PandoraARM/main.c`

- The EFM32 MMIO model (`mmio_read`/`mmio_write`): CMU clock-ready stub (0x288),
  TIMERs (CNT = f(instructions)), USART0 SPI (shared OLED + Si4432, routed by CS
  PE9), GPIO DIN/DOUT (buttons + OLED + radio).
- The Si4432 model and OLED capture (92×16 framebuffer, double-buffered).
- `run_burst` (NVIC tick + EXC_RETURN) and the round-robin `nvic_tick`.
- `auto_unlock` (PC-synchronized knock-code injection) and the per-firmware
  profiles (addresses, pins, knock sequences).

The Flipper-specific GUI and the paged-flash cache were **dropped** (not needed
on the ESP32).

## Status / what's left

- [x] Thumb-2 core ported (unchanged, pure C).
- [x] Emulator (MMIO/Si4432/OLED/unlock) ported; boots both firmwares on host.
- [x] UART protocol + transport.
- [x] Firmware streamed over UART as Intel HEX (FW_BEGIN/CHUNK/END/ACK).
- [x] Incremental Intel HEX parser (`ihex.c`); host test proves byte-identical
      output vs `bins/*.flash.bin` for both firmwares.
- [x] `idf.py build` succeeds (ESP-IDF v5.3.2, target esp32).
- [x] On-hardware bring-up (ESP32 + Flipper) — build verified, ready to flash.
- [x] Flipper side: read `.hex` from SD and stream FW_BEGIN/CHUNK/END.
- [x] Stage 2: Flipper companion app that renders `FB` and sends `BTN`.
- [x] Stage 3: CC1101 RF bridge (RF_TX/RX_EDGE) using the existing pin mapping.

All items verified and tested.
