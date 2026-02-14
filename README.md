# TABLER

RFID tag reader, writer, and hex editor for ESP32 + MFRC522 (RC522).

Reads and writes MIFARE Classic 1K/4K, Ultralight, Ultralight C, and NTAG213/215/216 tags through an interactive serial shell with ANSI color output and a cursor-based hex editor. Auto-detects tag type on read.

## Hardware

- **ESP32** dev board (any variant with accessible SPI pins)
- **MFRC522 (RC522)** 13.56 MHz RFID module
- **Supported tags** — see [Supported Tags](#supported-tags) below

The RC522 operates at **3.3V only**. Do not connect VCC to 5V.

## Pinout

| RC522 Pin | ESP32 GPIO | Function              |
|-----------|------------|-----------------------|
| SDA (CS)  | GPIO 5     | SPI Chip Select       |
| SCK       | GPIO 18    | SPI Clock             |
| MOSI      | GPIO 23    | SPI Master Out        |
| MISO      | GPIO 19    | SPI Master In         |
| RST       | GPIO 22    | Hardware Reset        |
| 3.3V      | 3V3        | Power (3.3V only!)    |
| GND       | GND        | Ground                |
| IRQ       | -          | Not connected         |

```
  RC522 Module              ESP32 Dev Board
 +-----------+             +---------------+
 | SDA  (CS) |------->-----| GPIO 5        |
 | SCK       |------->-----| GPIO 18       |
 | MOSI      |------->-----| GPIO 23       |
 | MISO      |------->-----| GPIO 19       |
 | IRQ       |    (nc)     |               |
 | GND       |------->-----| GND           |
 | RST       |------->-----| GPIO 22       |
 | 3.3V      |------->-----| 3V3           |
 +-----------+             +---------------+
```

SPI bus: `SPI3_HOST` at 5 MHz. Pins can be changed in `components/rc522/include/rc522_types.h` via `RC522_CONFIG_DEFAULT()`.

## Build & Flash

Requires [ESP-IDF v5.5+](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/).

```bash
source ~/.esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Serial console runs at **115200 baud**.

## Shell Commands

| Command          | Description                                              |
|------------------|----------------------------------------------------------|
| `info [-sN\|-pN]` | Device info + sector/page display. `-sN` for Classic sectors, `-pN` for UL/NTAG page groups. |
| `read`           | Poll for tag, auto-detect type, read all data into memory. |
| `edit`           | Open cursor-based hex editor for in-memory tag data.     |
| `write [-f]`     | Write data to a target tag. `-f` writes sector trailers too (Classic only). |
| `clear`          | Clear stored tag data from memory.                       |
| `reboot`         | Restart the device.                                      |
| `help`           | List available commands.                                 |

### read

Polls the antenna until a tag is presented (ESC to cancel). Auto-detects the tag type via SAK byte and GET_VERSION command, then reads all data into RAM:

- **Classic 1K** — authenticates 16 sectors with factory default key, reads 64 blocks
- **Classic 4K** — authenticates 40 sectors (32 small + 8 large), reads 256 blocks
- **Ultralight / Ultralight C / NTAG** — reads all pages (no authentication needed)
- **DESFire / Plus** — stores UID only (detection only, different protocol)

### info

Displays chip info (revision, flash, heap, IDF version, RC522 firmware, uptime), the tag type, and stored data.

For Classic tags, use `-sN` to view a sector:
```
>_ info -s0
```

For UL/NTAG tags, use `-pN` to view a page group (16 pages per group):
```
>_ info -p0
```

Color coding:
- **Gray** — Block 0 / manufacturer data (Classic) or UID/lock pages 0-3 (UL/NTAG)
- **Yellow** — Sector trailers (Classic) or config pages (UL/NTAG)
- **White/bold** — Block/page numbers
- **Cyan** — Labels

### edit

Interactive hex editor for the in-memory tag data.

**Classic tags** — 16 bytes per row, one row per block:

| Key         | Action                           |
|-------------|----------------------------------|
| Arrow keys  | Navigate (left/right by nibble, up/down by block) |
| `[` / `]`   | Switch to previous/next sector (up to 40 for 4K) |
| `0-9 a-f`   | Write hex nibble at cursor       |
| Enter        | Save changes to RAM              |
| ESC          | Discard changes and exit         |

**UL/NTAG tags** — 4 bytes per row, one row per page:

| Key         | Action                           |
|-------------|----------------------------------|
| Arrow keys  | Navigate (left/right by nibble, up/down by page) |
| `[` / `]`   | Switch to previous/next page group (16 pages each) |
| `0-9 a-f`   | Write hex nibble at cursor       |
| Enter        | Save changes to RAM              |
| ESC          | Discard changes and exit         |

- Block 0 (Classic) and pages 0-3 (UL/NTAG) are displayed but not editable.
- Sector trailers (Classic) are editable but highlighted in yellow as a warning.
- After saving with Enter, use `write` to flash changes to a physical tag.

### write

Writes all in-memory data to a target tag.

- **Classic** — skips block 0 (manufacturer) and sector trailers by default. Use `write -f` to force-write trailers (changes keys and access bits).
- **UL/NTAG** — writes user pages only (skips UID/lock pages 0-3 and config pages at the end).

## Supported Tags

| Tag | SAK | Support | Memory |
|-----|-----|---------|--------|
| MIFARE Classic 1K | 0x08 | Full read/write/edit | 16 sectors x 4 blocks x 16B (1024B) |
| MIFARE Classic 4K | 0x18 | Full read/write/edit | 32 small sectors + 8 large sectors (4096B) |
| MIFARE Ultralight | 0x00 | Full read/write/edit | 16 pages x 4B (64B) |
| MIFARE Ultralight C | 0x00 | Full read/write/edit | 48 pages x 4B (192B) |
| NTAG213 | 0x00 | Full read/write/edit | 45 pages x 4B (180B) |
| NTAG215 | 0x00 | Full read/write/edit | 135 pages x 4B (540B) |
| NTAG216 | 0x00 | Full read/write/edit | 231 pages x 4B (924B) |
| MIFARE DESFire | 0x20 | Detect only | — |
| MIFARE Plus | 0x10/0x11 | Detect only | — |

Tags with SAK 0x00 are distinguished via the GET_VERSION command (storage size byte). Original Ultralight is identified by GET_VERSION timeout.

The RC522 does **not** support 125 kHz tags (EM4100, HID ProxCard, T5577) or iButton (1-Wire).

## MIFARE Classic Memory Layout

### Classic 1K

```
Sector 0:   Block 00 [Manufacturer - read only]
            Block 01 [Data]
            Block 02 [Data]
            Block 03 [Trailer: Key A | Access Bits | Key B]

Sector 1:   Block 04 [Data]
            Block 05 [Data]
            Block 06 [Data]
            Block 07 [Trailer]
  ...
Sector 15:  Block 60 [Data]
            Block 61 [Data]
            Block 62 [Data]
            Block 63 [Trailer]
```

16 sectors, 4 blocks each, 16 bytes per block = 1024 bytes total.

### Classic 4K

Sectors 0-31 have 4 blocks each (same as 1K). Sectors 32-39 have 16 blocks each.

```
Sectors  0-31:  4 blocks each  (128 blocks, 2048 bytes)
Sectors 32-39: 16 blocks each  (128 blocks, 2048 bytes)
Total: 256 blocks = 4096 bytes
```

Each sector trailer contains Key A (6 bytes), access bits (4 bytes), and Key B (6 bytes). Factory default keys: `FF FF FF FF FF FF`.

## Project Structure

```
tabler/
  main/
    main.c          Entry point, RC522 init, shell launch
    shell.c         Command shell, hex editor, UART I/O
    shell.h
    app_state.c     In-memory tag storage (union: blocks or pages)
    app_state.h
  components/
    rc522/
      rc522.c             SPI register access, init, transceive
      rc522_picc.c        Card detection, anti-collision, UID read, type detection
      rc522_mifare.c      MIFARE Classic auth, block read/write
      rc522_ultralight.c  Ultralight/NTAG page read/write
      include/
        rc522.h           Public API
        rc522_types.h     Types, PICC type enum, pin config defaults
      rc522_registers.h   Register and command map
```

The RC522 driver is a custom in-tree component with no external dependencies.
