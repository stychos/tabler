# TABLER

RFID tag reader, writer, and hex editor for ESP32 + MFRC522 (RC522).

Reads and writes MIFARE Classic 1K tags (16 sectors, 64 blocks, 1024 bytes) through an interactive serial shell with ANSI color output and a cursor-based hex editor.

## Hardware

- **ESP32** dev board (any variant with accessible SPI pins)
- **MFRC522 (RC522)** 13.56 MHz RFID module
- **MIFARE Classic 1K** tags (S50) or compatible ISO 14443A cards/fobs

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

| Command       | Description                                              |
|---------------|----------------------------------------------------------|
| `info [-sN]`  | Device info + tag sector display. `-s0` through `-s15`.  |
| `read`        | Poll for tag, read all 16 sectors into memory.           |
| `edit`        | Open cursor-based hex editor for in-memory tag data.     |
| `write [-f]`  | Write all read blocks to a target tag. `-f` writes sector trailers too. |
| `clear`       | Clear stored tag data from memory.                       |
| `reboot`      | Restart the device.                                      |
| `help`        | List available commands.                                 |

### read

Polls the antenna until a tag is presented (ESC or Space to cancel). Authenticates each of the 16 sectors with the factory default key (`FF FF FF FF FF FF`) and reads all 64 blocks into RAM.

### info

Displays chip info (revision, flash, heap, IDF version, RC522 firmware, uptime) and the currently stored tag data. Defaults to showing sector 1; use `-sN` to view any sector:

```
>_ info -s0
```

Color coding:
- **Gray** - Block 0 (manufacturer, read-only)
- **Yellow** - Sector trailers (keys + access bits)
- **White/bold** - Block numbers
- **Cyan** - Labels

### edit

Interactive hex editor for the in-memory tag data. Opens on sector 1 by default.

| Key         | Action                           |
|-------------|----------------------------------|
| Arrow keys  | Navigate (left/right by nibble, up/down by block) |
| `[` / `]`   | Switch to previous/next sector   |
| `0-9 a-f`   | Write hex nibble at cursor       |
| Enter        | Save changes to RAM              |
| ESC          | Discard changes and exit         |

- Block 0 is displayed but not editable (manufacturer data).
- Sector trailers are editable (highlighted in yellow).
- After saving with Enter, use `write` to flash changes to a physical tag.

### write

Writes all in-memory blocks to a target tag. Skips block 0 (manufacturer) and sector trailers by default.

Use `write -f` to force-write sector trailers as well (changes keys and access bits on the target tag).

## MIFARE Classic 1K Memory Layout

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

- 16 sectors, 4 blocks each, 16 bytes per block = 1024 bytes total.
- Each sector trailer contains Key A (6 bytes), access bits (4 bytes), and Key B (6 bytes).
- Factory default keys: `FF FF FF FF FF FF` for both Key A and Key B.

## Project Structure

```
tabler/
  main/
    main.c          Entry point, RC522 init, shell launch
    shell.c         Command shell, hex editor, UART I/O
    shell.h
    app_state.c     In-memory tag storage (64 blocks)
    app_state.h
  components/
    rc522/
      rc522.c           SPI register access, init, transceive
      rc522_picc.c      Card detection, anti-collision, UID read
      rc522_mifare.c    MIFARE auth, block read/write
      include/
        rc522.h         Public API
        rc522_types.h   Types, pin config defaults
      rc522_registers.h Register map
```

The RC522 driver is a custom in-tree component with no external dependencies.

## Supported Tags

The RC522 module operates at 13.56 MHz (ISO 14443A) and supports:

- MIFARE Classic 1K (S50) - fully supported
- MIFARE Classic 4K (S70) - first 16 sectors
- MIFARE Ultralight / NTAG - detection only (different block structure)

It does **not** support 125 kHz tags (EM4100, HID ProxCard, T5577) or iButton (1-Wire).
