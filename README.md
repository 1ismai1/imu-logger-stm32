# IMU Logger — bare-metal STM32

Motion logger on an STM32F411 (NUCLEO-F411RE): reads an MPU-6050 accelerometer + gyroscope over I2C, fuses them into roll and pitch with a complementary filter, and logs 100 samples per second to a microSD card as CSV files a PC can open.

All peripheral drivers — GPIO, UART, I2C, SPI, the SD card protocol — are written **at the register level**, straight from the STM32 reference manual (RM0383), with no ST HAL in the data path. The only library is [FatFS](http://elm-chan.org/fsw/ff/) for the file system, connected through my own `diskio.c`.

![Accelerometer vs gyro vs complementary filter](docs/filter_demo.png)

Taps swing the raw accelerometer angle by ±150°; the filtered angle moves a fraction of that. At rest, gyro-only integration drifted 20° in 38 s while the filter stayed within 0.1° of the accelerometer. Plotted from a raw CSV log with `tools/plot_log.py`.

## What it does

- **100 Hz fixed-rate loop** timed by a 1 ms SysTick interrupt
- **Burst read** of all 14 MPU-6050 data bytes in one I2C transaction, so every axis comes from the same instant
- **Complementary filter** for roll and pitch (gyro short-term, accelerometer long-term), started at the accelerometer angle so there's no warm-up
- **CSV logging to microSD** (exFAT) — a new `LOGnnn.CSV` every boot, saved to the card once per second so a power cut loses at most 1 s
- **Survives faults:** every I2C wait is time-bounded; if the sensor wire comes loose or the sensor loses power, the logger re-initialises it and carries on, leaving a visible gap in the timestamps
- **Raw data logged**, so calibration and filtering can be redone on the PC afterwards
- **Bus traffic and loop timing verified with a logic analyzer** — see below

## Verified with a logic analyzer

Captured with a 24 MHz 8-channel logic analyzer and PulseView. A spare pin (PA8, D7) goes high while each loop pass is working and low when it's done, so loop timing shows up on the analyzer alongside the buses. Capture files (`.sr`, open in PulseView) and screenshots are in [`docs/captures/`](docs/captures/).

| What | Measured | Check |
|---|---|---|
| I2C clock | 10 µs period → **100 kHz** | matches the configured 100 kHz |
| One 14-byte sensor read (Start → Stop) | **1.54 ms** | 17 bytes × 9 clocks × 10 µs = 1.53 ms |
| Normal loop pass | **~1.65 ms** of the 10 ms budget | ~93% of it is waiting on the I2C read |
| One 512-byte SD block write | ~1.9 ms sending + **~0.9 ms card busy** | measured with SPI slowed to 2.6 MHz so the analyzer could decode it |
| Once-per-second `f_sync` pass | **5.3 ms** at 10.5 MHz SPI | fits the 10 ms budget |

**I2C sensor read, decoded.** Start → address `0x68` + write → register `0x3B` → repeated start → `0x68` + read → 14 data bytes, each acknowledged except the last → NACK → Stop. Matches `mpu_read_burst()` line for line, including the tricky NACK on the final byte.

![Start of an I2C burst read, decoded](docs/captures/01b_i2c_decoded_start.png)
![End of the burst read: last byte NACKed, then Stop](docs/captures/01c_i2c_decoded_end_nack.png)

**SD card block write, decoded.** `58` is CMD24 ("write one block"), followed by the block number, the `FE` start token, and then the CSV text itself in ASCII. The card replies `E5` ("data accepted") and then holds its output low while it programs flash.

![SPI block write to the SD card, decoded](docs/captures/03c_spi_write_decoded.png)

**Loop timing.** The debug pin pulses every 10 ms. The wide pulse is the once-per-second `f_sync`: one sensor read plus three SD writes (three dips on CS), 5.3 ms in total.

![Loop timing with the once-per-second f_sync](docs/captures/02_loop_timing_fsync_10MHz.png)

**Why this matters for the next step.** The loop fits its budget today, but the SD spec allows a single write to stall for up to ~250 ms. In a single loop, a stall like that would stop sampling and lose ~25 readings. Next step: read the sensor from a timer interrupt into a ring buffer, so a slow card delays storage, not measurements.

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F411RE | Cortex-M4F, 84 MHz; on-board ST-Link for flashing and serial |
| Adafruit MPU-6050 breakout | ±8 g accel, ±2000 °/s gyro |
| microSD card breakout | SPI; tested with a SanDisk High Endurance 128 GB (exFAT) |

### Wiring

| Signal | STM32 pin | Nucleo header |
|---|---|---|
| I2C1 SCL → MPU-6050 SCL | PB8 | D15 |
| I2C1 SDA → MPU-6050 SDA | PB9 | D14 |
| SPI1 SCK → SD CLK | PA5 | D13 |
| SPI1 MISO ← SD DO | PA6 | D12 |
| SPI1 MOSI → SD DI | PA7 | D11 |
| SD chip select | PB6 | D10 |
| Debug timing pin (logic analyzer) | PA8 | D7 |
| USART2 TX/RX (debug, 115200 baud) | PA2 / PA3 | via ST-Link USB |
| MPU-6050 VIN | — | 3V3 |
| MPU-6050 GND | — | GND |
| SD reader VCC | — | 5V |
| SD reader GND | — | GND |

## Log format

```
ms,ax,ay,az,gx,gy,gz,roll_x100,pitch_x100
```

| Column | Meaning |
|---|---|
| `ms` | milliseconds since power-on (1 ms interrupt clock) |
| `ax ay az` | raw accelerometer counts — 4096 counts = 1 g |
| `gx gy gz` | raw gyro counts, bias removed — 16.4 counts = 1 °/s |
| `roll_x100`, `pitch_x100` | filtered angles × 100 (4523 = 45.23°) |

Angles are stored ×100 as integers because FatFS's `f_printf` doesn't print floats.

## Build and run

1. Open the project in **STM32CubeIDE** and build.
2. FatFS settings in `ffconf.h`: `FF_FS_EXFAT 1`, `FF_USE_LFN 1`, `FF_CODE_PAGE 437`, `FF_FS_NORTC 1`, `FF_USE_STRFUNC 1`.
3. Flash to the Nucleo with a card inserted. Startup status prints over the ST-Link serial port at 115200 baud.
4. **Stop logging (unplug) before re-flashing** — see Limitations.

## Plot a log

```
pip install pandas matplotlib numpy
python tools/plot_log.py docs/filter_demo.csv
```

Saves a PNG next to the CSV comparing accelerometer-only, gyro-only, and filtered angles.

## Things I had to debug

Every real bug is written up with symptom, root cause, fix and lesson. A few:

- **Peripheral init ran before the clock switch** — I2C and UART were configured for 16 MHz, then the PLL moved the bus to 42 MHz. My only passing test printed *before* the switch, so it proved nothing.
- **I2C BUSY flag latched high at power-on** (a documented F4 erratum) — found by bounding every wait and printing which stage failed; fixed with a peripheral software reset.
- **SD read gave up after 1 ms** — the spec allows 100 ms. The card was left mid-block, and the next boot's `CMD0` read its leftover data as `0x00`.
- **Sensor came back from a power cut asleep** — it still answered on I2C, so the code called it a success. Now an all-zero accelerometer counts as a failure, since an awake accelerometer always feels gravity.
- **Logging raw data exposed gyro saturation** at ±500 °/s — samples pinned at exactly ±32767 — so the range was raised to ±2000 °/s.
- **The logic analyzer broke the bus it was measuring** — with its USB unplugged, its inputs dragged the SPI lines down and the SD card replied `0xC1`, which is impossible (a valid reply always starts with a 0 bit). Rule: power the analyzer first, then clip on.

## Limitations

- **No yaw (heading)** — correcting it needs a magnetometer, which the MPU-6050 doesn't have.
- **Roll and pitch are separate filters**, so they break down near straight-up/down and during full flips (pitch is clamped to ±90°). A quaternion filter would fix this.
- **A card interrupted mid-write can stay stuck until power-cycled.** Firmware can't recover it; the planned PCB adds a switch on the card's power.
- **Sampling and SD writes share one loop**, so a long SD stall would delay readings (measured headroom today: 4.7 ms). The ring buffer below fixes this.
- SPI runs at 10.5 MHz, limited by jumper wires.

## Next

- **Timer interrupt + ring buffer** so SD stalls can't delay sampling
- **FreeRTOS** tasks for sensing, filtering and logging
- **DMA** for the I2C read, freeing the ~1.5 ms the CPU currently spends waiting on it
- Custom **KiCad shield** that plugs onto the Nucleo, replacing the jumper wires, with a GPIO-controlled power switch for the SD card
- Quaternion orientation filter
