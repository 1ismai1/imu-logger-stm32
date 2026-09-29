# IMU Logger: bare-metal STM32

A motion logger built on an STM32F411 (NUCLEO-F411RE). It reads an MPU-6050 accelerometer and gyroscope over I2C, combines them into roll and pitch angles with a complementary filter, and saves 100 samples per second to a microSD card as CSV files.

Every peripheral driver (GPIO, UART, I2C, SPI and the SD card protocol) is written **at the register level**, straight from the STM32 reference manual (RM0383). There is no ST HAL in the data path. The only library is [FatFS](http://elm-chan.org/fsw/ff/) for the file system, connected through my own `diskio.c`.

![Accelerometer vs gyro vs complementary filter](docs/filter_demo.png)

Taps swing the raw accelerometer angle by ±150°, while the filtered angle barely moves. At rest, the gyro-only angle drifted 20° in 38 s, and the filter stayed within 0.1° of the accelerometer. Plotted from a raw CSV log with `tools/plot_log.py`.

## Features

- **100 Hz fixed-rate loop**, timed by a 1 ms SysTick interrupt.
- **Burst read** of all 14 MPU-6050 data bytes in one I2C transaction, so every axis comes from the same instant.
- **Complementary filter** for roll and pitch: the gyro is trusted short-term, the accelerometer long-term. It starts at the accelerometer angle, so there is no warm-up.
- **CSV logging to microSD** (exFAT). Each boot creates a new `LOGnnn.CSV`, and the file is saved to the card once per second, so a power cut loses at most 1 s of data.
- **Fault recovery.** Every I2C wait has a timeout. If a sensor wire comes loose or the sensor loses power, the logger re-initialises it and keeps going, leaving a visible gap in the timestamps.
- **Raw data is logged**, so calibration and filtering can be redone on a PC later.
- **Bus traffic and loop timing verified with a logic analyzer** (see below).

## Verified with a logic analyzer

All measurements were taken with a 24 MHz, 8-channel logic analyzer and PulseView. A spare pin (PA8, D7) goes high while each loop pass is working and low when it finishes, so loop timing shows up next to the bus signals. Capture files (`.sr`, open in PulseView) and screenshots are in [`docs/captures/`](docs/captures/).

| What | Measured | Check |
|---|---|---|
| I2C clock | 10 µs period, **100 kHz** | Matches the configured speed |
| One 14-byte sensor read (Start to Stop) | **1.54 ms** | 17 bytes × 9 clocks × 10 µs = 1.53 ms |
| Normal loop pass | **~1.65 ms** out of 10 ms | About 93% is waiting on the I2C read |
| One 512-byte SD block write | ~1.9 ms sending, then **~0.9 ms card busy** | Taken with SPI slowed to 2.6 MHz so the analyzer could decode it |
| Once-per-second `f_sync` pass | **5.3 ms** at 10.5 MHz SPI | Fits inside the 10 ms budget |

### I2C sensor read, decoded

Start, address `0x68` + write, register `0x3B`, repeated start, `0x68` + read, then 14 data bytes. Every byte is acknowledged except the last one, which gets a NACK, followed by Stop. This matches `mpu_read_burst()` line for line, including the timing-sensitive NACK on the final byte.

![Start of an I2C burst read, decoded](docs/captures/01b_i2c_decoded_start.png)
![End of the burst read: last byte NACKed, then Stop](docs/captures/01c_i2c_decoded_end_nack.png)

### SD card block write, decoded

`58` is CMD24 ("write one block"), followed by the block number and the `FE` start token. After that comes the CSV text itself, in ASCII. The card answers `E5` ("data accepted"), then holds its output low while it writes to flash.

![SPI block write to the SD card, decoded](docs/captures/03c_spi_write_decoded.png)

### Loop timing

The debug pin pulses every 10 ms. The wide pulse is the once-per-second `f_sync`: one sensor read plus three SD writes (three dips on CS), 5.3 ms in total.

![Loop timing with the once-per-second f_sync](docs/captures/02_loop_timing_fsync_10MHz.png)

### Why this matters

The loop fits its budget today, with 4.7 ms to spare. But the SD spec allows a single write to stall for up to ~250 ms. In a single loop, a stall like that would stop sampling and lose about 25 readings. The next step fixes this by reading the sensor from a timer interrupt into a ring buffer, so a slow card delays storage instead of measurements.

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F411RE | Cortex-M4F, 84 MHz, on-board ST-Link for flashing and serial |
| Adafruit MPU-6050 breakout | ±8 g accelerometer, ±2000 °/s gyro |
| microSD card breakout | SPI, tested with a SanDisk High Endurance 128 GB card (exFAT) |

### Wiring

| Signal | STM32 pin | Nucleo header |
|---|---|---|
| I2C1 SCL to MPU-6050 SCL | PB8 | D15 |
| I2C1 SDA to MPU-6050 SDA | PB9 | D14 |
| SPI1 SCK to SD CLK | PA5 | D13 |
| SPI1 MISO from SD DO | PA6 | D12 |
| SPI1 MOSI to SD DI | PA7 | D11 |
| SD chip select | PB6 | D10 |
| Debug timing pin (logic analyzer) | PA8 | D7 |
| USART2 TX/RX (debug, 115200 baud) | PA2 / PA3 | Via ST-Link USB |
| MPU-6050 VIN | Power | 3V3 |
| MPU-6050 GND | Ground | GND |
| SD reader VCC | Power | 5V |
| SD reader GND | Ground | GND |

## Log format

```
ms,ax,ay,az,gx,gy,gz,roll_x100,pitch_x100
```

| Column | Meaning |
|---|---|
| `ms` | Milliseconds since power-on (1 ms interrupt clock) |
| `ax ay az` | Raw accelerometer counts: 4096 counts = 1 g |
| `gx gy gz` | Raw gyro counts with bias removed: 16.4 counts = 1 °/s |
| `roll_x100`, `pitch_x100` | Filtered angles × 100 (4523 means 45.23°) |

Angles are stored as integers × 100 because FatFS's `f_printf` can't print floats.

## Build and run

1. Open the project in **STM32CubeIDE** and build.
2. FatFS settings in `ffconf.h`: `FF_FS_EXFAT 1`, `FF_USE_LFN 1`, `FF_CODE_PAGE 437`, `FF_FS_NORTC 1`, `FF_USE_STRFUNC 1`.
3. Flash the Nucleo with a card inserted. Startup status prints over the ST-Link serial port at 115200 baud.
4. **Stop logging (unplug) before re-flashing.** See Limitations.

## Plot a log

```
pip install pandas matplotlib numpy
python tools/plot_log.py docs/filter_demo.csv
```

This saves a PNG next to the CSV comparing the accelerometer-only, gyro-only and filtered angles.

## Bugs I tracked down

Every real bug is written up with its symptom, root cause, fix and lesson. Some highlights:

- **Peripherals were set up before the clock changed.** I2C and UART were configured for 16 MHz, then the PLL switched the bus to 42 MHz. My only passing test printed *before* the switch, so it proved nothing.
- **The I2C BUSY flag latched high at power-on**, a documented F4 erratum. I found it by adding a timeout to every wait and printing which step failed, then fixed it with a peripheral software reset.
- **SD reads gave up after 1 ms**, but the spec allows 100 ms. The card was left mid-transfer, and on the next boot `CMD0` read its leftover data as `0x00`.
- **The sensor came back from a power cut asleep.** It still answered on I2C, so the code treated it as working. Now an all-zero accelerometer reading counts as a failure, because an awake accelerometer always feels gravity.
- **Logging raw data revealed gyro saturation** at ±500 °/s: samples stuck at exactly ±32767. I raised the range to ±2000 °/s.
- **The logic analyzer broke the bus it was measuring.** With its USB unplugged, its inputs pulled the SPI lines down, and the SD card replied `0xC1`. That reply is impossible, since a valid one always starts with a 0 bit. The rule now: power the analyzer first, then clip it on.

## Limitations

- **No yaw (heading).** Correcting it needs a magnetometer, which the MPU-6050 doesn't have.
- **Roll and pitch use separate filters**, so they break down near straight up or down and during full flips. Pitch is clamped to ±90°. A quaternion filter would fix this.
- **A card interrupted mid-write can stay stuck until it is power-cycled.** Firmware can't recover it, so the planned PCB adds a switch on the card's power.
- **Sampling and SD writes share one loop**, so a long SD stall would delay readings. The ring buffer below fixes this.
- **SPI runs at 10.5 MHz**, limited by the jumper wires.

## Roadmap

- **Timer interrupt + ring buffer**, so SD stalls can't delay sampling
- **FreeRTOS** tasks for sensing, filtering and logging
- **DMA** for the I2C read, freeing the ~1.5 ms the CPU currently spends waiting on it
- **Custom KiCad shield** that plugs onto the Nucleo, replacing the jumper wires, with a GPIO-controlled power switch for the SD card
- **Quaternion orientation filter**
