# IMU Logger: STM32 + FreeRTOS

A motion logger built on an STM32F411 (NUCLEO-F411RE). It reads an MPU-6050 accelerometer and gyroscope over I2C, combines them into roll and pitch angles with a complementary filter, and saves 100 samples per second to a microSD card as CSV files.

Every peripheral driver (GPIO, UART, I2C, SPI and the SD card protocol) is written **at the register level**, straight from the STM32 reference manual (RM0383). There is no ST HAL in the data path. The firmware runs as **four FreeRTOS tasks joined by queues**, so a slow SD card can never delay a sensor reading, and both data paths use **DMA**: the CPU sleeps while the sensor bytes come in and while each 512-byte block goes out to the card. The only libraries are [FreeRTOS](https://www.freertos.org/) and [FatFS](http://elm-chan.org/fsw/ff/) (connected through my own `diskio.c`).

![Accelerometer vs gyro vs complementary filter](docs/filter_demo.png)

Taps swing the raw accelerometer angle by ±150°, while the filtered angle barely moves. At rest, the gyro-only angle drifted 20° in 38 s, and the filter stayed within 0.1° of the accelerometer. Plotted from a raw CSV log with `tools/plot_log.py`.

## Features

- **Four FreeRTOS tasks**, joined by queues. Sampling, filtering, CSV formatting and SD writing run separately, ranked by how urgent they are (see Architecture).
- **DMA on both buses** (register level). I2C1 → DMA1 Stream 5 for the 14-byte sensor read, DMA2 Stream 3 → SPI1 for SD card blocks. The task starts the transfer and sleeps; the DMA-complete interrupt wakes it with a FreeRTOS task notification. No polling loops in the data path.
- **Ping-pong (double) buffering** for the SD card: one 512-byte block is filled with CSV text while the other is being written. The blocks are swapped by passing an index through a queue, never copied.
- **100 Hz sampling with no drift.** `vTaskDelayUntil` wakes the sensor task exactly every 10 ms. In a 98 s test log, **8,297 of 8,299 sample gaps were exactly 10 ms**, with zero samples dropped.
- **Burst read** of all 14 MPU-6050 data bytes in one I2C transaction, so every axis comes from the same instant.
- **Complementary filter** for roll and pitch: the gyro is trusted short-term, the accelerometer long-term. It starts at the accelerometer angle, so there is no warm-up.
- **Vibration filter in the sensor.** The MPU-6050's digital low-pass filter is set to 44 Hz (accel) / 42 Hz (gyro). Sampling at 100 Hz can only represent motion below 50 Hz; faster vibration (motors, props) would otherwise alias into a fake slow wobble. It has to be filtered before sampling, so it's done inside the sensor.
- **Gyro bias measured at power-on.** The first 2 s of samples are averaged to find each axis's zero-rate offset, which drifts with temperature. If the board moves during that window (any axis swings more than ~3 °/s) it tries again, up to 5 times, then falls back to stored values. Logging runs the whole time. First run measured −92 / −47 / −11 counts against the old hard-coded −101 / −52 / −1: a 0.3–0.6 °/s difference that would have drifted a gyro-only angle by ~20–35° per minute.
- **CSV logging to microSD** (exFAT). Each boot creates a new `LOGnnn.CSV`. Data goes to the card in 512-byte blocks, and the file is saved once per second, so a power cut loses about the last second of data.
- **Fault recovery.** Every I2C wait has a timeout. If a sensor wire comes loose or the sensor loses power, the logger re-initialises it and keeps going, leaving a visible gap in the timestamps.
- **Live health stats** over the serial port every second (queue backlog, dropped samples, sensor faults), plus each task's stack usage every 10 s.
- **Stack overflow detection.** If any task runs past the end of its stack, the board prints the task's name and halts, instead of silently corrupting memory.
- **Raw data is logged**, so calibration and filtering can be redone on a PC later.
- **Timing verified with a logic analyzer** (see below).

## Architecture

```
imu_task ──[imu_q, 16]──▶ filter_task ──[log_q, 64]──▶ sd_task ──[full_q]──▶ sdw_task ──▶ microSD
 prio 40                     prio 32                   prio 16  ◀──[free_q]──  prio 8
 never waits                 never waits               formatter               writer
 I2C via DMA                                           (ping-pong blocks)      SPI via DMA
```

| Task | Priority | Job | Sleeps until |
|---|---|---|---|
| `imu_task` | 40 (highest) | Burst-reads the MPU-6050. Re-initialises the sensor itself if a read fails | The next 10 ms slot (`vTaskDelayUntil`) |
| `filter_task` | 32 | Raw bytes → g, °/s, roll and pitch | A sample arrives in `imu_q` |
| heartbeat | 24 | Toggles a pin every 250 ms, to show the scheduler is alive | 250 ms pass |
| `sd_task` | 16 | Formats rows as CSV text into the current 512-byte block. Hands full blocks to the writer. Prints stats every 1 s | A row arrives in `log_q`, or the next report is due |
| `sdw_task` | 8 (lowest) | The only task that touches FatFS/SPI. Writes each full block (SPI DMA reads straight from the block), returns it as empty, saves the file every 1 s | A full block arrives in `full_q`, or the next save is due |

**Design rules:**

- **Senders never wait.** If a queue is full, the item is dropped and counted, so sampling can't be held up by anything downstream.
- **Receivers wait forever** (`portMAX_DELAY`), so an idle task uses no CPU.
- **Only `imu_task` touches I2C.** It can reset the bus after a failed read with no locking, because nothing else can be mid-transfer.
- **SD writing is lowest priority.** When the card stalls, rows pile up in the 64-slot `log_q` (0.64 s of cushion) instead of delaying samples.
- **Only `sdw_task` touches FatFS and SPI**, for the same reason.
- **Every file write is a whole, aligned 512-byte block** (the CSV header goes into the first block instead of being written on its own). That lets FatFS pass our buffer straight to `disk_write`, so the DMA reads from it directly with no extra copy.

### Stack sizing, from measurement

Each task gets a fixed block of RAM for its stack. The first sizes were guesses. Then `uxTaskGetStackHighWaterMark` measured the least free stack each task ever had, and the stacks were resized to the measured use plus a safety margin.

| Task | First guess (words) | Measured peak use | Now | Margin |
|---|---|---|---|---|
| `imu_task` | 256 | 44 | **128** | 2.9× |
| `filter_task` | 512 | 96 | **256** | 2.7× |
| `sd_task` | 1024 | ~132 | **512** | 3.9×, extra because the SD card's heaviest path may not have run during the test |
| `sd_task` / `sdw_task` (after the DMA split) | — / 512 | 78 / 119 | **256 / 384** | 3.3× / 3.2× |
| heartbeat | 128 | 28 | 128 | 4.6× |

The guesses were 4–8× too big. Resizing freed **3.5 KB**: the FreeRTOS heap's minimum free space went from 5,440 B to **9,024 B**, matching the prediction exactly. `configCHECK_FOR_STACK_OVERFLOW = 2` catches overflows at runtime.

## DMA: how the transfers work

A DMA stream is a small copier that runs beside the CPU. It needs three things: **from**, **to**, and **how many**. The peripheral tells it *when* to copy each byte, and it raises an interrupt when the count reaches 0.

| | Sensor read | SD block write |
|---|---|---|
| Stream | DMA1 Stream 5, channel 1 (I2C1_RX) | DMA2 Stream 3, channel 3 (SPI1_TX) |
| From → to | `I2C1->DR` → `buf` | `blk[i]` → `SPI1->DR` |
| Address increment | memory only (`MINC`), never the peripheral | memory only (`MINC`) |
| Per-byte trigger | I2C1 sees `RXNE` (byte arrived) | SPI1 sees `TXE` (ready for next byte) |
| When done | ISR sends STOP, wakes `imu_task` | ISR wakes `sdw_task` |

Details that matter:

- **Sensor read:** the CPU still sends the register address (1 byte). Then `DMAEN` + `LAST` are set in `I2C1->CR2`. `LAST` makes the hardware NACK the final byte by itself, replacing the hand-timed BTF/ACK-off sequence of the polling version. STOP is issued in the DMA-complete interrupt. The I2C stream runs at *very high* DMA priority, because ST's errata warns the I2C DMA request must be served before the next byte arrives.
- **SD write:** "DMA done" means the last byte reached SPI1, not that it left the pin, so the driver then waits for `TXE`=1 and `BSY`=0. SPI receives a byte for every byte it sends; nobody reads those 512 replies, so `RXNE` and `OVR` are set afterwards and are cleared by reading `DR` then `SR`.
- **Tested on hardware.** `blk` rises 9–11 per second (~5.1 KB/s = 100 rows × ~51 B), with `wait 0`, `sderr 0` and zero dropped samples over a 60 s run. A 1,594-row log had every timestamp gap at exactly 10 ms and no corrupted rows. With the sensor unplugged for ~8 s, `fault` rose by exactly 100 per second (one failed DMA read and re-init per 10 ms slot) and logging resumed by itself on replug.
- **Before the scheduler starts** (`f_mount`, `f_open`) there are no tasks to wake, so `spi1_dma_send` turns the interrupt off and polls the DMA flag instead.
- **Interrupt priority 6.** A handler may only call FreeRTOS `...FromISR` functions if its priority number is ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` (5).
- **Card busy time:** after a block, the card holds MISO low while it programs flash (usually ~1 ms, sometimes 100+ ms). The writer spins for ~0.15 ms, then sleeps 1 ms between checks instead of burning the CPU.

## Verified with a logic analyzer

All measurements were taken with a 24 MHz, 8-channel logic analyzer and PulseView. Spare pins go high while specific work runs, so timing shows up next to the bus signals. Capture files (`.sr`, open in PulseView) and screenshots are in [`docs/captures/`](docs/captures/).

| What | Measured | Check |
|---|---|---|
| I2C clock | 10 µs period, **100 kHz** | Matches the configured speed |
| One 14-byte sensor read (Start to Stop) | **1.54 ms** | 17 bytes × 9 clocks × 10 µs = 1.53 ms |
| Sensor read period (FreeRTOS) | **10.000–10.001 ms** | After the clock fix below |
| Once-per-second save (`f_sync`) | **3.5–10.6 ms** | Varies with the card. The queues absorb it |
| One 512-byte SD block write | ~1.9 ms sending, then **~0.9 ms card busy** | Taken with SPI slowed to 2.6 MHz so the analyzer could decode it |

### Preemption: sampling never waits for the SD card

D0 = sensor read, D1 = SD save (`f_sync`), D2 = heartbeat. A sensor read fires right on schedule **in the middle of** a 10.6 ms SD save. FreeRTOS pauses the low-priority save, runs the high-priority read, then resumes the save where it left off.

![Sensor read preempting an SD save](docs/captures/04a_freertos_preemption.png)

### Clock accuracy

The chip originally ran on its internal RC oscillator (HSI). On the analyzer, 20 sample periods took **197 ms** instead of 200 ms: the whole board was running **1.5% fast**. The firmware couldn't notice, because its own timestamps are measured with the same clock. After switching to the 8 MHz crystal-derived clock from the on-board ST-Link (HSE bypass, PLL → 84 MHz), 20 periods measure **200.11 ms**.

![20 sample periods = 200.11 ms](docs/captures/04b_clock_hse_20_periods.png)

### Sensor unplug test

With the MPU-6050 unplugged mid-run, the fault counter climbed by 100 per second (one retry every 10 ms). The log shows a **14,900 ms gap**, and the fault counter read **1,489** (× 10 ms = 14.89 s). Logging resumed in the same file with clean 10 ms steps.

### Bus traffic, decoded

**I2C sensor read.** Start, address `0x68` + write, register `0x3B`, repeated start, `0x68` + read, then 14 data bytes. Every byte is acknowledged except the last one, which gets a NACK, followed by Stop. This matches `mpu_read_burst()` line for line, including the timing-sensitive NACK on the final byte.

![Start of an I2C burst read, decoded](docs/captures/01b_i2c_decoded_start.png)
![End of the burst read: last byte NACKed, then Stop](docs/captures/01c_i2c_decoded_end_nack.png)

**SD card block write.** `58` is CMD24 ("write one block"), followed by the block number and the `FE` start token. After that comes the CSV text itself, in ASCII. The card answers `E5` ("data accepted"), then holds its output low while it writes to flash.

![SPI block write to the SD card, decoded](docs/captures/03c_spi_write_decoded.png)

### Before FreeRTOS: why it was needed

The bare-metal version ran everything in one 10 ms loop. Its once-per-second `f_sync` pass took 5.3 ms, which fit. But the SD spec allows a single write to stall for up to ~250 ms, and in a single loop that would stop sampling and lose about 25 readings. That's what the task split fixes.

![Single-loop timing with the once-per-second f_sync](docs/captures/02_loop_timing_fsync_10MHz.png)

## Hardware

| Part | Notes |
|---|---|
| NUCLEO-F411RE | Cortex-M4F, 84 MHz, on-board ST-Link for flashing, serial and the 8 MHz clock |
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
| Debug pin: high during each sensor read | PA8 | D7 |
| Debug pin: high during each SD save (`f_sync`) | PA9 | D8 |
| Debug pin: heartbeat, toggles every 250 ms | PA10 | D2 |
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
| `ms` | Milliseconds since the scheduler started (FreeRTOS tick, taken at each read) |
| `ax ay az` | Raw accelerometer counts: 4096 counts = 1 g |
| `gx gy gz` | Raw gyro counts with bias removed: 16.4 counts = 1 °/s |
| `roll_x100`, `pitch_x100` | Filtered angles × 100 (4523 means 45.23°) |

Angles are stored as integers × 100, which keeps the CSV formatter a tiny integer-to-text routine (no floats, no `printf`).

## Serial output

```
file LOG040.CSV open 0
tasks running
imuq 1 logq 1 drop 0/0 fault 0 blk 9 wait 0 sderr 0
imuq 1 logq 1 drop 0/0 fault 0 blk 19 wait 0 sderr 0
gyro bias measured: -92 -47 -11
...
stack free (words): imu 84 filter 160 sd 178 sdw 265 hb 100 | heap min free (bytes): 8216
```

`imuq` / `logq` are the worst backlog seen in each queue, `drop` counts samples/rows thrown away because a queue was full, and `fault` counts failed sensor reads. `blk` is how many 512-byte blocks have been written, `wait` counts times the formatter found both blocks busy (the writer fell behind), and `sderr` counts failed writes/saves.

## Build and run

1. Open the project in **STM32CubeIDE** and build. FreeRTOS is configured through STM32CubeMX (CMSIS_V2 interface, HAL timebase on TIM1, HSE bypass clock).
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

- **The whole board ran 1.5% fast, and the firmware couldn't tell.** Its timestamps were measured with the same internal RC clock, so the CSV looked perfect. Only the logic analyzer, with its own crystal, showed 20 periods taking 197 ms. Switching to the ST-Link's crystal-derived 8 MHz fixed it.
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
- **The 8 MHz clock comes from the ST-Link chip.** Running without the ST-Link powered (on battery, for example) still needs checking.
- **SPI runs at 10.5 MHz**, limited by the jumper wires.

## Roadmap

- **Custom shield PCB** that plugs onto the Nucleo, replacing the jumper wires, with a GPIO-controlled power switch for the SD card and a socket for an ESP32-C6 radio
- **Over-the-air firmware updates** through the ESP32-C6 (Wi-Fi), then mounting the logger on a drone
- **Quaternion orientation filter**
