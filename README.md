# ESP32-S3 USB Audio Interface 🎛️

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20ESP32-blue.svg)](https://github.com/espressif/arduino-esp32)
[![Chip](https://img.shields.io/badge/Chip-ESP32--S3-red.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![USB Audio](https://img.shields.io/badge/USB%20Audio-UAC2%2048kHz%2016--bit-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)]()

A high-performance, full-duplex USB Audio Interface powered by the **ESP32-S3** microcontroller. It turns an ESP32-S3 into a plug-and-play USB Audio Class (UAC2) sound card featuring studio-grade I2S ADC/DAC integration, real-time hardware-accelerated DSP equalization, an interactive 128x64 OLED display, and rotary encoder navigation.

---

## 🌟 Key Features

- **Full-Duplex USB Audio (UAC2)**:
  - 48 kHz sample rate, 16-bit depth, 2-channel stereo input & output.
  - Driverless plug-and-play compatibility on **Windows, macOS, Linux, Android, and iOS**.
  - Built-in adaptive ring buffer (jitter buffer) with real-time drift compensation to eliminate audio glitching and sample slips.
- **Hardware I2S Master Clock (MCLK)**:
  - Generates a clean 12.288 MHz MCLK (256 × Fs) directly from ESP32-S3 GPIO 16, eliminating the need for external oscillators on ADCs like the **PCM1808**.
- **Real-Time DSP Engine**:
  - **DC Blocker Filter**: Removes ADC hardware DC offset from input streams.
  - **4-Channel 3-Band Biquad IIR Equalizer**:
    - Independent processing for **IN 1** (Mic L), **IN 2** (Mic R), **OUT 1** (DAC L), and **OUT 2** (DAC R).
    - **Low Shelf**: 120 Hz (±12 dB)
    - **Peaking / Mid**: 1000 Hz, Q=1.0 (±12 dB)
    - **High Shelf**: 6000 Hz (±12 dB)
    - **Zero CPU Overhead**: Transparent bypass mode when bands are set to 0 dB.
  - **Gain Smoothing**: Artifact-free volume attenuation and muting without clicks or pops.
- **Flexible Audio Routing**:
  - **Input Modes**:
    - `Stereo In`: Discrete L & R channels from ADC.
    - `Mono L In`: Duplicates Mic Left to both L & R (ideal for single microphone setups).
    - `Mono R In`: Duplicates Mic Right to both L & R.
    - `2 Mono In`: Mixes `(L + R) / 2` and broadcasts to both channels.
  - **Output Modes**:
    - `Stereo Out`: True stereo playback to DAC.
    - `Mono Mix`: Summed mono `(L + R) / 2` to both DAC channels.
- **Dual-Core FreeRTOS Architecture**:
  - **Core 1 (High Priority - Audio DSP Task)**: Handles I2S DMA transfers, USB audio streams, ring buffer management, and DSP biquad filtering.
  - **Core 0 (UI & System Task)**: Renders OLED graphics at ~30 FPS, handles rotary encoder inputs, and performs non-blocking NVS flash commits.
- **Interactive 128x64 OLED Display (SSD1306)**:
  - **Page 1 (Device Status)**: USB connection state, active stream status (`REC` / `PLAY`), routing configuration, volume percentage, and visual volume bar.
  - **Page 2 (VU Meters)**: 4 independent real-time VU level meters (`IN-L`, `IN-R`, `OT-L`, `OT-R`) with smooth logarithmic decay and dB scale ticks (-40dB, -12dB, 0dB).
  - **Page 3 (Routing Menu)**: Quick routing switching for input and output.
  - **Page 4 (Graphic Equalizer)**: Channel selector and graphical boost/cut sliders for Bass, Mid, and Treble.
- **Persistent Storage (NVS)**:
  - All configurations (Master Volume, Input Routing, Output Routing, and 4-Channel EQ values) are automatically saved to internal flash with a 1.5-second debounce.

---

## 📐 Signal Flow Diagram

```text
       ┌────────────────────────┐
       │     PCM1808 (ADC)      │
       └───────────┬────────────┘
                   │ I2S RX (MCLK: 12.288 MHz)
                   ▼
         ┌───────────────────┐
         │    DC Blocker     │
         └─────────┬─────────┘
                   ▼
         ┌───────────────────┐
         │  EQ IN 1 & IN 2   │
         └─────────┬─────────┘
                   ▼
       ┌──────────────────────┐
       │ Input Routing Matrix │ (Stereo / Mono L / Mono R / Dual Mono)
       └───────────┬──────────┘
                   ▼
         ┌───────────────────┐
         │ Smooth Master Vol │
         └─────────┬─────────┘
                   │
                   ▼ USB Audio IN (Tx)
         ┌───────────────────┐
         │     Host PC       │
         └─────────┬─────────┘
                   │ USB Audio OUT (Rx)
                   ▼
         ┌───────────────────┐
         │ Jitter RingBuffer │ (Drift Correction & Underrun Guard)
         └─────────┬─────────┘
                   ▼
       ┌──────────────────────┐
       │ Output Routing Matrix│ (Stereo / Mono Mix)
       └───────────┬──────────┘
                   ▼
         ┌───────────────────┐
         │  EQ OUT 1 & OUT 2 │
         └─────────┬─────────┘
                   │ I2S TX
                   ▼
       ┌────────────────────────┐
       │  PCM5102A / I2S (DAC)  │
       └────────────────────────┘
```

---

## 🔌 Hardware Pinout & Wiring

### 1. I2S Audio Bus (ADC & DAC)

| Signal | ESP32-S3 Pin | PCM1808 (ADC) | PCM5102A (DAC) | Description |
|---|---|---|---|---|
| **MCLK** | `GPIO 16` | `SCK` | - / Optional | Master Clock (12.288 MHz) |
| **BCLK** | `GPIO 17` | `BCK` | `BCK` | Bit Clock (Continuous) |
| **LRCK / WS** | `GPIO 18` | `LRCK` | `LCK` | Word Select (48 kHz) |
| **DIN (Data In)** | `GPIO 14` | `DOUT` | - | Audio from ADC into ESP32 |
| **DOUT (Data Out)**| `GPIO 15` | - | `DIN` | Audio from ESP32 into DAC |

> [!NOTE]
> **PCM1808 Configuration**: Connect `FMT` to GND (I2S Format), `MD0` to GND, `MD1` to GND (Slave Mode). Supply +5V to the analog supply (`VCC`) and +3.3V to `VDD`.

### 2. OLED Display (0.96" SSD1306 I2C)

| OLED Pin | ESP32-S3 Pin | Description |
|---|---|---|
| **SDA** | `GPIO 8` | I2C Data |
| **SCL** | `GPIO 9` | I2C Clock |
| **VCC** | `3.3V` / `5V`| Power |
| **GND** | `GND` | Ground |

### 3. Rotary Encoder (EC11)

| Encoder Pin | ESP32-S3 Pin | Description |
|---|---|---|
| **CLK (A)** | `GPIO 5` | Quadrature Phase A |
| **DT (B)** | `GPIO 6` | Quadrature Phase B |
| **SW** | `GPIO 7` | Push Button Switch (Active LOW) |
| **+** | `3.3V` | Pull-up supply (if module has resistors) |
| **GND** | `GND` | Common ground |

### 4. USB Connection

- Connect the host computer or mobile device to the **Native USB port** of the ESP32-S3 (Pins `GPIO 19` [D-] and `GPIO 20` [D+]).
- Do **not** use the UART / CH340 / CP2102 programming port for USB Audio streaming.

---

## 🎮 Navigation & UI Controls

| Screen | Encoder Turn | Short Click | Long Press (> 600ms) |
|---|---|---|---|
| **Page 1: Device Status** | Adjust Master Volume (5% - 100%) | Switch to Page 2 (VU Meters) | Toggle **MUTE** |
| **Page 2: VU Meters** | Adjust Master Volume (5% - 100%) | Switch to Page 3 (Routing Menu) | Toggle **MUTE** |
| **Page 3: Routing Menu** | Move cursor up / down | Select / Cycle option / Enter EQ | Return to Page 1 |
| **Page 4: Graphic EQ** | Select band / Adjust dB gain | Toggle band edit mode / Cycle channel | Return to Page 1 |

---

## 🛠️ Software Requirements & Libraries

All required libraries are automatically managed by PlatformIO:

- **Framework**: `espressif32` @ `arduino`
- **[arduino-audio-tools](https://github.com/pschatzmann/arduino-audio-tools)**: Low-level audio streaming & USB Audio Class stack.
- **[U8g2](https://github.com/olikraus/u8g2)**: High-performance monochrome graphics driver for SSD1306 OLED.
- **[ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder)**: Hardware interrupt-based quadrature encoder reader.

---

## 🚀 Building & Flashing

### Using PlatformIO IDE (VS Code)

1. Clone this repository:
   ```bash
   git clone https://github.com/aliefadityanugraha/esp32s3-audio-interface.git
   cd esp32s3-audio-interface
   ```
2. Open the project folder in **Visual Studio Code** with the **PlatformIO IDE** extension installed.
3. Connect your ESP32-S3 via the UART port (or Native USB with bootloader mode).
4. Click **Build** (checkmark icon) or **Upload** (right arrow icon) in the bottom status bar.

### Using PlatformIO CLI

```bash
# Compile project
pio run

# Upload firmware
pio run --target upload

# Monitor Serial Output (Optional)
pio device monitor -b 115200
```

---

## ⚙️ PlatformIO Configuration (`platformio.ini`)

Key build flags configured for Native USB OTG and TinyUSB audio streaming:

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino

build_unflags = 
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1

build_flags = 
    -D ARDUINO_USB_MODE=0
    -D ARDUINO_USB_CDC_ON_BOOT=0
    -D CONFIG_TINYUSB_AUDIO_ENABLED=1
    -D CFG_TUD_AUDIO_EP_SZ_IN=192
    -D CFG_TUD_AUDIO_EP_SZ_OUT=192
    -D CONFIG_I2S_SUPPRESS_DEPRECATE_WARN=1
    -D CONFIG_PCNT_SUPPRESS_DEPRECATE_WARN=1
    -Wno-format
    -Wno-ignored-qualifiers
    -Wno-unused-variable

lib_deps = 
    https://github.com/pschatzmann/arduino-audio-tools.git
    olikraus/U8g2 @ ^2.34.22
    madhephaestus/ESP32Encoder @ ^0.11.2
```

---

## 📜 License

This project is open-source and released under the [MIT License](LICENSE).

---

## 👤 Author

Developed by **[Alief Aditya Nugraha](https://github.com/aliefadityanugraha)**.  
Feedback and contributions are welcome! Feel free to open an issue or pull request.
