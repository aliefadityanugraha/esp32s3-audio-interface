# ESP32-S3 N16R8 Hybrid USB Audio Interface + DLMS 2-In 4-Out 🎛️

[![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange.svg)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20ESP32-blue.svg)](https://github.com/espressif/arduino-esp32)
[![Chip](https://img.shields.io/badge/Chip-ESP32--S3%20N16R8-red.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![USB Audio](https://img.shields.io/badge/USB%20Audio-UAC2%2048kHz%2016--bit-brightgreen.svg)]()
[![Unit Tests](https://img.shields.io/badge/Native%20Unity%20Tests-6%2F6%20Passing-success.svg)]()

Sistem audio profesional berbasis **ESP32-S3 N16R8 (16 MB QIO Flash + 8 MB OPI PSRAM)** yang menggabungkan **Full-Duplex USB Audio Interface (UAC2 48 kHz 16-bit)** dengan **Digital Loudspeaker Management System (DLMS) 2-In 4-Out** real-time 32-bit floating-point, layar **TFT ILI9341 2.0" (240×320, 30 FPS PSRAM double-buffering)**, kontrol **BLE NUS & WiFi Web App (`AU-WEB` / `AU-OTA`)**, serta **Native PC Unity Unit Tests**.

---

## 🌟 Fitur Utama

### 1. Hardware & Audio I/O Layer (Core 1 — Real-Time)
- **USB Audio Class 2.0 (UAC2)** Full-Duplex 48 kHz 16-bit Stereo (plug-and-play Windows, macOS, Linux, Android, iOS) melalui **Native USB OTG** (`GPIO 19/20`).
- **Adaptive Ring Buffer** dengan kompensasi clock-drift berbasis interpolasi linear (mencegah klik/pop akibat drift XTAL PC vs ESP32-S3).
- **Dual-I2S 100% Sinkron Fase (0-Sample Skew)**:
  - `I2S_NUM_0` (**Master Full-Duplex**): Membangkitkan `MCLK 12.288 MHz` (`256fs`), `BCLK 3.072 MHz`, `LRCK 48 kHz`; membaca 2 kanal ADC **PCM1808** (`GPIO 7`) dan mengirim OUT 1–2 ke **DAC1 PCM5102A** (`GPIO 6`).
  - `I2S_NUM_1` (**Slave TX**): Menerima `BCLK` & `LRCK` internal via **GPIO Matrix** (`GPIO_MODE_INPUT_OUTPUT`) tanpa jumper tambahan, mengirim OUT 3–4 ke **DAC2 PCM5102A** (`GPIO 21`).
  - Mendukung ESP-IDF v5.x (`driver/i2s_std.h`) dan ESP-IDF v4.x (`driver/i2s.h`).
- **Monitoring Suhu Chip Internal ESP32-S3** secara periodik dengan proteksi thermal warning (`≥ 75°C`).

### 2. 32-Bit Float DLMS 2-In 4-Out DSP Engine (Core 1)
- **Proteksi Anti-Denormal & Subnormal**: Alternating micro-offset (`±1e-25f`) + Flush-To-Zero (`< 1e-20f`) di seluruh state Biquad, Pink Noise, dan output agar bebas lonjakan CPU maupun bunyi *"tek"* saat hening.
- **Input Stage (`IN A` & `IN B`)**:
  - Gain (`-12` s.d. `+12 dB`), Mute, Phase Invert (`0° / 180°`).
  - **Noise Gate** (Threshold, Attack, Release, Hold, Hysteresis).
  - **9-Band Parametric EQ (PEQ)** Biquad IIR (presisi koefisien `double`, bypass 0% CPU saat `0 dB`).
  - **Compressor** (Threshold, Ratio `1:1–20:1`, Soft Knee `0–12 dB`, Attack, Release, Makeup Gain `0–18 dB`, & Gain Reduction real-time).
- **Signal Generator & Routing Matrix 4×4**:
  - Generator internal **White Noise** & **Pink Noise (7-pole Paul Kellet)** dengan **timer auto-off 30 detik** (1.440.000 sampel).
  - **Routing Matrix 4×4** fleksibel (`IN A`, `IN B`, `USB L`, `USB R` → `OUT 1–4`):
    - Default **OUT 1–2**: Mix monitor `IN 1–2` + `USB Playback` dari PC.
    - Default **OUT 3–4**: Direct/Bypass dari `IN 1–2`.
- **Output Stage (`OUT 1`, `OUT 2`, `OUT 3`, `OUT 4`)**:
  - Gain (`-12..+12 dB`), Mute, Phase Invert, Bypass DSP.
  - **Crossover HPF & LPF**: **Linkwitz-Riley**, **Butterworth**, dan **Bessel** (slope **12, 24, dan 48 dB/oct** kaskade biquad).
  - **9-Band PEQ** (`16 Hz – 20 kHz`, resolusi `0.5 Hz`, gain `±15 dB`, `Q 0.4 – 12.0`).
  - **Delay Line hingga 300 ms** per kanal (14.400 sampel @ 48 kHz) dialokasikan di **8 MB OPI PSRAM (`ps_malloc`)** dengan fast integer-delay path & anti-zipper slew.
  - **Output Auto-Gate** (default `-50 dB`), **Peak Limiter** (+ GR real-time), & **Hard/Soft Anti-Clip Limiter**.
- **Master, Metering, Helper & RTA**:
  - Master Volume (`0` s.d. `-60 dB` smooth ramp), Peak/RMS VU 6-kanal, flag Clip/Limiter, `copyChannelSettings(src, dst|ALL)`, dan **FFT Radix-2 256/512-point** (31/32-Band ISO RTA) non-blocking di Core 0.

### 3. UI Layar TFT ILI9341 2.0" & Kontrol Fisik (Core 0 — 30 FPS)
- **LovyanGFX + 2× Sprite PSRAM Double-Buffering**: Rendering 30 FPS bebas flicker dengan cache kurva EQ/Xover.
- **7 Tema Warna UI**: `RED`, `ICE LUXURY`, `UNGU-EMAS`, `MERAH-BIRU`, `MERAH-PUTIH`, `BIRU-KUNING`, dan **`RGB PELANGI`** dinamis.
- **4 Mode Warna Hardware Panel**: Normal, Negatif (Invert), Tukar R-B (RGB/BGR swap), Negatif+Tukar.
- **Boot & Standby Logo** dari LittleFS (`/logo_boot.bin`, `/logo_standby.bin`) + **Timer Auto-Standby (`5–600 dtk`)** + **Layar Peringatan Suhu Merah (`≥ 75°C`)**.
- **Navigasi Encoder & Panel Lock**:
  - Putar: Navigasi / ubah nilai (dengan akselerasi).
  - Klik (`< 350 ms`): Pilih / edit; Tahan singkat (`< 1 dtk`): Kembali; Tahan 2 detik di home: Quick-Edit kanal aktif.
  - **Panel Lock**: Blokir semua tombol, buka kunci dengan **tahan encoder 3 detik**.
- **LED RGB Status (`GPIO 48`)**: Animasi pelangi saat boot; redup saat Mute; menyala terang untuk indikator **Clip** (merah), **Limiter** (oranye/kuning), dan **Compressor** (cyan/biru).

### 4. Konektivitas Radio Eksklusif & Web App Mandiri (`data/index.html`)
- **Mode Default (`BLE NUS`)**: Nordic UART Service dengan Nama Perangkat & **PIN Pairing 6-digit** tersimpan di NVS (default `ESP32-S3-DLMS` / PIN `123456`). WiFi mati total agar bebas RF noise & hemat RAM.
- **Mode WiFi Web (`AU-WEB`)**: Deinit BLE penuh, menyalakan AP **`AU-WEB`** (`192.168.4.1`, WebSocket port `81`), menyajikan aplikasi web single-file offline `data/index.html` dengan **12 Tab penuh layar**, 4 pilihan tema web (default **Studio Carbon & Amber**), grafik drag-and-drop **Crossover + 9-Band PEQ**, kurva **Compressor/Limiter/Gate**, **32-Band RTA**, dan **Konverter & Uploader Logo RGB565 di browser**.
- **Mode WiFi OTA (`AU-OTA`)**: Mute halus + suspend aman Audio DSP Core 1, deinit BLE, menyalakan portal web **`AU-OTA`** (`192.168.4.1`) untuk upload Firmware `.bin` (`Update.h`), Boot/Standby Logo `.bin`, dan `index.html`.

---

## 📐 Arsitektur Dual-Core & Alur Sinyal

```text
=========================== CORE 1 (PRIO 6 - REAL-TIME AUDIO) ===========================
PCM1808 ADC (GPIO 7) ─▶ DC Blocker ─▶ [INPUT STAGE: Gain/Mute/Phase/Gate/9-PEQ/Comp] ─▶ USB TX (PC)
                                                         │
USB RX (PC) ─▶ Adaptive Ring Buffer ─────────────────────┼─▶ [4x4 ROUTING MATRIX + NOISE GEN]
                                                         ▼
                                    [OUTPUT STAGE 1..4: Gain/Mute/Phase/Xover(LR/BW/BES)
                                     9-PEQ/PSRAM Delay 300ms/Auto-Gate/Peak Limiter/Master/Anti-Clip]
                                                   │                          │
                                         I2S0 TX (GPIO 6)           I2S1 TX Slave (GPIO 21)
                                          DAC1 PCM5102A              DAC2 PCM5102A (0-Skew)
                                            (OUT 1–2)                      (OUT 3–4)
=========================== CORE 0 (UI 30FPS + RADIO + RTA) =============================
 • UiTft_Core0 (Prio 2) : ILI9341 2x PSRAM Sprite, Encoder/Lock, LED GPIO 48, FFT 512-pt RTA, Temp
 • Conn_Core0  (Prio 1) : BLE NUS (Default) <-> AU-WEB (HTTP+WS) <-> AU-OTA + Command Parser
```

---

## 🔌 Pinout Hardware (`include/pin_config.h`)

| Subsistem | Sinyal | GPIO ESP32-S3 | Keterangan |
|---|---|---|---|
| **I2S Shared Clock** | `BCLK` | `GPIO 4` | Bit Clock 3.072 MHz (Master I2S0 → ADC, DAC1, DAC2 & I2S1 internal) |
| **I2S Shared Clock** | `LRCK / WS` | `GPIO 5` | Word Select 48 kHz (100% sinkron fase) |
| **I2S Shared Clock** | `MCLK` | `GPIO 15` | Master Clock 12.288 MHz (256fs untuk PCM1808) |
| **ADC PCM1808** | `DOUT` | `GPIO 7` | Audio IN 1–2 (`FMT=GND, MD0=GND, MD1=GND`) |
| **DAC1 PCM5102A** | `DIN` | `GPIO 6` | Audio OUT 1–2 (`SCK=GND` atau ke MCLK) |
| **DAC2 PCM5102A** | `DIN` | `GPIO 21` | Audio OUT 3–4 (`SCK=GND` atau ke MCLK) |
| **LCD ILI9341 (SPI)** | `SCLK / MOSI` | `GPIO 12 / 13` | SPI2_HOST 40 MHz + DMA |
| **LCD ILI9341 (SPI)** | `CS / DC / RST` | `GPIO 10 / 9 / 8` | Layar TFT 2.0" 240×320 |
| **Rotary Encoder** | `CLK / DT / SW` | `GPIO 16 / 17 / 18` | EC11 Quadrature + Push Switch (Active LOW) |
| **LED Status** | `RGB WS2812` | `GPIO 48` | LED RGB bawaan ESP32-S3 DevKitC-1 |
| **Native USB OTG** | `D- / D+` | `GPIO 19 / 20` | Dedicated USB Audio UAC2 (jangan dipakai GPIO lain) |

> **Catatan Pin Internal N16R8**: `GPIO 26–32` (16MB QIO Flash) dan `GPIO 33–37` (8MB OPI PSRAM) dicadangkan hardware dan tidak digunakan oleh periferal eksternal.

---

## 💾 Peta Partisi Flash 16 MB (`partitions_16mb.csv`)

| Partisi | Tipe / Subtipe | Offset | Ukuran | Fungsi |
|---|---|---|---|---|
| `nvs` | `data / nvs` | `0x009000` | 20 KB | Nama/PIN BLE, kalibrasi cepat |
| `otadata` | `data / ota` | `0x00E000` | 8 KB | Selektor boot slot OTA |
| `app0` | `app / ota_0` | `0x010000` | **3 MB** | Firmware OTA Slot 0 |
| `app1` | `app / ota_1` | `0x310000` | **3 MB** | Firmware OTA Slot 1 |
| `spiffs` | `data / spiffs` | `0x610000` | **8 MB** | **LittleFS** (`index.html`, logo `.bin`, 8 slot preset, config UI) |
| `coredump` | `data / coredump` | `0xE10000` | 64 KB | Diagnostik crash |

---

## 📡 Protokol Perintah (BLE NUS & WebSocket)

Berlaku identik melalui teks maupun JSON ringan (`{"cmd":"SET","param":"OUT1.DELAY","val":12.5}`):

| Perintah | Deskripsi |
|---|---|
| `SET <param> <val>` / `GET <param>` | Kontrol seluruh parameter `IN1/2.*`, `OUT1..4.*` (`GAIN`, `MUTE`, `PHASE`, `PEQ1..9.*`, `HPF.*`, `LPF.*`, `DELAY`, `GATE.*`, `COMP.*`, `LIM.*`, `CLIP`), `MATRIX.*`, `GEN.*`, `RTA.*` |
| `DUMP` | Kirim seluruh konfigurasi aktif dalam JSON ringkas untuk sinkronisasi Web/Mobile App |
| `PRESET <LOAD\|SAVE\|DEL\|RENAME> <slot>` | Kelola slot preset `0..7` di LittleFS |
| `THEME <0-6>` / `COLORMODE <0-3>` | Ganti tema UI & mode warna hardware ILI9341 secara live |
| `STANDBY <detik>` / `LOCK <0\|1>` | Atur timer Auto-Standby (`0`=OFF, `5..600`s) & Panel Lock |
| `MASTER <dB>` / `LINK <mask>` / `COPY <src> <dst\|ALL>` | Master Volume (`-60..0` dB), Stereo Link (`NONE/IN/OUT12/OUT34/OUT/ALL`), Copy kanal |
| `LOGO <BOOT\|STANDBY> <size>` | Upload logo RGB565 (`153.600` byte) via chunk raw binary / `LOGO_DATA <base64>` |
| `BLENAME <nama>` / `BLEPIN <6-digit>` | Ubah Nama Perangkat & PIN Pairing BLE (tersimpan di NVS) |
| `INFO` / `TELEM <ms>` / `RESTART` | Info sistem, atur interval stream telemetry (default `80 ms`, 6-ch VU + GR + 32-band RTA), reboot aman |

---

## 📁 Struktur Proyek

```text
vps-esp32s3/
├── platformio.ini              # [env:esp32-s3-n16r8-dlms] & [env:native]
├── partitions_16mb.csv         # Partisi custom 16MB (OTA 2x3MB + LittleFS 8MB)
├── data/
│   └── index.html              # Web App kontrol mandiri 12-Tab (Offline / BLE / AU-WEB)
├── include/
│   ├── pin_config.h            # Definisi pinout hardware ESP32-S3 N16R8
│   ├── audio_config.h          # Konstanta audio 48kHz, ring buffer, & thermal
│   ├── dsp_engine.h            # Header 32-bit Float DLMS 2-In 4-Out DSP Engine
│   ├── input_ctrl.h            # Header Encoder, Panel Lock, & LED RGB GPIO 48
│   ├── ui_tft.h                # Header LovyanGFX ILI9341 PSRAM Double-Buffer UI
│   ├── cmd_parser.h            # Header Command Parser & Telemetry JSON
│   └── connectivity.h          # Header Radio Manager (BLE NUS / AU-WEB / AU-OTA)
├── src/
│   ├── main.cpp                # Entry point Dual-Core FreeRTOS
│   ├── audio_io.h / .cpp       # Dual-I2S 100% Phase-Sync + UAC2 + Temp Sensor
│   ├── dsp_engine.cpp          # Implementasi DSP Engine + 512-pt FFT RTA
│   ├── input_ctrl.cpp          # Implementasi Encoder & WS2812 Status LED
│   ├── ui_tft.cpp              # Implementasi 30 FPS UI ILI9341 & Preset LittleFS
│   ├── cmd_parser.cpp          # Implementasi Parser Perintah & Logo Uploader
│   └── connectivity.cpp        # Implementasi BLE NUS, WebServer/WS, & Portal OTA
└── test/
    └── test_native_dsp/
        └── test_dsp_pure.cpp   # 6 Suite Unit Test Unity murni di PC (tanpa hardware)
```

---

## 🚀 Build, Flash & Unit Testing

```bash
# 1. Jalankan Unit Test DSP murni di PC (tanpa hardware ESP32-S3)
pio test -e native

# 2. Compile & Upload Firmware ke ESP32-S3 N16R8 (via port UART)
pio run -e esp32-s3-n16r8-dlms --target upload

# 3. Upload Image LittleFS (mengunggah data/index.html ke partisi 8 MB)
pio run -e esp32-s3-n16r8-dlms --target uploadfs

# 4. Monitor Serial Debug (UART0)
pio device monitor -b 115200
```

### Cara Menggunakan Aplikasi Web Kontrol
1. **Via BLE NUS (Default saat boot)**: Buka [data/index.html](file:///Users/macbook/vps-esp32s3/data/index.html) di Chrome/Edge → klik **Scan & Connect (BLE NUS)** → masukkan PIN `123456`.
2. **Via WiFi `AU-WEB`**: Di layar TFT masuk `SETUP > WIFI` → pilih **`AU-WEB`** → sambungkan HP/PC ke WiFi `AU-WEB` (password `12345678`) → buka `http://192.168.4.1`.
3. **Update Firmware & Logo (`AU-OTA`)**: Di layar TFT masuk `SETUP > WIFI` → pilih **`AU-OTA`** → sambungkan ke WiFi `AU-OTA` → buka `http://192.168.4.1`.
4. **Mode Offline**: Buka `data/index.html` langsung di browser dan klik **Masuk Tanpa Alat (Mode Offline)** untuk simulasi DSP, penyusunan kurva EQ/Xover, dan konversi gambar logo RGB565.
