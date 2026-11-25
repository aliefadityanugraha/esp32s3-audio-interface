#pragma once

/**
 * @file audio_config.h
 * @brief Konstanta Konfigurasi Audio, Buffer, dan Telemetri Sistem
 */

#include <cstddef>
#include <cstdint>

// ============================================================================
// 1. FORMAT AUDIO UTAMA
// ============================================================================
constexpr uint32_t AUDIO_SAMPLE_RATE      = 48000;  // 48 kHz studio standard
constexpr uint8_t  AUDIO_USB_BIT_DEPTH    = 16;     // UAC2 16-bit PCM
constexpr uint8_t  AUDIO_USB_CHANNELS     = 2;      // UAC2 Stereo (2-In / 2-Out)
constexpr uint8_t  AUDIO_IN_CHANNELS      = 2;      // ADC PCM1808 (IN 1-2)
constexpr uint8_t  AUDIO_OUT_CHANNELS     = 4;      // DAC1 (OUT 1-2) + DAC2 (OUT 3-4)

// Master Clock Multiplier: 256 * 48kHz = 12.288 MHz (wajib untuk PCM1808 256fs)
constexpr uint32_t AUDIO_I2S_MCLK_MULT    = 256;
constexpr uint32_t AUDIO_I2S_MCLK_HZ      = AUDIO_SAMPLE_RATE * AUDIO_I2S_MCLK_MULT;

// ============================================================================
// 2. KONFIGURASI BLOCK DSP & I2S DMA
// ============================================================================
// 128 stereo frames @ 48kHz = 2.667 ms per blok (latensi rendah & efisien untuk DSP)
constexpr size_t   AUDIO_BLOCK_FRAMES     = 128;
constexpr size_t   AUDIO_STEREO_SAMPLES   = AUDIO_BLOCK_FRAMES * 2;
constexpr size_t   AUDIO_DMA_BUF_COUNT    = 8;

// ============================================================================
// 3. ADAPTIVE RING BUFFER (USB PC -> ESP32 I2S Jitter & Drift Compensator)
// ============================================================================
// Kapasitas wajib pangkat 2 agar modulo menggunakan bitmask (& mask) tanpa divisi CPU
constexpr size_t   AUDIO_RB_FRAMES        = 2048;   // ~42.67 ms kapasitas total
constexpr size_t   AUDIO_RB_TARGET_FILL   = 512;    // ~10.67 ms target steady-state (4 blok, kebal jitter USB 2ms)
constexpr size_t   AUDIO_RB_TOLERANCE     = 256;    // ±5.33 ms hysteresis sebelum koreksi drift halus

// ============================================================================
// 4. IDENTITAS PERANGKAT USB UAC2
// ============================================================================
constexpr const char* USB_AUDIO_MANUFACTURER = "DSP Audio Labs";
constexpr const char* USB_AUDIO_PRODUCT      = "DSP Audio Interface";

// ============================================================================
// 5. TELEMETRI & SENSOR SUHU INTERNAL ESP32-S3
// ============================================================================
constexpr uint32_t TEMP_READ_INTERVAL_MS  = 1000;   // Baca sensor suhu setiap 1 detik
constexpr float    TEMP_WARN_THRESHOLD_C  = 75.0f;  // Layar peringatan merah & LED warning (>= 75°C)
constexpr float    TEMP_CRIT_THRESHOLD_C  = 85.0f;  // Suhu kritis (LED Merah berkedip cepat)

// ============================================================================
// 6. PARAMETER FREERTOS TASK
// ============================================================================
constexpr uint32_t AUDIO_TASK_STACK_BYTES = 12288;  // Cukup lega untuk DSP DLMS di Tahap 2
constexpr uint8_t  AUDIO_TASK_PRIORITY    = 6;      // Prioritas tertinggi di Core 1
constexpr uint8_t  AUDIO_TASK_CORE        = 1;      // Dedicated Audio/DSP Core

constexpr uint32_t SYS_TASK_STACK_BYTES   = 8192;
constexpr uint8_t  SYS_TASK_PRIORITY      = 1;
constexpr uint8_t  SYS_TASK_CORE          = 0;      // UI, Telemetry, Wi-Fi, Storage Core
