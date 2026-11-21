#pragma once

/**
 * @file pin_config.h
 * @brief Definisi Pinout Hardware ESP32-S3 N16R8
 *        Hybrid USB Audio Interface (UAC2) + DLMS 2-In 4-Out
 *
 * Catatan Pin Terpakai Internal ESP32-S3 N16R8 (JANGAN DIGUNAKAN):
 *  - GPIO 19, 20     : Native USB OTG (D- / D+) untuk USB Audio UAC2
 *  - GPIO 26..32     : SPI Flash (16MB QIO)
 *  - GPIO 33..37     : Octal PSRAM (8MB OPI)
 *  - GPIO 43, 44     : UART0 (TX0 / RX0) untuk Serial Debug & Flashing
 *  - GPIO 0, 3, 45, 46 : Strapping Pins
 */

#include <driver/gpio.h>

// ============================================================================
// 1. LCD TFT ST7789 (240x320 SPI)
// ============================================================================
constexpr gpio_num_t PIN_LCD_SCLK = GPIO_NUM_12;
constexpr gpio_num_t PIN_LCD_MOSI = GPIO_NUM_13;
constexpr gpio_num_t PIN_LCD_CS   = GPIO_NUM_10;
constexpr gpio_num_t PIN_LCD_DC   = GPIO_NUM_9;
constexpr gpio_num_t PIN_LCD_RST  = GPIO_NUM_8;
constexpr int8_t     PIN_LCD_BL   = -1; // Set ke GPIO bebas (mis. 14) jika pakai PWM Backlight

// ============================================================================
// 2. ROTARY ENCODER (EC11 + Push Switch)
// ============================================================================
constexpr gpio_num_t PIN_ENC_CLK  = GPIO_NUM_16;
constexpr gpio_num_t PIN_ENC_DT   = GPIO_NUM_17;
constexpr gpio_num_t PIN_ENC_SW   = GPIO_NUM_18;

// ============================================================================
// 3. I2S SHARED CLOCK BUS (100% Sinkron Fase: ADC + DAC1 + DAC2)
// ============================================================================
// I2S0 berperan sebagai Master pembangkit MCLK, BCLK, dan LRCK.
// I2S1 berperan sebagai Slave TX yang membaca BCLK & LRCK internal via GPIO Matrix.
constexpr gpio_num_t PIN_I2S_BCLK = GPIO_NUM_4;   // 3.072 MHz (64 * 48 kHz)
constexpr gpio_num_t PIN_I2S_LRCK = GPIO_NUM_5;   // 48.000 kHz (Word Select)
constexpr gpio_num_t PIN_I2S_MCLK = GPIO_NUM_15;  // 12.288 MHz (256 * 48 kHz untuk PCM1808)

// ============================================================================
// 4. I2S DATA LINES (2-In ADC, 4-Out Dual Stereo DAC)
// ============================================================================
// ADC PCM1808 (Slave Mode: FMT=GND, MD0=GND, MD1=GND) -> I2S0 RX
constexpr gpio_num_t PIN_I2S_ADC_DOUT  = GPIO_NUM_7;   // PCM1808 DOUT -> ESP32-S3 DIN

// DAC1 PCM5102A (OUT 1-2: Main L/R atau High/Mid) -> I2S0 TX
constexpr gpio_num_t PIN_I2S_DAC1_DIN  = GPIO_NUM_6;   // ESP32-S3 I2S0 DOUT -> DAC1 DIN

// DAC2 PCM5102A (OUT 3-4: Sub/Aux atau Low/Sub) -> I2S1 TX (Phase-Locked ke I2S0)
constexpr gpio_num_t PIN_I2S_DAC2_DIN  = GPIO_NUM_21;  // ESP32-S3 I2S1 DOUT -> DAC2 DIN

// ============================================================================
// 5. LED STATUS (WS2812 RGB Bawaan ESP32-S3 DevKitC-1)
// ============================================================================
constexpr gpio_num_t PIN_LED_STATUS    = GPIO_NUM_48;

// ============================================================================
// 6. NATIVE USB OTG (Dokumentasi - Dikelola Hardware TinyUSB)
// ============================================================================
constexpr gpio_num_t PIN_USB_DN        = GPIO_NUM_19;
constexpr gpio_num_t PIN_USB_DP        = GPIO_NUM_20;
