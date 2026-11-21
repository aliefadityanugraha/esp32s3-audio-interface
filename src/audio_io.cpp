/**
 * @file audio_io.cpp
 * @brief Implementasi Hardware & Audio I/O Layer (Tahap 1)
 *        ESP32-S3 N16R8 Hybrid USB Audio Interface (UAC2) + DLMS 2-In 4-Out
 */

#include "audio_io.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "soc/gpio_sig_map.h"

namespace audio_io {

namespace {

constexpr float INV_INT16_MAX   = 1.0f / 32768.0f;
constexpr float INT16_SCALE     = 32767.0f;
constexpr float INV_INT32_SCALE = 1.0f / 2147483648.0f;

// Durasi 1 blok audio dalam mikrodetik: (128 / 48000) * 1e6 = 2666.667 us
constexpr float BLOCK_DURATION_US =
    (static_cast<float>(AUDIO_BLOCK_FRAMES) * 1000000.0f) /
    static_cast<float>(AUDIO_SAMPLE_RATE);

inline float clampAudio(float v) {
  if (v > 1.0f) return 1.0f;
  if (v < -1.0f) return -1.0f;
  return v;
}

inline int16_t floatToInt16(float v) {
  float c = clampAudio(v) * INT16_SCALE;
  return static_cast<int16_t>(c);
}

inline int32_t floatToInt32(float v) {
  // 24-bit signed left-aligned ke slot 32-bit I2S via single-precision HW FPU (0 software double)
  constexpr float SCALE_24 = 8388607.0f;
  const int32_t s24 = static_cast<int32_t>(clampAudio(v) * SCALE_24);
  return s24 << 8;
}

} // namespace

// ============================================================================
// 1. IMPLEMENTASI ADAPTIVE RING BUFFER (DRIFT COMPENSATION + INTERPOLASI)
// ============================================================================
void AdaptiveRingBuffer::reset() {
  head_ = 0;
  tail_ = 0;
  count_ = 0;
  primed_ = false;
  lastL_ = 0;
  lastR_ = 0;
}

void AdaptiveRingBuffer::pushStereo16(const int16_t *interleavedSrc, size_t frames) {
  for (size_t i = 0; i < frames; i++) {
    if (count_ >= CAPACITY) {
      // Buffer penuh (overrun): buang frame tertua agar pointer tetap konsisten
      tail_ = (tail_ + 1) & MASK;
      count_--;
      overrunCount_++;
    }
    buf_[head_ * 2]     = interleavedSrc[i * 2];
    buf_[head_ * 2 + 1] = interleavedSrc[i * 2 + 1];
    head_ = (head_ + 1) & MASK;
    count_++;
  }
}

bool AdaptiveRingBuffer::popBlockFloat(float *outL, float *outR, size_t frames) {
  // Priming: tunggu hingga terisi minimal AUDIO_RB_TARGET_FILL (~8 ms) sebelum mulai bunyi
  if (!primed_) {
    if (count_ >= AUDIO_RB_TARGET_FILL) {
      primed_ = true;
    } else {
      memset(outL, 0, frames * sizeof(float));
      memset(outR, 0, frames * sizeof(float));
      return false;
    }
  }

  // Deteksi underrun: butuh sedikit margin (+2 frame) untuk koreksi drift
  if (count_ < frames + 2) {
    primed_ = false;
    underrunCount_++;
    memset(outL, 0, frames * sizeof(float));
    memset(outR, 0, frames * sizeof(float));
    return false;
  }

  // Hitung koreksi clock-drift antara USB Host (PC) vs I2S Master (ESP32-S3):
  //   adj = +1 : PC sedikit lebih cepat -> lebur 2 frame menjadi 1 (drop 1 frame halus)
  //   adj = -1 : PC sedikit lebih lambat -> sisipkan 1 frame interpolasi linear
  int adj = 0;
  if (count_ > (AUDIO_RB_TARGET_FILL + AUDIO_RB_TOLERANCE)) {
    adj = 1;
    driftDropCount_++;
  } else if (count_ < (AUDIO_RB_TARGET_FILL - AUDIO_RB_TOLERANCE)) {
    adj = -1;
    driftInsertCount_++;
  }

  for (size_t i = 0; i < frames; i++) {
    int16_t curL = 0;
    int16_t curR = 0;

    if (i == 0 && adj == 1) {
      // Drop 1 frame dengan rata-rata 2 frame bersebelahan untuk meminimalkan klik
      int16_t aL = 0, aR = 0;
      int16_t bL = 0, bR = 0;
      popOne(aL, aR);
      popOne(bL, bR);
      curL = static_cast<int16_t>((static_cast<int32_t>(aL) + static_cast<int32_t>(bL)) >> 1);
      curR = static_cast<int16_t>((static_cast<int32_t>(aR) + static_cast<int32_t>(bR)) >> 1);
      lastL_ = bL;
      lastR_ = bR;
    } else if (i == 0 && adj == -1) {
      // Sisipkan 1 frame hasil interpolasi linear antara sampel terakhir dan sampel berikutnya
      int16_t nextL = 0, nextR = 0;
      peekOne(nextL, nextR);
      curL = static_cast<int16_t>((static_cast<int32_t>(lastL_) + static_cast<int32_t>(nextL)) >> 1);
      curR = static_cast<int16_t>((static_cast<int32_t>(lastR_) + static_cast<int32_t>(nextR)) >> 1);
    } else {
      popOne(curL, curR);
      lastL_ = curL;
      lastR_ = curR;
    }

    outL[i] = static_cast<float>(curL) * INV_INT16_MAX;
    outR[i] = static_cast<float>(curR) * INV_INT16_MAX;
  }

  return true;
}

// ============================================================================
// 2. INISIALISASI USB AUDIO UAC2 (48 kHz, 16-bit, Stereo Full-Duplex)
// ============================================================================
bool AudioIO::beginUSB(const char *manufacturer, const char *product) {
  audio_tools::AudioInfo info(AUDIO_SAMPLE_RATE, AUDIO_USB_CHANNELS, AUDIO_USB_BIT_DEPTH);
  auto cfg = usbAudio_.defaultConfig(audio_tools::RXTX_MODE);
  cfg.copyFrom(info);
  cfg.manufacturer = manufacturer;
  cfg.product      = product;
  cfg.begin_usb    = true;

  usbInitialized_ = usbAudio_.begin(cfg);
  usbRxRb_.reset();
  return usbInitialized_;
}

// ============================================================================
// 3. KONFIGURASI GPIO MATRIX: SHARED BCLK (4) & LRCK (5) UNTUK I2S0 & I2S1
// ============================================================================
void AudioIO::configureSharedClockMatrix() {
  // Jadikan BCLK (GPIO 4) dan LRCK (GPIO 5) sebagai INPUT_OUTPUT:
  // - Driver output mengirim sinyal clock dari I2S0 Master keluar ke PCM1808, DAC1, DAC2
  // - Buffer input membaca balik pad yang sama secara internal menuju I2S1 Slave TX
  gpio_config_t ioConf = {};
  ioConf.pin_bit_mask = (1ULL << PIN_I2S_BCLK) | (1ULL << PIN_I2S_LRCK);
  ioConf.mode         = GPIO_MODE_INPUT_OUTPUT;
  ioConf.pull_up_en   = GPIO_PULLUP_DISABLE;
  ioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ioConf.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&ioConf);

  // Sambungkan Output Clock I2S0 Master -> Pin Fisik BCLK & LRCK
  esp_rom_gpio_connect_out_signal(PIN_I2S_BCLK, I2S0O_BCK_OUT_IDX, false, false);
  esp_rom_gpio_connect_out_signal(PIN_I2S_LRCK, I2S0O_WS_OUT_IDX,  false, false);

  // Sambungkan Pin Fisik BCLK & LRCK -> Input Clock I2S1 Slave TX (Loopback Internal)
  esp_rom_gpio_connect_in_signal(PIN_I2S_BCLK, I2S1O_BCK_IN_IDX, false);
  esp_rom_gpio_connect_in_signal(PIN_I2S_LRCK, I2S1O_WS_IN_IDX,  false);

  // Perkuat drive capability sinyal clock & data I2S (mencegah ringing/rounding di PCB/jumper)
  gpio_set_drive_capability(PIN_I2S_MCLK,     GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability(PIN_I2S_BCLK,     GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability(PIN_I2S_LRCK,     GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability(PIN_I2S_DAC1_DIN, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability(PIN_I2S_DAC2_DIN, GPIO_DRIVE_CAP_2);
}

// ============================================================================
// 4. INISIALISASI DUAL-I2S 100% SINKRON FASE (2-IN ADC + 4-OUT DUAL DAC)
// ============================================================================
bool AudioIO::beginI2S() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // --------------------------------------------------------------------------
  // ESP-IDF v5.x (Arduino-ESP32 3.x) -> driver/i2s_std.h
  // --------------------------------------------------------------------------

  // A. Alokasi I2S_NUM_1 sebagai SLAVE TX (DAC2 PCM5102A: OUT 3-4)
  i2s_chan_config_t chan1Cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
  chan1Cfg.dma_desc_num  = AUDIO_DMA_BUF_COUNT;
  chan1Cfg.dma_frame_num = AUDIO_BLOCK_FRAMES;
  chan1Cfg.auto_clear    = true;

  if (i2s_new_channel(&chan1Cfg, &i2s1TxHandle_, nullptr) != ESP_OK) {
    return false;
  }

  i2s_std_config_t std1Cfg = {};
  std1Cfg.clk_cfg                = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  std1Cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256;
  std1Cfg.slot_cfg               = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                                       I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  std1Cfg.gpio_cfg.mclk          = I2S_GPIO_UNUSED;
  std1Cfg.gpio_cfg.bclk          = PIN_I2S_BCLK;
  std1Cfg.gpio_cfg.ws            = PIN_I2S_LRCK;
  std1Cfg.gpio_cfg.dout          = PIN_I2S_DAC2_DIN;
  std1Cfg.gpio_cfg.din           = I2S_GPIO_UNUSED;
  std1Cfg.gpio_cfg.invert_flags.mclk_inv = false;
  std1Cfg.gpio_cfg.invert_flags.bclk_inv = false;
  std1Cfg.gpio_cfg.invert_flags.ws_inv   = false;

  if (i2s_channel_init_std_mode(i2s1TxHandle_, &std1Cfg) != ESP_OK) {
    return false;
  }

  // B. Alokasi I2S_NUM_0 sebagai MASTER FULL-DUPLEX (ADC PCM1808 + DAC1 OUT 1-2)
  i2s_chan_config_t chan0Cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan0Cfg.dma_desc_num  = AUDIO_DMA_BUF_COUNT;
  chan0Cfg.dma_frame_num = AUDIO_BLOCK_FRAMES;
  chan0Cfg.auto_clear    = true;

  if (i2s_new_channel(&chan0Cfg, &i2s0TxHandle_, &i2s0RxHandle_) != ESP_OK) {
    return false;
  }

  i2s_std_config_t std0Cfg = {};
  std0Cfg.clk_cfg                = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  std0Cfg.clk_cfg.mclk_multiple  = I2S_MCLK_MULTIPLE_256; // 12.288 MHz MCLK untuk PCM1808
  std0Cfg.slot_cfg               = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                                       I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  std0Cfg.gpio_cfg.mclk          = PIN_I2S_MCLK;
  std0Cfg.gpio_cfg.bclk          = PIN_I2S_BCLK;
  std0Cfg.gpio_cfg.ws            = PIN_I2S_LRCK;
  std0Cfg.gpio_cfg.dout          = PIN_I2S_DAC1_DIN;
  std0Cfg.gpio_cfg.din           = PIN_I2S_ADC_DOUT;
  std0Cfg.gpio_cfg.invert_flags.mclk_inv = false;
  std0Cfg.gpio_cfg.invert_flags.bclk_inv = false;
  std0Cfg.gpio_cfg.invert_flags.ws_inv   = false;

  if (i2s_channel_init_std_mode(i2s0TxHandle_, &std0Cfg) != ESP_OK) {
    return false;
  }
  if (i2s_channel_init_std_mode(i2s0RxHandle_, &std0Cfg) != ESP_OK) {
    return false;
  }

  // C. Ikat Ulang GPIO Matrix agar BCLK (4) & LRCK (5) dibagi 100% sinkron fase
  configureSharedClockMatrix();

  // D. Start Sinkron 0-Sample-Skew:
  //    1. Preload silence ke DMA TX I2S1 & I2S0 saat clock Master masih diam
  //    2. Enable I2S1 Slave TX terlebih dahulu (siap di Sample 0, Bit 0 menunggu tepi clock)
  //    3. Enable I2S0 Master RX & TX -> keduanya bergeser keluar tepat di tepi LRCK pertama!
  memset(i2sTx0Buf_, 0, sizeof(i2sTx0Buf_));
  memset(i2sTx1Buf_, 0, sizeof(i2sTx1Buf_));
  size_t loadedBytes = 0;
  i2s_channel_preload_data(i2s1TxHandle_, i2sTx1Buf_, sizeof(i2sTx1Buf_), &loadedBytes);
  i2s_channel_preload_data(i2s0TxHandle_, i2sTx0Buf_, sizeof(i2sTx0Buf_), &loadedBytes);

  if (i2s_channel_enable(i2s1TxHandle_) != ESP_OK) return false;
  if (i2s_channel_enable(i2s0RxHandle_) != ESP_OK) return false;
  if (i2s_channel_enable(i2s0TxHandle_) != ESP_OK) return false;

#else
  // --------------------------------------------------------------------------
  // ESP-IDF v4.4 (Arduino-ESP32 2.x) -> Legacy driver/i2s.h
  // --------------------------------------------------------------------------

  // A. Instal I2S_NUM_1 sebagai SLAVE TX (DAC2 OUT 3-4)
  i2s_config_t i2s1Cfg = {};
  i2s1Cfg.mode                 = static_cast<i2s_mode_t>(I2S_MODE_SLAVE | I2S_MODE_TX);
  i2s1Cfg.sample_rate          = AUDIO_SAMPLE_RATE;
  i2s1Cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
  i2s1Cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s1Cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s1Cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2s1Cfg.dma_buf_count        = AUDIO_DMA_BUF_COUNT;
  i2s1Cfg.dma_buf_len          = AUDIO_BLOCK_FRAMES;
  i2s1Cfg.use_apll             = false;
  i2s1Cfg.tx_desc_auto_clear   = true;
  i2s1Cfg.fixed_mclk           = 0;

  i2s_pin_config_t pin1Cfg = {};
  pin1Cfg.mck_io_num   = I2S_PIN_NO_CHANGE;
  pin1Cfg.bck_io_num   = PIN_I2S_BCLK;
  pin1Cfg.ws_io_num    = PIN_I2S_LRCK;
  pin1Cfg.data_out_num = PIN_I2S_DAC2_DIN;
  pin1Cfg.data_in_num  = I2S_PIN_NO_CHANGE;

  if (i2s_driver_install(I2S_NUM_1, &i2s1Cfg, 0, nullptr) != ESP_OK) return false;
  if (i2s_set_pin(I2S_NUM_1, &pin1Cfg) != ESP_OK) return false;
  i2s_stop(I2S_NUM_1);

  // B. Instal I2S_NUM_0 sebagai MASTER FULL-DUPLEX (ADC + DAC1 OUT 1-2)
  i2s_config_t i2s0Cfg = {};
  i2s0Cfg.mode                 = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX);
  i2s0Cfg.sample_rate          = AUDIO_SAMPLE_RATE;
  i2s0Cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT;
  i2s0Cfg.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s0Cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s0Cfg.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
  i2s0Cfg.dma_buf_count        = AUDIO_DMA_BUF_COUNT;
  i2s0Cfg.dma_buf_len          = AUDIO_BLOCK_FRAMES;
  i2s0Cfg.use_apll             = false;
  i2s0Cfg.tx_desc_auto_clear   = true;
  i2s0Cfg.fixed_mclk           = AUDIO_I2S_MCLK_HZ; // 12.288 MHz (256fs)

  i2s_pin_config_t pin0Cfg = {};
  pin0Cfg.mck_io_num   = PIN_I2S_MCLK;
  pin0Cfg.bck_io_num   = PIN_I2S_BCLK;
  pin0Cfg.ws_io_num    = PIN_I2S_LRCK;
  pin0Cfg.data_out_num = PIN_I2S_DAC1_DIN;
  pin0Cfg.data_in_num  = PIN_I2S_ADC_DOUT;

  if (i2s_driver_install(I2S_NUM_0, &i2s0Cfg, 0, nullptr) != ESP_OK) return false;
  if (i2s_set_pin(I2S_NUM_0, &pin0Cfg) != ESP_OK) return false;
  i2s_stop(I2S_NUM_0);

  // C. Ikat Ulang GPIO Matrix untuk shared BCLK & LRCK
  configureSharedClockMatrix();

  // D. Bersihkan buffer DMA dan start serempak (Slave dulu baru Master)
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_1);
  i2s_start(I2S_NUM_0);
#endif

  dcAdc_[0].reset();
  dcAdc_[1].reset();
  i2sInitialized_ = true;

  portENTER_CRITICAL(&telemetryMux_);
  telemetry_.i2sReady = true;
  portEXIT_CRITICAL(&telemetryMux_);

  return true;
}

bool AudioIO::resyncI2S() {
  if (!i2sInitialized_) return false;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  i2s_channel_disable(i2s0TxHandle_);
  i2s_channel_disable(i2s0RxHandle_);
  i2s_channel_disable(i2s1TxHandle_);

  configureSharedClockMatrix();

  memset(i2sTx0Buf_, 0, sizeof(i2sTx0Buf_));
  memset(i2sTx1Buf_, 0, sizeof(i2sTx1Buf_));
  size_t loadedBytes = 0;
  i2s_channel_preload_data(i2s1TxHandle_, i2sTx1Buf_, sizeof(i2sTx1Buf_), &loadedBytes);
  i2s_channel_preload_data(i2s0TxHandle_, i2sTx0Buf_, sizeof(i2sTx0Buf_), &loadedBytes);

  if (i2s_channel_enable(i2s1TxHandle_) != ESP_OK) return false;
  if (i2s_channel_enable(i2s0RxHandle_) != ESP_OK) return false;
  if (i2s_channel_enable(i2s0TxHandle_) != ESP_OK) return false;
#else
  i2s_stop(I2S_NUM_0);
  i2s_stop(I2S_NUM_1);
  configureSharedClockMatrix();
  i2s_zero_dma_buffer(I2S_NUM_1);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_start(I2S_NUM_1);
  i2s_start(I2S_NUM_0);
#endif
  return true;
}

// ============================================================================
// 5. SENSOR SUHU INTERNAL ESP32-S3 (PERIODIK DI CORE 0)
// ============================================================================
bool AudioIO::beginTempSensor() {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Rentang 10..80 °C memiliki akurasi optimal (±1°C) untuk suhu operasi ESP32-S3
  temperature_sensor_config_t tempCfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  if (temperature_sensor_install(&tempCfg, &tempHandle_) != ESP_OK) {
    return false;
  }
  if (temperature_sensor_enable(tempHandle_) != ESP_OK) {
    return false;
  }
#else
  temp_sensor_config_t tempCfg = TSENS_CONFIG_DEFAULT();
  tempCfg.dac_offset = TSENS_DAC_L2; // Rentang -10°C s/d +80°C (error < 1°C)
  if (temp_sensor_set_config(tempCfg) != ESP_OK) {
    return false;
  }
  if (temp_sensor_start() != ESP_OK) {
    return false;
  }
#endif

  tempInitialized_ = true;
  tempHasFirstSample_ = false;
  lastTempReadMs_ = 0;

  portENTER_CRITICAL(&telemetryMux_);
  telemetry_.tempSensorReady = true;
  portEXIT_CRITICAL(&telemetryMux_);

  // Ambil sampel awal langsung saat boot
  updateTemperature(0);
  return true;
}

bool AudioIO::updateTemperature(uint32_t intervalMs) {
  if (!tempInitialized_) return false;

  uint32_t nowMs = millis();
  if (tempHasFirstSample_ && (nowMs - lastTempReadMs_ < intervalMs)) {
    return false;
  }
  lastTempReadMs_ = nowMs;

  float rawC = 0.0f;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  if (temperature_sensor_get_celsius(tempHandle_, &rawC) != ESP_OK) {
    return false;
  }
#else
  if (temp_sensor_read_celsius(&rawC) != ESP_OK) {
    return false;
  }
#endif

  portENTER_CRITICAL(&telemetryMux_);
  if (!tempHasFirstSample_) {
    telemetry_.chipTempC = rawC;
    telemetry_.chipTempPeakC = rawC;
    tempHasFirstSample_ = true;
  } else {
    // Exponential Moving Average (EMA) agar tampilan suhu halus dan tidak jitter
    telemetry_.chipTempC = (telemetry_.chipTempC * 0.8f) + (rawC * 0.2f);
    if (telemetry_.chipTempC > telemetry_.chipTempPeakC) {
      telemetry_.chipTempPeakC = telemetry_.chipTempC;
    }
  }
  telemetry_.tempWarning  = (telemetry_.chipTempC >= TEMP_WARN_THRESHOLD_C);
  telemetry_.tempCritical = (telemetry_.chipTempC >= TEMP_CRIT_THRESHOLD_C);
  portEXIT_CRITICAL(&telemetryMux_);

  return true;
}

// ============================================================================
// 6. KONTROL CALLBACK & PARAMETER TAHAP 1
// ============================================================================
void AudioIO::setProcessCallback(AudioProcessCallback cb, void *userCtx) {
  portENTER_CRITICAL(&telemetryMux_);
  processCb_  = cb;
  processCtx_ = userCtx;
  portEXIT_CRITICAL(&telemetryMux_);
}

void AudioIO::setDefaultRouting(DefaultRoutingMode mode) {
  defaultRouting_ = mode;
}

void AudioIO::setMasterGain(float linearGain) {
  if (linearGain < 0.0f) linearGain = 0.0f;
  if (linearGain > 4.0f) linearGain = 4.0f; // Maks +12 dB
  masterGainTarget_ = linearGain;
}

// ============================================================================
// 7. KURAS DATA USB UAC2 (PC -> ESP32) KE ADAPTIVE RING BUFFER
// ============================================================================
void AudioIO::drainUsbToRingBuffer() {
  if (!usbInitialized_) return;

  if (usbAudio_.mounted()) {
    int avail = 0;
    while ((avail = usbAudio_.available()) >= 4) {
      size_t toRead = static_cast<size_t>(avail);
      if (toRead > sizeof(usbRxRaw_)) {
        toRead = sizeof(usbRxRaw_);
      }
      toRead -= (toRead % 4); // Kelipatan 4 byte (1 frame stereo 16-bit)
      if (toRead == 0) break;

      size_t got = usbAudio_.readBytes(reinterpret_cast<uint8_t *>(usbRxRaw_), toRead);
      got -= (got % 4);
      if (got == 0) break;

      usbRxRb_.pushStereo16(usbRxRaw_, got / 4);
    }
  } else {
    usbRxRb_.reset();
  }
}

// ============================================================================
// 8. ROUTING BAWAAN TAHAP 1 (PASSTHROUGH / VERIFIKASI 2-IN 4-OUT)
// ============================================================================
void AudioIO::runDefaultRouting(AudioBlock &block) {
  const DefaultRoutingMode mode = defaultRouting_;
  const float targetGain = masterMuted_ ? 0.0f : masterGainTarget_;

  for (size_t i = 0; i < block.frames; i++) {
    // Smoothing gain agar perubahan volume/mute bebas klik/pop
    masterGainSmooth_ += (targetGain - masterGainSmooth_) * 0.0025f;

    const float adcL = block.adcIn[0][i];
    const float adcR = block.adcIn[1][i];
    const float usbL = block.usbIn[0][i];
    const float usbR = block.usbIn[1][i];

    // Jalur Record ke PC (USB UAC2 IN): selalu kirim sinyal ADC IN 1-2
    block.usbOut[0][i] = clampAudio(adcL);
    block.usbOut[1][i] = clampAudio(adcR);

    // Jalur 4-Output DAC (DAC1 = OUT 1-2, DAC2 = OUT 3-4)
    float o1 = 0.0f, o2 = 0.0f, o3 = 0.0f, o4 = 0.0f;
    switch (mode) {
      case DefaultRoutingMode::ADC_TO_ALL4:
        o1 = adcL; o2 = adcR;
        o3 = adcL; o4 = adcR;
        break;

      case DefaultRoutingMode::HYBRID_MIX_ALL4:
        o1 = (usbL + adcL) * 0.5f;
        o2 = (usbR + adcR) * 0.5f;
        o3 = o1;
        o4 = o2;
        break;

      case DefaultRoutingMode::SPLIT_USB_DAC1_ADC_DAC2:
        o1 = usbL; o2 = usbR;
        o3 = adcL; o4 = adcR;
        break;

      case DefaultRoutingMode::USB_TO_ALL4:
      default:
        o1 = usbL; o2 = usbR;
        o3 = usbL; o4 = usbR;
        break;
    }

    block.dacOut[0][i] = clampAudio(o1 * masterGainSmooth_);
    block.dacOut[1][i] = clampAudio(o2 * masterGainSmooth_);
    block.dacOut[2][i] = clampAudio(o3 * masterGainSmooth_);
    block.dacOut[3][i] = clampAudio(o4 * masterGainSmooth_);
  }
}

// ============================================================================
// 9. LOOP REAL-TIME AUDIO (DIPANGGIL KONTINU DI CORE 1 PRIORITAS TINGGI)
// ============================================================================
void AudioIO::processOneBlock() {
  if (!i2sInitialized_) {
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  // ---- 1. Kuras paket USB masuk ke Adaptive Ring Buffer ----
  drainUsbToRingBuffer();

  // ---- 2. Baca 128 frame dari ADC PCM1808 (Metronom Hardware 2.667 ms) ----
  size_t bytesRead = 0;
  esp_err_t rxErr = ESP_OK;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  rxErr = i2s_channel_read(i2s0RxHandle_, i2sRxBuf_, sizeof(i2sRxBuf_), &bytesRead, 20);
#else
  rxErr = i2s_read(I2S_NUM_0, i2sRxBuf_, sizeof(i2sRxBuf_), &bytesRead, pdMS_TO_TICKS(20));
#endif

  if (rxErr != ESP_OK || bytesRead < sizeof(i2sRxBuf_)) {
    portENTER_CRITICAL(&telemetryMux_);
    telemetry_.i2sRxErrorCount++;
    portEXIT_CRITICAL(&telemetryMux_);
    vTaskDelay(1);
    return;
  }

  // Mulai ukur waktu komputasi DSP/I/O per blok (di luar waktu tunggu DMA RX)
  const int64_t tStartUs = esp_timer_get_time();

  // ---- 3. Konversi ADC 32-bit -> float32 [-1..+1] + DC Blocker ----
  block_.frames = AUDIO_BLOCK_FRAMES;
  float pkAdc0 = 0.0f, pkAdc1 = 0.0f;

  for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
    // Pertahankan resolusi penuh 24-bit PCM1808 dalam slot 32-bit signed
    float sL = static_cast<float>(i2sRxBuf_[i * 2])     * INV_INT32_SCALE;
    float sR = static_cast<float>(i2sRxBuf_[i * 2 + 1]) * INV_INT32_SCALE;

    sL = dcAdc_[0].process(sL);
    sR = dcAdc_[1].process(sR);

    block_.adcIn[0][i] = sL;
    block_.adcIn[1][i] = sR;

    float aL = fabsf(sL);
    float aR = fabsf(sR);
    if (aL > pkAdc0) pkAdc0 = aL;
    if (aR > pkAdc1) pkAdc1 = aR;
  }

  // ---- 4. Tarik 128 frame USB Playback dari Adaptive Ring Buffer ----
  block_.usbPlaybackActive = usbRxRb_.popBlockFloat(block_.usbIn[0], block_.usbIn[1], AUDIO_BLOCK_FRAMES);

  float pkUsb0 = 0.0f, pkUsb1 = 0.0f;
  if (block_.usbPlaybackActive) {
    for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
      float aL = fabsf(block_.usbIn[0][i]);
      float aR = fabsf(block_.usbIn[1][i]);
      if (aL > pkUsb0) pkUsb0 = aL;
      if (aR > pkUsb1) pkUsb1 = aR;
    }
  }

  // ---- 5. Jalankan DSP Callback (Tahap 2 DLMS) atau Default Routing (Tahap 1) ----
  AudioProcessCallback cb = processCb_;
  void *ctx = processCtx_;
  if (cb != nullptr) {
    cb(block_, ctx);
  } else {
    runDefaultRouting(block_);
  }

  // ---- 6. Packing Output ke USB UAC2 TX (16-bit) & Dual DAC I2S0/I2S1 (32-bit) ----
  float pkDac[AUDIO_OUT_CHANNELS] = {0.0f, 0.0f, 0.0f, 0.0f};

  for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
    // USB Record ke PC (16-bit stereo)
    usbTxRaw_[i * 2]     = floatToInt16(block_.usbOut[0][i]);
    usbTxRaw_[i * 2 + 1] = floatToInt16(block_.usbOut[1][i]);

    // DAC1 (I2S0 TX: OUT 1-2) & DAC2 (I2S1 TX: OUT 3-4) 32-bit stereo
    const float d0 = block_.dacOut[0][i];
    const float d1 = block_.dacOut[1][i];
    const float d2 = block_.dacOut[2][i];
    const float d3 = block_.dacOut[3][i];

    i2sTx0Buf_[i * 2]     = floatToInt32(d0);
    i2sTx0Buf_[i * 2 + 1] = floatToInt32(d1);

    i2sTx1Buf_[i * 2]     = floatToInt32(d2);
    i2sTx1Buf_[i * 2 + 1] = floatToInt32(d3);

    float a0 = fabsf(d0); if (a0 > pkDac[0]) pkDac[0] = a0;
    float a1 = fabsf(d1); if (a1 > pkDac[1]) pkDac[1] = a1;
    float a2 = fabsf(d2); if (a2 > pkDac[2]) pkDac[2] = a2;
    float a3 = fabsf(d3); if (a3 > pkDac[3]) pkDac[3] = a3;
  }

  // ---- 7. Kirim ke Endpoint USB UAC2 TX (ESP32 -> PC) ----
  const bool mounted = usbInitialized_ && usbAudio_.mounted();
  if (mounted) {
    usbAudio_.write(reinterpret_cast<const uint8_t *>(usbTxRaw_), sizeof(usbTxRaw_));
  }

  // ---- 8. Tulis Serempak ke DMA DAC2 (I2S1 Slave) & DAC1 (I2S0 Master) ----
  // Kedua DMA mengonsumsi tepat 128 frame per 128 frame BCLK/LRCK bersama -> 0 drift!
  size_t wBytes1 = 0;
  size_t wBytes0 = 0;
  esp_err_t tx1Err = ESP_OK;
  esp_err_t tx0Err = ESP_OK;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  tx1Err = i2s_channel_write(i2s1TxHandle_, i2sTx1Buf_, sizeof(i2sTx1Buf_), &wBytes1, 10);
  tx0Err = i2s_channel_write(i2s0TxHandle_, i2sTx0Buf_, sizeof(i2sTx0Buf_), &wBytes0, 10);
#else
  tx1Err = i2s_write(I2S_NUM_1, i2sTx1Buf_, sizeof(i2sTx1Buf_), &wBytes1, pdMS_TO_TICKS(10));
  tx0Err = i2s_write(I2S_NUM_0, i2sTx0Buf_, sizeof(i2sTx0Buf_), &wBytes0, pdMS_TO_TICKS(10));
#endif

  // Hitung CPU Load Core 1 (%)
  const int64_t elapsedUs = esp_timer_get_time() - tStartUs;
  const float cpuLoadPct = (static_cast<float>(elapsedUs) / BLOCK_DURATION_US) * 100.0f;

  // ---- 9. Update Snapshot Telemetri secara Atomic ----
  portENTER_CRITICAL(&telemetryMux_);
  telemetry_.usbMounted        = mounted;
  telemetry_.usbRxStreaming    = mounted && (usbAudio_.isStreamingActiveRx() || block_.usbPlaybackActive);
  telemetry_.usbTxStreaming    = mounted && usbAudio_.isStreamingActiveTx();
  telemetry_.ringBufferPrimed  = block_.usbPlaybackActive;
  telemetry_.rbFillFrames      = usbRxRb_.count();
  telemetry_.rbUnderrunCount   = usbRxRb_.underruns();
  telemetry_.rbOverrunCount    = usbRxRb_.overruns();
  telemetry_.rbDriftDropCount  = usbRxRb_.driftDrops();
  telemetry_.rbDriftInsertCount= usbRxRb_.driftInserts();

  if (tx0Err != ESP_OK || wBytes0 < sizeof(i2sTx0Buf_)) telemetry_.i2sTx0ErrorCount++;
  if (tx1Err != ESP_OK || wBytes1 < sizeof(i2sTx1Buf_)) telemetry_.i2sTx1ErrorCount++;

  telemetry_.peakAdc[0]   = pkAdc0;
  telemetry_.peakAdc[1]   = pkAdc1;
  telemetry_.peakUsbIn[0] = pkUsb0;
  telemetry_.peakUsbIn[1] = pkUsb1;
  for (size_t ch = 0; ch < AUDIO_OUT_CHANNELS; ch++) {
    telemetry_.peakDac[ch] = pkDac[ch];
  }

  telemetry_.dspCpuLoadPct = (telemetry_.dspCpuLoadPct * 0.9f) + (cpuLoadPct * 0.1f);
  if (cpuLoadPct > telemetry_.dspCpuLoadPeakPct) {
    telemetry_.dspCpuLoadPeakPct = cpuLoadPct;
  }
  portEXIT_CRITICAL(&telemetryMux_);
}

// ============================================================================
// 10. TELEMETRI HELPER
// ============================================================================
AudioTelemetry AudioIO::getTelemetry() const {
  AudioTelemetry copy = {};
  portENTER_CRITICAL(&telemetryMux_);
  copy = telemetry_;
  portEXIT_CRITICAL(&telemetryMux_);
  return copy;
}

void AudioIO::resetPeakTelemetry() {
  portENTER_CRITICAL(&telemetryMux_);
  telemetry_.dspCpuLoadPeakPct = telemetry_.dspCpuLoadPct;
  telemetry_.chipTempPeakC     = telemetry_.chipTempC;
  portEXIT_CRITICAL(&telemetryMux_);
}

float AudioIO::linearToDbfs(float linear) {
  if (linear <= 1.58489e-5f) { // -96 dBFS floor
    return -96.0f;
  }
  return 20.0f * log10f(linear);
}

} // namespace audio_io
