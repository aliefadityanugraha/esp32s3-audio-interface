#pragma once

/**
 * @file audio_io.h
 * @brief Tahap 1: Hardware & Audio I/O Layer untuk ESP32-S3 N16R8
 *        Hybrid USB Audio Interface (UAC2 48kHz 16-bit) + DLMS 2-In 4-Out
 *
 * Fitur Utama:
 *  1. Dual-I2S 100% Sinkron Fase:
 *     - I2S0 Master Full-Duplex : MCLK (15), BCLK (4), LRCK (5),
 *                                 RX ADC PCM1808 (7), TX DAC1 PCM5102A OUT 1-2 (6)
 *     - I2S1 Slave TX           : BCLK (4) & LRCK (5) internal via GPIO Matrix,
 *                                 TX DAC2 PCM5102A OUT 3-4 (21)
 *     - Start Sinkron 0-Sample-Skew: Preload DMA + Arm I2S1 Slave sebelum Master clock aktif.
 *  2. Integrasi USB Audio UAC2 (2-In / 2-Out) via Adaptive Ring Buffer dengan
 *     interpolasi linear saat koreksi drift clock PC vs ESP32-S3.
 *  3. Pembacaan Sensor Suhu Internal ESP32-S3 secara periodik + telemetri real-time
 *     (Peak 6-channel, DSP CPU load, status Ring Buffer, status termal).
 *  4. Hook DSP (`AudioProcessCallback`) agar Tahap 2 (DLMS 2-In 4-Out Engine)
 *     dapat dipasang tanpa mengubah layer Audio I/O.
 */

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #include "driver/i2s_std.h"
  #include "driver/temperature_sensor.h"
#else
  #include "driver/i2s.h"
  #include "driver/temp_sensor.h"
#endif

#include "audio_config.h"
#include "pin_config.h"

namespace audio_io {

// ============================================================================
// 1. STRUKTUR BLOK AUDIO (FLOAT32 NORMALIZED [-1.0f, +1.0f])
// ============================================================================
// Semua kanal menggunakan format float32 [-1.0, +1.0] agar dynamic range 24-bit
// ADC PCM1808 dan DAC PCM5102A tetap utuh saat diproses oleh DSP DLMS di Tahap 2.
struct AudioBlock {
  float adcIn[AUDIO_IN_CHANNELS][AUDIO_BLOCK_FRAMES];     // IN 1-2 dari ADC PCM1808 (post DC blocker)
  float usbIn[AUDIO_USB_CHANNELS][AUDIO_BLOCK_FRAMES];    // Playback Stereo dari PC (USB UAC2 RX)
  float usbOut[AUDIO_USB_CHANNELS][AUDIO_BLOCK_FRAMES];   // Record Stereo ke PC (USB UAC2 TX)
  float dacOut[AUDIO_OUT_CHANNELS][AUDIO_BLOCK_FRAMES];   // OUT 1-2 (DAC1) & OUT 3-4 (DAC2)
  size_t frames;                                          // Selalu AUDIO_BLOCK_FRAMES (128)
  bool usbPlaybackActive;                                 // True jika Ring Buffer USB sedang primed
};

// Callback pemrosesan DSP (dipasang oleh Modul DLMS di Tahap 2)
using AudioProcessCallback = void (*)(AudioBlock &block, void *userCtx);

// Mode routing bawaan untuk Tahap 1 (sebelum DSP DLMS aktif)
enum class DefaultRoutingMode : uint8_t {
  USB_TO_ALL4 = 0,       // USB Playback -> DAC1 (OUT 1-2) & DAC2 (OUT 3-4); ADC -> USB Record
  ADC_TO_ALL4 = 1,       // ADC Direct -> DAC1 & DAC2 (uji DLMS analog tanpa PC); ADC -> USB Record
  HYBRID_MIX_ALL4 = 2,   // (USB + ADC) Mix -> DAC1 & DAC2; ADC -> USB Record
  SPLIT_USB_DAC1_ADC_DAC2 = 3 // USB -> DAC1 (OUT 1-2), ADC -> DAC2 (OUT 3-4) untuk uji isolasi 4-ch
};

// ============================================================================
// 2. STRUKTUR TELEMETRI REAL-TIME (DIBACA CORE 0: UI / LED / SERIAL / WEB)
// ============================================================================
struct AudioTelemetry {
  // Status USB UAC2
  bool usbMounted;
  bool usbRxStreaming;
  bool usbTxStreaming;
  bool ringBufferPrimed;

  // Statistik Adaptive Ring Buffer
  size_t rbFillFrames;
  uint32_t rbUnderrunCount;
  uint32_t rbOverrunCount;
  uint32_t rbDriftDropCount;
  uint32_t rbDriftInsertCount;

  // Status Sinkronisasi I2S Dual-Port
  bool i2sReady;
  uint32_t i2sRxErrorCount;
  uint32_t i2sTx0ErrorCount;
  uint32_t i2sTx1ErrorCount;

  // Peak Level Linear [0.0f .. 1.0f] per blok
  float peakAdc[AUDIO_IN_CHANNELS];     // IN 1, IN 2
  float peakDac[AUDIO_OUT_CHANNELS];    // OUT 1, OUT 2, OUT 3, OUT 4
  float peakUsbIn[AUDIO_USB_CHANNELS];  // USB Play L, R

  // Beban CPU Core 1 Audio Loop (persen dari durasi blok 2.667 ms)
  float dspCpuLoadPct;
  float dspCpuLoadPeakPct;

  // Sensor Suhu Internal ESP32-S3
  bool tempSensorReady;
  float chipTempC;
  float chipTempPeakC;
  bool tempWarning;
  bool tempCritical;
};

// ============================================================================
// 3. FILTER DC BLOCKER (HPF ~6 Hz @ 48kHz - bass utuh tanpa DC Offset ADC)
// ============================================================================
struct DcBlocker {
  float xPrev = 0.0f;
  float yPrev = 0.0f;

  inline float process(float x) {
    if (!std::isfinite(x)) x = 0.0f;
    float y = x - xPrev + 0.9992f * yPrev;
    if (fabsf(y) < 1.0e-20f) y = 0.0f;
    xPrev = x;
    yPrev = y;
    return y;
  }

  inline void reset() {
    xPrev = 0.0f;
    yPrev = 0.0f;
  }
};

// ============================================================================
// 4. ADAPTIVE RING BUFFER (USB PC -> ESP32 I2S Jitter & Clock-Drift Guard)
// ============================================================================
class AdaptiveRingBuffer {
public:
  static_assert((AUDIO_RB_FRAMES & (AUDIO_RB_FRAMES - 1)) == 0,
                "AUDIO_RB_FRAMES harus kelipatan pangkat 2!");

  static constexpr size_t CAPACITY = AUDIO_RB_FRAMES;
  static constexpr size_t MASK     = AUDIO_RB_FRAMES - 1;

  void reset();

  // Push sejumlah frame stereo int16 dari endpoint USB UAC2 RX
  void pushStereo16(const int16_t *interleavedSrc, size_t frames);

  // Pop tepat `frames` stereo float32 dengan kompensasi drift adaptif (interpolasi linear)
  // Mengembalikan true jika buffer sedang primed (audio valid), false jika silence (underrun)
  bool popBlockFloat(float *outL, float *outR, size_t frames);

  inline size_t count() const {
    portENTER_CRITICAL(&rbMux_);
    size_t c = count_;
    portEXIT_CRITICAL(&rbMux_);
    return c;
  }
  inline bool isPrimed() const { return primed_; }
  inline uint32_t underruns() const { return underrunCount_; }
  inline uint32_t overruns() const { return overrunCount_; }
  inline uint32_t driftDrops() const { return driftDropCount_; }
  inline uint32_t driftInserts() const { return driftInsertCount_; }

private:
  inline void popOne(int16_t &l, int16_t &r) {
    l = buf_[tail_ * 2];
    r = buf_[tail_ * 2 + 1];
    tail_ = (tail_ + 1) & MASK;
    count_--;
  }

  inline void peekOne(int16_t &l, int16_t &r) const {
    l = buf_[tail_ * 2];
    r = buf_[tail_ * 2 + 1];
  }

  mutable portMUX_TYPE rbMux_ = portMUX_INITIALIZER_UNLOCKED;
  int16_t buf_[CAPACITY * 2] = {0};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t count_ = 0;
  bool primed_ = false;
  bool needFadeIn_ = false;
  bool wasPrimed_ = false;
  uint8_t driftCooldown_ = 0;

  int16_t lastL_ = 0;
  int16_t lastR_ = 0;

  uint32_t underrunCount_ = 0;
  uint32_t overrunCount_ = 0;
  uint32_t driftDropCount_ = 0;
  uint32_t driftInsertCount_ = 0;
};

// ============================================================================
// 5. KELAS UTAMA AUDIO I/O LAYER
// ============================================================================
class AudioIO {
public:
  AudioIO() = default;

  // Langkah 1 (WAJIB pertama di setup() agar enumerasi USB PC tidak timeout)
  bool beginUSB(const char *manufacturer = USB_AUDIO_MANUFACTURER,
                const char *product      = USB_AUDIO_PRODUCT);

  // Langkah 2: Inisialisasi I2S0 (Master Full-Duplex) + I2S1 (Slave TX) sinkron fase
  bool beginI2S();

  // Langkah 3: Inisialisasi Sensor Suhu Internal ESP32-S3
  bool beginTempSensor();

  // Re-sinkronisasi ulang DMA I2S0 & I2S1 (0-sample-skew recovery jika diperlukan)
  bool resyncI2S();

  // Menjalankan 1 iterasi siklus audio (dipanggil terus-menerus di Task Core 1)
  void processOneBlock();

  // Pembacaan periodik sensor suhu internal ESP32-S3 (dipanggil dari Task Core 0)
  // Mengembalikan true jika pembacaan baru dilakukan pada interval TEMP_READ_INTERVAL_MS
  bool updateTemperature(uint32_t intervalMs = TEMP_READ_INTERVAL_MS);

  // Pasang callback DSP kustom (untuk integrasi DLMS 2-In 4-Out di Tahap 2)
  void setProcessCallback(AudioProcessCallback cb, void *userCtx = nullptr);

  // Pengaturan routing bawaan Tahap 1 (aktif saat processCallback == nullptr)
  void setDefaultRouting(DefaultRoutingMode mode);
  DefaultRoutingMode getDefaultRouting() const { return defaultRouting_; }

  // Master Output Mute & Gain sementara untuk pengujian Tahap 1
  void setMasterMute(bool muted) { masterMuted_ = muted; }
  bool isMasterMuted() const { return masterMuted_; }
  void setMasterGain(float linearGain);
  float getMasterGain() const { return masterGainTarget_; }

  // Ambil snapshot telemetri secara aman untuk UI / Serial / Web
  AudioTelemetry getTelemetry() const;
  void resetPeakTelemetry();

  // Helper konversi linear [0..1] ke dBFS (floor -96.0 dB)
  static float linearToDbfs(float linear);

  // Callback internal dari native Espressif USBAudioCard (dipanggil dari _uacReceiveTask / USB event)
  void onUsbRxData(const int16_t *interleavedSrc, size_t frames);
  void onUsbLinkChanged(bool mounted);
  void onUsbInterfaceEnable(bool isMic, bool enabled);

private:
  // Konfigurasi internal GPIO Matrix untuk berbagi BCLK (4) & LRCK (5) antara I2S0 & I2S1
  void configureSharedClockMatrix();

  // Pemrosesan routing default Tahap 1 jika belum ada callback DSP DLMS
  void runDefaultRouting(AudioBlock &block);

  // Status USB Audio native (USBAudioCard + TinyUSB built-in audio_device.c)
  bool usbInitialized_ = false;
  volatile bool usbMounted_        = false;
  volatile bool usbRxStreamActive_ = false;
  volatile bool usbTxStreamActive_ = false;

  // Hardware I2S Handles (mendukung IDF v5.x dan IDF v4.x)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  i2s_chan_handle_t i2s0TxHandle_ = nullptr;
  i2s_chan_handle_t i2s0RxHandle_ = nullptr;
  i2s_chan_handle_t i2s1TxHandle_ = nullptr;
  temperature_sensor_handle_t tempHandle_ = nullptr;
#endif
  bool i2sInitialized_ = false;
  bool tempInitialized_ = false;

  // Adaptive Ring Buffer & DC Blockers
  AdaptiveRingBuffer usbRxRb_;
  DcBlocker dcAdc_[AUDIO_IN_CHANNELS];

  // Buffer DMA & Blok Kerja (dialokasikan statis/member agar bebas heap fragmentation)
  alignas(16) int32_t i2sRxBuf_[AUDIO_STEREO_SAMPLES]  = {0};
  alignas(16) int32_t i2sTx0Buf_[AUDIO_STEREO_SAMPLES] = {0};
  alignas(16) int32_t i2sTx1Buf_[AUDIO_STEREO_SAMPLES] = {0};
  alignas(16) int16_t usbTxRaw_[AUDIO_STEREO_SAMPLES]  = {0};
  AudioBlock block_ = {};

  // DSP Callback & Stage-1 Controls
  AudioProcessCallback processCb_ = nullptr;
  void *processCtx_ = nullptr;
  volatile DefaultRoutingMode defaultRouting_ = DefaultRoutingMode::USB_TO_ALL4;
  volatile bool masterMuted_ = false;
  volatile float masterGainTarget_ = 1.0f;
  float masterGainSmooth_ = 1.0f;

  // Telemetri & State Sensor Suhu
  mutable portMUX_TYPE telemetryMux_ = portMUX_INITIALIZER_UNLOCKED;
  AudioTelemetry telemetry_ = {};
  uint32_t lastTempReadMs_ = 0;
  bool tempHasFirstSample_ = false;
};

} // namespace audio_io
