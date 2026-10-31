#include "AudioTools.h"
#include "AudioTools/Communication/USB/USBAudioStream.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <cmath>

// ==========================================
// 1. DEFINISI PIN & AUDIO FORMAT
// ==========================================
#define I2S_MCLK 16
#define I2S_BCLK 17
#define I2S_LRCK 18
#define I2S_DIN 14
#define I2S_DOUT 15

#define OLED_SDA 8
#define OLED_SCL 9

#define ENC_CLK 5
#define ENC_DT 6
#define ENC_SW 7

#define SAMPLE_RATE 48000
#define BIT_DEPTH 16
#define CHANNELS 2

#define DEBUG_AUDIO 0

// Mode routing input & output
enum InputMode {
  IN_STEREO = 0,    // 1 Stereo In (L -> In L, R -> In R asli PCM1808)
  IN_MONO_L = 1,    // 1 Mono In L (Channel L diduplikasi ke L & R)
  IN_MONO_R = 2,    // 1 Mono In R (Channel R diduplikasi ke L & R)
  IN_DUAL_MONO = 3  // 2 Mono In ((L + R)/2 diduplikasi ke L & R)
};

enum OutputMode {
  OUT_STEREO = 0,   // Stereo Out (L -> DAC L, R -> DAC R)
  OUT_MONO_MIX = 1  // 2 Mono / Mix Out ((L + R)/2 dikirim ke DAC L & R)
};

// 4 Channel Audio Equalizer
enum EqChannel {
  EQ_CH_IN1 = 0,   // Input 1 (Mic Left PCM1808)
  EQ_CH_IN2 = 1,   // Input 2 (Mic Right PCM1808)
  EQ_CH_OUT1 = 2,  // Output 1 (DAC Left)
  EQ_CH_OUT2 = 3   // Output 2 (DAC Right)
};

struct ChannelEQ {
  int8_t bass;   // -12 s/d +12 dB (Low Shelf @ 120 Hz)
  int8_t mid;    // -12 s/d +12 dB (Peaking @ 1000 Hz, Q=1.0)
  int8_t treble; // -12 s/d +12 dB (High Shelf @ 6000 Hz)
};

// ==========================================
// 2. OBJEK & VARIABEL GLOBAL
// ==========================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
ESP32Encoder encoder;
USBAudioStream usb_audio;
Preferences prefs;

volatile int currentVolume = 50;
volatile bool isMuted = false;

// 4 Peak terpisah untuk 2 VU IN dan 2 VU OUT
volatile int peakInL = 0;
volatile int peakInR = 0;
volatile int peakOutL = 0;
volatile int peakOutR = 0;

// Setting routing audio (Default Mono L - koneksi 1 mic paling umum)
volatile InputMode currentInputMode = IN_MONO_L;
volatile OutputMode currentOutputMode = OUT_STEREO;

// Pengaturan 4 Channel Equalizer (IN 1, IN 2, OUT 1, OUT 2)
ChannelEQ channelEqSettings[4] = {
  {0, 0, 0}, // IN 1
  {0, 0, 0}, // IN 2
  {0, 0, 0}, // OUT 1
  {0, 0, 0}  // OUT 2
};

// Flag sinkronisasi koefisien audio filter antar core
volatile bool eqNeedsUpdate = true;

// Auto-save setting debounce timer
bool settingsChanged = false;
unsigned long lastChangeTime = 0;

// Halaman aktif UI (0 = Status Device, 1 = VU Meters, 2 = Routing Menu, 3 = EQ Settings)
volatile int currentPage = 0;

// ==========================================
// 3. RING BUFFER (jitter buffer USB -> I2S)
// ==========================================
constexpr size_t BLOCK_FRAMES = 128;               // 128 frame stereo (~2.67 ms)
constexpr size_t BLOCK_SAMPLES = BLOCK_FRAMES * 2; // L + R
constexpr size_t RB_FRAMES = 2048;                 // kapasitas ring buffer (~42 ms)
constexpr size_t TARGET_FILL = 384;                // level target (~8 ms)
constexpr size_t FILL_TOLERANCE = 192;             // toleransi sebelum koreksi drift

static int16_t rb[RB_FRAMES * 2];
static size_t rbHead = 0, rbTail = 0, rbCount = 0;

static inline void rbReset() { rbHead = rbTail = rbCount = 0; }

static inline void rbPush(const int16_t *src, size_t frames) {
  for (size_t i = 0; i < frames; i++) {
    if (rbCount >= RB_FRAMES) { // penuh: buang frame tertua
      rbTail = (rbTail + 1) % RB_FRAMES;
      rbCount--;
    }
    rb[rbHead * 2] = src[i * 2];
    rb[rbHead * 2 + 1] = src[i * 2 + 1];
    rbHead = (rbHead + 1) % RB_FRAMES;
    rbCount++;
  }
}

static inline void rbPop(int16_t &l, int16_t &r) {
  l = rb[rbTail * 2];
  r = rb[rbTail * 2 + 1];
  rbTail = (rbTail + 1) % RB_FRAMES;
  rbCount--;
}

// ==========================================
// 4. KONFIGURASI I2S
// ==========================================
void setupI2S() {
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Hardware wajib 32-bit slot
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 8,
      .dma_buf_len = 128, // dalam frame; sama dengan BLOCK_FRAMES
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = SAMPLE_RATE * 256}; // MCLK 12.288 MHz untuk PCM1808 (256fs)

  i2s_pin_config_t pin_config = {.mck_io_num = I2S_MCLK,
                                 .bck_io_num = I2S_BCLK,
                                 .ws_io_num = I2S_LRCK,
                                 .data_out_num = I2S_DOUT,
                                 .data_in_num = I2S_DIN};

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // Drive kuat supaya tepi sinyal tegas di kabel jumper
  gpio_set_drive_capability((gpio_num_t)I2S_BCLK, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_LRCK, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_DOUT, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_MCLK, GPIO_DRIVE_CAP_2);
}

// ==========================================
// 5. DSP: DC BLOCKER & BIQUAD EQUALIZER
// ==========================================
struct DcBlocker {
  float xPrev = 0, yPrev = 0;
  inline float process(float x) {
    float y = x - xPrev + 0.995f * yPrev;
    xPrev = x;
    yPrev = y;
    return y;
  }
};

// Filter Biquad IIR Direct Form II Transposed
class Biquad {
public:
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float s1 = 0.0f, s2 = 0.0f;
  bool active = false;

  inline float process(float x) {
    if (!active) return x; // 0 dB bypass: 0 CPU overhead
    float y = b0 * x + s1;
    s1 = b1 * x - a1 * y + s2;
    s2 = b2 * x - a2 * y;
    if (fabsf(s1) < 1.0e-15f) s1 = 0.0f;
    if (fabsf(s2) < 1.0e-15f) s2 = 0.0f;
    return y;
  }

  void reset() {
    s1 = 0.0f;
    s2 = 0.0f;
  }

  void setLowShelf(float f0, float Fs, float gain_dB) {
    if (fabsf(gain_dB) < 0.1f) {
      active = false;
      return;
    }
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * (float)M_PI * f0 / Fs;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 1.41421356f;
    float two_sqrt_A_alpha = 2.0f * sqrtf(A) * alpha;

    float a0 = (A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
    b0 = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha)) / a0;
    b1 = (2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / a0;
    b2 = (A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha)) / a0;
    a1 = (-2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / a0;
    a2 = ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha) / a0;
    active = true;
  }

  void setPeaking(float f0, float Fs, float Q, float gain_dB) {
    if (fabsf(gain_dB) < 0.1f) {
      active = false;
      return;
    }
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * (float)M_PI * f0 / Fs;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / (2.0f * Q);

    float a0 = 1.0f + alpha / A;
    b0 = (1.0f + alpha * A) / a0;
    b1 = (-2.0f * cos_w0) / a0;
    b2 = (1.0f - alpha * A) / a0;
    a1 = (-2.0f * cos_w0) / a0;
    a2 = (1.0f - alpha / A) / a0;
    active = true;
  }

  void setHighShelf(float f0, float Fs, float gain_dB) {
    if (fabsf(gain_dB) < 0.1f) {
      active = false;
      return;
    }
    float A = powf(10.0f, gain_dB / 40.0f);
    float w0 = 2.0f * (float)M_PI * f0 / Fs;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha = sin_w0 / 1.41421356f;
    float two_sqrt_A_alpha = 2.0f * sqrtf(A) * alpha;

    float a0 = (A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrt_A_alpha;
    b0 = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrt_A_alpha)) / a0;
    b1 = (-2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0)) / a0;
    b2 = (A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrt_A_alpha)) / a0;
    a1 = (2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0)) / a0;
    a2 = ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrt_A_alpha) / a0;
    active = true;
  }
};

// 3-Band Equalizer untuk 1 Channel Audio
class Equalizer3Band {
public:
  Biquad low;
  Biquad mid;
  Biquad high;

  inline float process(float x) {
    x = low.process(x);
    x = mid.process(x);
    x = high.process(x);
    return x;
  }

  void update(int bass_dB, int mid_dB, int treb_dB) {
    low.setLowShelf(120.0f, (float)SAMPLE_RATE, (float)bass_dB);
    mid.setPeaking(1000.0f, (float)SAMPLE_RATE, 1.0f, (float)mid_dB);
    high.setHighShelf(6000.0f, (float)SAMPLE_RATE, (float)treb_dB);
  }

  void reset() {
    low.reset();
    mid.reset();
    high.reset();
  }
};

// Instance EQ untuk masing-masing 4 channel audio
Equalizer3Band eqInL;
Equalizer3Band eqInR;
Equalizer3Band eqOutL;
Equalizer3Band eqOutR;

// ==========================================
// 6. PENYIMPANAN PENGATURAN (NVS FLASH)
// ==========================================
void loadSettings() {
  prefs.begin("audio_cfg", true); // Read-only
  if (prefs.isKey("eq_data")) {
    prefs.getBytes("eq_data", channelEqSettings, sizeof(channelEqSettings));
  } else {
    memset(channelEqSettings, 0, sizeof(channelEqSettings));
  }
  
  int loadedVol = prefs.getInt("vol", 50);
  if (loadedVol <= 0) loadedVol = 50; // Pastikan volume tidak 0 agar mic langsung bersuara
  currentVolume = constrain(loadedVol, 5, 100);

  int loadedInMode = prefs.getInt("in_mode", (int)IN_MONO_L);
  if (loadedInMode < 0 || loadedInMode > 3) loadedInMode = (int)IN_MONO_L;
  currentInputMode = (InputMode)loadedInMode;

  int loadedOutMode = prefs.getInt("out_mode", (int)OUT_STEREO);
  if (loadedOutMode < 0 || loadedOutMode > 1) loadedOutMode = (int)OUT_STEREO;
  currentOutputMode = (OutputMode)loadedOutMode;

  prefs.end();

  // Sanitasi EQ range
  for (int ch = 0; ch < 4; ch++) {
    channelEqSettings[ch].bass = constrain(channelEqSettings[ch].bass, -12, 12);
    channelEqSettings[ch].mid = constrain(channelEqSettings[ch].mid, -12, 12);
    channelEqSettings[ch].treble = constrain(channelEqSettings[ch].treble, -12, 12);
  }
  eqNeedsUpdate = true;
}

void saveSettings() {
  prefs.begin("audio_cfg", false); // Read-write
  prefs.putBytes("eq_data", (const void*)channelEqSettings, sizeof(channelEqSettings));
  prefs.putInt("vol", currentVolume);
  prefs.putInt("in_mode", (int)currentInputMode);
  prefs.putInt("out_mode", (int)currentOutputMode);
  prefs.end();
}

// ==========================================
// 7. AUDIO TASK (DSP & USB I/O) - CORE 1
// ==========================================
void audioTask(void *pvParameters) {
  size_t bytes_read = 0;
  size_t bytes_written = 0;

  static int32_t i2s_rx_buf[BLOCK_SAMPLES];
  static int16_t usb_tx_buf[BLOCK_SAMPLES];
  static int16_t usb_rx_buf[BLOCK_SAMPLES];
  static int32_t i2s_tx_buf[BLOCK_SAMPLES];

  DcBlocker dcL, dcR;
  float gainSmooth = (currentVolume * 5) / 100.0f; // Mulai langsung di volume target
  bool primed = false;
  int16_t lastL = 0, lastR = 0;

  while (1) {
    // ---- 0. Update koefisien EQ pada boundary block secara atomic ----
    if (eqNeedsUpdate) {
      eqNeedsUpdate = false;
      eqInL.update(channelEqSettings[EQ_CH_IN1].bass,
                   channelEqSettings[EQ_CH_IN1].mid,
                   channelEqSettings[EQ_CH_IN1].treble);
      eqInR.update(channelEqSettings[EQ_CH_IN2].bass,
                   channelEqSettings[EQ_CH_IN2].mid,
                   channelEqSettings[EQ_CH_IN2].treble);
      eqOutL.update(channelEqSettings[EQ_CH_OUT1].bass,
                    channelEqSettings[EQ_CH_OUT1].mid,
                    channelEqSettings[EQ_CH_OUT1].treble);
      eqOutR.update(channelEqSettings[EQ_CH_OUT2].bass,
                    channelEqSettings[EQ_CH_OUT2].mid,
                    channelEqSettings[EQ_CH_OUT2].treble);
    }

    // ---- 1. Kuras data USB (PC -> ESP) ke ring buffer ----
    if (usb_audio.mounted()) {
      int avail;
      while ((avail = usb_audio.available()) >= 4) {
        size_t n = (size_t)avail;
        if (n > sizeof(usb_rx_buf))
          n = sizeof(usb_rx_buf);
        n -= n % 4; // kelipatan 4 byte (1 frame stereo 16-bit)
        size_t got = usb_audio.readBytes((uint8_t *)usb_rx_buf, n);
        got -= got % 4;
        if (!got)
          break;
        rbPush(usb_rx_buf, got / 4);
      }
    } else {
      rbReset();
      primed = false;
    }

    // ---- 2. ADC (I2S RX) -> USB mic (Metronom loop) ----
    esp_err_t err = i2s_read(I2S_NUM_0, i2s_rx_buf, sizeof(i2s_rx_buf),
                             &bytes_read, pdMS_TO_TICKS(20));
    if (err != ESP_OK || bytes_read < sizeof(i2s_rx_buf)) {
      vTaskDelay(1);
      continue;
    }

    float targetGain = isMuted ? 0.0f : (currentVolume * 5) / 100.0f;
    int pkInL = 0, pkInR = 0;

    InputMode inMode = currentInputMode; // Snapshot mode input

    for (size_t i = 0; i < BLOCK_FRAMES; i++) {
      float rawL = (float)(i2s_rx_buf[i * 2] >> 16);
      float rawR = (float)(i2s_rx_buf[i * 2 + 1] >> 16);

      // Filter DC Offset bawaan ADC
      rawL = dcL.process(rawL);
      rawR = dcR.process(rawR);

      // EQ fisik independen sebelum routing:
      // In 1 (Mic L) diproses oleh EQ IN 1
      // In 2 (Mic R) diproses oleh EQ IN 2
      rawL = eqInL.process(rawL);
      rawR = eqInR.process(rawR);

      float l = rawL;
      float r = rawR;

      // Routing Input Channel Mode
      switch (inMode) {
        case IN_MONO_L:
          r = l; // Mic Left diduplikasi ke L & R
          break;
        case IN_MONO_R:
          l = r; // Mic Right diduplikasi ke L & R
          break;
        case IN_DUAL_MONO: {
          float mix = (rawL + rawR) * 0.5f;
          l = mix;
          r = mix;
          break;
        }
        case IN_STEREO:
        default:
          // Stereo asli (L tetap L, R tetap R)
          break;
      }

      // Haluskan perubahan gain
      gainSmooth += (targetGain - gainSmooth) * 0.002f;

      l *= gainSmooth;
      r *= gainSmooth;

      if (l > 32767.0f)  l = 32767.0f;
      if (l < -32768.0f) l = -32768.0f;
      if (r > 32767.0f)  r = 32767.0f;
      if (r < -32768.0f) r = -32768.0f;

      int16_t sl = (int16_t)l;
      int16_t sr = (int16_t)r;
      usb_tx_buf[i * 2] = sl;
      usb_tx_buf[i * 2 + 1] = sr;

      int al = abs((int)sl);
      int ar = abs((int)sr);
      if (al > pkInL) pkInL = al;
      if (ar > pkInR) pkInR = ar;
    }

    if (usb_audio.mounted()) {
      usb_audio.write((const uint8_t *)usb_tx_buf, sizeof(usb_tx_buf));
    }

    // ---- 3. Ring buffer -> DAC (I2S TX), tepat 128 frame ----
    if (!primed && rbCount >= TARGET_FILL)
      primed = true;
    if (primed && rbCount < BLOCK_FRAMES + 2) { // underrun
      primed = false;
    }

    int pkOutL = 0, pkOutR = 0;
    OutputMode outMode = currentOutputMode;

    if (primed) {
      int adj = 0;
      if (rbCount > TARGET_FILL + FILL_TOLERANCE)
        adj = 1; // terlalu penuh -> buang 1 frame
      else if (rbCount < TARGET_FILL - FILL_TOLERANCE)
        adj = -1; // terlalu kosong -> ulang 1 frame

      for (size_t i = 0; i < BLOCK_FRAMES; i++) {
        if (i == 0 && adj == 1) {
          int16_t a, b;
          rbPop(a, b);
        }
        if (!(i == 0 && adj == -1))
          rbPop(lastL, lastR);

        int16_t outL = lastL;
        int16_t outR = lastR;

        // Routing Output Channel Mode
        if (outMode == OUT_MONO_MIX) {
          int32_t mix = ((int32_t)outL + (int32_t)outR) / 2;
          outL = (int16_t)mix;
          outR = (int16_t)mix;
        }

        // EQ OUT 1 (DAC Left) & EQ OUT 2 (DAC Right) independen
        float fl = (float)outL;
        float fr = (float)outR;

        fl = eqOutL.process(fl);
        fr = eqOutR.process(fr);

        if (fl > 32767.0f)  fl = 32767.0f;
        if (fl < -32768.0f) fl = -32768.0f;
        if (fr > 32767.0f)  fr = 32767.0f;
        if (fr < -32768.0f) fr = -32768.0f;

        outL = (int16_t)fl;
        outR = (int16_t)fr;

        i2s_tx_buf[i * 2] = (int32_t)outL * 65536;
        i2s_tx_buf[i * 2 + 1] = (int32_t)outR * 65536;

        int al = abs((int)outL);
        int ar = abs((int)outR);
        if (al > pkOutL) pkOutL = al;
        if (ar > pkOutR) pkOutR = ar;
      }
    } else {
      memset(i2s_tx_buf, 0, sizeof(i2s_tx_buf));
    }

    i2s_write(I2S_NUM_0, i2s_tx_buf, sizeof(i2s_tx_buf), &bytes_written, portMAX_DELAY);

    // Update level peak independen untuk IN dan OUT
    peakInL = pkInL;
    peakInR = pkInR;
    peakOutL = pkOutL;
    peakOutR = pkOutR;
  }
}

// ==========================================
// 8. FUNGSI GAMBAR HALAMAN OLED (128x64)
// ==========================================

const char* getInputModeName(InputMode mode) {
  switch (mode) {
    case IN_STEREO:    return "Stereo In";
    case IN_MONO_L:    return "Mono L In";
    case IN_MONO_R:    return "Mono R In";
    case IN_DUAL_MONO: return "2 Mono In";
    default:           return "Mono L In";
  }
}

const char* getOutputModeName(OutputMode mode) {
  switch (mode) {
    case OUT_STEREO:   return "Stereo Out";
    case OUT_MONO_MIX: return "Mono Mix";
    default:           return "Stereo Out";
  }
}

const char* getChannelName(int ch) {
  switch (ch) {
    case EQ_CH_IN1:  return "IN 1";
    case EQ_CH_IN2:  return "IN 2";
    case EQ_CH_OUT1: return "OUT 1";
    case EQ_CH_OUT2: return "OUT 2";
    default:         return "IN 1";
  }
}

// Halaman 1: Status Device (1/4)
void drawPageStatus() {
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 9, "DEVICE STATUS");
  u8g2.drawStr(105, 9, "1/4");
  u8g2.drawHLine(0, 11, 128);

  u8g2.drawStr(0, 22, "USB :");
  if (usb_audio.mounted()) {
    u8g2.drawStr(36, 22, "48k 16b UAC2");
  } else {
    u8g2.drawStr(36, 22, "WAITING...");
  }

  u8g2.drawStr(0, 33, "MIC :");
  if (usb_audio.isStreamingActiveTx()) {
    u8g2.drawBox(36, 24, 26, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(38, 33, "REC");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(36, 33, "IDLE");
  }

  u8g2.drawStr(68, 33, "SPK:");
  if (usb_audio.isStreamingActiveRx() || rbCount >= BLOCK_FRAMES) {
    u8g2.drawBox(96, 24, 30, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(98, 33, "PLAY");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(96, 33, "IDLE");
  }

  u8g2.drawStr(0, 44, "IN  :");
  u8g2.drawStr(36, 44, getInputModeName(currentInputMode));

  u8g2.drawStr(0, 54, "OUT :");
  u8g2.drawStr(36, 54, getOutputModeName(currentOutputMode));

  u8g2.drawStr(0, 64, "VOL :");
  if (isMuted) {
    u8g2.drawBox(36, 55, 38, 9);
    u8g2.setDrawColor(0);
    u8g2.drawStr(38, 64, "MUTED");
    u8g2.setDrawColor(1);
  } else {
    char volStr[16];
    snprintf(volStr, sizeof(volStr), "%d%%", currentVolume);
    u8g2.drawStr(36, 64, volStr);

    int barW = constrain(map(currentVolume, 0, 100, 0, 55), 0, 55);
    u8g2.drawFrame(68, 56, 57, 8);
    u8g2.drawBox(69, 57, barW, 6);
  }
}

// Halaman 2: 4 Channel VU Meter (2/4)
void drawPageVUMeters(int sInL, int sInR, int sOutL, int sOutR) {
  u8g2.setFont(u8g2_font_5x8_tr);

  u8g2.drawStr(0, 8, "VU METERS");
  if (isMuted) {
    u8g2.drawStr(58, 8, "[MUTED]");
  } else {
    char vStr[16];
    snprintf(vStr, sizeof(vStr), "VOL:%d%%", currentVolume);
    u8g2.drawStr(58, 8, vStr);
  }
  u8g2.drawStr(110, 8, "2/4");
  u8g2.drawHLine(0, 10, 128);

  constexpr int barX = 26;
  constexpr int barMaxW = 100;

  // 1. INPUT LEFT
  int wInL = constrain(map(sInL, 0, 32767, 0, barMaxW - 2), 0, barMaxW - 2);
  u8g2.drawStr(0, 20, "IN-L");
  u8g2.drawFrame(barX, 13, barMaxW, 9);
  u8g2.drawBox(barX + 1, 14, wInL, 7);

  // 2. INPUT RIGHT
  int wInR = constrain(map(sInR, 0, 32767, 0, barMaxW - 2), 0, barMaxW - 2);
  u8g2.drawStr(0, 32, "IN-R");
  u8g2.drawFrame(barX, 25, barMaxW, 9);
  u8g2.drawBox(barX + 1, 26, wInR, 7);

  // 3. OUTPUT LEFT
  int wOutL = constrain(map(sOutL, 0, 32767, 0, barMaxW - 2), 0, barMaxW - 2);
  u8g2.drawStr(0, 44, "OT-L");
  u8g2.drawFrame(barX, 37, barMaxW, 9);
  u8g2.drawBox(barX + 1, 38, wOutL, 7);

  // 4. OUTPUT RIGHT
  int wOutR = constrain(map(sOutR, 0, 32767, 0, barMaxW - 2), 0, barMaxW - 2);
  u8g2.drawStr(0, 56, "OT-R");
  u8g2.drawFrame(barX, 49, barMaxW, 9);
  u8g2.drawBox(barX + 1, 50, wOutR, 7);

  u8g2.drawStr(barX, 64, "-40dB");
  u8g2.drawStr(barX + 38, 64, "-12dB");
  u8g2.drawStr(barX + 85, 64, "0dB");
}

// Halaman 3: Menu Konfigurasi Routing (3/4)
void drawPageMenu(int cursor) {
  u8g2.setFont(u8g2_font_6x10_tf);

  u8g2.drawStr(0, 9, "ROUTING MENU");
  u8g2.drawStr(105, 9, "3/4");
  u8g2.drawHLine(0, 11, 128);

  // Menu Baris 0: Input Mode
  if (cursor == 0) {
    u8g2.drawBox(0, 13, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 22, "> IN :");
    u8g2.drawStr(44, 22, getInputModeName(currentInputMode));
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 22, "  IN :");
    u8g2.drawStr(44, 22, getInputModeName(currentInputMode));
  }

  // Menu Baris 1: Output Mode
  if (cursor == 1) {
    u8g2.drawBox(0, 25, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 34, "> OUT:");
    u8g2.drawStr(44, 34, getOutputModeName(currentOutputMode));
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 34, "  OUT:");
    u8g2.drawStr(44, 34, getOutputModeName(currentOutputMode));
  }

  // Menu Baris 2: Tombol ke Halaman EQ
  if (cursor == 2) {
    u8g2.drawBox(0, 37, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 46, "> [ NEXT: EQ PAGE ]");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 46, "  [ NEXT: EQ PAGE ]");
  }

  // Menu Baris 3: Tombol Back
  if (cursor == 3) {
    u8g2.drawBox(0, 49, 128, 11);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 58, "> [ BACK TO STATUS ]");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 58, "  [ BACK TO STATUS ]");
  }
}

// Halaman 4: Equalizer Settings (4/4)
void drawPageEQ(int cursor, bool editing, int ch) {
  u8g2.setFont(u8g2_font_6x10_tf);

  char hdr[24];
  snprintf(hdr, sizeof(hdr), "EQ: %s", getChannelName(ch));
  u8g2.drawStr(0, 9, hdr);
  u8g2.drawStr(105, 9, "4/4");
  u8g2.drawHLine(0, 11, 128);

  // Line 0: Channel Selector
  if (cursor == 0) {
    u8g2.drawBox(0, 13, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 21, "> CH :");
    char chStr[20];
    snprintf(chStr, sizeof(chStr), "< %s > (Click)", getChannelName(ch));
    u8g2.drawStr(42, 21, chStr);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 21, "  CH :");
    u8g2.drawStr(42, 21, getChannelName(ch));
  }

  // Draw 3 Bands: BASS, MID, TREBLE
  const char* bandNames[3] = {"BAS", "MID", "TRE"};
  int8_t bandVals[3] = {
    channelEqSettings[ch].bass,
    channelEqSettings[ch].mid,
    channelEqSettings[ch].treble
  };

  for (int b = 0; b < 3; b++) {
    int itemIdx = b + 1; // 1, 2, 3
    int yBox = 23 + (b * 10);
    int yTxt = yBox + 8;
    int val = bandVals[b];

    bool isSelected = (cursor == itemIdx);

    if (isSelected && editing) {
      u8g2.drawBox(0, yBox, 128, 10);
      u8g2.setDrawColor(0);
    } else if (isSelected && !editing) {
      u8g2.drawFrame(0, yBox, 128, 10);
    }

    char lbl[12];
    if (isSelected) {
      snprintf(lbl, sizeof(lbl), "%c %s:", editing ? '*' : '>', bandNames[b]);
    } else {
      snprintf(lbl, sizeof(lbl), "  %s:", bandNames[b]);
    }
    u8g2.drawStr(2, yTxt, lbl);

    char valStr[12];
    if (val > 0) {
      snprintf(valStr, sizeof(valStr), "+%ddB", val);
    } else {
      snprintf(valStr, sizeof(valStr), "%ddB", val);
    }
    u8g2.drawStr(38, yTxt, valStr);

    constexpr int barX = 74;
    constexpr int barW = 50;
    constexpr int barMid = barX + (barW / 2);
    constexpr int barMaxHalf = (barW / 2) - 2;

    u8g2.drawFrame(barX, yBox + 1, barW, 8);
    u8g2.drawVLine(barMid, yBox + 1, 8);

    if (val > 0) {
      int fillW = map(val, 0, 12, 0, barMaxHalf);
      if (fillW > barMaxHalf) fillW = barMaxHalf;
      u8g2.drawBox(barMid + 1, yBox + 2, fillW, 6);
    } else if (val < 0) {
      int fillW = map(-val, 0, 12, 0, barMaxHalf);
      if (fillW > barMaxHalf) fillW = barMaxHalf;
      u8g2.drawBox(barMid - fillW, yBox + 2, fillW, 6);
    }

    if (isSelected && editing) {
      u8g2.setDrawColor(1);
    }
  }

  // Line 4: Back Button
  if (cursor == 4) {
    u8g2.drawBox(0, 53, 128, 10);
    u8g2.setDrawColor(0);
    u8g2.drawStr(2, 61, "> [ BACK TO STATUS ]");
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(2, 61, "  [ BACK TO STATUS ]");
  }
}

// ==========================================
// 9. UI TASK (OLED & Encoder) - CORE 0
// ==========================================
void uiTask(void *pvParameters) {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  u8g2.begin();

  int smoothInL = 0, smoothInR = 0;
  int smoothOutL = 0, smoothOutR = 0;

  // Inisialisasi posisi encoder awal secara tepat (mencegah phantom diff saat boot)
  int lastEncoderCount = encoder.getCount() / 2;
  int menuCursor = 0;

  int selectedEqChannel = 0;
  int eqCursor = 0;
  bool eqEditing = false;

  bool btnPrev = false;
  unsigned long btnPressStart = 0;
  bool btnHandled = false;

  while (1) {
    // 1. Baca Rotary Encoder secara relatif
    int curCount = encoder.getCount() / 2;
    int diff = curCount - lastEncoderCount;
    lastEncoderCount = curCount;

    if (diff != 0) {
      if (currentPage == 0 || currentPage == 1) {
        currentVolume = constrain(currentVolume + (diff * 2), 5, 100);
        settingsChanged = true;
        lastChangeTime = millis();
      } else if (currentPage == 2) {
        menuCursor += diff;
        if (menuCursor < 0) menuCursor = 3;
        if (menuCursor > 3) menuCursor = 0;
      } else if (currentPage == 3) {
        if (!eqEditing) {
          eqCursor += diff;
          if (eqCursor < 0) eqCursor = 4;
          if (eqCursor > 4) eqCursor = 0;
        } else {
          if (eqCursor == 1) {
            channelEqSettings[selectedEqChannel].bass =
                constrain(channelEqSettings[selectedEqChannel].bass + diff, -12, 12);
          } else if (eqCursor == 2) {
            channelEqSettings[selectedEqChannel].mid =
                constrain(channelEqSettings[selectedEqChannel].mid + diff, -12, 12);
          } else if (eqCursor == 3) {
            channelEqSettings[selectedEqChannel].treble =
                constrain(channelEqSettings[selectedEqChannel].treble + diff, -12, 12);
          }
          eqNeedsUpdate = true;
          settingsChanged = true;
          lastChangeTime = millis();
        }
      }
    }

    // 2. Baca Tombol Encoder
    bool btnNow = (digitalRead(ENC_SW) == LOW);

    if (btnNow && !btnPrev) {
      btnPressStart = millis();
      btnHandled = false;
    } else if (btnNow && btnPrev) {
      if (!btnHandled && (millis() - btnPressStart >= 600)) {
        btnHandled = true; // Long Press
        if (currentPage == 2) {
          currentPage = 0;
          if (settingsChanged) { saveSettings(); settingsChanged = false; }
        } else if (currentPage == 3) {
          if (eqEditing) {
            eqEditing = false;
          } else {
            currentPage = 0;
            if (settingsChanged) { saveSettings(); settingsChanged = false; }
          }
        } else {
          isMuted = !isMuted;
        }
      }
    } else if (!btnNow && btnPrev) {
      unsigned long duration = millis() - btnPressStart;
      if (!btnHandled && duration >= 40) { // Short Click
        if (currentPage == 0) {
          currentPage = 1; // Pindah ke VU Meter
        } else if (currentPage == 1) {
          currentPage = 2; // Pindah ke Routing Menu
          menuCursor = 0;
        } else if (currentPage == 2) {
          if (menuCursor == 0) {
            currentInputMode = (InputMode)((currentInputMode + 1) % 4);
            settingsChanged = true;
            lastChangeTime = millis();
          } else if (menuCursor == 1) {
            currentOutputMode = (OutputMode)((currentOutputMode + 1) % 2);
            settingsChanged = true;
            lastChangeTime = millis();
          } else if (menuCursor == 2) {
            currentPage = 3; // Pindah ke EQ Page
            eqCursor = 0;
            eqEditing = false;
          } else if (menuCursor == 3) {
            currentPage = 0; // Kembali ke Status
            if (settingsChanged) { saveSettings(); settingsChanged = false; }
          }
        } else if (currentPage == 3) {
          if (eqEditing) {
            eqEditing = false;
          } else {
            if (eqCursor == 0) {
              selectedEqChannel = (selectedEqChannel + 1) % 4;
            } else if (eqCursor >= 1 && eqCursor <= 3) {
              eqEditing = true;
            } else if (eqCursor == 4) {
              currentPage = 0;
              if (settingsChanged) { saveSettings(); settingsChanged = false; }
            }
          }
        }
      }
    }
    btnPrev = btnNow;

    // 3. Auto-save debounce (simpan 1.5s setelah selesai memutar encoder)
    if (settingsChanged && (millis() - lastChangeTime >= 1500)) {
      saveSettings();
      settingsChanged = false;
    }

    // 4. Smooth Decay untuk VU Meter
    int rawInL = peakInL;
    int rawInR = peakInR;
    int rawOutL = peakOutL;
    int rawOutR = peakOutR;

    if (rawInL > smoothInL)   smoothInL = rawInL;   else smoothInL = (smoothInL * 85) / 100;
    if (rawInR > smoothInR)   smoothInR = rawInR;   else smoothInR = (smoothInR * 85) / 100;
    if (rawOutL > smoothOutL) smoothOutL = rawOutL; else smoothOutL = (smoothOutL * 85) / 100;
    if (rawOutR > smoothOutR) smoothOutR = rawOutR; else smoothOutR = (smoothOutR * 85) / 100;

    // 5. Render Layar OLED
    u8g2.clearBuffer();

    switch (currentPage) {
      case 0:
        drawPageStatus();
        break;
      case 1:
        drawPageVUMeters(smoothInL, smoothInR, smoothOutL, smoothOutR);
        break;
      case 2:
        drawPageMenu(menuCursor);
        break;
      case 3:
        drawPageEQ(eqCursor, eqEditing, selectedEqChannel);
        break;
      default:
        currentPage = 0;
        break;
    }

    u8g2.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
  }
}

// ==========================================
// 10. MAIN SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  // 1. Inisialisasi USB Audio PERTAMA KALI agar enumerasi PC tidak timeout
  AudioInfo audio_info(SAMPLE_RATE, CHANNELS, BIT_DEPTH);
  auto config = usb_audio.defaultConfig(RXTX_MODE);
  config.copyFrom(audio_info);
  config.manufacturer = "aliefadityanugraha";
  config.product = "ESP32-S3 Audio Interface";
  config.begin_usb = true;
  usb_audio.begin(config);

  // 2. Setup Pin Rotary Encoder
  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  encoder.attachHalfQuad(ENC_DT, ENC_CLK);

  // 3. Setup Hardware I2S (MCLK, BCLK, LRCK, DIN, DOUT)
  setupI2S();

  // 4. Muat Pengaturan Tersimpan dari Flash (Volume, Routing, EQ)
  loadSettings();
  encoder.setCount(currentVolume * 2);

  // 5. Pisahkan task audio ke Core 1 (prioritas tertinggi) dan UI ke Core 0
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(uiTask, "UITask", 8192, NULL, 1, NULL, 0); // 8KB stack untuk keamanan NVS
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
