#include "AudioTools.h"
#include "AudioTools/Communication/USB/USBAudioStream.h"
#include "driver/gpio.h"
#include "driver/i2s.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <U8g2lib.h>
#include <Wire.h>

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

// Sumber mic: 0 = channel L diduplikasi ke R, 1 = channel R diduplikasi ke L, 2
// = stereo asli
#define MIC_SOURCE 0

// Aktifkan untuk melihat debug di Serial Monitor
#define DEBUG_AUDIO 0

// ==========================================
// 2. OBJEK GLOBAL
// ==========================================
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL,
                                         OLED_SDA);
ESP32Encoder encoder;

// Objek USB Audio (1 Input Mic, 1 Output Speaker)
USBAudioStream usb_audio;

volatile int currentVolume = 50;
volatile int peakLeft = 0;
volatile int peakRight = 0;
volatile bool isMuted = false;

// ==========================================
// 3. RING BUFFER (jitter buffer USB -> I2S)
// ==========================================
constexpr size_t BLOCK_FRAMES = 128; // 128 frame stereo (~2.67 ms)
constexpr size_t BLOCK_SAMPLES = BLOCK_FRAMES * 2; // L + R
constexpr size_t RB_FRAMES = 2048;     // kapasitas ring buffer (~42 ms)
constexpr size_t TARGET_FILL = 384;    // level target (~8 ms)
constexpr size_t FILL_TOLERANCE = 192; // toleransi sebelum koreksi drift

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
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, // Hardware wajib 32-bit
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

  // Drive kuat supaya tepi sinyal tegas di kabel jumper (MCLK 12.288 MHz).
  // Disarankan juga resistor seri 33-47 ohm dekat pin ESP32.
  gpio_set_drive_capability((gpio_num_t)I2S_BCLK, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_LRCK, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_DOUT, GPIO_DRIVE_CAP_2);
  gpio_set_drive_capability((gpio_num_t)I2S_MCLK, GPIO_DRIVE_CAP_2);
}

// ==========================================
// 5. DC BLOCKER (menghilangkan DC offset PCM1808)
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

// ==========================================
// 6. AUDIO TASK (DSP & USB I/O)
// ==========================================
void audioTask(void *pvParameters) {
  size_t bytes_read = 0;
  size_t bytes_written = 0;

  static int32_t i2s_rx_buf[BLOCK_SAMPLES];
  static int16_t usb_tx_buf[BLOCK_SAMPLES];
  static int16_t usb_rx_buf[BLOCK_SAMPLES];
  static int32_t i2s_tx_buf[BLOCK_SAMPLES];

  DcBlocker dcL, dcR;
  float gainSmooth =
      0.0f; // gain dihaluskan agar tidak ada pop saat mute / ubah volume
  bool primed = false;
  int16_t lastL = 0, lastR = 0;

  while (1) {
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

    // ---- 2. ADC (I2S RX) -> USB mic. Juga berfungsi sebagai "metronom" loop
    // ----
    esp_err_t err = i2s_read(I2S_NUM_0, i2s_rx_buf, sizeof(i2s_rx_buf),
                             &bytes_read, pdMS_TO_TICKS(20));
    if (err != ESP_OK || bytes_read < sizeof(i2s_rx_buf)) {
      vTaskDelay(1);
      continue;
    }

    float targetGain = isMuted ? 0.0f : (currentVolume * 5) / 100.0f;
    int pkL = 0, pkR = 0;

    for (size_t i = 0; i < BLOCK_FRAMES; i++) {
      float l = (float)(i2s_rx_buf[i * 2] >> 16);
      float r = (float)(i2s_rx_buf[i * 2 + 1] >> 16);

#if MIC_SOURCE == 0
      r = l;
#elif MIC_SOURCE == 1
      l = r;
#endif

      l = dcL.process(l);
      r = dcR.process(r);

      // Haluskan perubahan gain (one-pole)
      gainSmooth += (targetGain - gainSmooth) * 0.002f;

      l *= gainSmooth;
      r *= gainSmooth;

      if (l > 32767.0f)
        l = 32767.0f;
      if (l < -32768.0f)
        l = -32768.0f;
      if (r > 32767.0f)
        r = 32767.0f;
      if (r < -32768.0f)
        r = -32768.0f;

      int16_t sl = (int16_t)l;
      int16_t sr = (int16_t)r;
      usb_tx_buf[i * 2] = sl;
      usb_tx_buf[i * 2 + 1] = sr;

      int al = abs((int)sl);
      int ar = abs((int)sr);
      if (al > pkL)
        pkL = al;
      if (ar > pkR)
        pkR = ar;
    }

    if (usb_audio.mounted()) {
      size_t w =
          usb_audio.write((const uint8_t *)usb_tx_buf, sizeof(usb_tx_buf));
#if DEBUG_AUDIO
      if (w != sizeof(usb_tx_buf))
        Serial.printf("mic drop: %u/%u\n", (unsigned)w,
                      (unsigned)sizeof(usb_tx_buf));
#endif
      (void)w;
    }

    // ---- 3. Ring buffer -> DAC (I2S TX), selalu tepat 128 frame ----
    if (!primed && rbCount >= TARGET_FILL)
      primed = true;
    if (primed && rbCount < BLOCK_FRAMES + 2) { // underrun
      primed = false;
    }

    if (primed) {
      int adj = 0;
      if (rbCount > TARGET_FILL + FILL_TOLERANCE)
        adj = 1; // terlalu penuh -> buang 1 frame
      else if (rbCount < TARGET_FILL - FILL_TOLERANCE)
        adj = -1; // terlalu kosong -> ulang 1 frame

      int mL = 0, mR = 0;
      for (size_t i = 0; i < BLOCK_FRAMES; i++) {
        if (i == 0 && adj == 1) {
          int16_t a, b;
          rbPop(a, b);
        }
        if (!(i == 0 && adj == -1))
          rbPop(lastL, lastR);

        i2s_tx_buf[i * 2] = (int32_t)lastL * 65536;
        i2s_tx_buf[i * 2 + 1] = (int32_t)lastR * 65536;

        int al = abs((int)lastL);
        int ar = abs((int)lastR);
        if (al > mL)
          mL = al;
        if (ar > mR)
          mR = ar;
      }
      // VU meter dari musik PC jika mic tidak aktif
      if (pkL < 500 && pkR < 500) {
        pkL = mL;
        pkR = mR;
      }
    } else {
      memset(i2s_tx_buf, 0, sizeof(i2s_tx_buf));
    }

    i2s_write(I2S_NUM_0, i2s_tx_buf, sizeof(i2s_tx_buf), &bytes_written,
              portMAX_DELAY);

    peakLeft = pkL;
    peakRight = pkR;

#if DEBUG_AUDIO
    static uint32_t lastDbg = 0;
    if (millis() - lastDbg > 1000) {
      lastDbg = millis();
      Serial.printf("mounted=%d txActive=%d rbCount=%u primed=%d\n",
                    usb_audio.mounted(), usb_audio.isStreamingActiveTx(),
                    (unsigned)rbCount, primed);
    }
#endif
  }
}

// ==========================================
// 7. UI TASK (OLED & Encoder)
// ==========================================
void uiTask(void *pvParameters) {
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  u8g2.begin();

  int smoothPeakL = 0;
  int smoothPeakR = 0;
  unsigned long lastBtnPress = 0;

  while (1) {
    int encoderVal = encoder.getCount() / 2;
    if (encoderVal != currentVolume) {
      currentVolume = constrain(encoderVal, 0, 100);
      encoder.setCount(currentVolume * 2);
    }

    // Debounce tombol mute non-blocking
    if (digitalRead(ENC_SW) == LOW && (millis() - lastBtnPress > 250)) {
      isMuted = !isMuted;
      lastBtnPress = millis();
    }

    // Smooth decay untuk VU meter (attack cepat, decay ~15% per frame)
    int curL = peakLeft;
    int curR = peakRight;
    if (curL > smoothPeakL)
      smoothPeakL = curL;
    else
      smoothPeakL = (smoothPeakL * 85) / 100;

    if (curR > smoothPeakR)
      smoothPeakR = curR;
    else
      smoothPeakR = (smoothPeakR * 85) / 100;

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);

    u8g2.setCursor(0, 10);
    if (usb_audio.mounted())
      u8g2.print("USB: 16-BIT 48K");
    else
      u8g2.print("USB: WAITING...");

    u8g2.setCursor(0, 25);
    if (isMuted) {
      u8g2.print("MUTED");
    } else {
      u8g2.print("G:" + String(currentVolume) + "% ");
      if (usb_audio.isStreamingActiveTx()) {
        u8g2.print("MIC:REC");
      } else {
        u8g2.print("MIC:IDLE");
      }
    }

    int barL = constrain(map(smoothPeakL, 0, 32767, 0, 110), 0, 110);
    int barR = constrain(map(smoothPeakR, 0, 32767, 0, 110), 0, 110);

    u8g2.drawStr(0, 45, "L:");
    u8g2.drawFrame(15, 37, 110, 10);
    u8g2.drawBox(15, 37, barL, 10);

    u8g2.drawStr(0, 60, "R:");
    u8g2.drawFrame(15, 52, 110, 10);
    u8g2.drawBox(15, 52, barR, 10);

    u8g2.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
  }
}

// ==========================================
// 8. MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // USB Audio Setup
  AudioInfo audio_info(SAMPLE_RATE, CHANNELS, BIT_DEPTH);
  auto config = usb_audio.defaultConfig(RXTX_MODE);
  config.copyFrom(audio_info);
  config.manufacturer = "aliefadityanugraha";
  config.product = "ESP32-S3 Audio Interface";
  config.begin_usb = true;
  usb_audio.begin(config);

  pinMode(ENC_SW, INPUT_PULLUP);
  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  encoder.attachHalfQuad(ENC_DT, ENC_CLK);
  encoder.setCount(50 * 2);

  setupI2S();

  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(uiTask, "UITask", 4096, NULL, 1, NULL, 0);
}

void loop() { delay(1000); }
