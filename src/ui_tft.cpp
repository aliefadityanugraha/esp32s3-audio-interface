/**
 * @file ui_tft.cpp
 * @brief Implementasi Tahap 3: UI TFT ST7789 2.0" (320x240 Landscape, LovyanGFX)
 *        Double-Buffered PSRAM Sprites (30 FPS, Core 0)
 */

#include "ui_tft.h"

#include <LittleFS.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ui_tft {

namespace {

constexpr const char *PATH_UI_CONFIG   = "/ui_settings.bin";
constexpr const char *PATH_BOOT_LOGO   = "/logo_boot.bin";
constexpr const char *PATH_STBY_LOGO   = "/logo_standby.bin";
constexpr const char *DIR_PRESETS      = "/presets";

constexpr const char *TOP_MENU_LABELS[NUM_TOP_MENU_ITEMS] = {
    "METER", "IN A", "IN B", "OUT 1", "OUT 2", "OUT 3", "OUT 4", "PRESET", "SETUP"};

constexpr const char *SETUP_LABELS[NUM_SETUP_ITEMS] = {
    "MATRIX", "PRESET", "LEVEL", "SINYAL", "TEMA", "WIFI", "INPUT", "SISTEM"};

constexpr const char *THEME_NAMES[7] = {
    "RED", "ICE LUXURY", "UNGU-EMAS", "MERAH-BIRU",
    "MERAH-PUTIH", "BIRU-KUNING", "RGB PELANGI"};

constexpr const char *PANEL_MODE_NAMES[4] = {
    "NORMAL", "NEGATIF", "TUKAR R-B", "NEG+R-B"};

constexpr const char *WIFI_MODE_NAMES[3] = {
    "OFF", "AU-WEB", "AU-OTA"};

constexpr const char *XOVER_TYPE_SHORT[3] = {"LR", "BW", "BES"};
constexpr const char *PEQ_TYPE_SHORT[7]   = {"PK", "LS", "HS", "LP", "HP", "NT", "AP"};
constexpr const char *GEN_TYPE_NAMES[3]   = {"OFF", "WHITE NOISE", "PINK NOISE"};
constexpr const char *RTA_SRC_NAMES[8]    = {
    "IN A", "IN B", "OUT 1", "OUT 2", "OUT 3", "OUT 4", "IN MIX", "OUT12"};

constexpr const char RENAME_CHARSET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_";
constexpr size_t RENAME_CHARSET_LEN   = sizeof(RENAME_CHARSET) - 1;

constexpr float PI_F = 3.14159265358979323846f;
constexpr double PI_D = 3.14159265358979323846;

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Evaluasi respon magnitudo kuadrat |H(e^jw)|^2 dari 1 Biquad yang stabil di frekuensi rendah
inline double evalBiquadMagSq(double b0, double b1, double b2,
                              double a1, double a2, double w) {
  const double s = sin(0.5 * w);
  const double phi = s * s; // sin^2(w/2)
  const double numA = b0 + b1 + b2;
  const double denA = 1.0 + a1 + a2;
  const double num = numA * numA - 4.0 * (b0 * b1 + 4.0 * b0 * b2 + b1 * b2) * phi
                     + 16.0 * b0 * b2 * phi * phi;
  const double den = denA * denA - 4.0 * (a1 + 4.0 * a2 + a1 * a2) * phi
                     + 16.0 * a2 * phi * phi;
  if (den < 1.0e-18) return 1.0;
  return (num < 1.0e-18) ? 1.0e-18 : (num / den);
}

// Evaluasi |H(e^jw)|^2 dari 1 band PEQ
double evalPeqMagSq(const dsp::PeqBandConfig &band, double freqHz, double fs) {
  if (!band.enabled) return 1.0;
  const bool isGain = (band.type == dsp::PeqFilterType::PEAKING ||
                       band.type == dsp::PeqFilterType::LOW_SHELF ||
                       band.type == dsp::PeqFilterType::HIGH_SHELF);
  if (isGain && fabsf(band.gainDb) < 0.05f) return 1.0;

  const double fc = clampf(band.freqHz, 16.0f, 20000.0f);
  const double Q  = clampf(band.q, 0.4f, 12.0f);
  const double A  = pow(10.0, static_cast<double>(clampf(band.gainDb, -15.0f, 15.0f)) / 40.0);
  const double w0 = 2.0 * PI_D * fc / fs;
  const double cw = cos(w0);
  const double sw = sin(w0);
  const double al = sw / (2.0 * Q);

  double b0 = 1.0, b1 = 0.0, b2 = 0.0, a0 = 1.0, a1 = 0.0, a2 = 0.0;
  switch (band.type) {
    case dsp::PeqFilterType::PEAKING:
      b0 = 1.0 + al * A; b1 = -2.0 * cw; b2 = 1.0 - al * A;
      a0 = 1.0 + al / A; a1 = -2.0 * cw; a2 = 1.0 - al / A;
      break;
    case dsp::PeqFilterType::LOW_SHELF: {
      const double aS = sw / 2.0 * sqrt(fmax(0.01, (A + 1.0 / A) * (1.0 / Q - 1.0) + 2.0));
      const double tA = 2.0 * sqrt(A) * aS;
      b0 = A * ((A + 1.0) - (A - 1.0) * cw + tA);
      b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cw);
      b2 = A * ((A + 1.0) - (A - 1.0) * cw - tA);
      a0 = (A + 1.0) + (A - 1.0) * cw + tA;
      a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cw);
      a2 = (A + 1.0) + (A - 1.0) * cw - tA;
      break;
    }
    case dsp::PeqFilterType::HIGH_SHELF: {
      const double aS = sw / 2.0 * sqrt(fmax(0.01, (A + 1.0 / A) * (1.0 / Q - 1.0) + 2.0));
      const double tA = 2.0 * sqrt(A) * aS;
      b0 = A * ((A + 1.0) + (A - 1.0) * cw + tA);
      b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cw);
      b2 = A * ((A + 1.0) + (A - 1.0) * cw - tA);
      a0 = (A + 1.0) - (A - 1.0) * cw + tA;
      a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cw);
      a2 = (A + 1.0) - (A - 1.0) * cw - tA;
      break;
    }
    case dsp::PeqFilterType::LOW_PASS:
      b0 = (1.0 - cw) * 0.5; b1 = (1.0 - cw); b2 = b0;
      a0 = 1.0 + al; a1 = -2.0 * cw; a2 = 1.0 - al;
      break;
    case dsp::PeqFilterType::HIGH_PASS:
      b0 = (1.0 + cw) * 0.5; b1 = -(1.0 + cw); b2 = b0;
      a0 = 1.0 + al; a1 = -2.0 * cw; a2 = 1.0 - al;
      break;
    case dsp::PeqFilterType::NOTCH:
      b0 = 1.0; b1 = -2.0 * cw; b2 = 1.0;
      a0 = 1.0 + al; a1 = -2.0 * cw; a2 = 1.0 - al;
      break;
    case dsp::PeqFilterType::ALL_PASS:
    default:
      return 1.0;
  }

  const double invA0 = 1.0 / a0;
  const double w = 2.0 * PI_D * freqHz / fs;
  return evalBiquadMagSq(b0 * invA0, b1 * invA0, b2 * invA0, a1 * invA0, a2 * invA0, w);
}

// Evaluasi |H(e^jw)|^2 dari Crossover (LR / BW / Bessel @ 12/24/48 dB/oct)
double evalXoverMagSq(const dsp::XoverFilterConfig &xo, bool isHpf,
                      double freqHz, double fs) {
  if (!xo.enabled) return 1.0;

  struct Sec { float sc; float q; };
  static constexpr Sec BW12[1]  = {{1.0f, 0.707107f}};
  static constexpr Sec BW24[2]  = {{1.0f, 0.541196f}, {1.0f, 1.306563f}};
  static constexpr Sec BW48[4]  = {{1.0f, 0.509796f}, {1.0f, 0.601345f}, {1.0f, 0.899976f}, {1.0f, 2.562915f}};
  static constexpr Sec LR12[1]  = {{1.0f, 0.500000f}};
  static constexpr Sec LR24[2]  = {{1.0f, 0.707107f}, {1.0f, 0.707107f}};
  static constexpr Sec LR48[4]  = {{1.0f, 0.541196f}, {1.0f, 1.306563f}, {1.0f, 0.541196f}, {1.0f, 1.306563f}};
  static constexpr Sec BES12[1] = {{1.272020f, 0.577350f}};
  static constexpr Sec BES24[2] = {{1.430172f, 0.521935f}, {1.603358f, 0.805538f}};
  static constexpr Sec BES48[4] = {{1.778466f, 0.505991f}, {1.832093f, 0.559609f}, {1.953196f, 0.710852f}, {2.188726f, 1.225669f}};

  const Sec *s = BW24;
  int n = 2;
  if (xo.type == dsp::XoverType::LINKWITZ_RILEY) {
    if (xo.slope == dsp::XoverSlope::SLOPE_12DB) { s = LR12; n = 1; }
    else if (xo.slope == dsp::XoverSlope::SLOPE_48DB) { s = LR48; n = 4; }
    else { s = LR24; n = 2; }
  } else if (xo.type == dsp::XoverType::BESSEL) {
    if (xo.slope == dsp::XoverSlope::SLOPE_12DB) { s = BES12; n = 1; }
    else if (xo.slope == dsp::XoverSlope::SLOPE_48DB) { s = BES48; n = 4; }
    else { s = BES24; n = 2; }
  } else {
    if (xo.slope == dsp::XoverSlope::SLOPE_12DB) { s = BW12; n = 1; }
    else if (xo.slope == dsp::XoverSlope::SLOPE_48DB) { s = BW48; n = 4; }
    else { s = BW24; n = 2; }
  }

  const double w = 2.0 * PI_D * freqHz / fs;
  double magSq = 1.0;
  for (int i = 0; i < n; i++) {
    const double fc = clampf(isHpf ? (xo.freqHz / s[i].sc) : (xo.freqHz * s[i].sc),
                             10.0f, static_cast<float>(fs * 0.475));
    const double w0 = 2.0 * PI_D * fc / fs;
    const double cw = cos(w0);
    const double al = sin(w0) / (2.0 * static_cast<double>(s[i].q));
    const double invA0 = 1.0 / (1.0 + al);
    const double b0 = (isHpf ? (1.0 + cw) : (1.0 - cw)) * 0.5 * invA0;
    const double b1 = (isHpf ? -(1.0 + cw) : (1.0 - cw)) * invA0;
    const double b2 = b0;
    const double a1 = (-2.0 * cw) * invA0;
    const double a2 = (1.0 - al) * invA0;
    magSq *= evalBiquadMagSq(b0, b1, b2, a1, a2, w);
  }
  return magSq;
}

} // namespace

// ============================================================================
// 1. KONFIGURASI HARDWARE LOVYANGFX ST7789 (SPI2_HOST)
// ============================================================================
LgfxSt7789::LgfxSt7789() {
  {
    auto cfg = bus_.config();
    cfg.spi_host   = SPI2_HOST;
    cfg.spi_mode   = 0;
    cfg.freq_write = 40000000; // 40 MHz stabil & cepat untuk 30 FPS DMA
    cfg.freq_read  = 16000000;
    cfg.spi_3wire  = false;
    cfg.use_lock   = true;
    cfg.dma_channel= SPI_DMA_CH_AUTO;
    cfg.pin_sclk   = PIN_LCD_SCLK; // GPIO 12
    cfg.pin_mosi   = PIN_LCD_MOSI; // GPIO 13
    cfg.pin_miso   = -1;
    cfg.pin_dc     = PIN_LCD_DC;   // GPIO 9
    bus_.config(cfg);
    panel_.setBus(&bus_);
  }
  {
    auto cfg = panel_.config();
    cfg.pin_cs        = PIN_LCD_CS;  // GPIO 10
    cfg.pin_rst       = PIN_LCD_RST; // GPIO 8
    cfg.pin_busy      = -1;
    cfg.panel_width   = 240;
    cfg.panel_height  = 320;
    cfg.memory_width  = 240;
    cfg.memory_height = 320;
    cfg.offset_x      = 0;
    cfg.offset_y      = 0;
    cfg.readable      = false;
    cfg.invert        = false;
    cfg.rgb_order     = false; // false = RGB, true = BGR
    cfg.dlen_16bit    = false;
    cfg.bus_shared    = false;
    panel_.config(cfg);
  }
  setPanel(&panel_);
}

void LgfxSt7789::applyPanelColorMode(PanelColorMode mode) {
  const bool doInvert = (mode == PanelColorMode::NEGATIF ||
                         mode == PanelColorMode::NEGATIF_TUKAR_RB);
  const bool swapRB   = (mode == PanelColorMode::TUKAR_RB ||
                         mode == PanelColorMode::NEGATIF_TUKAR_RB);

  auto cfg = panel_.config();
  cfg.rgb_order = swapRB;
  cfg.invert    = doInvert;
  panel_.config(cfg);

  // Tulis ulang MADCTL (RGB/BGR bit) & INVON/INVOFF ke controller ST7789 secara live
  invertDisplay(doInvert);
  setRotation(getRotation());
}

// ============================================================================
// 2. LIFECYCLE, PSRAM DOUBLE BUFFERING, & PERSISTENCE LITTLEFS
// ============================================================================
UiTft::~UiTft() {
  fb_[0].deleteSprite();
  fb_[1].deleteSprite();
  if (bootLogoBuf_)    { free(bootLogoBuf_);    bootLogoBuf_ = nullptr; }
  if (standbyLogoBuf_) { free(standbyLogoBuf_); standbyLogoBuf_ = nullptr; }
}

uint16_t UiTft::rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return static_cast<uint16_t>(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3));
}

bool UiTft::begin(audio_io::AudioIO &audioIO,
                  dsp::DspEngine &dspEngine,
                  ui_ctrl::InputController &inputCtrl) {
  audioIO_ = &audioIO;
  dsp_     = &dspEngine;
  input_   = &inputCtrl;

  loadUiConfig();

  // 1. Inisialisasi panel ST7789 (240x320 -> Landscape 320x240)
  lcd_.init();
  lcd_.initDMA();
  lcd_.setRotation(1);
  lcd_.setColorDepth(16);
  lcd_.applyPanelColorMode(uiCfg_.panelMode);
  lcd_.fillScreen(TFT_BLACK);

  // 2. Buat 2 Sprite Double-Buffer Full-Screen (320x240 RGB565 = 153.6 KB x 2) di PSRAM
  bool ok0 = false, ok1 = false;
  for (int i = 0; i < 2; i++) {
    fb_[i].setPsram(true);
    fb_[i].setColorDepth(16);
    fb_[i].setSwapBytes(true);
    bool created = (fb_[i].createSprite(TFT_WIDTH, TFT_HEIGHT) != nullptr);
    if (i == 0) ok0 = created; else ok1 = created;
  }
  psramSprites_ = (ok0 && ok1);

  // 3. Sinkronkan status Panel Lock awal ke InputController
  input_->setPanelLocked(uiCfg_.panelLocked);

  // 4. Cache Boot & Standby Logo dari LittleFS ke PSRAM (jika file tersedia)
  loadRawImagePsram(PATH_BOOT_LOGO, bootLogoBuf_, bootLogoPixels_);
  loadRawImagePsram(PATH_STBY_LOGO, standbyLogoBuf_, standbyLogoPix_);

  // 5. Pastikan folder /presets tersedia & baca daftar nama preset
  if (!LittleFS.exists(DIR_PRESETS)) {
    LittleFS.mkdir(DIR_PRESETS);
  }
  refreshPresetNames();

  lastActivityMs_ = millis();
  return psramSprites_;
}

void UiTft::loadUiConfig() {
  // Default pabrik
  uiCfg_.magic             = UI_CFG_MAGIC;
  uiCfg_.theme             = UiTheme::ICE_LUXURY;
  uiCfg_.panelMode         = PanelColorMode::NORMAL;
  uiCfg_.standbyTimeoutSec = 120; // 2 menit (rentang 5 - 600 detik)
  uiCfg_.standbyEnabled    = true;
  uiCfg_.panelLocked       = false;
  uiCfg_.wifiMode          = WifiMode::OFF;
  uiCfg_.bleEnabled        = true; // Mode Default Tahap 4: BLE NUS aktif saat WiFi OFF
  uiCfg_.brightness        = 240;

  if (LittleFS.exists(PATH_UI_CONFIG)) {
    File f = LittleFS.open(PATH_UI_CONFIG, "r");
    if (f) {
      UiConfig tmp = {};
      if (f.read(reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp)) == sizeof(tmp) &&
          tmp.magic == UI_CFG_MAGIC) {
        tmp.standbyTimeoutSec = constrain(tmp.standbyTimeoutSec, 5, 600);
        if (static_cast<uint8_t>(tmp.theme) > 6)     tmp.theme = UiTheme::ICE_LUXURY;
        if (static_cast<uint8_t>(tmp.panelMode) > 3) tmp.panelMode = PanelColorMode::NORMAL;
        if (static_cast<uint8_t>(tmp.wifiMode) > 2)  tmp.wifiMode = WifiMode::OFF;
        uiCfg_ = tmp;
      }
      f.close();
    }
  }
}

void UiTft::saveUiConfig() {
  File f = LittleFS.open(PATH_UI_CONFIG, "w");
  if (f) {
    f.write(reinterpret_cast<const uint8_t *>(&uiCfg_), sizeof(uiCfg_));
    f.close();
  }
}

void UiTft::applyUiConfig(const UiConfig &cfg, bool persist) {
  uiCfg_ = cfg;
  uiCfg_.standbyTimeoutSec = constrain(uiCfg_.standbyTimeoutSec, 5, 600);
  lcd_.applyPanelColorMode(uiCfg_.panelMode);
  if (input_) {
    input_->setPanelLocked(uiCfg_.panelLocked);
  }
  if (persist) {
    saveUiConfig();
  }
}

bool UiTft::loadRawImagePsram(const char *path, uint16_t *&dstBuf, size_t &dstPixels) {
  if (dstBuf) { free(dstBuf); dstBuf = nullptr; dstPixels = 0; }
  if (!LittleFS.exists(path)) return false;

  File f = LittleFS.open(path, "r");
  if (!f) return false;

  const size_t expectedBytes = static_cast<size_t>(TFT_WIDTH) * TFT_HEIGHT * sizeof(uint16_t);
  if (f.size() < expectedBytes) {
    f.close();
    return false;
  }

  uint16_t *buf = static_cast<uint16_t *>(ps_malloc(expectedBytes));
  if (!buf) {
    f.close();
    return false;
  }

  const size_t readBytes = f.read(reinterpret_cast<uint8_t *>(buf), expectedBytes);
  f.close();
  if (readBytes != expectedBytes) {
    free(buf);
    return false;
  }

  dstBuf    = buf;
  dstPixels = TFT_WIDTH * TFT_HEIGHT;
  return true;
}

// ============================================================================
// 3. SISTEM 7 TEMA WARNA UI (TERMASUK RGB PELANGI DINAMIS)
// ============================================================================
ThemePalette UiTft::resolveTheme(uint32_t nowMs) const {
  ThemePalette p = {};
  p.ok     = rgb565(30, 225, 95);
  p.warn   = rgb565(255, 190, 20);
  p.danger = rgb565(255, 35, 45);

  switch (uiCfg_.theme) {
    case UiTheme::RED:
      p.bg       = rgb565(10, 6, 8);
      p.panel    = rgb565(26, 12, 16);
      p.headerBg = rgb565(48, 10, 14);
      p.accent   = rgb565(255, 40, 50);
      p.accent2  = rgb565(255, 140, 40);
      p.text     = rgb565(248, 240, 242);
      p.textDim  = rgb565(160, 115, 120);
      p.grid     = rgb565(55, 25, 30);
      p.rtaBar   = rgb565(120, 24, 34);
      p.curve    = rgb565(255, 90, 75);
      break;

    case UiTheme::ICE_LUXURY:
      p.bg       = rgb565(6, 12, 22);
      p.panel    = rgb565(14, 26, 44);
      p.headerBg = rgb565(10, 34, 62);
      p.accent   = rgb565(56, 232, 255);
      p.accent2  = rgb565(175, 225, 255);
      p.text     = rgb565(238, 248, 255);
      p.textDim  = rgb565(120, 158, 185);
      p.grid     = rgb565(28, 52, 78);
      p.rtaBar   = rgb565(24, 96, 138);
      p.curve    = rgb565(90, 245, 255);
      break;

    case UiTheme::UNGU_EMAS:
      p.bg       = rgb565(14, 8, 24);
      p.panel    = rgb565(30, 18, 50);
      p.headerBg = rgb565(44, 20, 72);
      p.accent   = rgb565(255, 204, 36);
      p.accent2  = rgb565(186, 92, 255);
      p.text     = rgb565(252, 246, 235);
      p.textDim  = rgb565(168, 140, 195);
      p.grid     = rgb565(58, 36, 86);
      p.rtaBar   = rgb565(105, 48, 155);
      p.curve    = rgb565(255, 215, 65);
      break;

    case UiTheme::MERAH_BIRU:
      p.bg       = rgb565(8, 10, 20);
      p.panel    = rgb565(18, 22, 42);
      p.headerBg = rgb565(42, 14, 32);
      p.accent   = rgb565(255, 45, 72);
      p.accent2  = rgb565(45, 145, 255);
      p.text     = rgb565(242, 246, 255);
      p.textDim  = rgb565(132, 148, 182);
      p.grid     = rgb565(36, 44, 74);
      p.rtaBar   = rgb565(28, 76, 165);
      p.curve    = rgb565(255, 72, 95);
      break;

    case UiTheme::MERAH_PUTIH:
      p.bg       = rgb565(14, 14, 16);
      p.panel    = rgb565(30, 30, 34);
      p.headerBg = rgb565(165, 18, 28);
      p.accent   = rgb565(255, 255, 255);
      p.accent2  = rgb565(240, 42, 52);
      p.text     = rgb565(255, 255, 255);
      p.textDim  = rgb565(180, 180, 185);
      p.grid     = rgb565(64, 64, 70);
      p.rtaBar   = rgb565(145, 24, 34);
      p.curve    = rgb565(255, 255, 255);
      break;

    case UiTheme::BIRU_KUNING:
      p.bg       = rgb565(6, 14, 32);
      p.panel    = rgb565(14, 28, 62);
      p.headerBg = rgb565(16, 42, 96);
      p.accent   = rgb565(255, 214, 10);
      p.accent2  = rgb565(52, 165, 255);
      p.text     = rgb565(245, 250, 255);
      p.textDim  = rgb565(135, 165, 210);
      p.grid     = rgb565(32, 56, 102);
      p.rtaBar   = rgb565(34, 96, 178);
      p.curve    = rgb565(255, 224, 48);
      break;

    case UiTheme::RGB_PELANGI:
    default: {
      const uint16_t h1 = static_cast<uint16_t>((nowMs / 18) % 360);
      const uint16_t h2 = static_cast<uint16_t>((h1 + 120) % 360);
      const uint16_t h3 = static_cast<uint16_t>((h1 + 220) % 360);
      uint8_t r1, g1, b1, r2, g2, b2, r3, g3, b3;
      ui_ctrl::InputController::hsvToRgb(h1, 240, 255, r1, g1, b1);
      ui_ctrl::InputController::hsvToRgb(h2, 225, 255, r2, g2, b2);
      ui_ctrl::InputController::hsvToRgb(h3, 210, 130, r3, g3, b3);
      p.bg       = rgb565(8, 8, 14);
      p.panel    = rgb565(20, 20, 32);
      p.headerBg = rgb565(r1 >> 2, g1 >> 2, b1 >> 2);
      p.accent   = rgb565(r1, g1, b1);
      p.accent2  = rgb565(r2, g2, b2);
      p.text     = rgb565(250, 250, 255);
      p.textDim  = rgb565(155, 160, 185);
      p.grid     = rgb565(42, 42, 62);
      p.rtaBar   = rgb565(r3, g3, b3);
      p.curve    = rgb565(r2, g2, b2);
      break;
    }
  }
  return p;
}

// ============================================================================
// 4. BOOT LOGO & STANDBY LOGO (LITTLEFS / FALLBACK VEKTOR)
// ============================================================================
void UiTft::renderFallbackLogo(LGFX_Sprite &sp, const ThemePalette &pal,
                               bool isStandby, float phase) {
  sp.fillScreen(pal.bg);

  // Frame neon halus di sekeliling layar
  sp.drawRoundRect(8, 8, TFT_WIDTH - 16, TFT_HEIGHT - 16, 10, pal.accent);
  sp.drawRoundRect(11, 11, TFT_WIDTH - 22, TFT_HEIGHT - 22, 8, pal.grid);

  // Grafik gelombang audio animasi di tengah
  const int cy = isStandby ? 112 : 108;
  for (int x = 32; x < TFT_WIDTH - 32; x += 3) {
    const float norm = static_cast<float>(x - 32) / static_cast<float>(TFT_WIDTH - 64);
    const float env  = sinf(norm * PI_F);
    const float wave = sinf(norm * 18.0f + phase * 6.0f) * env;
    const int h = static_cast<int>(fabsf(wave) * 26.0f) + 2;
    sp.drawFastVLine(x, cy - h, h * 2, (x & 1) ? pal.accent : pal.accent2);
  }

  sp.setTextDatum(middle_center);
  sp.setTextColor(pal.text);
  sp.setTextSize(2);
  sp.drawString("ESP32-S3 DLMS 2x4", TFT_WIDTH / 2, 48);

  sp.setTextSize(1);
  sp.setTextColor(pal.accent2);
  sp.drawString("HYBRID UAC2 48kHz + DIGITAL SPEAKER PROCESSOR", TFT_WIDTH / 2, 72);

  if (!isStandby) {
    // Progress bar boot
    const int barX = 48, barY = 176, barW = TFT_WIDTH - 96, barH = 10;
    sp.drawRoundRect(barX, barY, barW, barH, 4, pal.accent);
    const int fillW = static_cast<int>(clampf(phase, 0.0f, 1.0f) * (barW - 4));
    if (fillW > 0) {
      sp.fillRoundRect(barX + 2, barY + 2, fillW, barH - 4, 2, pal.accent);
    }
    sp.setTextColor(pal.textDim);
    sp.drawString("Memuat Audio I/O & 32-Bit Float DSP...", TFT_WIDTH / 2, 202);
  } else {
    sp.setTextColor(pal.textDim);
    sp.drawString("MODE STANDBY - PUTAR / KLIK ENCODER UNTUK AKTIF", TFT_WIDTH / 2, 198);
  }
}

void UiTft::showBootScreen(uint32_t durationMs) {
  const uint32_t startMs = millis();
  while (millis() - startMs < durationMs) {
    const uint32_t elapsed = millis() - startMs;
    const float prog = static_cast<float>(elapsed) / static_cast<float>(durationMs);
    if (input_) {
      input_->updateBootAnimation(elapsed);
    }

    LGFX_Sprite &sp = fb_[curFb_];
    const ThemePalette pal = resolveTheme(millis());

    if (bootLogoBuf_ != nullptr) {
      sp.pushImage(0, 0, TFT_WIDTH, TFT_HEIGHT, bootLogoBuf_);
      // Overlay bar tipis kemajuan boot di bawah logo
      const int fillW = static_cast<int>(prog * (TFT_WIDTH - 40));
      sp.fillRect(20, TFT_HEIGHT - 12, TFT_WIDTH - 40, 4, pal.grid);
      sp.fillRect(20, TFT_HEIGHT - 12, fillW, 4, pal.accent);
    } else {
      renderFallbackLogo(sp, pal, false, prog);
    }

    lcd_.waitDMA();
    sp.pushSprite(&lcd_, 0, 0);
    curFb_ ^= 1;
    vTaskDelay(pdMS_TO_TICKS(33));
  }
  lastActivityMs_ = millis();
}

// ============================================================================
// 5. PRESET STORAGE LITTLEFS (SAVE / LOAD / RENAME 8 SLOT)
// ============================================================================
void UiTft::showToast(const char *msg, uint32_t durationMs) {
  strncpy(toastMsg_, msg, sizeof(toastMsg_) - 1);
  toastMsg_[sizeof(toastMsg_) - 1] = '\0';
  toastUntilMs_ = millis() + durationMs;
}

void UiTft::refreshPresetNames() {
  for (uint8_t s = 0; s < NUM_PRESET_SLOTS; s++) {
    char path[32];
    snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, s);
    presetExists_[s] = false;
    snprintf(presetNames_[s], sizeof(presetNames_[s]), "EMPTY SLOT %u", s + 1);

    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      if (f) {
        dsp::SystemConfig tmp = {};
        if (f.read(reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp)) == sizeof(tmp) &&
            tmp.magic == dsp::DSP_CONFIG_MAGIC) {
          presetExists_[s] = true;
          strncpy(presetNames_[s], tmp.presetName, sizeof(presetNames_[s]) - 1);
          presetNames_[s][sizeof(presetNames_[s]) - 1] = '\0';
        }
        f.close();
      }
    }
  }
}

bool UiTft::savePresetSlot(uint8_t slot, const char *name) {
  if (!dsp_ || slot >= NUM_PRESET_SLOTS) return false;
  dsp::SystemConfig cfg = dsp_->getConfig();
  if (name != nullptr && name[0] != '\0') {
    strncpy(cfg.presetName, name, sizeof(cfg.presetName) - 1);
    cfg.presetName[sizeof(cfg.presetName) - 1] = '\0';
  } else if (!presetExists_[slot]) {
    snprintf(cfg.presetName, sizeof(cfg.presetName), "PRESET %u", slot + 1);
  } else {
    strncpy(cfg.presetName, presetNames_[slot], sizeof(cfg.presetName) - 1);
  }

  char path[32];
  snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, slot);
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  const size_t written = f.write(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg));
  f.close();

  if (written == sizeof(cfg)) {
    dsp_->applyConfig(cfg);
    refreshPresetNames();
    showToast("PRESET TERSIMPAN!");
    return true;
  }
  return false;
}

bool UiTft::loadPresetSlot(uint8_t slot) {
  if (!dsp_ || slot >= NUM_PRESET_SLOTS) return false;
  char path[32];
  snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, slot);
  if (!LittleFS.exists(path)) {
    showToast("SLOT KOSONG!");
    return false;
  }

  File f = LittleFS.open(path, "r");
  if (!f) return false;
  dsp::SystemConfig cfg = {};
  const size_t got = f.read(reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg));
  f.close();

  if (got == sizeof(cfg) && cfg.magic == dsp::DSP_CONFIG_MAGIC) {
    dsp_->applyConfig(cfg);
    showToast("PRESET DIMUAT!");
    return true;
  }
  showToast("PRESET RUSAK!");
  return false;
}

bool UiTft::renamePresetSlot(uint8_t slot, const char *newName) {
  if (slot >= NUM_PRESET_SLOTS || !newName || newName[0] == '\0') return false;
  if (!presetExists_[slot]) {
    return savePresetSlot(slot, newName);
  }

  char path[32];
  snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, slot);
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  dsp::SystemConfig cfg = {};
  const size_t got = f.read(reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg));
  f.close();
  if (got != sizeof(cfg)) return false;

  strncpy(cfg.presetName, newName, sizeof(cfg.presetName) - 1);
  cfg.presetName[sizeof(cfg.presetName) - 1] = '\0';

  File fw = LittleFS.open(path, "w");
  if (!fw) return false;
  fw.write(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg));
  fw.close();

  refreshPresetNames();
  showToast("NAMA DIPERBARUI!");
  return true;
}

bool UiTft::deletePresetSlot(uint8_t slot) {
  if (slot >= NUM_PRESET_SLOTS) return false;
  char path[32];
  snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, slot);
  bool ok = true;
  if (LittleFS.exists(path)) {
    ok = LittleFS.remove(path);
  }
  refreshPresetNames();
  if (ok) showToast("PRESET DIHAPUS!");
  return ok;
}

void UiTft::reloadLogos() {
  loadRawImagePsram(PATH_BOOT_LOGO, bootLogoBuf_, bootLogoPixels_);
  loadRawImagePsram(PATH_STBY_LOGO, standbyLogoBuf_, standbyLogoPix_);
}

// ============================================================================
// 6. STATUS & INPUT ROUTING STATE MACHINE
// ============================================================================
bool UiTft::isOnHomeScreen() const {
  return (!inStandby_ && !overheatActive_ && !inDetailPage_);
}

void UiTft::handleInput(const ui_ctrl::InputEvent &ev) {
  const uint32_t nowMs = millis();

  if (ev.anyActivity) {
    lastActivityMs_ = nowMs;
  }

  // Buka kunci Panel Lock (tahan 3 detik)
  if (ev.button == ui_ctrl::ButtonEvent::UNLOCK_3S) {
    uiCfg_.panelLocked = false;
    saveUiConfig();
    inStandby_ = false;
    showToast("PANEL TERBUKA!");
    return;
  }

  // Input diblokir karena Panel Lock aktif
  if (ev.button == ui_ctrl::ButtonEvent::LOCKED_BLOCKED || ev.panelLocked) {
    if (ev.anyActivity) {
      inStandby_ = false;
      lockBannerUntilMs_ = nowMs + 1500;
    }
    return;
  }

  // Jika sedang Standby -> sentuhan/putaran pertama hanya membangunkan layar
  if (inStandby_) {
    if (ev.anyActivity) {
      inStandby_ = false;
    }
    return;
  }

  // Jika layar peringatan suhu >= 75°C aktif -> klik untuk dismiss sementara
  if (overheatActive_ && !overheatDismissed_) {
    if (ev.button == ui_ctrl::ButtonEvent::CLICK ||
        ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
      overheatDismissed_ = true;
    }
    return;
  }

  if (!inDetailPage_) {
    handleHomeInput(ev);
  } else {
    switch (activeMenu_) {
      case TopMenuId::IN_A:
      case TopMenuId::IN_B:
      case TopMenuId::OUT_1:
      case TopMenuId::OUT_2:
      case TopMenuId::OUT_3:
      case TopMenuId::OUT_4:
        handleChannelInput(ev);
        break;
      case TopMenuId::PRESET:
        handlePresetInput(ev);
        break;
      case TopMenuId::SETUP:
        handleSetupInput(ev);
        break;
      case TopMenuId::METER:
      default:
        inDetailPage_ = false;
        break;
    }
  }
}

void UiTft::handleHomeInput(const ui_ctrl::InputEvent &ev) {
  // Putar encoder: geser horizontal AUDIO INTERFACE MENU
  if (ev.delta != 0) {
    int idx = static_cast<int>(activeMenu_) + ev.delta;
    while (idx < 0) idx += NUM_TOP_MENU_ITEMS;
    idx %= NUM_TOP_MENU_ITEMS;
    activeMenu_ = static_cast<TopMenuId>(idx);
  }

  // Tahan 2 detik dari layar utama: langsung masuk ke halaman edit kanal aktif
  if (ev.button == ui_ctrl::ButtonEvent::HOLD_2S) {
    if (activeMenu_ == TopMenuId::METER ||
        activeMenu_ == TopMenuId::PRESET ||
        activeMenu_ == TopMenuId::SETUP) {
      activeMenu_ = TopMenuId::OUT_1; // Default loncat cepat ke OUT 1
    }
    inDetailPage_   = true;
    chTab_          = 0;
    chParamIdx_     = 0;
    chEditingParam_ = true;
    return;
  }

  // Klik singkat: masuk ke halaman terpilih
  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    if (activeMenu_ == TopMenuId::METER) {
      // Pada halaman METER, klik singkat bisa langsung toggle Master Mute
      dsp::SystemConfig cfg = dsp_->getConfig();
      cfg.master.mute = !cfg.master.mute;
      dsp_->applyConfig(cfg);
      showToast(cfg.master.mute ? "MASTER MUTE: ON" : "MASTER MUTE: OFF", 900);
    } else {
      inDetailPage_    = true;
      chTab_           = 0;
      chParamIdx_      = 0;
      chEditingParam_  = false;
      setupSub_        = SetupSubmenu::NONE;
      subCursor_       = 0;
      subEditing_      = false;
      presetActionMenu_= false;
      presetRenaming_  = false;
      if (activeMenu_ == TopMenuId::PRESET) {
        refreshPresetNames();
      }
    }
  }
}

void UiTft::handleChannelInput(const ui_ctrl::InputEvent &ev) {
  const bool isOutput = (static_cast<uint8_t>(activeMenu_) >= static_cast<uint8_t>(TopMenuId::OUT_1));
  const int chIdx = isOutput
      ? (static_cast<int>(activeMenu_) - static_cast<int>(TopMenuId::OUT_1))
      : (static_cast<int>(activeMenu_) - static_cast<int>(TopMenuId::IN_A));

  // Jumlah item parameter per tab:
  // Tab 0 (EQ/XOVER):
  //   IN  (9 items): [0]TAB, [1]Gain, [2]Mute, [3]Phase, [4]PEQ Band, [5]Freq, [6]Gain, [7]Q, [8]Copy
  //   OUT (15 items): [0]TAB, [1]Gain, [2]Mute, [3]Phase, [4]HPF En/Type/Slope, [5]HPF Freq,
  //                   [6]LPF En/Type/Slope, [7]LPF Freq, [8]PEQ Band, [9]Freq, [10]Gain, [11]Q,
  //                   [12]Delay ms, [13]Bypass DSP, [14]Copy -> ALL
  // Tab 1 (DYNAMICS):
  //   IN  (10 items): [0]TAB, [1]Gate En, [2]Gate Thr, [3]Gate Atk, [4]Gate Rel,
  //                   [5]Comp En, [6]Comp Thr, [7]Comp Ratio, [8]Comp Knee, [9]Comp Makeup
  //   OUT (10 items): [0]TAB, [1]Gate En, [2]Gate Thr, [3]Gate Atk, [4]Gate Rel,
  //                   [5]Lim En, [6]Lim Thr, [7]Lim Atk, [8]Lim Rel, [9]AntiClip
  const uint8_t maxParams = (chTab_ == 0) ? (isOutput ? 15 : 9) : 10;

  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (chEditingParam_) {
      chEditingParam_ = false;
    } else {
      inDetailPage_ = false;
    }
    return;
  }

  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    if (chParamIdx_ == 0) {
      chTab_ ^= 1; // Tukar Tab 0 (XOVER+PEQ+RTA) <-> Tab 1 (DYNAMICS)
      chParamIdx_ = 0;
      chEditingParam_ = false;
      return;
    }

    // Aksi toggle / copy langsung pada klik
    dsp::SystemConfig cfg = dsp_->getConfig();
    if (chTab_ == 0) {
      if (!isOutput) {
        if (chParamIdx_ == 2) { cfg.inputs[chIdx].mute = !cfg.inputs[chIdx].mute; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 3) { cfg.inputs[chIdx].phaseInvert = !cfg.inputs[chIdx].phaseInvert; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 8) {
          dsp_->copyChannelSettings(chIdx == 0 ? dsp::ChannelId::IN1 : dsp::ChannelId::IN2,
                                    chIdx == 0 ? dsp::ChannelId::IN2 : dsp::ChannelId::IN1);
          showToast("DISALIN KE IN LAIN!");
          return;
        }
      } else {
        if (chParamIdx_ == 2) { cfg.outputs[chIdx].mute = !cfg.outputs[chIdx].mute; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 3) { cfg.outputs[chIdx].phaseInvert = !cfg.outputs[chIdx].phaseInvert; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 13) { cfg.outputs[chIdx].bypassDsp = !cfg.outputs[chIdx].bypassDsp; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 14) {
          dsp_->copyChannelSettings(static_cast<dsp::ChannelId>(2 + chIdx), dsp::ChannelId::ALL_OUTPUTS);
          showToast("DISALIN KE SEMUA OUT!");
          return;
        }
      }
    } else {
      if (!isOutput) {
        if (chParamIdx_ == 1) { cfg.inputs[chIdx].gate.enabled = !cfg.inputs[chIdx].gate.enabled; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 5) { cfg.inputs[chIdx].comp.enabled = !cfg.inputs[chIdx].comp.enabled; dsp_->applyConfig(cfg); return; }
      } else {
        if (chParamIdx_ == 1) { cfg.outputs[chIdx].gate.enabled = !cfg.outputs[chIdx].gate.enabled; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 5) { cfg.outputs[chIdx].limiter.enabled = !cfg.outputs[chIdx].limiter.enabled; dsp_->applyConfig(cfg); return; }
        if (chParamIdx_ == 9) {
          cfg.outputs[chIdx].antiClipMode = (cfg.outputs[chIdx].antiClipMode == dsp::AntiClipMode::SOFT_TANH)
              ? dsp::AntiClipMode::HARD_CLIP : dsp::AntiClipMode::SOFT_TANH;
          dsp_->applyConfig(cfg);
          return;
        }
      }
    }

    chEditingParam_ = !chEditingParam_;
    return;
  }

  if (ev.delta == 0) return;

  if (!chEditingParam_) {
    int next = static_cast<int>(chParamIdx_) + ev.delta;
    if (next < 0) next = maxParams - 1;
    if (next >= maxParams) next = 0;
    chParamIdx_ = static_cast<uint8_t>(next);
    return;
  }

  // Mode edit nilai parameter aktif
  dsp::SystemConfig cfg = dsp_->getConfig();
  const float d  = static_cast<float>(ev.delta);
  const float ad = static_cast<float>(ev.accelDelta);

  if (chTab_ == 0) {
    if (!isOutput) {
      auto &in = cfg.inputs[chIdx];
      auto &band = in.peq[chActivePeqBand_];
      switch (chParamIdx_) {
        case 1: in.gainDb = clampf(in.gainDb + d * 0.5f, -12.0f, 12.0f); break;
        case 4: chActivePeqBand_ = static_cast<uint8_t>((chActivePeqBand_ + ev.delta + 9) % 9); break;
        case 5: {
          // Resolusi 0.5 Hz di bawah 200 Hz, proporsional saat diputar cepat
          const float step = (band.freqHz < 200.0f) ? 0.5f : (band.freqHz < 2000.0f ? 5.0f : 25.0f);
          band.freqHz = dsp::DspEngine::quantizeFreqHalfHz(band.freqHz + ad * step);
          break;
        }
        case 6: band.gainDb = clampf(band.gainDb + d * 0.5f, -15.0f, 15.0f); break;
        case 7: band.q = clampf(band.q + d * 0.1f, 0.4f, 12.0f); break;
        default: break;
      }
    } else {
      auto &out  = cfg.outputs[chIdx];
      auto &band = out.peq[chActivePeqBand_];
      auto cycleXoverMode = [](dsp::XoverFilterConfig &xo, int delta) {
        // 10 state: OFF + (3 tipe x 3 slope)
        int state = 0;
        if (xo.enabled) {
          int sIdx = (xo.slope == dsp::XoverSlope::SLOPE_12DB) ? 0 : (xo.slope == dsp::XoverSlope::SLOPE_24DB ? 1 : 2);
          state = 1 + static_cast<int>(xo.type) * 3 + sIdx;
        }
        state = (state + delta + 10) % 10;
        if (state == 0) {
          xo.enabled = false;
        } else {
          xo.enabled = true;
          int m = state - 1;
          xo.type = static_cast<dsp::XoverType>(m / 3);
          const dsp::XoverSlope slopes[3] = {
              dsp::XoverSlope::SLOPE_12DB, dsp::XoverSlope::SLOPE_24DB, dsp::XoverSlope::SLOPE_48DB};
          xo.slope = slopes[m % 3];
        }
      };

      switch (chParamIdx_) {
        case 1: out.gainDb = clampf(out.gainDb + d * 0.5f, -12.0f, 12.0f); break;
        case 4: cycleXoverMode(out.hpf, ev.delta); break;
        case 5: {
          const float step = (out.hpf.freqHz < 200.0f) ? 1.0f : (out.hpf.freqHz < 2000.0f ? 10.0f : 50.0f);
          out.hpf.freqHz = dsp::DspEngine::quantizeFreqHalfHz(out.hpf.freqHz + ad * step);
          break;
        }
        case 6: cycleXoverMode(out.lpf, ev.delta); break;
        case 7: {
          const float step = (out.lpf.freqHz < 200.0f) ? 1.0f : (out.lpf.freqHz < 2000.0f ? 10.0f : 50.0f);
          out.lpf.freqHz = dsp::DspEngine::quantizeFreqHalfHz(out.lpf.freqHz + ad * step);
          break;
        }
        case 8: chActivePeqBand_ = static_cast<uint8_t>((chActivePeqBand_ + ev.delta + 9) % 9); break;
        case 9: {
          const float step = (band.freqHz < 200.0f) ? 0.5f : (band.freqHz < 2000.0f ? 5.0f : 25.0f);
          band.freqHz = dsp::DspEngine::quantizeFreqHalfHz(band.freqHz + ad * step);
          break;
        }
        case 10: band.gainDb = clampf(band.gainDb + d * 0.5f, -15.0f, 15.0f); break;
        case 11: band.q = clampf(band.q + d * 0.1f, 0.4f, 12.0f); break;
        case 12: out.delayMs = clampf(out.delayMs + ad * 0.5f, 0.0f, dsp::DSP_MAX_DELAY_MS); break;
        default: break;
      }
    }
  } else {
    // Tab 1: Dynamics
    if (!isOutput) {
      auto &in = cfg.inputs[chIdx];
      switch (chParamIdx_) {
        case 2: in.gate.thresholdDb  = clampf(in.gate.thresholdDb + d, -90.0f, 0.0f); break;
        case 3: in.gate.attackMs     = clampf(in.gate.attackMs + d * 0.5f, 0.1f, 100.0f); break;
        case 4: in.gate.releaseMs    = clampf(in.gate.releaseMs + ad * 10.0f, 10.0f, 2000.0f); break;
        case 6: in.comp.thresholdDb  = clampf(in.comp.thresholdDb + d, -60.0f, 0.0f); break;
        case 7: in.comp.ratio        = clampf(in.comp.ratio + d * 0.5f, 1.0f, 20.0f); break;
        case 8: in.comp.kneeDb       = clampf(in.comp.kneeDb + d * 0.5f, 0.0f, 12.0f); break;
        case 9: in.comp.makeupGainDb = clampf(in.comp.makeupGainDb + d * 0.5f, 0.0f, 18.0f); break;
        default: break;
      }
    } else {
      auto &out = cfg.outputs[chIdx];
      switch (chParamIdx_) {
        case 2: out.gate.thresholdDb    = clampf(out.gate.thresholdDb + d, -90.0f, 0.0f); break;
        case 3: out.gate.attackMs       = clampf(out.gate.attackMs + d * 0.5f, 0.1f, 100.0f); break;
        case 4: out.gate.releaseMs      = clampf(out.gate.releaseMs + ad * 10.0f, 10.0f, 2000.0f); break;
        case 6: out.limiter.thresholdDb = clampf(out.limiter.thresholdDb + d * 0.5f, -30.0f, 0.0f); break;
        case 7: out.limiter.attackMs    = clampf(out.limiter.attackMs + d * 0.2f, 0.05f, 50.0f); break;
        case 8: out.limiter.releaseMs   = clampf(out.limiter.releaseMs + ad * 10.0f, 10.0f, 2000.0f); break;
        default: break;
      }
    }
  }

  dsp_->applyConfig(cfg);
}

void UiTft::handlePresetInput(const ui_ctrl::InputEvent &ev) {
  if (presetRenaming_) {
    if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
      // Selesai & simpan nama baru
      presetRenaming_ = false;
      presetActionMenu_ = false;
      renamePresetSlot(presetSlotCursor_, renameBuf_);
      return;
    }
    if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
      renameCharPos_ = (renameCharPos_ + 1) % 12;
      if (renameBuf_[renameCharPos_] == '\0') renameBuf_[renameCharPos_] = ' ';
      return;
    }
    if (ev.delta != 0) {
      const char cur = renameBuf_[renameCharPos_];
      const char *pos = strchr(RENAME_CHARSET, cur);
      int idx = pos ? static_cast<int>(pos - RENAME_CHARSET) : 0;
      idx = (idx + ev.delta + static_cast<int>(RENAME_CHARSET_LEN)) % static_cast<int>(RENAME_CHARSET_LEN);
      renameBuf_[renameCharPos_] = RENAME_CHARSET[idx];
    }
    return;
  }

  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (presetActionMenu_) {
      presetActionMenu_ = false;
    } else if (activeMenu_ == TopMenuId::SETUP && setupSub_ == SetupSubmenu::PRESET) {
      setupSub_ = SetupSubmenu::NONE;
    } else {
      inDetailPage_ = false;
    }
    return;
  }

  if (presetActionMenu_) {
    if (ev.delta != 0) {
      presetAction_ = static_cast<uint8_t>((presetAction_ + ev.delta + 3) % 3);
    }
    if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
      if (presetAction_ == 0) {
        loadPresetSlot(presetSlotCursor_);
        presetActionMenu_ = false;
      } else if (presetAction_ == 1) {
        savePresetSlot(presetSlotCursor_);
        presetActionMenu_ = false;
      } else {
        // Mulai editor Rename karakter
        memset(renameBuf_, ' ', 12);
        renameBuf_[12] = '\0';
        const size_t len = strlen(presetNames_[presetSlotCursor_]);
        memcpy(renameBuf_, presetNames_[presetSlotCursor_], (len < 12) ? len : 12);
        renameCharPos_  = 0;
        presetRenaming_ = true;
      }
    }
    return;
  }

  if (ev.delta != 0) {
    presetSlotCursor_ = static_cast<uint8_t>((presetSlotCursor_ + ev.delta + NUM_PRESET_SLOTS) % NUM_PRESET_SLOTS);
  }
  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    presetAction_     = 0;
    presetActionMenu_ = true;
  }
}

void UiTft::handleSetupInput(const ui_ctrl::InputEvent &ev) {
  if (setupSub_ == SetupSubmenu::PRESET) {
    handlePresetInput(ev);
    return;
  }

  if (setupSub_ == SetupSubmenu::NONE) {
    if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
      inDetailPage_ = false;
      return;
    }
    if (ev.delta != 0) {
      setupCursor_ = static_cast<uint8_t>((setupCursor_ + ev.delta + NUM_SETUP_ITEMS) % NUM_SETUP_ITEMS);
    }
    if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
      setupSub_   = static_cast<SetupSubmenu>(setupCursor_ + 1);
      subCursor_  = 0;
      subEditing_ = false;
      if (setupSub_ == SetupSubmenu::PRESET) {
        refreshPresetNames();
      }
    }
    return;
  }

  // Di dalam salah satu Submenu SETUP
  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (subEditing_) {
      subEditing_ = false;
    } else {
      setupSub_ = SetupSubmenu::NONE;
    }
    return;
  }

  dsp::SystemConfig cfg = dsp_->getConfig();

  switch (setupSub_) {
    case SetupSubmenu::MATRIX: {
      // 16 sel crosspoint (4 OUT x 4 SRC)
      if (ev.delta != 0) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 16) % 16);
      }
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        const uint8_t o = subCursor_ / 4;
        const uint8_t s = subCursor_ % 4;
        // Cycle cepat: 0.0 -> 0.5 -> 1.0 -> 0.0
        float g = cfg.matrix.mixGain[o][s];
        if (g < 0.25f)      g = 0.5f;
        else if (g < 0.75f) g = 1.0f;
        else                g = 0.0f;
        cfg.matrix.mixGain[o][s] = g;
        dsp_->applyConfig(cfg);
      }
      break;
    }

    case SetupSubmenu::LEVEL: {
      // 7 baris: [0]Master, [1]IN A, [2]IN B, [3..6]OUT 1..4
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        subEditing_ = !subEditing_;
        break;
      }
      if (ev.delta == 0) break;
      if (!subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 7) % 7);
      } else {
        const float d = static_cast<float>(ev.delta);
        if (subCursor_ == 0) {
          cfg.master.volumeDb = clampf(cfg.master.volumeDb + d, -60.0f, 0.0f);
        } else if (subCursor_ <= 2) {
          cfg.inputs[subCursor_ - 1].gainDb = clampf(cfg.inputs[subCursor_ - 1].gainDb + d * 0.5f, -12.0f, 12.0f);
        } else {
          cfg.outputs[subCursor_ - 3].gainDb = clampf(cfg.outputs[subCursor_ - 3].gainDb + d * 0.5f, -12.0f, 12.0f);
        }
        dsp_->applyConfig(cfg);
      }
      break;
    }

    case SetupSubmenu::SINYAL: {
      // 4 item: [0]Type (OFF/WHITE/PINK), [1]Level dB, [2]Dest Mask, [3]Auto-Off Sec
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          uint8_t t = (static_cast<uint8_t>(cfg.generator.type) + 1) % 3;
          dsp_->setGenerator(static_cast<dsp::GeneratorType>(t),
                             cfg.generator.levelDb, cfg.generator.destMask,
                             cfg.generator.autoOffSeconds);
        } else {
          subEditing_ = !subEditing_;
        }
        break;
      }
      if (ev.delta == 0) break;
      if (!subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 4) % 4);
      } else {
        if (subCursor_ == 1) {
          cfg.generator.levelDb = clampf(cfg.generator.levelDb + ev.delta, -60.0f, -6.0f);
        } else if (subCursor_ == 2) {
          // Cycle common masks: 0x0F (All), 0x03 (OUT1-2), 0x0C (OUT3-4), 0x01, 0x02, 0x04, 0x08
          static constexpr uint8_t MASKS[7] = {0x0F, 0x03, 0x0C, 0x01, 0x02, 0x04, 0x08};
          int idx = 0;
          for (int i = 0; i < 7; i++) if (MASKS[i] == cfg.generator.destMask) idx = i;
          cfg.generator.destMask = MASKS[(idx + ev.delta + 7) % 7];
        } else if (subCursor_ == 3) {
          int sec = static_cast<int>(cfg.generator.autoOffSeconds) + ev.delta * 5;
          cfg.generator.autoOffSeconds = static_cast<uint16_t>(constrain(sec, 5, 120));
        }
        dsp_->setGenerator(cfg.generator.type, cfg.generator.levelDb,
                           cfg.generator.destMask, cfg.generator.autoOffSeconds);
      }
      break;
    }

    case SetupSubmenu::TEMA: {
      // 2 item: [0] Tema UI (7 pilihan), [1] Mode Warna Panel (4 pilihan)
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        subEditing_ = !subEditing_;
        break;
      }
      if (ev.delta == 0) break;
      if (!subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 2) % 2);
      } else {
        if (subCursor_ == 0) {
          uiCfg_.theme = static_cast<UiTheme>((static_cast<int>(uiCfg_.theme) + ev.delta + 7) % 7);
        } else {
          uiCfg_.panelMode = static_cast<PanelColorMode>((static_cast<int>(uiCfg_.panelMode) + ev.delta + 4) % 4);
        }
        applyUiConfig(uiCfg_, true);
      }
      break;
    }

    case SetupSubmenu::WIFI: {
      // 2 item: [0] WiFi Mode (OFF / AU-WEB / AU-OTA), [1] BLE Control Toggle
      if (ev.delta != 0) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 2) % 2);
      }
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          uiCfg_.wifiMode = static_cast<WifiMode>((static_cast<uint8_t>(uiCfg_.wifiMode) + 1) % 3);
          applyUiConfig(uiCfg_, true);
          showToast(uiCfg_.wifiMode == WifiMode::AU_WEB ? "WIFI: AU-WEB AKTIF"
                    : (uiCfg_.wifiMode == WifiMode::AU_OTA ? "WIFI: AU-OTA AKTIF" : "WIFI: OFF"));
        } else {
          uiCfg_.bleEnabled = !uiCfg_.bleEnabled;
          applyUiConfig(uiCfg_, true);
        }
      }
      break;
    }

    case SetupSubmenu::INPUT: {
      // 3 item: [0] USB Rec Source (Post-DSP / Raw), [1] RTA Tap Source, [2] RTA FFT Size (256/512)
      if (ev.delta != 0 && !subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 3) % 3);
      }
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          cfg.matrix.usbRecPostInputDsp = !cfg.matrix.usbRecPostInputDsp;
          dsp_->applyConfig(cfg);
        } else if (subCursor_ == 1) {
          uint8_t ns = (static_cast<uint8_t>(cfg.master.rtaSource) + 1) % 8;
          dsp_->setRtaConfig(static_cast<dsp::RtaSource>(ns), cfg.master.rtaFftSize);
        } else {
          uint16_t nf = (cfg.master.rtaFftSize == 512) ? 256 : 512;
          dsp_->setRtaConfig(cfg.master.rtaSource, nf);
        }
      }
      break;
    }

    case SetupSubmenu::SISTEM: {
      // 4 item: [0] Kunci Panel (Lock), [1] Standby Enable, [2] Standby Timer (5-600s), [3] Factory Reset DSP
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          uiCfg_.panelLocked = true;
          applyUiConfig(uiCfg_, true);
          showToast("PANEL DIKUNCI (TAHAN 3s)");
        } else if (subCursor_ == 1) {
          uiCfg_.standbyEnabled = !uiCfg_.standbyEnabled;
          applyUiConfig(uiCfg_, true);
        } else if (subCursor_ == 2) {
          subEditing_ = !subEditing_;
        } else if (subCursor_ == 3) {
          dsp::SystemConfig defCfg = {};
          dsp::DspEngine::loadDefaultConfig(defCfg);
          dsp_->applyConfig(defCfg);
          showToast("DSP RESET PABRIK!");
        }
        break;
      }
      if (ev.delta == 0) break;
      if (!subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 4) % 4);
      } else if (subCursor_ == 2) {
        int t = static_cast<int>(uiCfg_.standbyTimeoutSec) + ev.accelDelta * 5;
        uiCfg_.standbyTimeoutSec = static_cast<uint16_t>(constrain(t, 5, 600));
        applyUiConfig(uiCfg_, true);
      }
      break;
    }

    default:
      break;
  }
}

// ============================================================================
// 7. EVALUASI KURVA FREKUENSI GABUNGAN (XOVER + 9-BAND PEQ)
// ============================================================================
float UiTft::evalChannelMagDb(int chIdx, bool isOutput, float freqHz,
                              const dsp::SystemConfig &cfg) const {
  const double fs = static_cast<double>(AUDIO_SAMPLE_RATE);
  double magSq = 1.0;

  if (!isOutput) {
    const auto &in = cfg.inputs[chIdx];
    for (uint8_t b = 0; b < dsp::DSP_PEQ_BANDS; b++) {
      magSq *= evalPeqMagSq(in.peq[b], freqHz, fs);
    }
  } else {
    const auto &out = cfg.outputs[chIdx];
    if (!out.bypassDsp) {
      magSq *= evalXoverMagSq(out.hpf, true,  freqHz, fs);
      magSq *= evalXoverMagSq(out.lpf, false, freqHz, fs);
      for (uint8_t b = 0; b < dsp::DSP_PEQ_BANDS; b++) {
        magSq *= evalPeqMagSq(out.peq[b], freqHz, fs);
      }
    }
  }

  if (magSq < 1.0e-12) return -60.0f;
  return static_cast<float>(10.0 * log10(magSq));
}

// ============================================================================
// 8. RENDERER: HEADER, MENU HORIZONTAL, VU METER, KANAL, PRESET, SETUP
// ============================================================================
void UiTft::drawHeader(LGFX_Sprite &sp, const ThemePalette &pal,
                       const audio_io::AudioTelemetry &audioTel,
                       const dsp::SystemConfig &cfg) {
  sp.fillRect(0, 0, TFT_WIDTH, HEADER_HEIGHT, pal.headerBg);
  sp.drawFastHLine(0, HEADER_HEIGHT - 1, TFT_WIDTH, pal.accent);

  sp.setTextSize(1);
  sp.setTextDatum(middle_left);

  // 1. Badge USB UAC2
  const uint16_t usbCol = audioTel.usbMounted
      ? (audioTel.usbRxStreaming || audioTel.usbTxStreaming ? pal.accent : pal.ok)
      : pal.textDim;
  sp.fillRoundRect(4, 4, 42, 17, 3, audioTel.usbMounted ? rgb565(12, 48, 32) : pal.panel);
  sp.drawRoundRect(4, 4, 42, 17, 3, usbCol);
  sp.setTextColor(usbCol);
  sp.drawString(audioTel.usbMounted ? "USB" : "WAIT", 10, 13);

  // 2. Badge WiFi / BLE
  const char *netTxt = (uiCfg_.wifiMode != WifiMode::OFF)
      ? WIFI_MODE_NAMES[static_cast<uint8_t>(uiCfg_.wifiMode)]
      : (uiCfg_.bleEnabled ? "BLE" : "OFF");
  const uint16_t netCol = (uiCfg_.wifiMode != WifiMode::OFF) ? pal.accent2
      : (uiCfg_.bleEnabled ? rgb565(90, 180, 255) : pal.textDim);
  sp.fillRoundRect(50, 4, 48, 17, 3, pal.panel);
  sp.drawRoundRect(50, 4, 48, 17, 3, netCol);
  sp.setTextColor(netCol);
  sp.drawString(netTxt, 56, 13);

  // 3. Ikon Gembok jika Panel Lock aktif
  if (uiCfg_.panelLocked) {
    sp.fillRoundRect(104, 10, 10, 9, 2, pal.warn);
    sp.drawCircle(109, 9, 3, pal.warn);
  }

  // 4. Master Volume
  sp.setTextDatum(middle_center);
  char volBuf[20];
  if (cfg.master.mute) {
    sp.fillRoundRect(148, 4, 82, 17, 3, pal.danger);
    sp.setTextColor(TFT_WHITE);
    sp.drawString("MUTE", 189, 13);
  } else {
    snprintf(volBuf, sizeof(volBuf), "VOL %+.0fdB", cfg.master.volumeDb);
    sp.setTextColor(pal.text);
    sp.drawString(volBuf, 192, 13);
  }

  // 5. Indikator Suhu Chip ESP32-S3
  const float tc = audioTel.chipTempC;
  const uint16_t tCol = (tc >= TEMP_WARN_THRESHOLD_C) ? pal.danger
      : (tc >= 65.0f ? pal.warn : pal.ok);
  char tempBuf[14];
  snprintf(tempBuf, sizeof(tempBuf), "%.0fC", tc);
  sp.fillRoundRect(268, 4, 48, 17, 3, pal.panel);
  sp.drawRoundRect(268, 4, 48, 17, 3, tCol);
  sp.setTextColor(tCol);
  sp.setTextDatum(middle_center);
  sp.drawString(tempBuf, 292, 13);
}

void UiTft::drawHorizontalMenuBar(LGFX_Sprite &sp, const ThemePalette &pal) {
  const int16_t yTop = HEADER_HEIGHT + 2;
  const int16_t hBar = 32;

  // Judul strip "AUDIO INTERFACE MENU"
  sp.setTextSize(1);
  sp.setTextDatum(top_center);
  sp.setTextColor(pal.textDim);
  sp.drawString("AUDIO INTERFACE MENU", TFT_WIDTH / 2, yTop);

  // Smooth horizontal scroll agar item aktif selalu berada di tengah
  constexpr int16_t ITEM_W = 66;
  constexpr int16_t ITEM_GAP = 4;
  constexpr int16_t STRIDE = ITEM_W + ITEM_GAP;
  const float targetScroll = static_cast<float>(static_cast<int>(activeMenu_) * STRIDE)
                             - static_cast<float>((TFT_WIDTH - ITEM_W) / 2);
  menuScrollX_ += (targetScroll - menuScrollX_) * 0.30f;

  const int16_t pillY = yTop + 11;
  for (uint8_t i = 0; i < NUM_TOP_MENU_ITEMS; i++) {
    const int16_t x = static_cast<int16_t>(i * STRIDE - menuScrollX_);
    if (x + ITEM_W < 0 || x > TFT_WIDTH) continue;

    const bool sel = (i == static_cast<uint8_t>(activeMenu_));
    if (sel) {
      sp.fillRoundRect(x, pillY, ITEM_W, 20, 5, pal.accent);
      sp.setTextColor(pal.bg);
    } else {
      sp.fillRoundRect(x, pillY, ITEM_W, 20, 5, pal.panel);
      sp.drawRoundRect(x, pillY, ITEM_W, 20, 5, pal.grid);
      sp.setTextColor(pal.textDim);
    }
    sp.setTextDatum(middle_center);
    sp.drawString(TOP_MENU_LABELS[i], x + ITEM_W / 2, pillY + 10);
  }
  sp.drawFastHLine(0, yTop + hBar + 2, TFT_WIDTH, pal.grid);
}

void UiTft::drawLiveVuMeterPage(LGFX_Sprite &sp, const ThemePalette &pal,
                                const dsp::DspMeters &meters,
                                const dsp::SystemConfig &cfg) {
  const uint32_t nowMs = millis();
  const int16_t topY   = HEADER_HEIGHT + 38; // Di bawah horizontal menu bar
  const int16_t barTop = topY + 20;
  const int16_t barH   = 118;
  const int16_t barW   = 32;

  static constexpr const char *CH_LABELS[6] = {"IN A", "IN B", "OUT1", "OUT2", "OUT3", "OUT4"};

  // Skala dB di sisi kiri
  sp.setTextSize(1);
  sp.setTextDatum(middle_right);
  sp.setTextColor(pal.textDim);
  const float scaleMarks[5] = {0.0f, -6.0f, -12.0f, -24.0f, -48.0f};
  for (int m = 0; m < 5; m++) {
    const float norm = clampf((scaleMarks[m] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t y  = barTop + barH - static_cast<int16_t>(norm * barH);
    char lbl[8];
    snprintf(lbl, sizeof(lbl), "%.0f", scaleMarks[m]);
    sp.drawString(lbl, 24, y);
    sp.drawFastHLine(27, y, TFT_WIDTH - 34, pal.grid);
  }

  for (int ch = 0; ch < 6; ch++) {
    const bool isOut = (ch >= 2);
    const float rawDb = isOut ? meters.outPeakDb[ch - 2] : meters.inPeakDb[ch];
    const bool  clip  = isOut ? meters.outClip[ch - 2]   : meters.inClip[ch];
    const bool  lim   = isOut ? meters.outLimiterActive[ch - 2] : false;
    const bool  muted = isOut ? cfg.outputs[ch - 2].mute : cfg.inputs[ch].mute;

    // Smooth VU ballistic & Peak Hold (tahan 1.2 detik lalu jatuh perlahan)
    if (rawDb > vuSmoothDb_[ch]) {
      vuSmoothDb_[ch] = rawDb;
    } else {
      vuSmoothDb_[ch] = fmaxf(-60.0f, vuSmoothDb_[ch] - 1.4f);
    }

    if (rawDb >= vuPeakHoldDb_[ch]) {
      vuPeakHoldDb_[ch] = rawDb;
      vuPeakHoldMs_[ch] = nowMs;
    } else if (nowMs - vuPeakHoldMs_[ch] > 1200) {
      vuPeakHoldDb_[ch] = fmaxf(-60.0f, vuPeakHoldDb_[ch] - 0.6f);
    }

    if (clip) {
      vuClipHoldMs_[ch] = nowMs + 1500;
    }

    const int16_t x = 32 + ch * 47 + (isOut ? 6 : 0);

    // Badge CLIP / LIM di atas bar
    sp.setTextDatum(middle_center);
    if (nowMs < vuClipHoldMs_[ch]) {
      sp.fillRoundRect(x, topY + 2, barW, 13, 2, pal.danger);
      sp.setTextColor(TFT_WHITE);
      sp.drawString("CLIP", x + barW / 2, topY + 9);
    } else if (lim) {
      sp.fillRoundRect(x, topY + 2, barW, 13, 2, pal.warn);
      sp.setTextColor(TFT_BLACK);
      sp.drawString("LIM", x + barW / 2, topY + 9);
    } else if (muted) {
      sp.fillRoundRect(x, topY + 2, barW, 13, 2, pal.grid);
      sp.setTextColor(pal.textDim);
      sp.drawString("MUTE", x + barW / 2, topY + 9);
    } else {
      char dbStr[8];
      snprintf(dbStr, sizeof(dbStr), "%.0f", clampf(vuPeakHoldDb_[ch], -60.0f, 0.0f));
      sp.setTextColor(pal.textDim);
      sp.drawString(dbStr, x + barW / 2, topY + 9);
    }

    // Kotak latar VU
    sp.fillRect(x, barTop, barW, barH, pal.panel);
    sp.drawRect(x, barTop, barW, barH, pal.grid);

    // Bar tersegmentasi warna hijau -> kuning -> merah
    const float fillNorm = clampf((vuSmoothDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t fillPx = static_cast<int16_t>(fillNorm * (barH - 4));
    for (int16_t seg = 0; seg < fillPx; seg += 4) {
      const float segRatio = static_cast<float>(seg) / static_cast<float>(barH - 4);
      const uint16_t col = (segRatio > 0.88f) ? pal.danger
                         : (segRatio > 0.70f ? pal.warn : pal.ok);
      const int16_t segY = barTop + barH - 2 - seg - 3;
      if (segY >= barTop + 2) {
        sp.fillRect(x + 2, segY, barW - 4, 3, muted ? pal.grid : col);
      }
    }

    // Garis Peak Hold
    const float pkNorm = clampf((vuPeakHoldDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t pkY  = barTop + barH - 2 - static_cast<int16_t>(pkNorm * (barH - 4));
    sp.fillRect(x + 2, pkY, barW - 4, 2, pal.text);

    // Label kanal di bawah
    sp.setTextColor(isOut ? pal.accent : pal.accent2);
    sp.drawString(CH_LABELS[ch], x + barW / 2, barTop + barH + 10);

    // Info GR ringkas di bawah IN A / IN B
    if (!isOut && meters.inCompGrDb[ch] < -0.2f) {
      char grTxt[10];
      snprintf(grTxt, sizeof(grTxt), "%.1f", meters.inCompGrDb[ch]);
      sp.setTextColor(pal.warn);
      sp.drawString(grTxt, x + barW / 2, barTop + barH + 21);
    }
  }

  // Hint footer
  sp.setTextDatum(bottom_center);
  sp.setTextColor(pal.textDim);
  sp.drawString("Putar: Menu | Klik: Mute | Tahan 2s: Edit Kanal", TFT_WIDTH / 2, TFT_HEIGHT - 3);
}

void UiTft::drawChannelFreqCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                                 int chIdx, bool isOutput,
                                 const dsp::RtaSpectrumData &rtaData,
                                 const dsp::SystemConfig &cfg) {
  const int16_t gx = 24, gy = HEADER_HEIGHT + 18;
  const int16_t gw = 290, gh = 116;

  sp.fillRect(gx, gy, gw, gh, pal.panel);
  sp.drawRect(gx, gy, gw, gh, pal.grid);

  // Grid horizontal dB (+18, +9, 0, -9, -18 dB)
  const int16_t midY = gy + gh / 2;
  sp.drawFastHLine(gx + 1, midY, gw - 2, pal.textDim);
  sp.drawFastHLine(gx + 1, gy + gh / 4, gw - 2, pal.grid);
  sp.drawFastHLine(gx + 1, gy + (gh * 3) / 4, gw - 2, pal.grid);

  // Grid vertikal logaritmik (100 Hz, 1 kHz, 10 kHz)
  const float logMin = log10f(20.0f);
  const float logMax = log10f(20000.0f);
  const float logSpan = logMax - logMin;

  const float fMarks[3] = {100.0f, 1000.0f, 10000.0f};
  const char *fLbls[3]  = {"100", "1k", "10k"};
  sp.setTextSize(1);
  sp.setTextDatum(top_center);
  sp.setTextColor(pal.textDim);
  for (int i = 0; i < 3; i++) {
    const int16_t x = gx + static_cast<int16_t>(((log10f(fMarks[i]) - logMin) / logSpan) * gw);
    sp.drawFastVLine(x, gy + 1, gh - 2, pal.grid);
    sp.drawString(fLbls[i], x, gy + gh + 2);
  }

  // Label dB sumbu Y
  sp.setTextDatum(middle_right);
  sp.drawString("+18", gx - 2, gy + 6);
  sp.drawString("0",   gx - 2, midY);
  sp.drawString("-18", gx - 2, gy + gh - 6);

  // 1. Gambar Spektrum RTA Live 31-Band di latar belakang grafik
  if (rtaData.valid) {
    const float barStep = static_cast<float>(gw - 4) / static_cast<float>(dsp::DSP_RTA_ISO_BANDS);
    const int16_t bw = static_cast<int16_t>(fmaxf(2.0f, barStep - 2.0f));
    for (uint8_t b = 0; b < dsp::DSP_RTA_ISO_BANDS; b++) {
      const float norm = clampf((rtaData.bandDb[b] + 72.0f) / 72.0f, 0.0f, 1.0f);
      const int16_t bh = static_cast<int16_t>(norm * (gh - 4));
      const int16_t bx = gx + 2 + static_cast<int16_t>(b * barStep);
      if (bh > 1) {
        sp.fillRect(bx, gy + gh - 2 - bh, bw, bh, pal.rtaBar);
      }
    }
  }

  // 2. Gambar Kurva Gabungan XOVER + 9-Band PEQ Real-Time (dengan cache hash hemat CPU)
  uint32_t h = static_cast<uint32_t>(isOutput ? (100 + chIdx) : chIdx) ^ 0x811C9DC5u;
  const uint8_t *rawPtr = isOutput
      ? reinterpret_cast<const uint8_t *>(&cfg.outputs[chIdx])
      : reinterpret_cast<const uint8_t *>(&cfg.inputs[chIdx]);
  const size_t rawLen = isOutput ? sizeof(dsp::OutputChannelConfig) : sizeof(dsp::InputChannelConfig);
  for (size_t k = 0; k < rawLen; k++) {
    h ^= rawPtr[k];
    h *= 16777619u;
  }

  constexpr int SAMPLES_X = CURVE_POINTS - 1;
  if (h != curveCacheHash_) {
    for (int i = 0; i <= SAMPLES_X; i++) {
      const float t = static_cast<float>(i) / static_cast<float>(SAMPLES_X);
      const float f = powf(10.0f, logMin + t * logSpan);
      curveCacheDb_[i] = clampf(evalChannelMagDb(chIdx, isOutput, f, cfg), -24.0f, 18.0f);
    }
    curveCacheHash_ = h;
  }

  int16_t prevX = gx + 1;
  int16_t prevY = midY;
  for (int i = 0; i <= SAMPLES_X; i++) {
    const float t  = static_cast<float>(i) / static_cast<float>(SAMPLES_X);
    const float db = curveCacheDb_[i];

    const int16_t px = gx + 1 + static_cast<int16_t>(t * (gw - 2));
    const int16_t py = midY - static_cast<int16_t>((db / 18.0f) * (gh * 0.46f));
    const int16_t cy = constrain(py, gy + 2, gy + gh - 2);
    if (i > 0) {
      sp.drawLine(prevX, prevY, px, cy, pal.curve);
      sp.drawLine(prevX, prevY + 1, px, cy + 1, pal.curve); // Tebal 2px agar tajam
    }
    prevX = px;
    prevY = cy;
  }

  // 3. Node titik band PEQ aktif di atas kurva
  const dsp::PeqBandConfig &ab = isOutput
      ? cfg.outputs[chIdx].peq[chActivePeqBand_]
      : cfg.inputs[chIdx].peq[chActivePeqBand_];
  const float nt = clampf((log10f(clampf(ab.freqHz, 20.0f, 20000.0f)) - logMin) / logSpan, 0.0f, 1.0f);
  const float ndb = clampf(evalChannelMagDb(chIdx, isOutput, ab.freqHz, cfg), -24.0f, 18.0f);
  const int16_t nx = gx + 1 + static_cast<int16_t>(nt * (gw - 2));
  const int16_t ny = constrain(midY - static_cast<int16_t>((ndb / 18.0f) * (gh * 0.46f)), gy + 4, gy + gh - 4);
  sp.fillCircle(nx, ny, 4, pal.accent);
  sp.drawCircle(nx, ny, 5, TFT_WHITE);
}

void UiTft::drawChannelDynamicsCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                                     int chIdx, bool isOutput,
                                     const dsp::DspMeters &meters,
                                     const dsp::SystemConfig &cfg) {
  const int16_t gx = 24, gy = HEADER_HEIGHT + 18;
  const int16_t gs = 114; // Kotak transfer 114x114 (-60..0 dB)

  sp.fillRect(gx, gy, gs, gs, pal.panel);
  sp.drawRect(gx, gy, gs, gs, pal.grid);

  // Garis referensi 1:1 (diagonal)
  sp.drawLine(gx, gy + gs, gx + gs, gy, pal.grid);

  // Hitung fungsi transfer Output dB dari Input dB [-60..0]
  auto evalTransfer = [&](float inDb) -> float {
    float yDb = inDb;
    const dsp::NoiseGateConfig &g = isOutput ? cfg.outputs[chIdx].gate : cfg.inputs[chIdx].gate;
    if (g.enabled && inDb < g.thresholdDb) {
      yDb = fmaxf(-60.0f, inDb + g.rangeDb * 0.5f);
    }
    if (!isOutput) {
      const auto &c = cfg.inputs[chIdx].comp;
      if (c.enabled) {
        const float over = yDb - c.thresholdDb;
        const float hk   = c.kneeDb * 0.5f;
        if (c.kneeDb > 0.05f && fabsf(over) <= hk) {
          const float t = over + hk;
          yDb += ((1.0f / c.ratio) - 1.0f) * (t * t) / (2.0f * c.kneeDb);
        } else if (over > hk) {
          yDb = c.thresholdDb + over / c.ratio;
        }
        yDb += c.makeupGainDb;
      }
    } else {
      const auto &lim = cfg.outputs[chIdx].limiter;
      if (lim.enabled && yDb > lim.thresholdDb) {
        yDb = lim.thresholdDb;
      }
    }
    return clampf(yDb, -60.0f, 0.0f);
  };

  int16_t px = gx, py = gy + gs;
  for (int i = 0; i <= 40; i++) {
    const float inDb  = -60.0f + (static_cast<float>(i) / 40.0f) * 60.0f;
    const float outDb = evalTransfer(inDb);
    const int16_t x   = gx + static_cast<int16_t>(((inDb + 60.0f) / 60.0f) * gs);
    const int16_t y   = gy + gs - static_cast<int16_t>(((outDb + 60.0f) / 60.0f) * gs);
    if (i > 0) {
      sp.drawLine(px, py, x, y, pal.curve);
      sp.drawLine(px, py + 1, x, y + 1, pal.curve);
    }
    px = x; py = y;
  }

  // Titik sinyal live bergerak di kurva transfer
  const float liveIn  = clampf(isOutput ? meters.outPeakDb[chIdx] : meters.inPeakDb[chIdx], -60.0f, 0.0f);
  const float liveOut = evalTransfer(liveIn);
  const int16_t lx = gx + static_cast<int16_t>(((liveIn + 60.0f) / 60.0f) * gs);
  const int16_t ly = gy + gs - static_cast<int16_t>(((liveOut + 60.0f) / 60.0f) * gs);
  sp.fillCircle(lx, ly, 4, pal.accent);

  // Meter Gain Reduction (GR) Vertikal di sebelah kanan kotak kurva
  const int16_t grX = gx + gs + 14;
  const int16_t grW = 18;
  const float grDb  = isOutput ? meters.outLimiterGrDb[chIdx] : meters.inCompGrDb[chIdx];
  const int chSlot  = isOutput ? (2 + chIdx) : chIdx;
  grSmoothDb_[chSlot] = (grDb < grSmoothDb_[chSlot]) ? grDb : (grSmoothDb_[chSlot] * 0.85f);

  sp.fillRect(grX, gy, grW, gs, pal.panel);
  sp.drawRect(grX, gy, grW, gs, pal.grid);
  const float grNorm = clampf(-grSmoothDb_[chSlot] / 24.0f, 0.0f, 1.0f);
  const int16_t grH  = static_cast<int16_t>(grNorm * (gs - 4));
  if (grH > 0) {
    sp.fillRect(grX + 2, gy + 2, grW - 4, grH, pal.warn);
  }
  sp.setTextSize(1);
  sp.setTextDatum(top_center);
  sp.setTextColor(pal.warn);
  char grBuf[12];
  snprintf(grBuf, sizeof(grBuf), "GR%.1f", grSmoothDb_[chSlot]);
  sp.drawString(grBuf, grX + grW / 2, gy + gs + 2);

  // Ringkasan parameter Dynamics di sebelah kanan (x = 178..314)
  const int16_t tx = 178;
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  sp.drawString(isOutput ? "GATE & LIMITER" : "GATE & COMP", tx, gy);

  char line[28];
  const dsp::NoiseGateConfig &g = isOutput ? cfg.outputs[chIdx].gate : cfg.inputs[chIdx].gate;
  sp.setTextColor(pal.text);
  snprintf(line, sizeof(line), "GATE: %s %.0fdB", g.enabled ? "ON" : "OFF", g.thresholdDb);
  sp.drawString(line, tx, gy + 18);
  snprintf(line, sizeof(line), "A:%.1fms R:%.0fms", g.attackMs, g.releaseMs);
  sp.setTextColor(pal.textDim);
  sp.drawString(line, tx, gy + 32);

  if (!isOutput) {
    const auto &c = cfg.inputs[chIdx].comp;
    sp.setTextColor(pal.text);
    snprintf(line, sizeof(line), "COMP: %s %.0fdB", c.enabled ? "ON" : "OFF", c.thresholdDb);
    sp.drawString(line, tx, gy + 54);
    snprintf(line, sizeof(line), "RAT:%.1f:1 KN:%.0fdB", c.ratio, c.kneeDb);
    sp.drawString(line, tx, gy + 68);
    snprintf(line, sizeof(line), "MKP:+%.1fdB", c.makeupGainDb);
    sp.setTextColor(pal.accent2);
    sp.drawString(line, tx, gy + 82);
  } else {
    const auto &l = cfg.outputs[chIdx].limiter;
    sp.setTextColor(pal.text);
    snprintf(line, sizeof(line), "LIM : %s %.1fdB", l.enabled ? "ON" : "OFF", l.thresholdDb);
    sp.drawString(line, tx, gy + 54);
    snprintf(line, sizeof(line), "A:%.2fms R:%.0fms", l.attackMs, l.releaseMs);
    sp.drawString(line, tx, gy + 68);
    snprintf(line, sizeof(line), "CLIP: %s",
             cfg.outputs[chIdx].antiClipMode == dsp::AntiClipMode::SOFT_TANH ? "SOFT" : "HARD");
    sp.setTextColor(pal.accent2);
    sp.drawString(line, tx, gy + 82);
  }
}

void UiTft::drawChannelPage(LGFX_Sprite &sp, const ThemePalette &pal,
                            const dsp::DspMeters &meters,
                            const dsp::RtaSpectrumData &rtaData,
                            const dsp::SystemConfig &cfg) {
  const bool isOutput = (static_cast<uint8_t>(activeMenu_) >= static_cast<uint8_t>(TopMenuId::OUT_1));
  const int chIdx = isOutput
      ? (static_cast<int>(activeMenu_) - static_cast<int>(TopMenuId::OUT_1))
      : (static_cast<int>(activeMenu_) - static_cast<int>(TopMenuId::IN_A));

  // Sub-header kanal + switcher Tab
  const int16_t shY = HEADER_HEIGHT + 2;
  sp.setTextSize(1);
  sp.setTextDatum(middle_left);
  sp.setTextColor(pal.accent);
  sp.drawString(TOP_MENU_LABELS[static_cast<uint8_t>(activeMenu_)], 6, shY + 7);

  // Pill Tab 0 (EQ/XO+RTA) & Tab 1 (DYNAMICS)
  const bool tabFocus = (chParamIdx_ == 0);
  sp.fillRoundRect(68, shY, 114, 14, 3, (chTab_ == 0) ? pal.accent : pal.panel);
  if (tabFocus) sp.drawRoundRect(68, shY, 114, 14, 3, TFT_WHITE);
  sp.setTextDatum(middle_center);
  sp.setTextColor((chTab_ == 0) ? pal.bg : pal.textDim);
  sp.drawString(isOutput ? "XOVER+PEQ+RTA" : "PEQ+RTA", 125, shY + 7);

  sp.fillRoundRect(188, shY, 124, 14, 3, (chTab_ == 1) ? pal.accent : pal.panel);
  if (tabFocus) sp.drawRoundRect(188, shY, 124, 14, 3, TFT_WHITE);
  sp.setTextColor((chTab_ == 1) ? pal.bg : pal.textDim);
  sp.drawString(isOutput ? "GATE/LIMITER" : "GATE/COMP/GR", 250, shY + 7);

  // Area Grafik Utama
  if (chTab_ == 0) {
    drawChannelFreqCurve(sp, pal, chIdx, isOutput, rtaData, cfg);
  } else {
    drawChannelDynamicsCurve(sp, pal, chIdx, isOutput, meters, cfg);
  }

  // Strip Parameter Interaktif di Bawah Grafik (y = 174..236)
  const int16_t py = 174;
  sp.fillRoundRect(4, py, TFT_WIDTH - 8, 62, 5, pal.panel);
  sp.drawRoundRect(4, py, TFT_WIDTH - 8, 62, 5, chEditingParam_ ? pal.accent : pal.grid);

  // Format nama & nilai parameter yang sedang disorot
  char pName[28] = "";
  char pVal[36]  = "";
  if (chParamIdx_ == 0) {
    strcpy(pName, "GANTI TAB GRAFIK");
    strcpy(pVal, (chTab_ == 0) ? "[XOVER+PEQ] -> Klik" : "[DYNAMICS] -> Klik");
  } else if (chTab_ == 0) {
    if (!isOutput) {
      const auto &in = cfg.inputs[chIdx];
      const auto &b  = in.peq[chActivePeqBand_];
      switch (chParamIdx_) {
        case 1: strcpy(pName, "INPUT GAIN");   snprintf(pVal, sizeof(pVal), "%+.1f dB", in.gainDb); break;
        case 2: strcpy(pName, "INPUT MUTE");   strcpy(pVal, in.mute ? "MUTED" : "ACTIVE"); break;
        case 3: strcpy(pName, "PHASE INVERT"); strcpy(pVal, in.phaseInvert ? "180 deg" : "0 deg"); break;
        case 4: strcpy(pName, "PEQ BAND");     snprintf(pVal, sizeof(pVal), "BAND %u / 9 (%s)", chActivePeqBand_ + 1, PEQ_TYPE_SHORT[static_cast<uint8_t>(b.type)]); break;
        case 5: strcpy(pName, "PEQ FREQ");     snprintf(pVal, sizeof(pVal), "%.1f Hz", b.freqHz); break;
        case 6: strcpy(pName, "PEQ GAIN");     snprintf(pVal, sizeof(pVal), "%+.1f dB", b.gainDb); break;
        case 7: strcpy(pName, "PEQ Q FACTOR"); snprintf(pVal, sizeof(pVal), "Q = %.1f", b.q); break;
        case 8: strcpy(pName, "COPY CHANNEL"); strcpy(pVal, "Klik: Salin ke IN lain"); break;
      }
    } else {
      const auto &out = cfg.outputs[chIdx];
      const auto &b   = out.peq[chActivePeqBand_];
      switch (chParamIdx_) {
        case 1:  strcpy(pName, "OUTPUT GAIN");  snprintf(pVal, sizeof(pVal), "%+.1f dB", out.gainDb); break;
        case 2:  strcpy(pName, "OUTPUT MUTE");  strcpy(pVal, out.mute ? "MUTED" : "ACTIVE"); break;
        case 3:  strcpy(pName, "PHASE INVERT"); strcpy(pVal, out.phaseInvert ? "180 deg" : "0 deg"); break;
        case 4:  strcpy(pName, "XOVER HPF");    if (!out.hpf.enabled) strcpy(pVal, "OFF"); else snprintf(pVal, sizeof(pVal), "%s %u dB/oct", XOVER_TYPE_SHORT[static_cast<uint8_t>(out.hpf.type)], static_cast<unsigned>(out.hpf.slope)); break;
        case 5:  strcpy(pName, "HPF FREQ");     snprintf(pVal, sizeof(pVal), "%.1f Hz", out.hpf.freqHz); break;
        case 6:  strcpy(pName, "XOVER LPF");    if (!out.lpf.enabled) strcpy(pVal, "OFF"); else snprintf(pVal, sizeof(pVal), "%s %u dB/oct", XOVER_TYPE_SHORT[static_cast<uint8_t>(out.lpf.type)], static_cast<unsigned>(out.lpf.slope)); break;
        case 7:  strcpy(pName, "LPF FREQ");     snprintf(pVal, sizeof(pVal), "%.1f Hz", out.lpf.freqHz); break;
        case 8:  strcpy(pName, "PEQ BAND");     snprintf(pVal, sizeof(pVal), "BAND %u / 9 (%s)", chActivePeqBand_ + 1, PEQ_TYPE_SHORT[static_cast<uint8_t>(b.type)]); break;
        case 9:  strcpy(pName, "PEQ FREQ");     snprintf(pVal, sizeof(pVal), "%.1f Hz", b.freqHz); break;
        case 10: strcpy(pName, "PEQ GAIN");     snprintf(pVal, sizeof(pVal), "%+.1f dB", b.gainDb); break;
        case 11: strcpy(pName, "PEQ Q FACTOR"); snprintf(pVal, sizeof(pVal), "Q = %.1f", b.q); break;
        case 12: strcpy(pName, "DELAY PSRAM");  snprintf(pVal, sizeof(pVal), "%.1f ms (%.1fm)", out.delayMs, out.delayMs * 0.343f); break;
        case 13: strcpy(pName, "BYPASS DSP");   strcpy(pVal, out.bypassDsp ? "BYPASSED" : "DSP ACTIVE"); break;
        case 14: strcpy(pName, "COPY TO ALL");  strcpy(pVal, "Klik: Salin -> OUT 1-4"); break;
      }
    }
  } else {
    if (!isOutput) {
      const auto &in = cfg.inputs[chIdx];
      switch (chParamIdx_) {
        case 1: strcpy(pName, "GATE ENABLE");  strcpy(pVal, in.gate.enabled ? "ON" : "OFF"); break;
        case 2: strcpy(pName, "GATE THRESH");  snprintf(pVal, sizeof(pVal), "%.1f dB", in.gate.thresholdDb); break;
        case 3: strcpy(pName, "GATE ATTACK");  snprintf(pVal, sizeof(pVal), "%.1f ms", in.gate.attackMs); break;
        case 4: strcpy(pName, "GATE RELEASE"); snprintf(pVal, sizeof(pVal), "%.0f ms", in.gate.releaseMs); break;
        case 5: strcpy(pName, "COMP ENABLE");  strcpy(pVal, in.comp.enabled ? "ON" : "OFF"); break;
        case 6: strcpy(pName, "COMP THRESH");  snprintf(pVal, sizeof(pVal), "%.1f dB", in.comp.thresholdDb); break;
        case 7: strcpy(pName, "COMP RATIO");   snprintf(pVal, sizeof(pVal), "%.1f : 1", in.comp.ratio); break;
        case 8: strcpy(pName, "COMP KNEE");    snprintf(pVal, sizeof(pVal), "%.1f dB", in.comp.kneeDb); break;
        case 9: strcpy(pName, "COMP MAKEUP");  snprintf(pVal, sizeof(pVal), "+%.1f dB", in.comp.makeupGainDb); break;
      }
    } else {
      const auto &out = cfg.outputs[chIdx];
      switch (chParamIdx_) {
        case 1: strcpy(pName, "GATE ENABLE");  strcpy(pVal, out.gate.enabled ? "ON (-50dB def)" : "OFF"); break;
        case 2: strcpy(pName, "GATE THRESH");  snprintf(pVal, sizeof(pVal), "%.1f dB", out.gate.thresholdDb); break;
        case 3: strcpy(pName, "GATE ATTACK");  snprintf(pVal, sizeof(pVal), "%.1f ms", out.gate.attackMs); break;
        case 4: strcpy(pName, "GATE RELEASE"); snprintf(pVal, sizeof(pVal), "%.0f ms", out.gate.releaseMs); break;
        case 5: strcpy(pName, "LIMITER EN");   strcpy(pVal, out.limiter.enabled ? "ON" : "OFF"); break;
        case 6: strcpy(pName, "LIM THRESH");   snprintf(pVal, sizeof(pVal), "%.1f dB", out.limiter.thresholdDb); break;
        case 7: strcpy(pName, "LIM ATTACK");   snprintf(pVal, sizeof(pVal), "%.2f ms", out.limiter.attackMs); break;
        case 8: strcpy(pName, "LIM RELEASE");  snprintf(pVal, sizeof(pVal), "%.0f ms", out.limiter.releaseMs); break;
        case 9: strcpy(pName, "ANTI-CLIP");    strcpy(pVal, out.antiClipMode == dsp::AntiClipMode::SOFT_TANH ? "SOFT TANH" : "HARD CLIP"); break;
      }
    }
  }

  sp.setTextDatum(top_left);
  sp.setTextColor(pal.textDim);
  sp.drawString(chEditingParam_ ? "EDIT PARAMETER (*):" : "PILIH PARAMETER:", 12, py + 6);

  sp.setTextColor(pal.accent2);
  sp.drawString(pName, 12, py + 20);

  sp.setTextSize(2);
  sp.setTextDatum(middle_right);
  sp.setTextColor(chEditingParam_ ? pal.accent : pal.text);
  sp.drawString(pVal, TFT_WIDTH - 14, py + 31);

  sp.setTextSize(1);
  sp.setTextDatum(bottom_left);
  sp.setTextColor(pal.textDim);
  sp.drawString("Klik: Edit/Toggle | Tahan <1s: Kembali", 12, py + 58);
}

void UiTft::drawPresetPage(LGFX_Sprite &sp, const ThemePalette &pal) {
  sp.setTextSize(1);
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  sp.drawString("PRESET MANAGER (LITTLEFS 8MB)", 10, HEADER_HEIGHT + 6);

  const int16_t listY = HEADER_HEIGHT + 22;
  for (uint8_t s = 0; s < NUM_PRESET_SLOTS; s++) {
    const int16_t y = listY + s * 22;
    const bool sel = (s == presetSlotCursor_);
    if (sel) {
      sp.fillRoundRect(8, y, TFT_WIDTH - 16, 20, 4, pal.accent);
      sp.setTextColor(pal.bg);
    } else {
      sp.fillRoundRect(8, y, TFT_WIDTH - 16, 20, 4, pal.panel);
      sp.setTextColor(presetExists_[s] ? pal.text : pal.textDim);
    }
    char row[40];
    snprintf(row, sizeof(row), "%u. %s", s + 1, presetNames_[s]);
    sp.setTextDatum(middle_left);
    sp.drawString(row, 16, y + 10);
  }

  // Popup Aksi (LOAD / SAVE / RENAME) atau Editor Rename
  if (presetRenaming_) {
    sp.fillRoundRect(24, 72, TFT_WIDTH - 48, 92, 8, pal.bg);
    sp.drawRoundRect(24, 72, TFT_WIDTH - 48, 92, 8, pal.accent);
    sp.setTextDatum(top_center);
    sp.setTextColor(pal.accent);
    sp.drawString("RENAME PRESET", TFT_WIDTH / 2, 82);

    sp.setTextSize(2);
    for (int i = 0; i < 12; i++) {
      const int16_t cx = 44 + i * 19;
      if (i == renameCharPos_) {
        sp.fillRect(cx - 2, 104, 16, 22, pal.accent);
        sp.setTextColor(pal.bg);
      } else {
        sp.setTextColor(pal.text);
      }
      char cStr[2] = {renameBuf_[i], '\0'};
      sp.setTextDatum(top_left);
      sp.drawString(cStr, cx, 107);
    }
    sp.setTextSize(1);
    sp.setTextDatum(bottom_center);
    sp.setTextColor(pal.textDim);
    sp.drawString("Putar: Huruf | Klik: Geser | Tahan: Simpan", TFT_WIDTH / 2, 156);
  } else if (presetActionMenu_) {
    sp.fillRoundRect(50, 78, TFT_WIDTH - 100, 84, 8, pal.bg);
    sp.drawRoundRect(50, 78, TFT_WIDTH - 100, 84, 8, pal.accent);
    const char *acts[3] = {"LOAD PRESET", "SAVE PRESET", "RENAME PRESET"};
    for (uint8_t a = 0; a < 3; a++) {
      const int16_t ay = 88 + a * 22;
      if (a == presetAction_) {
        sp.fillRoundRect(64, ay, TFT_WIDTH - 128, 18, 4, pal.accent);
        sp.setTextColor(pal.bg);
      } else {
        sp.setTextColor(pal.text);
      }
      sp.setTextDatum(middle_center);
      sp.drawString(acts[a], TFT_WIDTH / 2, ay + 9);
    }
  }
}

void UiTft::drawSetupMenuPage(LGFX_Sprite &sp, const ThemePalette &pal) {
  sp.setTextSize(1);
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  sp.drawString("SETUP & KONFIGURASI SISTEM", 10, HEADER_HEIGHT + 6);

  // Grid 2 kolom x 4 baris untuk 8 Submenu SETUP
  for (uint8_t i = 0; i < NUM_SETUP_ITEMS; i++) {
    const int16_t col = i % 2;
    const int16_t row = i / 2;
    const int16_t x   = 10 + col * 154;
    const int16_t y   = HEADER_HEIGHT + 24 + row * 44;
    const bool sel    = (i == setupCursor_);

    sp.fillRoundRect(x, y, 146, 36, 6, sel ? pal.accent : pal.panel);
    sp.drawRoundRect(x, y, 146, 36, 6, sel ? TFT_WHITE : pal.grid);
    sp.setTextDatum(middle_center);
    sp.setTextSize(2);
    sp.setTextColor(sel ? pal.bg : pal.text);
    sp.drawString(SETUP_LABELS[i], x + 73, y + 18);
  }
  sp.setTextSize(1);
}

void UiTft::drawSetupSubPage(LGFX_Sprite &sp, const ThemePalette &pal,
                             const audio_io::AudioTelemetry &audioTel,
                             const dsp::DspMeters &meters,
                             const dsp::SystemConfig &cfg) {
  if (setupSub_ == SetupSubmenu::PRESET) {
    drawPresetPage(sp, pal);
    return;
  }

  sp.setTextSize(1);
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  char hdr[36];
  snprintf(hdr, sizeof(hdr), "SETUP > %s", SETUP_LABELS[static_cast<uint8_t>(setupSub_) - 1]);
  sp.drawString(hdr, 10, HEADER_HEIGHT + 6);

  const int16_t y0 = HEADER_HEIGHT + 26;

  switch (setupSub_) {
    case SetupSubmenu::MATRIX: {
      const char *srcNames[4] = {"IN A", "IN B", "USB L", "USB R"};
      const char *outNames[4] = {"OUT 1", "OUT 2", "OUT 3", "OUT 4"};
      sp.setTextDatum(middle_center);
      for (int s = 0; s < 4; s++) {
        sp.setTextColor(pal.accent2);
        sp.drawString(srcNames[s], 90 + s * 60, y0 + 6);
      }
      for (int o = 0; o < 4; o++) {
        const int16_t ry = y0 + 20 + o * 34;
        sp.setTextDatum(middle_left);
        sp.setTextColor(pal.text);
        sp.drawString(outNames[o], 10, ry + 13);
        for (int s = 0; s < 4; s++) {
          const int16_t cx = 62 + s * 60;
          const bool sel   = (subCursor_ == o * 4 + s);
          const float g    = cfg.matrix.mixGain[o][s];
          const uint16_t fill = (g > 0.75f) ? pal.ok : (g > 0.25f ? pal.warn : pal.panel);
          sp.fillRoundRect(cx, ry, 52, 26, 4, fill);
          sp.drawRoundRect(cx, ry, 52, 26, 4, sel ? TFT_WHITE : pal.grid);
          if (sel) sp.drawRoundRect(cx - 1, ry - 1, 54, 28, 5, pal.accent);
          char pct[8];
          snprintf(pct, sizeof(pct), "%d%%", static_cast<int>(g * 100.0f));
          sp.setTextDatum(middle_center);
          sp.setTextColor((g > 0.25f) ? TFT_BLACK : pal.textDim);
          sp.drawString(pct, cx + 26, ry + 13);
        }
      }
      break;
    }

    case SetupSubmenu::LEVEL: {
      const char *names[7] = {"MASTER VOL", "IN A GAIN", "IN B GAIN", "OUT 1 GAIN", "OUT 2 GAIN", "OUT 3 GAIN", "OUT 4 GAIN"};
      for (int i = 0; i < 7; i++) {
        const int16_t y = y0 + i * 24;
        const bool sel  = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 21, 4, sel ? (subEditing_ ? pal.accent : pal.headerBg) : pal.panel);
        if (sel) sp.drawRoundRect(10, y, TFT_WIDTH - 20, 21, 4, pal.accent);
        const float val = (i == 0) ? cfg.master.volumeDb
                        : (i <= 2 ? cfg.inputs[i - 1].gainDb : cfg.outputs[i - 3].gainDb);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel && subEditing_ ? pal.bg : pal.text);
        sp.drawString(names[i], 18, y + 10);
        char vStr[16];
        snprintf(vStr, sizeof(vStr), "%+.1f dB", val);
        sp.setTextDatum(middle_right);
        sp.drawString(vStr, TFT_WIDTH - 18, y + 10);
      }
      break;
    }

    case SetupSubmenu::SINYAL: {
      char vals[4][28];
      snprintf(vals[0], sizeof(vals[0]), "%s", GEN_TYPE_NAMES[static_cast<uint8_t>(cfg.generator.type)]);
      snprintf(vals[1], sizeof(vals[1]), "%.1f dBFS", cfg.generator.levelDb);
      snprintf(vals[2], sizeof(vals[2]), "MASK 0x%02X", cfg.generator.destMask);
      snprintf(vals[3], sizeof(vals[3]), "%u dtk (Sisa: %us)", cfg.generator.autoOffSeconds, meters.generatorRemainingSec);
      const char *lbls[4] = {"MODE NOISE", "LEVEL OUTPUT", "TUJUAN OUT", "AUTO-OFF TIMER"};

      for (int i = 0; i < 4; i++) {
        const int16_t y = y0 + 6 + i * 34;
        const bool sel = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 28, 5, sel ? (subEditing_ ? pal.accent : pal.headerBg) : pal.panel);
        if (sel) sp.drawRoundRect(10, y, TFT_WIDTH - 20, 28, 5, pal.accent);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel && subEditing_ ? pal.bg : pal.text);
        sp.drawString(lbls[i], 18, y + 14);
        sp.setTextDatum(middle_right);
        sp.drawString(vals[i], TFT_WIDTH - 18, y + 14);
      }
      break;
    }

    case SetupSubmenu::TEMA: {
      const char *lbls[2] = {"TEMA WARNA UI (7)", "MODE PANEL LCD (4)"};
      const char *vals[2] = {
          THEME_NAMES[static_cast<uint8_t>(uiCfg_.theme)],
          PANEL_MODE_NAMES[static_cast<uint8_t>(uiCfg_.panelMode)]};
      for (int i = 0; i < 2; i++) {
        const int16_t y = y0 + 10 + i * 44;
        const bool sel  = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 34, 6, sel ? (subEditing_ ? pal.accent : pal.headerBg) : pal.panel);
        if (sel) sp.drawRoundRect(10, y, TFT_WIDTH - 20, 34, 6, pal.accent);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel && subEditing_ ? pal.bg : pal.text);
        sp.drawString(lbls[i], 18, y + 17);
        sp.setTextDatum(middle_right);
        sp.drawString(vals[i], TFT_WIDTH - 18, y + 17);
      }
      // Swatch preview warna tema
      sp.fillRoundRect(10,  y0 + 112, 92, 34, 5, pal.accent);
      sp.fillRoundRect(114, y0 + 112, 92, 34, 5, pal.accent2);
      sp.fillRoundRect(218, y0 + 112, 92, 34, 5, pal.curve);
      sp.setTextDatum(middle_center);
      sp.setTextColor(TFT_BLACK);
      sp.drawString("PRIMARY", 56, y0 + 129);
      sp.drawString("SECOND",  160, y0 + 129);
      sp.drawString("CURVE",   264, y0 + 129);
      break;
    }

    case SetupSubmenu::WIFI: {
      const char *lbls[2] = {"MODE WIFI", "BLUETOOTH BLE"};
      const char *vals[2] = {
          WIFI_MODE_NAMES[static_cast<uint8_t>(uiCfg_.wifiMode)],
          uiCfg_.bleEnabled ? "ON" : "OFF"};
      for (int i = 0; i < 2; i++) {
        const int16_t y = y0 + 10 + i * 40;
        const bool sel = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 32, 6, sel ? pal.accent : pal.panel);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel ? pal.bg : pal.text);
        sp.drawString(lbls[i], 18, y + 16);
        sp.setTextDatum(middle_right);
        sp.drawString(vals[i], TFT_WIDTH - 18, y + 16);
      }
      sp.fillRoundRect(10, y0 + 102, TFT_WIDTH - 20, 52, 6, pal.panel);
      sp.setTextDatum(top_left);
      sp.setTextColor(pal.accent2);
      sp.drawString(uiCfg_.wifiMode == WifiMode::AU_WEB ? "AP: ESP32-DLMS-WEB (192.168.4.1)"
                    : (uiCfg_.wifiMode == WifiMode::AU_OTA ? "AP: ESP32-DLMS-OTA (Siap Update)"
                                                           : "WiFi Nonaktif (Noise RF 0 dB)"),
                    18, y0 + 114);
      sp.setTextColor(pal.textDim);
      sp.drawString("Modul Web/OTA penuh aktif di Tahap 4", 18, y0 + 132);
      break;
    }

    case SetupSubmenu::INPUT: {
      char vals[3][24];
      strcpy(vals[0], cfg.matrix.usbRecPostInputDsp ? "POST INPUT DSP" : "RAW ADC");
      strcpy(vals[1], RTA_SRC_NAMES[static_cast<uint8_t>(cfg.master.rtaSource)]);
      snprintf(vals[2], sizeof(vals[2]), "%u POINT", cfg.master.rtaFftSize);
      const char *lbls[3] = {"USB RECORD TAP", "SUMBER RTA", "RESOLUSI FFT"};
      for (int i = 0; i < 3; i++) {
        const int16_t y = y0 + 8 + i * 38;
        const bool sel = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 30, 5, sel ? pal.accent : pal.panel);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel ? pal.bg : pal.text);
        sp.drawString(lbls[i], 18, y + 15);
        sp.setTextDatum(middle_right);
        sp.drawString(vals[i], TFT_WIDTH - 18, y + 15);
      }
      break;
    }

    case SetupSubmenu::SISTEM: {
      char vals[4][24];
      strcpy(vals[0], uiCfg_.panelLocked ? "TERKUNCI" : "KLIK KUNCI");
      strcpy(vals[1], uiCfg_.standbyEnabled ? "AKTIF" : "NONAKTIF");
      snprintf(vals[2], sizeof(vals[2]), "%u DETIK", uiCfg_.standbyTimeoutSec);
      strcpy(vals[3], "KLIK RESET");
      const char *lbls[4] = {"PANEL LOCK", "AUTO STANDBY", "TIMER STANDBY", "FACTORY RESET"};
      for (int i = 0; i < 4; i++) {
        const int16_t y = y0 + i * 28;
        const bool sel = (subCursor_ == i);
        sp.fillRoundRect(10, y, TFT_WIDTH - 20, 24, 4, sel ? (subEditing_ ? pal.accent : pal.headerBg) : pal.panel);
        if (sel) sp.drawRoundRect(10, y, TFT_WIDTH - 20, 24, 4, pal.accent);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel && subEditing_ ? pal.bg : pal.text);
        sp.drawString(lbls[i], 18, y + 12);
        sp.setTextDatum(middle_right);
        sp.drawString(vals[i], TFT_WIDTH - 18, y + 12);
      }

      // Box Info Hardware & DSP
      char info1[48], info2[48];
      snprintf(info1, sizeof(info1), "PSRAM Free: %.2fMB | DSP CPU: %.1f%%",
               ESP.getFreePsram() / 1048576.0f, audioTel.dspCpuLoadPct);
      snprintf(info2, sizeof(info2), "Chip: %.1fC (Pk %.1fC) | LittleFS: %.1fMB",
               audioTel.chipTempC, audioTel.chipTempPeakC, LittleFS.totalBytes() / 1048576.0f);
      sp.fillRoundRect(10, y0 + 118, TFT_WIDTH - 20, 40, 5, pal.panel);
      sp.setTextDatum(top_left);
      sp.setTextColor(pal.accent2);
      sp.drawString(info1, 16, y0 + 124);
      sp.setTextColor(pal.textDim);
      sp.drawString(info2, 16, y0 + 140);
      break;
    }

    default:
      break;
  }
}

void UiTft::drawStandbyScreen(LGFX_Sprite &sp, const ThemePalette &pal,
                              const audio_io::AudioTelemetry &audioTel) {
  if (standbyLogoBuf_ != nullptr) {
    sp.pushImage(0, 0, TFT_WIDTH, TFT_HEIGHT, standbyLogoBuf_);
  } else {
    renderFallbackLogo(sp, pal, true, static_cast<float>(millis()) * 0.001f);
  }
  // Badge suhu & USB minimalis di sudut bawah
  char st[32];
  snprintf(st, sizeof(st), "USB:%s  |  %.1fC", audioTel.usbMounted ? "OK" : "WAIT", audioTel.chipTempC);
  sp.setTextSize(1);
  sp.setTextDatum(bottom_right);
  sp.setTextColor(pal.textDim);
  sp.drawString(st, TFT_WIDTH - 14, TFT_HEIGHT - 12);
}

void UiTft::drawOverheatScreen(LGFX_Sprite &sp, float tempC) {
  const bool flash = ((millis() / 250) & 1u) != 0u;
  sp.fillScreen(flash ? rgb565(200, 0, 0) : rgb565(120, 0, 0));
  sp.drawRect(6, 6, TFT_WIDTH - 12, TFT_HEIGHT - 12, TFT_WHITE);

  sp.setTextDatum(middle_center);
  sp.setTextColor(TFT_WHITE);
  sp.setTextSize(2);
  sp.drawString("PERINGATAN SUHU!", TFT_WIDTH / 2, 56);

  char tStr[20];
  snprintf(tStr, sizeof(tStr), "%.1f C", tempC);
  sp.setTextSize(4);
  sp.setTextColor(rgb565(255, 240, 80));
  sp.drawString(tStr, TFT_WIDTH / 2, 118);

  sp.setTextSize(1);
  sp.setTextColor(TFT_WHITE);
  sp.drawString("SUHU CHIP ESP32-S3 >= 75 C! PERIKSA VENTILASI!", TFT_WIDTH / 2, 174);
  sp.drawString("Klik Encoder untuk abaikan sementara", TFT_WIDTH / 2, 204);
}

void UiTft::drawLockOverlay(LGFX_Sprite &sp, const ThemePalette &pal,
                            const ui_ctrl::InputEvent &ev) {
  const uint32_t nowMs = millis();
  const bool showHoldingUnlock = uiCfg_.panelLocked && ev.isHoldingBtn && (ev.holdDurationMs > 120);
  if (!showHoldingUnlock && nowMs >= lockBannerUntilMs_) return;

  sp.fillRoundRect(46, 86, TFT_WIDTH - 92, 68, 8, pal.bg);
  sp.drawRoundRect(46, 86, TFT_WIDTH - 92, 68, 8, pal.warn);

  // Ikon Gembok
  sp.drawCircle(72, 111, 7, pal.warn);
  sp.fillRoundRect(62, 113, 20, 16, 3, pal.warn);

  sp.setTextSize(1);
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.warn);
  sp.drawString("PANEL TERKUNCI", 92, 98);
  sp.setTextColor(pal.textDim);
  sp.drawString("Tahan 3 detik buka kunci", 92, 114);

  // Progress bar 3 detik
  const float prog = showHoldingUnlock ? clampf(ev.holdDurationMs / 3000.0f, 0.0f, 1.0f) : 0.0f;
  sp.drawRect(92, 132, 164, 10, pal.grid);
  if (prog > 0.0f) {
    sp.fillRect(94, 134, static_cast<int16_t>(prog * 160.0f), 6, pal.warn);
  }
}

// ============================================================================
// 9. MAIN 30 FPS UPDATE & DOUBLE-BUFFER PUSH (CORE 0)
// ============================================================================
void UiTft::update(const ui_ctrl::InputEvent &ev,
                   const audio_io::AudioTelemetry &audioTel,
                   const dsp::DspMeters &dspMeters,
                   const dsp::RtaSpectrumData &rtaData) {
  const uint32_t nowMs = millis();

  // 1. Cek Peringatan Suhu >= 75°C (hysteresis keluar di < 72°C)
  if (audioTel.chipTempC >= TEMP_WARN_THRESHOLD_C) {
    overheatActive_ = true;
    inStandby_      = false;
  } else if (audioTel.chipTempC < (TEMP_WARN_THRESHOLD_C - 3.0f)) {
    overheatActive_    = false;
    overheatDismissed_ = false;
  }

  // 2. Proses input encoder
  handleInput(ev);

  // 3. Cek Timer Auto-Standby (5 - 600 detik)
  if (uiCfg_.standbyEnabled && !inStandby_ && !overheatActive_) {
    const uint32_t idleSec = (nowMs - lastActivityMs_) / 1000u;
    if (idleSec >= uiCfg_.standbyTimeoutSec) {
      inStandby_ = true;
    }
  }

  // 4. Render seluruh frame ke PSRAM Sprite aktif (bebas flicker)
  LGFX_Sprite &sp = fb_[curFb_];
  const ThemePalette pal = resolveTheme(nowMs);
  const dsp::SystemConfig cfg = dsp_->getConfig();

  if (overheatActive_ && !overheatDismissed_) {
    drawOverheatScreen(sp, audioTel.chipTempC);
  } else if (inStandby_) {
    drawStandbyScreen(sp, pal, audioTel);
  } else {
    sp.fillScreen(pal.bg);
    drawHeader(sp, pal, audioTel, cfg);

    if (!inDetailPage_) {
      drawHorizontalMenuBar(sp, pal);
      drawLiveVuMeterPage(sp, pal, dspMeters, cfg);
    } else {
      switch (activeMenu_) {
        case TopMenuId::IN_A:
        case TopMenuId::IN_B:
        case TopMenuId::OUT_1:
        case TopMenuId::OUT_2:
        case TopMenuId::OUT_3:
        case TopMenuId::OUT_4:
          drawChannelPage(sp, pal, dspMeters, rtaData, cfg);
          break;
        case TopMenuId::PRESET:
          drawPresetPage(sp, pal);
          break;
        case TopMenuId::SETUP:
          if (setupSub_ == SetupSubmenu::NONE) {
            drawSetupMenuPage(sp, pal);
          } else {
            drawSetupSubPage(sp, pal, audioTel, dspMeters, cfg);
          }
          break;
        case TopMenuId::METER:
        default:
          drawHorizontalMenuBar(sp, pal);
          drawLiveVuMeterPage(sp, pal, dspMeters, cfg);
          break;
      }
    }

    // Overlay Panel Lock & Toast Notifikasi
    drawLockOverlay(sp, pal, ev);

    if (nowMs < toastUntilMs_) {
      sp.fillRoundRect(54, TFT_HEIGHT - 34, TFT_WIDTH - 108, 22, 6, pal.accent);
      sp.setTextSize(1);
      sp.setTextDatum(middle_center);
      sp.setTextColor(pal.bg);
      sp.drawString(toastMsg_, TFT_WIDTH / 2, TFT_HEIGHT - 23);
    }
  }

  // 5. Push Double-Buffer Sprite ke ST7789 via SPI DMA & tukar buffer
  lcd_.waitDMA();
  sp.pushSprite(&lcd_, 0, 0);
  curFb_ ^= 1;
}

} // namespace ui_tft
