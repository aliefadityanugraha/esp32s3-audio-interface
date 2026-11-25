/**
 * @file ui_tft.cpp
 * @brief UI Layar TFT ILI9341 (320x240 Landscape, LovyanGFX)
 *        Desain Studio Minimalis 5-Tab Tanpa Scroll — Double-Buffered PSRAM (30 FPS, Core 0)
 */

#include "ui_tft.h"

#include <LittleFS.h>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace ui_tft {

namespace {

constexpr const char *PATH_UI_CONFIG = "/ui_settings.bin";
constexpr const char *PATH_BOOT_LOGO = "/logo_boot.bin";
constexpr const char *PATH_STBY_LOGO = "/logo_standby.bin";
constexpr const char *DIR_PRESETS    = "/presets";

constexpr const char *TOP_MENU_LABELS[NUM_TOP_MENU_ITEMS] = {
    "METER", "INPUT", "MATRIX", "OUT", "SETUP"};

constexpr const char *SETUP_LABELS[NUM_SETUP_ITEMS] = {
    "PRESET", "KONEKSI", "TAMPILAN", "SISTEM"};

constexpr const char *SETUP_SUBTITLES[NUM_SETUP_ITEMS] = {
    "8 Slot Memori", "WiFi & BLE", "Tema & Layar", "Kunci & Reset"};

constexpr const char *THEME_NAMES[7] = {
    "RED", "ICE LUXURY", "UNGU-EMAS", "MERAH-BIRU",
    "MERAH-PUTIH", "BIRU-KUNING", "RGB PELANGI"};

constexpr const char *PANEL_MODE_NAMES[4] = {
    "NORMAL", "NEGATIF", "TUKAR R-B", "NEG+R-B"};

constexpr const char *WIFI_MODE_NAMES[3] = {
    "OFF", "AU-WEB", "AU-OTA"};

constexpr const char *XOVER_TYPE_SHORT[3] = {"LR", "BW", "BES"};
constexpr const char *PEQ_TYPE_SHORT[7]   = {"PK", "LS", "HS", "LP", "HP", "NT", "AP"};

constexpr const char RENAME_CHARSET[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_";
constexpr size_t RENAME_CHARSET_LEN   = sizeof(RENAME_CHARSET) - 1;

constexpr float  PI_F = 3.14159265358979323846f;
constexpr double PI_D = 3.14159265358979323846;

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Evaluasi respon magnitudo kuadrat |H(e^jw)|^2 dari 1 Biquad
inline double evalBiquadMagSq(double b0, double b1, double b2,
                              double a1, double a2, double w) {
  const double s    = sin(0.5 * w);
  const double phi  = s * s;
  const double numA = b0 + b1 + b2;
  const double denA = 1.0 + a1 + a2;
  const double num  = numA * numA - 4.0 * (b0 * b1 + 4.0 * b0 * b2 + b1 * b2) * phi
                      + 16.0 * b0 * b2 * phi * phi;
  const double den  = denA * denA - 4.0 * (a1 + 4.0 * a2 + a1 * a2) * phi
                      + 16.0 * a2 * phi * phi;
  if (den < 1.0e-18) return 1.0;
  return (num < 1.0e-18) ? 1.0e-18 : (num / den);
}

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
  const double w     = 2.0 * PI_D * freqHz / fs;
  return evalBiquadMagSq(b0 * invA0, b1 * invA0, b2 * invA0, a1 * invA0, a2 * invA0, w);
}

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
  double magSq   = 1.0;
  for (int i = 0; i < n; i++) {
    const double fc    = clampf(isHpf ? (xo.freqHz / s[i].sc) : (xo.freqHz * s[i].sc),
                                10.0f, static_cast<float>(fs * 0.475));
    const double w0    = 2.0 * PI_D * fc / fs;
    const double cw    = cos(w0);
    const double al    = sin(w0) / (2.0 * static_cast<double>(s[i].q));
    const double invA0 = 1.0 / (1.0 + al);
    const double b0    = (isHpf ? (1.0 + cw) : (1.0 - cw)) * 0.5 * invA0;
    const double b1    = (isHpf ? -(1.0 + cw) : (1.0 - cw)) * invA0;
    const double b2    = b0;
    const double a1    = (-2.0 * cw) * invA0;
    const double a2    = (1.0 - al) * invA0;
    magSq *= evalBiquadMagSq(b0, b1, b2, a1, a2, w);
  }
  return magSq;
}

} // namespace

// ============================================================================
// 1. KONFIGURASI HARDWARE LOVYANGFX ILI9341 (240x320 SPI2_HOST)
// ============================================================================
LgfxIli9341::LgfxIli9341() {
  {
    auto cfg = bus_.config();
    cfg.spi_host    = SPI2_HOST;
    cfg.spi_mode    = 0;
    cfg.freq_write  = 40000000;
    cfg.freq_read   = 16000000;
    cfg.spi_3wire   = false;
    cfg.use_lock    = true;
    cfg.dma_channel = SPI_DMA_CH_AUTO;
    cfg.pin_sclk    = PIN_LCD_SCLK;
    cfg.pin_mosi    = PIN_LCD_MOSI;
    cfg.pin_miso    = -1;
    cfg.pin_dc      = PIN_LCD_DC;
    bus_.config(cfg);
    panel_.setBus(&bus_);
  }
  {
    auto cfg = panel_.config();
    cfg.pin_cs        = PIN_LCD_CS;
    cfg.pin_rst       = PIN_LCD_RST;
    cfg.pin_busy      = -1;
    cfg.panel_width   = 240;
    cfg.panel_height  = 320;
    cfg.memory_width  = 240;
    cfg.memory_height = 320;
    cfg.offset_x      = 0;
    cfg.offset_y      = 0;
    cfg.readable      = false;
    cfg.invert        = false;
    cfg.rgb_order     = false;
    cfg.dlen_16bit    = false;
    cfg.bus_shared    = false;
    panel_.config(cfg);
  }
  setPanel(&panel_);
}

void LgfxIli9341::applyPanelColorMode(PanelColorMode mode) {
  const bool doInvert = (mode == PanelColorMode::NEGATIF ||
                         mode == PanelColorMode::NEGATIF_TUKAR_RB);
  const bool swapRB   = (mode == PanelColorMode::TUKAR_RB ||
                         mode == PanelColorMode::NEGATIF_TUKAR_RB);

  auto cfg = panel_.config();
  cfg.rgb_order = swapRB;
  cfg.invert    = doInvert;
  panel_.config(cfg);

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

  lcd_.init();
  lcd_.initDMA();
  lcd_.setRotation(1);
  lcd_.setColorDepth(16);
  lcd_.applyPanelColorMode(uiCfg_.panelMode);
  lcd_.fillScreen(TFT_BLACK);

  bool ok0 = false, ok1 = false;
  for (int i = 0; i < 2; i++) {
    fb_[i].setPsram(true);
    fb_[i].setColorDepth(16);
    fb_[i].setSwapBytes(true);
    bool created = (fb_[i].createSprite(TFT_WIDTH, TFT_HEIGHT) != nullptr);
    if (i == 0) ok0 = created; else ok1 = created;
  }
  psramSprites_ = (ok0 && ok1);

  input_->setPanelLocked(uiCfg_.panelLocked);

  loadRawImagePsram(PATH_BOOT_LOGO, bootLogoBuf_, bootLogoPixels_);
  loadRawImagePsram(PATH_STBY_LOGO, standbyLogoBuf_, standbyLogoPix_);

  if (!LittleFS.exists(DIR_PRESETS)) {
    LittleFS.mkdir(DIR_PRESETS);
  }
  refreshPresetNames();

  lastActivityMs_ = millis();
  return psramSprites_;
}

void UiTft::loadUiConfig() {
  uiCfg_.magic             = UI_CFG_MAGIC;
  uiCfg_.theme             = UiTheme::ICE_LUXURY;
  uiCfg_.panelMode         = PanelColorMode::NORMAL;
  uiCfg_.standbyTimeoutSec = 120;
  uiCfg_.standbyEnabled    = true;
  uiCfg_.panelLocked       = false;
  uiCfg_.wifiMode          = WifiMode::OFF;
  uiCfg_.bleEnabled        = true;
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

void UiTft::setRadioUiState(WifiMode wifiMode, bool bleEnabled) {
  uiCfg_.wifiMode   = wifiMode;
  uiCfg_.bleEnabled = bleEnabled;
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
// 3. SISTEM 7 TEMA STUDIO HIGH-CONTRAST
// ============================================================================
ThemePalette UiTft::resolveTheme(uint32_t nowMs) const {
  ThemePalette p = {};
  p.ok     = rgb565(24, 218, 118);
  p.warn   = rgb565(255, 190, 32);
  p.danger = rgb565(250, 52, 68);

  switch (uiCfg_.theme) {
    case UiTheme::RED:
      p.bg        = rgb565(10, 8, 10);
      p.panel     = rgb565(22, 14, 18);
      p.headerBg  = rgb565(34, 12, 16);
      p.accent    = rgb565(255, 56, 68);
      p.accent2   = rgb565(255, 168, 72);
      p.text      = rgb565(248, 244, 246);
      p.textDim   = rgb565(142, 116, 122);
      p.grid      = rgb565(46, 26, 32);
      p.curveFill = rgb565(64, 20, 26);
      p.curve     = rgb565(255, 96, 88);
      break;

    case UiTheme::ICE_LUXURY:
      p.bg        = rgb565(8, 11, 17);
      p.panel     = rgb565(16, 23, 35);
      p.headerBg  = rgb565(12, 20, 32);
      p.accent    = rgb565(48, 218, 255);
      p.accent2   = rgb565(148, 212, 255);
      p.text      = rgb565(242, 248, 255);
      p.textDim   = rgb565(118, 144, 170);
      p.grid      = rgb565(30, 44, 64);
      p.curveFill = rgb565(18, 56, 78);
      p.curve     = rgb565(86, 238, 255);
      break;

    case UiTheme::UNGU_EMAS:
      p.bg        = rgb565(11, 9, 18);
      p.panel     = rgb565(23, 18, 38);
      p.headerBg  = rgb565(30, 18, 52);
      p.accent    = rgb565(255, 206, 52);
      p.accent2   = rgb565(192, 116, 255);
      p.text      = rgb565(250, 246, 240);
      p.textDim   = rgb565(148, 132, 176);
      p.grid      = rgb565(48, 36, 74);
      p.curveFill = rgb565(62, 46, 26);
      p.curve     = rgb565(255, 216, 78);
      break;

    case UiTheme::MERAH_BIRU:
      p.bg        = rgb565(8, 10, 18);
      p.panel     = rgb565(17, 21, 36);
      p.headerBg  = rgb565(28, 16, 30);
      p.accent    = rgb565(255, 58, 86);
      p.accent2   = rgb565(68, 164, 255);
      p.text      = rgb565(244, 247, 255);
      p.textDim   = rgb565(126, 140, 168);
      p.grid      = rgb565(34, 42, 66);
      p.curveFill = rgb565(62, 22, 36);
      p.curve     = rgb565(255, 84, 108);
      break;

    case UiTheme::MERAH_PUTIH:
      p.bg        = rgb565(12, 12, 14);
      p.panel     = rgb565(24, 24, 28);
      p.headerBg  = rgb565(148, 22, 32);
      p.accent    = rgb565(255, 255, 255);
      p.accent2   = rgb565(245, 64, 72);
      p.text      = rgb565(255, 255, 255);
      p.textDim   = rgb565(158, 158, 166);
      p.grid      = rgb565(52, 52, 58);
      p.curveFill = rgb565(64, 28, 32);
      p.curve     = rgb565(255, 255, 255);
      break;

    case UiTheme::BIRU_KUNING:
      p.bg        = rgb565(7, 12, 24);
      p.panel     = rgb565(14, 24, 48);
      p.headerBg  = rgb565(14, 30, 68);
      p.accent    = rgb565(255, 212, 28);
      p.accent2   = rgb565(68, 174, 255);
      p.text      = rgb565(245, 250, 255);
      p.textDim   = rgb565(124, 152, 194);
      p.grid      = rgb565(28, 48, 86);
      p.curveFill = rgb565(58, 52, 22);
      p.curve     = rgb565(255, 222, 64);
      break;

    case UiTheme::RGB_PELANGI:
    default: {
      const uint16_t h1 = static_cast<uint16_t>((nowMs / 18) % 360);
      const uint16_t h2 = static_cast<uint16_t>((h1 + 120) % 360);
      uint8_t r1, g1, b1, r2, g2, b2;
      ui_ctrl::InputController::hsvToRgb(h1, 225, 255, r1, g1, b1);
      ui_ctrl::InputController::hsvToRgb(h2, 210, 255, r2, g2, b2);
      p.bg        = rgb565(9, 10, 15);
      p.panel     = rgb565(19, 20, 30);
      p.headerBg  = rgb565(r1 >> 3, g1 >> 3, b1 >> 3);
      p.accent    = rgb565(r1, g1, b1);
      p.accent2   = rgb565(r2, g2, b2);
      p.text      = rgb565(248, 250, 255);
      p.textDim   = rgb565(142, 148, 172);
      p.grid      = rgb565(38, 40, 58);
      p.curveFill = rgb565(r2 >> 2, g2 >> 2, b2 >> 2);
      p.curve     = rgb565(r2, g2, b2);
      break;
    }
  }
  return p;
}

// ============================================================================
// 4. BOOT SCREEN & STANDBY SCREEN (MINIMALIS STUDIO)
// ============================================================================
void UiTft::renderFallbackLogo(LGFX_Sprite &sp, const ThemePalette &pal,
                               bool isStandby, float phase) {
  sp.fillScreen(pal.bg);

  // Frame tipis minimalis dengan 4 sudut aksen
  sp.drawRoundRect(10, 10, TFT_WIDTH - 20, TFT_HEIGHT - 20, 8, pal.grid);
  constexpr int16_t B = 14;
  sp.drawFastHLine(14, 14, B, pal.accent);
  sp.drawFastVLine(14, 14, B, pal.accent);
  sp.drawFastHLine(TFT_WIDTH - 14 - B, 14, B, pal.accent);
  sp.drawFastVLine(TFT_WIDTH - 15, 14, B, pal.accent);
  sp.drawFastHLine(14, TFT_HEIGHT - 15, B, pal.accent);
  sp.drawFastVLine(14, TFT_HEIGHT - 14 - B, B, pal.accent);
  sp.drawFastHLine(TFT_WIDTH - 14 - B, TFT_HEIGHT - 15, B, pal.accent);
  sp.drawFastVLine(TFT_WIDTH - 15, TFT_HEIGHT - 14 - B, B, pal.accent);

  sp.setTextDatum(middle_center);
  sp.setTextColor(pal.text);
  sp.setTextSize(2);
  sp.drawString("DSP Audio Interface", TFT_WIDTH / 2, 54);

  sp.fillRect(TFT_WIDTH / 2 - 40, 69, 80, 2, pal.accent);

  sp.setTextSize(1);
  sp.setTextColor(pal.textDim);
  sp.drawString("48 kHz  |  2-IN 4-OUT", TFT_WIDTH / 2, 82);

  // Gelombang spektrum halus di tengah
  const int16_t cy = isStandby ? 136 : 132;
  sp.drawFastHLine(36, cy, TFT_WIDTH - 72, pal.grid);
  for (int x = 40; x < TFT_WIDTH - 40; x += 5) {
    const float norm = static_cast<float>(x - 40) / static_cast<float>(TFT_WIDTH - 80);
    const float env  = sinf(norm * PI_F);
    const float w1   = sinf(norm * 14.0f + phase * 5.0f) * env;
    const float w2   = cosf(norm * 8.0f - phase * 3.0f) * env * 0.5f;
    const int h      = static_cast<int>(fabsf(w1 + w2) * 18.0f) + 2;
    const uint16_t col = ((x / 5) & 1) ? pal.accent : pal.accent2;
    sp.fillRect(x, cy - h, 2, h * 2, col);
  }

  if (!isStandby) {
    const float clamped = clampf(phase, 0.0f, 1.0f);
    const int16_t barX = 60, barY = 186, barW = TFT_WIDTH - 120, barH = 6;
    sp.fillRoundRect(barX, barY, barW, barH, 3, pal.panel);
    const int16_t fillW = static_cast<int16_t>(clamped * barW);
    if (fillW > 2) {
      sp.fillRoundRect(barX, barY, fillW, barH, 3, pal.accent);
    }
  } else {
    sp.setTextColor(pal.textDim);
    sp.drawString("STANDBY  -  Putar / Klik", TFT_WIDTH / 2, 194);
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
      const int fillW = static_cast<int>(prog * (TFT_WIDTH - 40));
      sp.fillRect(20, TFT_HEIGHT - 10, TFT_WIDTH - 40, 3, pal.grid);
      sp.fillRect(20, TFT_HEIGHT - 10, fillW, 3, pal.accent);
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
// 5. PRESET STORAGE LITTLEFS (8 SLOT)
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
    snprintf(presetNames_[s], sizeof(presetNames_[s]), "Kosong");

    if (LittleFS.exists(path)) {
      File f = LittleFS.open(path, "r");
      if (f) {
        dsp::SystemConfig tmp = {};
        if (f.read(reinterpret_cast<uint8_t *>(&tmp), sizeof(tmp)) == sizeof(tmp) &&
            tmp.magic == dsp::DSP_CONFIG_MAGIC &&
            tmp.version == dsp::DSP_CONFIG_VERSION) {
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
    showToast("PRESET TERSIMPAN");
    return true;
  }
  return false;
}

bool UiTft::loadPresetSlot(uint8_t slot) {
  if (!dsp_ || slot >= NUM_PRESET_SLOTS) return false;
  char path[32];
  snprintf(path, sizeof(path), "%s/slot_%u.bin", DIR_PRESETS, slot);
  if (!LittleFS.exists(path)) {
    showToast("SLOT KOSONG");
    return false;
  }

  File f = LittleFS.open(path, "r");
  if (!f) return false;
  dsp::SystemConfig cfg = {};
  const size_t got = f.read(reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg));
  f.close();

  if (got == sizeof(cfg) && cfg.magic == dsp::DSP_CONFIG_MAGIC &&
      cfg.version == dsp::DSP_CONFIG_VERSION) {
    dsp_->applyConfig(cfg);
    showToast("PRESET DIMUAT");
    return true;
  }
  showToast("PRESET TIDAK VALID");
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
  if (got != sizeof(cfg) || cfg.magic != dsp::DSP_CONFIG_MAGIC ||
      cfg.version != dsp::DSP_CONFIG_VERSION) {
    return false;
  }

  strncpy(cfg.presetName, newName, sizeof(cfg.presetName) - 1);
  cfg.presetName[sizeof(cfg.presetName) - 1] = '\0';

  File fw = LittleFS.open(path, "w");
  if (!fw) return false;
  fw.write(reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg));
  fw.close();

  refreshPresetNames();
  showToast("NAMA DIPERBARUI");
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
  if (ok) showToast("PRESET DIHAPUS");
  return ok;
}

void UiTft::reloadLogos() {
  loadRawImagePsram(PATH_BOOT_LOGO, bootLogoBuf_, bootLogoPixels_);
  loadRawImagePsram(PATH_STBY_LOGO, standbyLogoBuf_, standbyLogoPix_);
}

// ============================================================================
// 6. NAVIGASI & INPUT STATE MACHINE (5 TAB SEDERHANA)
// ============================================================================
bool UiTft::isOnHomeScreen() const {
  return (!inStandby_ && !overheatActive_ && !inDetailPage_ && !masterEditing_);
}

void UiTft::handleInput(const ui_ctrl::InputEvent &ev) {
  const uint32_t nowMs = millis();
  if (ev.anyActivity) {
    lastActivityMs_ = nowMs;
  }

  if (ev.button == ui_ctrl::ButtonEvent::UNLOCK_3S) {
    uiCfg_.panelLocked = false;
    saveUiConfig();
    inStandby_ = false;
    showToast("PANEL TERBUKA");
    return;
  }

  if (ev.button == ui_ctrl::ButtonEvent::LOCKED_BLOCKED || ev.panelLocked) {
    if (ev.anyActivity) {
      inStandby_ = false;
      lockBannerUntilMs_ = nowMs + 1500;
    }
    return;
  }

  if (inStandby_) {
    if (ev.anyActivity) inStandby_ = false;
    return;
  }

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
      case TopMenuId::IN_CH:
        handleInputPageInput(ev);
        break;
      case TopMenuId::MATRIX:
        handleMatrixInput(ev);
        break;
      case TopMenuId::OUT_CH:
        handleOutputInput(ev);
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
  // Mode Kontrol Cepat Master Volume langsung di layar METER
  if (masterEditing_) {
    if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
      masterEditing_ = false;
      return;
    }
    if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
      dsp::SystemConfig cfg = dsp_->getConfig();
      cfg.master.mute = !cfg.master.mute;
      dsp_->applyConfig(cfg);
      showToast(cfg.master.mute ? "MASTER MUTE" : "MASTER AKTIF", 800);
      return;
    }
    if (ev.delta != 0) {
      dsp::SystemConfig cfg = dsp_->getConfig();
      cfg.master.volumeDb = clampf(cfg.master.volumeDb + static_cast<float>(ev.delta) * 0.5f, -60.0f, 0.0f);
      if (cfg.master.mute) cfg.master.mute = false;
      dsp_->applyConfig(cfg);
    }
    return;
  }

  // Putar encoder: pilih 1 dari 5 tab bawah (METER / INPUT / MATRIX / OUT / SETUP)
  if (ev.delta != 0) {
    int idx = static_cast<int>(activeMenu_) + ev.delta;
    while (idx < 0) idx += NUM_TOP_MENU_ITEMS;
    idx %= NUM_TOP_MENU_ITEMS;
    activeMenu_ = static_cast<TopMenuId>(idx);
  }

  // Tahan 2 detik di Home: langsung masuk ke halaman OUT DSP
  if (ev.button == ui_ctrl::ButtonEvent::HOLD_2S) {
    activeMenu_      = TopMenuId::OUT_CH;
    inDetailPage_    = true;
    outTab_          = 0;
    outParamIdx_     = 2; // Output Gain
    outEditingParam_ = true;
    return;
  }

  // Klik singkat: aktifkan Master Control (jika METER) atau buka halaman terpilih
  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    if (activeMenu_ == TopMenuId::METER) {
      masterEditing_ = true;
    } else {
      inDetailPage_     = true;
      inEditing_        = false;
      outParamIdx_      = 0;
      outEditingParam_  = false;
      setupSub_         = SetupSubmenu::NONE;
      subCursor_        = 0;
      subEditing_       = false;
      presetActionMenu_ = false;
      presetRenaming_   = false;
    }
  }
}

void UiTft::handleInputPageInput(const ui_ctrl::InputEvent &ev) {
  // 6 kontrol: [0] IN A Gain, [1] IN A Mute, [2] IN A Phase,
  //            [3] IN B Gain, [4] IN B Mute, [5] IN B Phase
  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (inEditing_) inEditing_ = false;
    else inDetailPage_ = false;
    return;
  }

  const int ch  = (inCursor_ >= 3) ? 1 : 0;
  const int fld = inCursor_ % 3;

  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    if (fld == 0) {
      inEditing_ = !inEditing_;
    } else {
      dsp::SystemConfig cfg = dsp_->getConfig();
      if (fld == 1) cfg.inputs[ch].mute = !cfg.inputs[ch].mute;
      else          cfg.inputs[ch].phaseInvert = !cfg.inputs[ch].phaseInvert;
      dsp_->applyConfig(cfg);
    }
    return;
  }

  if (ev.delta == 0) return;

  if (!inEditing_) {
    inCursor_ = static_cast<uint8_t>((inCursor_ + ev.delta + 6) % 6);
  } else if (fld == 0) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    cfg.inputs[ch].gainDb = clampf(cfg.inputs[ch].gainDb + static_cast<float>(ev.delta) * 0.5f, -12.0f, 12.0f);
    dsp_->applyConfig(cfg);
  }
}

void UiTft::handleMatrixInput(const ui_ctrl::InputEvent &ev) {
  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    inDetailPage_ = false;
    return;
  }
  if (ev.delta != 0) {
    matrixCursor_ = static_cast<uint8_t>((matrixCursor_ + ev.delta + 16) % 16);
  }
  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    const uint8_t o = matrixCursor_ / 4;
    const uint8_t s = matrixCursor_ % 4;
    float g = cfg.matrix.mixGain[o][s];
    if (g < 0.25f)      g = 0.5f;
    else if (g < 0.75f) g = 1.0f;
    else                g = 0.0f;
    cfg.matrix.mixGain[o][s] = g;
    dsp_->applyConfig(cfg);
  }
}

void UiTft::handleOutputInput(const ui_ctrl::InputEvent &ev) {
  // Tab 0 (EQ / XO, 16 items):
  //   [0] Kanal (OUT1..4), [1] Mode (EQ/DYN), [2] Gain, [3] Mute, [4] Phase,
  //   [5] HPF Mode, [6] HPF Freq, [7] LPF Mode, [8] LPF Freq,
  //   [9] PEQ Band, [10] PEQ Freq, [11] PEQ Gain, [12] PEQ Q,
  //   [13] Delay ms, [14] Bypass DSP, [15] Copy -> ALL
  // Tab 1 (DYN, 11 items):
  //   [0] Kanal (OUT1..4), [1] Mode (EQ/DYN),
  //   [2] Gate En, [3] Gate Thr, [4] Gate Atk, [5] Gate Rel,
  //   [6] Lim En,  [7] Lim Thr,  [8] Lim Atk,  [9] Lim Rel, [10] Anti-Clip
  const uint8_t maxParams = (outTab_ == 0) ? 16 : 11;

  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (outEditingParam_) outEditingParam_ = false;
    else inDetailPage_ = false;
    return;
  }

  if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
    if (outParamIdx_ == 0) {
      outIdx_ = static_cast<uint8_t>((outIdx_ + 1) % 4);
      curveCacheHash_ = 0;
      return;
    }
    if (outParamIdx_ == 1) {
      outTab_ ^= 1;
      outParamIdx_ = 1;
      outEditingParam_ = false;
      return;
    }

    dsp::SystemConfig cfg = dsp_->getConfig();
    auto &out = cfg.outputs[outIdx_];
    if (outTab_ == 0) {
      if (outParamIdx_ == 3)  { out.mute = !out.mute; dsp_->applyConfig(cfg); return; }
      if (outParamIdx_ == 4)  { out.phaseInvert = !out.phaseInvert; dsp_->applyConfig(cfg); return; }
      if (outParamIdx_ == 14) { out.bypassDsp = !out.bypassDsp; dsp_->applyConfig(cfg); return; }
      if (outParamIdx_ == 15) {
        dsp_->copyChannelSettings(static_cast<dsp::ChannelId>(2 + outIdx_), dsp::ChannelId::ALL_OUTPUTS);
        showToast("DISALIN KE OUT 1-4");
        return;
      }
    } else {
      if (outParamIdx_ == 2) { out.gate.enabled = !out.gate.enabled; dsp_->applyConfig(cfg); return; }
      if (outParamIdx_ == 6) { out.limiter.enabled = !out.limiter.enabled; dsp_->applyConfig(cfg); return; }
      if (outParamIdx_ == 10) {
        out.antiClipMode = (out.antiClipMode == dsp::AntiClipMode::SOFT_TANH)
            ? dsp::AntiClipMode::HARD_CLIP : dsp::AntiClipMode::SOFT_TANH;
        dsp_->applyConfig(cfg);
        return;
      }
    }

    outEditingParam_ = !outEditingParam_;
    return;
  }

  if (ev.delta == 0) return;

  if (!outEditingParam_) {
    int next = static_cast<int>(outParamIdx_) + ev.delta;
    if (next < 0) next = maxParams - 1;
    if (next >= maxParams) next = 0;
    outParamIdx_ = static_cast<uint8_t>(next);
    return;
  }

  dsp::SystemConfig cfg = dsp_->getConfig();
  const float d  = static_cast<float>(ev.delta);
  const float ad = static_cast<float>(ev.accelDelta);
  auto &out = cfg.outputs[outIdx_];

  if (outTab_ == 0) {
    auto &band = out.peq[outActiveBand_];
    auto cycleXoverMode = [](dsp::XoverFilterConfig &xo, int delta) {
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

    switch (outParamIdx_) {
      case 2: out.gainDb = clampf(out.gainDb + d * 0.5f, -12.0f, 12.0f); break;
      case 5: cycleXoverMode(out.hpf, ev.delta); break;
      case 6: {
        const float step = (out.hpf.freqHz < 200.0f) ? 1.0f : (out.hpf.freqHz < 2000.0f ? 10.0f : 50.0f);
        out.hpf.freqHz = dsp::DspEngine::quantizeFreqHalfHz(out.hpf.freqHz + ad * step);
        break;
      }
      case 7: cycleXoverMode(out.lpf, ev.delta); break;
      case 8: {
        const float step = (out.lpf.freqHz < 200.0f) ? 1.0f : (out.lpf.freqHz < 2000.0f ? 10.0f : 50.0f);
        out.lpf.freqHz = dsp::DspEngine::quantizeFreqHalfHz(out.lpf.freqHz + ad * step);
        break;
      }
      case 9: outActiveBand_ = static_cast<uint8_t>((outActiveBand_ + ev.delta + 9) % 9); break;
      case 10: {
        const float step = (band.freqHz < 200.0f) ? 0.5f : (band.freqHz < 2000.0f ? 5.0f : 25.0f);
        band.freqHz = dsp::DspEngine::quantizeFreqHalfHz(band.freqHz + ad * step);
        break;
      }
      case 11: band.gainDb = clampf(band.gainDb + d * 0.5f, -15.0f, 15.0f); break;
      case 12: band.q = clampf(band.q + d * 0.1f, 0.4f, 12.0f); break;
      case 13: out.delayMs = clampf(out.delayMs + ad * 0.5f, 0.0f, dsp::DSP_MAX_DELAY_MS); break;
      default: break;
    }
  } else {
    switch (outParamIdx_) {
      case 3: out.gate.thresholdDb    = clampf(out.gate.thresholdDb + d, -90.0f, 0.0f); break;
      case 4: out.gate.attackMs       = clampf(out.gate.attackMs + d * 0.5f, 0.1f, 100.0f); break;
      case 5: out.gate.releaseMs      = clampf(out.gate.releaseMs + ad * 10.0f, 10.0f, 2000.0f); break;
      case 7: out.limiter.thresholdDb = clampf(out.limiter.thresholdDb + d * 0.5f, -30.0f, 0.0f); break;
      case 8: out.limiter.attackMs    = clampf(out.limiter.attackMs + d * 0.2f, 0.05f, 50.0f); break;
      case 9: out.limiter.releaseMs   = clampf(out.limiter.releaseMs + ad * 10.0f, 10.0f, 2000.0f); break;
      default: break;
    }
  }

  dsp_->applyConfig(cfg);
}

void UiTft::handlePresetInput(const ui_ctrl::InputEvent &ev) {
  if (presetRenaming_) {
    if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
      presetRenaming_   = false;
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
      const char cur  = renameBuf_[renameCharPos_];
      const char *pos = strchr(RENAME_CHARSET, cur);
      int idx = pos ? static_cast<int>(pos - RENAME_CHARSET) : 0;
      idx = (idx + ev.delta + static_cast<int>(RENAME_CHARSET_LEN)) % static_cast<int>(RENAME_CHARSET_LEN);
      renameBuf_[renameCharPos_] = RENAME_CHARSET[idx];
    }
    return;
  }

  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (presetActionMenu_) presetActionMenu_ = false;
    else setupSub_ = SetupSubmenu::NONE;
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

  if (ev.button == ui_ctrl::ButtonEvent::SHORT_HOLD) {
    if (subEditing_) subEditing_ = false;
    else setupSub_ = SetupSubmenu::NONE;
    return;
  }

  switch (setupSub_) {
    case SetupSubmenu::KONEKSI: {
      // [0] Mode WiFi (OFF / AU-WEB / AU-OTA), [1] Bluetooth BLE (Eksklusif)
      if (ev.delta != 0) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 2) % 2);
      }
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          uiCfg_.wifiMode = static_cast<WifiMode>((static_cast<uint8_t>(uiCfg_.wifiMode) + 1) % 3);
          if (uiCfg_.wifiMode != WifiMode::OFF) uiCfg_.bleEnabled = false;
          applyUiConfig(uiCfg_, true);
          showToast(uiCfg_.wifiMode == WifiMode::AU_WEB ? "WIFI AU-WEB AKTIF"
                    : (uiCfg_.wifiMode == WifiMode::AU_OTA ? "WIFI AU-OTA AKTIF" : "WIFI OFF"));
        } else {
          uiCfg_.bleEnabled = !uiCfg_.bleEnabled;
          if (uiCfg_.bleEnabled) uiCfg_.wifiMode = WifiMode::OFF;
          applyUiConfig(uiCfg_, true);
          showToast(uiCfg_.bleEnabled ? "BLUETOOTH AKTIF" : "BLUETOOTH OFF");
        }
      }
      break;
    }

    case SetupSubmenu::TAMPILAN: {
      // [0] Tema Warna (7), [1] Kalibrasi Panel (4), [2] Auto Standby (ON/OFF), [3] Waktu Standby
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 2) {
          uiCfg_.standbyEnabled = !uiCfg_.standbyEnabled;
          applyUiConfig(uiCfg_, true);
        } else {
          subEditing_ = !subEditing_;
        }
        break;
      }
      if (ev.delta == 0) break;
      if (!subEditing_) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 4) % 4);
      } else {
        if (subCursor_ == 0) {
          uiCfg_.theme = static_cast<UiTheme>((static_cast<int>(uiCfg_.theme) + ev.delta + 7) % 7);
        } else if (subCursor_ == 1) {
          uiCfg_.panelMode = static_cast<PanelColorMode>((static_cast<int>(uiCfg_.panelMode) + ev.delta + 4) % 4);
        } else if (subCursor_ == 3) {
          int t = static_cast<int>(uiCfg_.standbyTimeoutSec) + ev.accelDelta * 5;
          uiCfg_.standbyTimeoutSec = static_cast<uint16_t>(constrain(t, 5, 600));
        }
        applyUiConfig(uiCfg_, true);
      }
      break;
    }

    case SetupSubmenu::SISTEM: {
      // [0] Kunci Panel, [1] Reset Pabrik DSP
      if (ev.delta != 0) {
        subCursor_ = static_cast<uint8_t>((subCursor_ + ev.delta + 2) % 2);
      }
      if (ev.button == ui_ctrl::ButtonEvent::CLICK) {
        if (subCursor_ == 0) {
          uiCfg_.panelLocked = true;
          applyUiConfig(uiCfg_, true);
          showToast("PANEL DIKUNCI (3s)");
        } else {
          dsp::SystemConfig defCfg = {};
          dsp::DspEngine::loadDefaultConfig(defCfg);
          dsp_->applyConfig(defCfg);
          showToast("RESET PABRIK OK");
        }
      }
      break;
    }

    default:
      break;
  }
}

// ============================================================================
// 7. EVALUASI KURVA FREKUENSI OUTPUT (XOVER + 9-BAND PEQ)
// ============================================================================
float UiTft::evalChannelMagDb(int outIdx, float freqHz,
                              const dsp::SystemConfig &cfg) const {
  const double fs = static_cast<double>(AUDIO_SAMPLE_RATE);
  double magSq = 1.0;

  const auto &out = cfg.outputs[outIdx];
  if (!out.bypassDsp) {
    magSq *= evalXoverMagSq(out.hpf, true,  freqHz, fs);
    magSq *= evalXoverMagSq(out.lpf, false, freqHz, fs);
    for (uint8_t b = 0; b < dsp::DSP_PEQ_BANDS; b++) {
      magSq *= evalPeqMagSq(out.peq[b], freqHz, fs);
    }
  }

  if (magSq < 1.0e-12) return -60.0f;
  return static_cast<float>(10.0 * log10(magSq));
}

// ============================================================================
// 8. RENDERER MINIMALIS STUDIO
// ============================================================================
void UiTft::drawHeader(LGFX_Sprite &sp, const ThemePalette &pal,
                       const audio_io::AudioTelemetry &audioTel,
                       const char *titleOverride) {
  sp.fillRect(0, 0, TFT_WIDTH, HEADER_HEIGHT, pal.headerBg);
  sp.drawFastHLine(0, HEADER_HEIGHT - 1, TFT_WIDTH, pal.accent);

  sp.setTextSize(1);
  sp.setTextDatum(middle_left);

  // Kiri: Titik aksen + Judul / Breadcrumb aktif
  sp.fillCircle(10, 12, 3, pal.accent);
  sp.setTextColor(pal.text);
  sp.drawString(titleOverride ? titleOverride : "DSP Audio Interface", 18, 12);

  // Kanan: Indikator status ringkas tanpa kotak berat (Lock, USB, Radio, Suhu)
  if (uiCfg_.panelLocked) {
    sp.fillRoundRect(184, 9, 8, 7, 1, pal.warn);
    sp.drawCircle(188, 8, 2, pal.warn);
  }

  // USB
  const bool usbLive = audioTel.usbMounted && (audioTel.usbRxStreaming || audioTel.usbTxStreaming);
  const uint16_t usbDot = usbLive ? pal.accent : (audioTel.usbMounted ? pal.ok : pal.grid);
  sp.fillCircle(202, 12, 3, usbDot);
  sp.setTextColor(audioTel.usbMounted ? pal.text : pal.textDim);
  sp.drawString("USB", 208, 12);

  // Radio (BLE / WEB / OTA / OFF)
  const char *netTxt = (uiCfg_.wifiMode == WifiMode::AU_WEB) ? "WEB"
                     : (uiCfg_.wifiMode == WifiMode::AU_OTA ? "OTA"
                                                            : (uiCfg_.bleEnabled ? "BLE" : "OFF"));
  const bool radioOn = (uiCfg_.wifiMode != WifiMode::OFF) || uiCfg_.bleEnabled;
  const uint16_t netCol = (uiCfg_.wifiMode == WifiMode::AU_OTA) ? pal.warn
                        : (uiCfg_.wifiMode == WifiMode::AU_WEB ? pal.ok
                        : (uiCfg_.bleEnabled ? rgb565(90, 190, 255) : pal.grid));
  sp.fillCircle(240, 12, 3, netCol);
  sp.setTextColor(radioOn ? pal.text : pal.textDim);
  sp.drawString(netTxt, 246, 12);

  // Suhu
  const float tc = audioTel.chipTempC;
  const uint16_t tCol = (tc >= TEMP_WARN_THRESHOLD_C) ? pal.danger
                      : (tc >= 65.0f ? pal.warn : pal.textDim);
  char tempBuf[10];
  snprintf(tempBuf, sizeof(tempBuf), "%.0fC", tc);
  sp.setTextDatum(middle_right);
  sp.setTextColor(tCol);
  sp.drawString(tempBuf, TFT_WIDTH - 8, 12);
}

void UiTft::drawBottomTabBar(LGFX_Sprite &sp, const ThemePalette &pal) {
  constexpr int16_t BAR_Y  = 212;
  constexpr int16_t BAR_H  = TFT_HEIGHT - BAR_Y;
  constexpr int16_t TAB_W  = 58;
  constexpr int16_t GAP    = 4;
  constexpr int16_t STARTX = 7;

  sp.fillRect(0, BAR_Y, TFT_WIDTH, BAR_H, pal.headerBg);
  sp.drawFastHLine(0, BAR_Y, TFT_WIDTH, pal.grid);

  sp.setTextSize(1);
  sp.setTextDatum(middle_center);

  for (uint8_t i = 0; i < NUM_TOP_MENU_ITEMS; i++) {
    const int16_t x = STARTX + i * (TAB_W + GAP);
    const bool sel  = (i == static_cast<uint8_t>(activeMenu_));
    if (sel) {
      sp.fillRect(x + 8, BAR_Y, TAB_W - 16, 2, pal.accent2);
      sp.fillRoundRect(x, BAR_Y + 4, TAB_W, 20, 5, pal.accent);
      sp.setTextColor(pal.bg);
    } else {
      sp.setTextColor(pal.textDim);
    }
    sp.drawString(TOP_MENU_LABELS[i], x + TAB_W / 2, BAR_Y + 14);
  }
}

void UiTft::drawFooterHint(LGFX_Sprite &sp, const ThemePalette &pal, const char *hint) {
  sp.fillRect(0, TFT_HEIGHT - 15, TFT_WIDTH, 15, pal.headerBg);
  sp.drawFastHLine(0, TFT_HEIGHT - 15, TFT_WIDTH, pal.grid);
  sp.setTextSize(1);
  sp.setTextDatum(middle_center);
  sp.setTextColor(pal.textDim);
  sp.drawString(hint, TFT_WIDTH / 2, TFT_HEIGHT - 7);
}

void UiTft::drawLiveVuMeterPage(LGFX_Sprite &sp, const ThemePalette &pal,
                                const dsp::DspMeters &meters,
                                const dsp::SystemConfig &cfg) {
  const uint32_t nowMs = millis();

  // Baris ringkas Preset & Master Volume di atas VU Meter (y = 28..48)
  sp.setTextSize(1);
  char pShort[18];
  strncpy(pShort, cfg.presetName, sizeof(pShort) - 1);
  pShort[sizeof(pShort) - 1] = '\0';
  sp.setTextDatum(middle_left);
  sp.setTextColor(pal.textDim);
  sp.drawString(pShort, 10, 38);

  // Pill Master Volume / Mute (kanan atas)
  const int16_t mx = 166, my = 29, mw = 146, mh = 18;
  if (cfg.master.mute) {
    sp.fillRoundRect(mx, my, mw, mh, 4, pal.danger);
    if (masterEditing_) sp.drawRoundRect(mx, my, mw, mh, 4, TFT_WHITE);
    sp.setTextDatum(middle_center);
    sp.setTextColor(TFT_WHITE);
    sp.drawString("MASTER MUTE", mx + mw / 2, my + 9);
  } else {
    sp.fillRoundRect(mx, my, mw, mh, 4, pal.panel);
    sp.drawRoundRect(mx, my, mw, mh, 4, masterEditing_ ? pal.accent : pal.grid);

    // Bar tipis level volume di dalam pill
    const float vNorm = clampf((cfg.master.volumeDb + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t vw  = static_cast<int16_t>(vNorm * (mw - 10));
    sp.fillRect(mx + 5, my + mh - 4, mw - 10, 2, pal.bg);
    if (vw > 0) sp.fillRect(mx + 5, my + mh - 4, vw, 2, pal.accent);

    char mTxt[24];
    if (masterEditing_) snprintf(mTxt, sizeof(mTxt), "< MASTER %+.1fdB >", cfg.master.volumeDb);
    else                snprintf(mTxt, sizeof(mTxt), "MASTER  %+.1fdB", cfg.master.volumeDb);
    sp.setTextDatum(middle_center);
    sp.setTextColor(masterEditing_ ? pal.accent : pal.text);
    sp.drawString(mTxt, mx + mw / 2, my + 7);
  }

  // 6-Channel Studio VU Bridge (y = 54..206)
  constexpr int16_t GRP_Y   = 53;
  constexpr int16_t BADGE_Y = 63;
  constexpr int16_t BAR_TOP = 78;
  constexpr int16_t BAR_H   = 112;
  constexpr int16_t BAR_W   = 36;
  static constexpr const char *CH_LABELS[6] = {"IN A", "IN B", "OUT1", "OUT2", "OUT3", "OUT4"};
  static constexpr int16_t CH_X[6] = {28, 70, 124, 170, 216, 262};

  // Label grup INPUT & OUTPUT + garis pemisah tipis
  sp.setTextDatum(middle_center);
  sp.setTextColor(pal.accent2);
  sp.drawString("INPUT", 67, GRP_Y);
  sp.setTextColor(pal.accent);
  sp.drawString("OUTPUT", 211, GRP_Y);
  sp.drawFastVLine(114, GRP_Y - 2, BAR_TOP + BAR_H - GRP_Y + 18, pal.grid);

  // Skala dB kiri
  sp.setTextDatum(middle_right);
  sp.setTextColor(pal.textDim);
  const float scaleMarks[4] = {0.0f, -12.0f, -24.0f, -48.0f};
  for (int m = 0; m < 4; m++) {
    const float norm = clampf((scaleMarks[m] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t y  = BAR_TOP + BAR_H - static_cast<int16_t>(norm * BAR_H);
    char lbl[6];
    snprintf(lbl, sizeof(lbl), "%.0f", scaleMarks[m]);
    sp.drawString(lbl, 23, y);
    sp.drawFastHLine(26, y, TFT_WIDTH - 34, pal.grid);
  }

  for (int ch = 0; ch < 6; ch++) {
    const bool  isOut = (ch >= 2);
    const float rawDb = isOut ? meters.outPeakDb[ch - 2] : meters.inPeakDb[ch];
    const bool  clip  = isOut ? meters.outClip[ch - 2]   : meters.inClip[ch];
    const bool  lim   = isOut ? meters.outLimiterActive[ch - 2] : false;
    const bool  muted = isOut ? (cfg.outputs[ch - 2].mute || cfg.master.mute) : cfg.inputs[ch].mute;

    if (rawDb > vuSmoothDb_[ch]) vuSmoothDb_[ch] = rawDb;
    else vuSmoothDb_[ch] = fmaxf(-60.0f, vuSmoothDb_[ch] - 1.4f);

    if (rawDb >= vuPeakHoldDb_[ch]) {
      vuPeakHoldDb_[ch] = rawDb;
      vuPeakHoldMs_[ch] = nowMs;
    } else if (nowMs - vuPeakHoldMs_[ch] > 1100) {
      vuPeakHoldDb_[ch] = fmaxf(-60.0f, vuPeakHoldDb_[ch] - 0.6f);
    }
    if (clip) vuClipHoldMs_[ch] = nowMs + 1400;

    const int16_t x = CH_X[ch];

    // Badge status di atas bar
    sp.setTextDatum(middle_center);
    if (nowMs < vuClipHoldMs_[ch]) {
      sp.fillRoundRect(x, BADGE_Y, BAR_W, 12, 2, pal.danger);
      sp.setTextColor(TFT_WHITE);
      sp.drawString("CLIP", x + BAR_W / 2, BADGE_Y + 6);
    } else if (lim) {
      sp.fillRoundRect(x, BADGE_Y, BAR_W, 12, 2, pal.warn);
      sp.setTextColor(TFT_BLACK);
      sp.drawString("LIM", x + BAR_W / 2, BADGE_Y + 6);
    } else if (muted) {
      sp.drawRoundRect(x, BADGE_Y, BAR_W, 12, 2, pal.danger);
      sp.setTextColor(pal.danger);
      sp.drawString("MUTE", x + BAR_W / 2, BADGE_Y + 6);
    } else {
      char dbStr[8];
      snprintf(dbStr, sizeof(dbStr), "%.0f", clampf(vuPeakHoldDb_[ch], -60.0f, 0.0f));
      sp.setTextColor(pal.textDim);
      sp.drawString(dbStr, x + BAR_W / 2, BADGE_Y + 6);
    }

    // Kotak meter + segmen LED halus
    sp.fillRoundRect(x, BAR_TOP, BAR_W, BAR_H, 3, pal.panel);
    sp.drawRoundRect(x, BAR_TOP, BAR_W, BAR_H, 3, pal.grid);

    const float fillNorm = clampf((vuSmoothDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t fillPx = static_cast<int16_t>(fillNorm * (BAR_H - 6));
    for (int16_t seg = 0; seg < (BAR_H - 6); seg += 4) {
      const int16_t segY = BAR_TOP + BAR_H - 3 - seg - 3;
      if (segY < BAR_TOP + 2) break;
      if (seg < fillPx) {
        const float r = static_cast<float>(seg) / static_cast<float>(BAR_H - 6);
        const uint16_t col = (r > 0.88f) ? pal.danger : (r > 0.68f ? pal.warn : pal.ok);
        sp.fillRect(x + 3, segY, BAR_W - 6, 3, muted ? pal.grid : col);
      } else {
        sp.fillRect(x + 3, segY, BAR_W - 6, 3, pal.bg);
      }
    }

    if (vuPeakHoldDb_[ch] > -59.0f && !muted) {
      const float pkNorm = clampf((vuPeakHoldDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
      const int16_t pkY  = BAR_TOP + BAR_H - 3 - static_cast<int16_t>(pkNorm * (BAR_H - 6));
      sp.fillRect(x + 3, pkY, BAR_W - 6, 2, TFT_WHITE);
    }

    sp.setTextColor(isOut ? pal.accent : pal.accent2);
    sp.drawString(CH_LABELS[ch], x + BAR_W / 2, BAR_TOP + BAR_H + 10);
  }

  // Bawah: Bar Tab 5-Menu atau Bar Kontrol Master saat edit
  if (masterEditing_) {
    sp.fillRect(0, 212, TFT_WIDTH, 28, pal.headerBg);
    sp.drawFastHLine(0, 212, TFT_WIDTH, pal.accent);
    sp.setTextDatum(middle_center);
    sp.setTextColor(pal.accent);
    sp.drawString("Putar: Volume  |  Klik: Mute  |  Tahan: Selesai", TFT_WIDTH / 2, 226);
  } else {
    drawBottomTabBar(sp, pal);
  }
}

void UiTft::drawUnifiedInputPage(LGFX_Sprite &sp, const ThemePalette &pal,
                                 const dsp::DspMeters &meters,
                                 const dsp::SystemConfig &cfg) {
  const uint32_t nowMs = millis();

  for (int ch = 0; ch < 2; ch++) {
    const auto &in    = cfg.inputs[ch];
    const int16_t cy  = 32 + ch * 94;
    const int16_t chH = 86;

    sp.fillRoundRect(8, cy, TFT_WIDTH - 16, chH, 6, pal.panel);
    sp.drawRoundRect(8, cy, TFT_WIDTH - 16, chH, 6, meters.inClip[ch] ? pal.danger : pal.grid);

    // Baris atas kartu: Badge kanal + nilai dB live
    sp.setTextSize(1);
    sp.fillRoundRect(16, cy + 8, 42, 16, 4, pal.headerBg);
    sp.setTextDatum(middle_center);
    sp.setTextColor(pal.accent2);
    sp.drawString(ch == 0 ? "IN A" : "IN B", 37, cy + 16);

    sp.setTextDatum(middle_left);
    sp.setTextColor(pal.textDim);
    sp.drawString("DIRECT INPUT", 66, cy + 16);

    const float rawDb = meters.inPeakDb[ch];
    if (rawDb > vuSmoothDb_[ch]) vuSmoothDb_[ch] = rawDb;
    else vuSmoothDb_[ch] = fmaxf(-60.0f, vuSmoothDb_[ch] - 1.4f);
    if (rawDb >= vuPeakHoldDb_[ch]) { vuPeakHoldDb_[ch] = rawDb; vuPeakHoldMs_[ch] = nowMs; }
    else if (nowMs - vuPeakHoldMs_[ch] > 1100) { vuPeakHoldDb_[ch] = fmaxf(-60.0f, vuPeakHoldDb_[ch] - 0.6f); }

    sp.setTextDatum(middle_right);
    if (meters.inClip[ch]) {
      sp.setTextColor(pal.danger);
      sp.drawString("CLIP!", TFT_WIDTH - 16, cy + 16);
    } else if (in.mute) {
      sp.setTextColor(pal.danger);
      sp.drawString("MUTED", TFT_WIDTH - 16, cy + 16);
    } else {
      char dbBuf[14];
      snprintf(dbBuf, sizeof(dbBuf), "%.1f dB", clampf(vuSmoothDb_[ch], -60.0f, 0.0f));
      sp.setTextColor(pal.text);
      sp.drawString(dbBuf, TFT_WIDTH - 16, cy + 16);
    }

    // Live Horizontal VU Bar
    const int16_t vx = 16, vy = cy + 30, vw = TFT_WIDTH - 32, vh = 10;
    sp.fillRoundRect(vx, vy, vw, vh, 3, pal.bg);
    const float norm = clampf((vuSmoothDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
    const int16_t fillPx = static_cast<int16_t>(norm * (vw - 4));
    for (int16_t seg = 0; seg < fillPx; seg += 5) {
      const float r = static_cast<float>(seg) / static_cast<float>(vw - 4);
      const uint16_t col = (r > 0.88f) ? pal.danger : (r > 0.68f ? pal.warn : pal.ok);
      sp.fillRect(vx + 2 + seg, vy + 2, 3, vh - 4, in.mute ? pal.grid : col);
    }
    if (vuPeakHoldDb_[ch] > -59.0f && !in.mute) {
      const float pkNorm = clampf((vuPeakHoldDb_[ch] + 60.0f) / 60.0f, 0.0f, 1.0f);
      const int16_t pkX  = vx + 2 + static_cast<int16_t>(pkNorm * (vw - 6));
      sp.fillRect(pkX, vy + 1, 2, vh - 2, TFT_WHITE);
    }

    // 3 Tombol Kontrol Taktil: GAIN | MUTE | PHASE
    constexpr int16_t BTN_W = 90;
    constexpr int16_t BTN_H = 30;
    constexpr int16_t GAP   = 9;
    const int16_t by = cy + 48;

    for (int f = 0; f < 3; f++) {
      const int16_t bx = 16 + f * (BTN_W + GAP);
      const bool sel   = (inCursor_ == ch * 3 + f);
      const bool edit  = (sel && f == 0 && inEditing_);

      uint16_t fillCol = pal.bg;
      uint16_t bdrCol  = sel ? pal.accent : pal.grid;
      uint16_t txtCol  = pal.text;

      if (edit) {
        fillCol = pal.accent; bdrCol = TFT_WHITE; txtCol = pal.bg;
      } else if (f == 1 && in.mute) {
        fillCol = pal.danger; txtCol = TFT_WHITE;
      } else if (f == 2 && in.phaseInvert) {
        fillCol = pal.warn; txtCol = TFT_BLACK;
      } else if (sel) {
        fillCol = pal.headerBg;
      }

      sp.fillRoundRect(bx, by, BTN_W, BTN_H, 5, fillCol);
      sp.drawRoundRect(bx, by, BTN_W, BTN_H, 5, bdrCol);

      sp.setTextDatum(middle_center);
      sp.setTextSize(1);
      if (f == 0) {
        sp.setTextColor(edit ? pal.bg : pal.textDim);
        sp.drawString("GAIN", bx + BTN_W / 2, by + 9);
        char gStr[16];
        if (edit) snprintf(gStr, sizeof(gStr), "<%+.1fdB>", in.gainDb);
        else      snprintf(gStr, sizeof(gStr), "%+.1f dB", in.gainDb);
        sp.setTextColor(txtCol);
        sp.drawString(gStr, bx + BTN_W / 2, by + 21);
      } else if (f == 1) {
        sp.setTextColor(in.mute ? TFT_WHITE : pal.textDim);
        sp.drawString("MUTE", bx + BTN_W / 2, by + 9);
        sp.setTextColor(in.mute ? TFT_WHITE : pal.ok);
        sp.drawString(in.mute ? "MUTED" : "AKTIF", bx + BTN_W / 2, by + 21);
      } else {
        sp.setTextColor(in.phaseInvert ? TFT_BLACK : pal.textDim);
        sp.drawString("FASE", bx + BTN_W / 2, by + 9);
        sp.setTextColor(txtCol);
        sp.drawString(in.phaseInvert ? "180 deg" : "0 deg", bx + BTN_W / 2, by + 21);
      }
    }
  }

  drawFooterHint(sp, pal, inEditing_
      ? "Putar: Ubah Gain  |  Klik: Selesai"
      : "Putar: Pilih  |  Klik: Ubah  |  Tahan: Kembali");
}

void UiTft::drawMatrixPage(LGFX_Sprite &sp, const ThemePalette &pal,
                           const dsp::SystemConfig &cfg) {
  const char *srcNames[4] = {"IN A", "IN B", "USB L", "USB R"};
  const char *outNames[4] = {"OUT 1", "OUT 2", "OUT 3", "OUT 4"};

  sp.setTextSize(1);
  sp.setTextDatum(middle_center);
  for (int s = 0; s < 4; s++) {
    sp.setTextColor(s < 2 ? pal.accent2 : pal.accent);
    sp.drawString(srcNames[s], 88 + s * 62, 36);
  }

  for (int o = 0; o < 4; o++) {
    const int16_t ry = 49 + o * 41;
    sp.setTextDatum(middle_left);
    sp.setTextColor(pal.text);
    sp.drawString(outNames[o], 10, ry + 15);

    for (int s = 0; s < 4; s++) {
      const int16_t cx = 60 + s * 62;
      const bool sel   = (matrixCursor_ == o * 4 + s);
      const float g    = cfg.matrix.mixGain[o][s];
      const bool isOn  = (g > 0.75f);
      const bool isMid = (!isOn && g > 0.25f);

      const uint16_t fill = isOn ? pal.ok : (isMid ? pal.warn : pal.panel);
      sp.fillRoundRect(cx, ry, 56, 31, 6, fill);
      sp.drawRoundRect(cx, ry, 56, 31, 6, sel ? TFT_WHITE : pal.grid);
      if (sel) sp.drawRoundRect(cx - 1, ry - 1, 58, 33, 7, pal.accent);

      sp.setTextDatum(middle_center);
      sp.setTextColor((isOn || isMid) ? TFT_BLACK : pal.textDim);
      sp.drawString(isOn ? "100%" : (isMid ? "50%" : "-"), cx + 28, ry + 15);
    }
  }

  drawFooterHint(sp, pal, "Putar: Pilih  |  Klik: OFF / 50% / 100%  |  Tahan: Kembali");
}

void UiTft::drawOutputFreqCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                                int outIdx, const dsp::SystemConfig &cfg) {
  constexpr int16_t GX = 26, GY = 50, GW = 286, GH = 100;
  const int16_t midY = GY + GH / 2;

  sp.fillRoundRect(GX, GY, GW, GH, 4, pal.panel);
  sp.drawRoundRect(GX, GY, GW, GH, 4, pal.grid);

  // Grid horizontal dB (+18 / 0 / -18)
  sp.drawFastHLine(GX + 1, midY, GW - 2, pal.textDim);
  sp.drawFastHLine(GX + 1, GY + GH / 4, GW - 2, pal.grid);
  sp.drawFastHLine(GX + 1, GY + (GH * 3) / 4, GW - 2, pal.grid);

  const float logMin  = log10f(20.0f);
  const float logMax  = log10f(20000.0f);
  const float logSpan = logMax - logMin;

  // Grid vertikal frekuensi
  const float fMarks[3] = {100.0f, 1000.0f, 10000.0f};
  const char *fLbls[3]  = {"100", "1k", "10k"};
  sp.setTextSize(1);
  sp.setTextDatum(top_center);
  sp.setTextColor(pal.textDim);
  for (int i = 0; i < 3; i++) {
    const int16_t x = GX + static_cast<int16_t>(((log10f(fMarks[i]) - logMin) / logSpan) * GW);
    sp.drawFastVLine(x, GY + 1, GH - 2, pal.grid);
    sp.drawString(fLbls[i], x, GY + GH + 2);
  }

  sp.setTextDatum(middle_right);
  sp.drawString("+18", GX - 3, GY + 6);
  sp.drawString("0",   GX - 3, midY);
  sp.drawString("-18", GX - 3, GY + GH - 6);

  // Hash cache kurva
  uint32_t h = static_cast<uint32_t>(100 + outIdx) ^ 0x811C9DC5u;
  const uint8_t *rawPtr = reinterpret_cast<const uint8_t *>(&cfg.outputs[outIdx]);
  for (size_t k = 0; k < sizeof(dsp::OutputChannelConfig); k++) {
    h ^= rawPtr[k];
    h *= 16777619u;
  }

  constexpr int SAMPLES_X = CURVE_POINTS - 1;
  if (h != curveCacheHash_) {
    for (int i = 0; i <= SAMPLES_X; i++) {
      const float t = static_cast<float>(i) / static_cast<float>(SAMPLES_X);
      const float f = powf(10.0f, logMin + t * logSpan);
      curveCacheDb_[i] = clampf(evalChannelMagDb(outIdx, f, cfg), -24.0f, 18.0f);
    }
    curveCacheHash_ = h;
  }

  // Shading halus antara garis 0 dB dan kurva (gaya studio EQ modern)
  for (int i = 0; i <= SAMPLES_X; i++) {
    const float t  = static_cast<float>(i) / static_cast<float>(SAMPLES_X);
    const int16_t px = GX + 2 + static_cast<int16_t>(t * (GW - 4));
    const int16_t py = constrain(midY - static_cast<int16_t>((curveCacheDb_[i] / 18.0f) * (GH * 0.45f)),
                                 GY + 2, GY + GH - 2);
    if (py < midY) sp.drawFastVLine(px, py, midY - py, pal.curveFill);
    else if (py > midY) sp.drawFastVLine(px, midY + 1, py - midY, pal.curveFill);
  }

  // Garis penanda cutoff HPF / LPF jika aktif
  const auto &out = cfg.outputs[outIdx];
  if (!out.bypassDsp && out.hpf.enabled) {
    const float ht = clampf((log10f(clampf(out.hpf.freqHz, 20.0f, 20000.0f)) - logMin) / logSpan, 0.0f, 1.0f);
    const int16_t hx = GX + 2 + static_cast<int16_t>(ht * (GW - 4));
    for (int16_t y = GY + 3; y < GY + GH - 3; y += 6) sp.drawFastVLine(hx, y, 3, pal.ok);
  }
  if (!out.bypassDsp && out.lpf.enabled) {
    const float lt = clampf((log10f(clampf(out.lpf.freqHz, 20.0f, 20000.0f)) - logMin) / logSpan, 0.0f, 1.0f);
    const int16_t lx = GX + 2 + static_cast<int16_t>(lt * (GW - 4));
    for (int16_t y = GY + 3; y < GY + GH - 3; y += 6) sp.drawFastVLine(lx, y, 3, pal.warn);
  }

  // Kurva respon 2px tajam
  int16_t prevX = GX + 2, prevY = midY;
  for (int i = 0; i <= SAMPLES_X; i++) {
    const float t  = static_cast<float>(i) / static_cast<float>(SAMPLES_X);
    const int16_t px = GX + 2 + static_cast<int16_t>(t * (GW - 4));
    const int16_t cy = constrain(midY - static_cast<int16_t>((curveCacheDb_[i] / 18.0f) * (GH * 0.45f)),
                                 GY + 2, GY + GH - 2);
    if (i > 0) {
      sp.drawLine(prevX, prevY, px, cy, pal.curve);
      sp.drawLine(prevX, prevY + 1, px, cy + 1, pal.curve);
    }
    prevX = px;
    prevY = cy;
  }

  // Titik band PEQ aktif + sorotan band terpilih
  for (uint8_t b = 0; b < dsp::DSP_PEQ_BANDS; b++) {
    const auto &pb = out.peq[b];
    if (!pb.enabled && b != outActiveBand_) continue;
    const float bt  = clampf((log10f(clampf(pb.freqHz, 20.0f, 20000.0f)) - logMin) / logSpan, 0.0f, 1.0f);
    const float bdb = clampf(evalChannelMagDb(outIdx, pb.freqHz, cfg), -24.0f, 18.0f);
    const int16_t bx = GX + 2 + static_cast<int16_t>(bt * (GW - 4));
    const int16_t by = constrain(midY - static_cast<int16_t>((bdb / 18.0f) * (GH * 0.45f)), GY + 5, GY + GH - 5);
    if (b == outActiveBand_) {
      sp.fillCircle(bx, by, 4, pal.accent);
      sp.drawCircle(bx, by, 6, TFT_WHITE);
    } else if (fabsf(pb.gainDb) >= 0.1f) {
      sp.fillCircle(bx, by, 2, pal.accent2);
    }
  }

  // Badge ringan di pojok kanan atas plot jika BYPASS / MUTE
  if (out.mute || out.bypassDsp) {
    sp.fillRoundRect(GX + GW - 52, GY + 4, 48, 13, 3, out.mute ? pal.danger : pal.warn);
    sp.setTextDatum(middle_center);
    sp.setTextColor(out.mute ? TFT_WHITE : TFT_BLACK);
    sp.drawString(out.mute ? "MUTED" : "BYPASS", GX + GW - 28, GY + 11);
  }
}

void UiTft::drawOutputDynamicsCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                                    int outIdx, const dsp::DspMeters &meters,
                                    const dsp::SystemConfig &cfg) {
  constexpr int16_t GX = 24, GY = 52, GS = 100;

  sp.fillRoundRect(GX, GY, GS, GS, 4, pal.panel);
  sp.drawRoundRect(GX, GY, GS, GS, 4, pal.grid);
  sp.drawLine(GX + 2, GY + GS - 2, GX + GS - 2, GY + 2, pal.grid);

  const auto &out = cfg.outputs[outIdx];
  auto evalTransfer = [&](float inDb) -> float {
    float yDb = inDb;
    if (out.gate.enabled && inDb < out.gate.thresholdDb) {
      yDb = fmaxf(-60.0f, inDb + out.gate.rangeDb * 0.5f);
    }
    if (out.limiter.enabled && yDb > out.limiter.thresholdDb) {
      yDb = out.limiter.thresholdDb;
    }
    return clampf(yDb, -60.0f, 0.0f);
  };

  int16_t px = GX + 2, py = GY + GS - 2;
  for (int i = 0; i <= 40; i++) {
    const float inDb  = -60.0f + (static_cast<float>(i) / 40.0f) * 60.0f;
    const float outDb = evalTransfer(inDb);
    const int16_t x   = GX + 2 + static_cast<int16_t>(((inDb + 60.0f) / 60.0f) * (GS - 4));
    const int16_t y   = GY + GS - 2 - static_cast<int16_t>(((outDb + 60.0f) / 60.0f) * (GS - 4));
    if (i > 0) {
      sp.drawLine(px, py, x, y, pal.curve);
      sp.drawLine(px, py + 1, x, y + 1, pal.curve);
    }
    px = x; py = y;
  }

  const float liveIn  = clampf(meters.outPeakDb[outIdx], -60.0f, 0.0f);
  const float liveOut = evalTransfer(liveIn);
  const int16_t lx = GX + 2 + static_cast<int16_t>(((liveIn + 60.0f) / 60.0f) * (GS - 4));
  const int16_t ly = GY + GS - 2 - static_cast<int16_t>(((liveOut + 60.0f) / 60.0f) * (GS - 4));
  sp.fillCircle(lx, ly, 4, pal.accent);

  // Meter GR vertikal tipis
  const int16_t grX = GX + GS + 12;
  const int16_t grW = 14;
  const float grDb  = meters.outLimiterGrDb[outIdx];
  grSmoothDb_[outIdx] = (grDb < grSmoothDb_[outIdx]) ? grDb : (grSmoothDb_[outIdx] * 0.85f);

  sp.fillRoundRect(grX, GY, grW, GS, 3, pal.panel);
  sp.drawRoundRect(grX, GY, grW, GS, 3, pal.grid);
  const float grNorm = clampf(-grSmoothDb_[outIdx] / 24.0f, 0.0f, 1.0f);
  const int16_t grH  = static_cast<int16_t>(grNorm * (GS - 4));
  if (grH > 0) sp.fillRect(grX + 2, GY + 2, grW - 4, grH, pal.warn);

  sp.setTextSize(1);
  sp.setTextDatum(top_center);
  sp.setTextColor(pal.warn);
  char grBuf[12];
  snprintf(grBuf, sizeof(grBuf), "%.1f", grSmoothDb_[outIdx]);
  sp.drawString(grBuf, grX + grW / 2, GY + GS + 2);

  // Ringkasan bersih di sisi kanan (x = 164..312)
  const int16_t tx = 164;
  sp.fillRoundRect(tx, GY, 148, GS, 4, pal.panel);
  sp.drawRoundRect(tx, GY, 148, GS, 4, pal.grid);

  char line[26];
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  sp.drawString("GATE", tx + 10, GY + 8);
  sp.setTextDatum(top_right);
  sp.setTextColor(out.gate.enabled ? pal.ok : pal.textDim);
  snprintf(line, sizeof(line), "%s  %.0fdB", out.gate.enabled ? "ON" : "OFF", out.gate.thresholdDb);
  sp.drawString(line, tx + 138, GY + 8);

  sp.drawFastHLine(tx + 8, GY + 26, 132, pal.grid);

  sp.setTextDatum(top_left);
  sp.setTextColor(pal.accent);
  sp.drawString("LIMITER", tx + 10, GY + 34);
  sp.setTextDatum(top_right);
  sp.setTextColor(out.limiter.enabled ? pal.warn : pal.textDim);
  snprintf(line, sizeof(line), "%s  %.1fdB", out.limiter.enabled ? "ON" : "OFF", out.limiter.thresholdDb);
  sp.drawString(line, tx + 138, GY + 34);

  sp.drawFastHLine(tx + 8, GY + 52, 132, pal.grid);

  sp.setTextDatum(top_left);
  sp.setTextColor(pal.textDim);
  sp.drawString("ANTI-CLIP", tx + 10, GY + 60);
  sp.setTextDatum(top_right);
  sp.setTextColor(pal.accent2);
  sp.drawString(out.antiClipMode == dsp::AntiClipMode::SOFT_TANH ? "SOFT TANH" : "HARD CLIP", tx + 138, GY + 60);

  sp.setTextDatum(top_left);
  sp.setTextColor(pal.textDim);
  sp.drawString("DELAY", tx + 10, GY + 78);
  sp.setTextDatum(top_right);
  sp.setTextColor(pal.text);
  snprintf(line, sizeof(line), "%.1f ms", out.delayMs);
  sp.drawString(line, tx + 138, GY + 78);
}

void UiTft::drawOutputPage(LGFX_Sprite &sp, const ThemePalette &pal,
                           const dsp::DspMeters &meters,
                           const dsp::SystemConfig &cfg) {
  // Baris atas: 4 pill OUT 1..4 + 1 pill Mode (EQ/XO | DYN)
  constexpr int16_t TOP_Y = 28;
  sp.setTextSize(1);
  sp.setTextDatum(middle_center);

  for (uint8_t o = 0; o < 4; o++) {
    const int16_t x = 8 + o * 53;
    const bool active = (o == outIdx_);
    sp.fillRoundRect(x, TOP_Y, 49, 17, 4, active ? pal.accent : pal.panel);
    sp.drawRoundRect(x, TOP_Y, 49, 17, 4, (active && outParamIdx_ == 0) ? TFT_WHITE : pal.grid);
    sp.setTextColor(active ? pal.bg : pal.textDim);
    char lbl[8];
    snprintf(lbl, sizeof(lbl), "OUT %u", o + 1);
    sp.drawString(lbl, x + 24, TOP_Y + 9);
  }

  // Mode toggle pill di kanan
  const bool modeFocus = (outParamIdx_ == 1);
  sp.fillRoundRect(224, TOP_Y, 88, 17, 4, pal.headerBg);
  sp.drawRoundRect(224, TOP_Y, 88, 17, 4, modeFocus ? TFT_WHITE : pal.accent);
  sp.setTextColor(pal.accent);
  sp.drawString(outTab_ == 0 ? "EQ / XOVER" : "GATE / LIM", 268, TOP_Y + 9);

  // Area Grafik
  if (outTab_ == 0) {
    drawOutputFreqCurve(sp, pal, outIdx_, cfg);
  } else {
    drawOutputDynamicsCurve(sp, pal, outIdx_, meters, cfg);
  }

  // Dock Parameter Interaktif yang bersih & lega (y = 166..221)
  const int16_t py = 166;
  sp.fillRoundRect(8, py, TFT_WIDTH - 16, 55, 6, pal.panel);
  sp.drawRoundRect(8, py, TFT_WIDTH - 16, 55, 6, outEditingParam_ ? pal.accent : pal.grid);
  if (outEditingParam_) {
    sp.drawRoundRect(9, py + 1, TFT_WIDTH - 18, 53, 5, pal.accent);
  }

  const uint8_t maxParams = (outTab_ == 0) ? 16 : 11;
  const auto &out = cfg.outputs[outIdx_];
  char pName[28] = "";
  char pVal[36]  = "";

  if (outParamIdx_ == 0) {
    strcpy(pName, "PILIH OUTPUT");
    snprintf(pVal, sizeof(pVal), "OUT %u", outIdx_ + 1);
  } else if (outParamIdx_ == 1) {
    strcpy(pName, "HALAMAN DSP");
    strcpy(pVal, outTab_ == 0 ? "EQ / XOVER" : "GATE / LIM");
  } else if (outTab_ == 0) {
    const auto &b = out.peq[outActiveBand_];
    switch (outParamIdx_) {
      case 2:  strcpy(pName, "OUTPUT GAIN");   snprintf(pVal, sizeof(pVal), "%+.1f dB", out.gainDb); break;
      case 3:  strcpy(pName, "OUTPUT MUTE");   strcpy(pVal, out.mute ? "MUTED" : "AKTIF"); break;
      case 4:  strcpy(pName, "FASE SINYAL");   strcpy(pVal, out.phaseInvert ? "180 deg" : "0 deg"); break;
      case 5:  strcpy(pName, "XOVER HPF");     if (!out.hpf.enabled) strcpy(pVal, "OFF"); else snprintf(pVal, sizeof(pVal), "%s %udB", XOVER_TYPE_SHORT[static_cast<uint8_t>(out.hpf.type)], static_cast<unsigned>(out.hpf.slope)); break;
      case 6:  strcpy(pName, "HPF FREKUENSI"); snprintf(pVal, sizeof(pVal), "%.1f Hz", out.hpf.freqHz); break;
      case 7:  strcpy(pName, "XOVER LPF");     if (!out.lpf.enabled) strcpy(pVal, "OFF"); else snprintf(pVal, sizeof(pVal), "%s %udB", XOVER_TYPE_SHORT[static_cast<uint8_t>(out.lpf.type)], static_cast<unsigned>(out.lpf.slope)); break;
      case 8:  strcpy(pName, "LPF FREKUENSI"); snprintf(pVal, sizeof(pVal), "%.1f Hz", out.lpf.freqHz); break;
      case 9:  strcpy(pName, "PEQ BAND");      snprintf(pVal, sizeof(pVal), "B%u/9 (%s)", outActiveBand_ + 1, PEQ_TYPE_SHORT[static_cast<uint8_t>(b.type)]); break;
      case 10: strcpy(pName, "PEQ FREKUENSI"); snprintf(pVal, sizeof(pVal), "%.1f Hz", b.freqHz); break;
      case 11: strcpy(pName, "PEQ GAIN");      snprintf(pVal, sizeof(pVal), "%+.1f dB", b.gainDb); break;
      case 12: strcpy(pName, "PEQ Q-FACTOR");  snprintf(pVal, sizeof(pVal), "Q %.1f", b.q); break;
      case 13: strcpy(pName, "OUTPUT DELAY");  snprintf(pVal, sizeof(pVal), "%.1f ms", out.delayMs); break;
      case 14: strcpy(pName, "BYPASS DSP");    strcpy(pVal, out.bypassDsp ? "BYPASS" : "AKTIF"); break;
      case 15: strcpy(pName, "SALIN OUTPUT");  strcpy(pVal, "-> OUT 1-4"); break;
    }
  } else {
    switch (outParamIdx_) {
      case 2:  strcpy(pName, "NOISE GATE");   strcpy(pVal, out.gate.enabled ? "ON" : "OFF"); break;
      case 3:  strcpy(pName, "GATE THRESH");  snprintf(pVal, sizeof(pVal), "%.1f dB", out.gate.thresholdDb); break;
      case 4:  strcpy(pName, "GATE ATTACK");  snprintf(pVal, sizeof(pVal), "%.1f ms", out.gate.attackMs); break;
      case 5:  strcpy(pName, "GATE RELEASE"); snprintf(pVal, sizeof(pVal), "%.0f ms", out.gate.releaseMs); break;
      case 6:  strcpy(pName, "PEAK LIMITER"); strcpy(pVal, out.limiter.enabled ? "ON" : "OFF"); break;
      case 7:  strcpy(pName, "LIM THRESH");   snprintf(pVal, sizeof(pVal), "%.1f dB", out.limiter.thresholdDb); break;
      case 8:  strcpy(pName, "LIM ATTACK");   snprintf(pVal, sizeof(pVal), "%.2f ms", out.limiter.attackMs); break;
      case 9:  strcpy(pName, "LIM RELEASE");  snprintf(pVal, sizeof(pVal), "%.0f ms", out.limiter.releaseMs); break;
      case 10: strcpy(pName, "ANTI-CLIP");    strcpy(pVal, out.antiClipMode == dsp::AntiClipMode::SOFT_TANH ? "SOFT" : "HARD"); break;
    }
  }

  sp.setTextDatum(middle_left);
  sp.setTextColor(pal.accent2);
  sp.drawString(pName, 18, py + 12);

  char idxStr[12];
  snprintf(idxStr, sizeof(idxStr), "%u/%u", static_cast<unsigned>(outParamIdx_ + 1), static_cast<unsigned>(maxParams));
  sp.setTextDatum(middle_right);
  sp.setTextColor(pal.textDim);
  sp.drawString(idxStr, TFT_WIDTH - 18, py + 12);

  sp.setTextSize(2);
  sp.setTextDatum(middle_center);
  sp.setTextColor(outEditingParam_ ? pal.accent : pal.text);
  char valDisp[44];
  if (outEditingParam_) snprintf(valDisp, sizeof(valDisp), "< %s >", pVal);
  else                  snprintf(valDisp, sizeof(valDisp), "%s", pVal);
  sp.drawString(valDisp, TFT_WIDTH / 2, py + 37);

  drawFooterHint(sp, pal, outEditingParam_
      ? "Putar: Ubah Nilai  |  Klik: Selesai"
      : "Putar: Pilih  |  Klik: Ubah  |  Tahan: Kembali");
}

void UiTft::drawPresetPage(LGFX_Sprite &sp, const ThemePalette &pal) {
  const int16_t listY = HEADER_HEIGHT + 8;
  sp.setTextSize(1);

  for (uint8_t s = 0; s < NUM_PRESET_SLOTS; s++) {
    const int16_t y = listY + s * 23;
    const bool sel  = (s == presetSlotCursor_);
    sp.fillRoundRect(10, y, TFT_WIDTH - 20, 20, 4, sel ? pal.accent : pal.panel);
    if (!sel) sp.drawRoundRect(10, y, TFT_WIDTH - 20, 20, 4, pal.grid);

    if (presetExists_[s]) sp.fillCircle(20, y + 10, 3, sel ? pal.bg : pal.ok);
    else                  sp.drawCircle(20, y + 10, 3, sel ? pal.bg : pal.textDim);

    char row[40];
    snprintf(row, sizeof(row), "%u. %s", s + 1, presetNames_[s]);
    sp.setTextDatum(middle_left);
    sp.setTextColor(sel ? pal.bg : (presetExists_[s] ? pal.text : pal.textDim));
    sp.drawString(row, 30, y + 10);

    if (presetExists_[s]) {
      sp.setTextDatum(middle_right);
      sp.setTextColor(sel ? pal.bg : pal.accent2);
      sp.drawString("OK", TFT_WIDTH - 18, y + 10);
    }
  }

  drawFooterHint(sp, pal, "Putar: Pilih Slot  |  Klik: Aksi  |  Tahan: Kembali");

  if (presetRenaming_) {
    sp.fillRoundRect(18, 66, TFT_WIDTH - 36, 98, 8, pal.bg);
    sp.drawRoundRect(18, 66, TFT_WIDTH - 36, 98, 8, pal.accent);
    sp.setTextDatum(top_center);
    sp.setTextColor(pal.accent);
    sp.drawString("UBAH NAMA PRESET", TFT_WIDTH / 2, 76);

    sp.setTextSize(2);
    for (int i = 0; i < 12; i++) {
      const int16_t cx = 44 + i * 19;
      if (i == renameCharPos_) {
        sp.fillRoundRect(cx - 2, 102, 16, 24, 3, pal.accent);
        sp.setTextColor(pal.bg);
      } else {
        sp.setTextColor(pal.text);
      }
      char cStr[2] = {renameBuf_[i], '\0'};
      sp.setTextDatum(top_left);
      sp.drawString(cStr, cx, 106);
    }
    sp.setTextSize(1);
    sp.setTextDatum(bottom_center);
    sp.setTextColor(pal.textDim);
    sp.drawString("Putar: Huruf  |  Klik: Geser  |  Tahan: Simpan", TFT_WIDTH / 2, 156);
  } else if (presetActionMenu_) {
    sp.fillRoundRect(52, 72, TFT_WIDTH - 104, 94, 8, pal.bg);
    sp.drawRoundRect(52, 72, TFT_WIDTH - 104, 94, 8, pal.accent);
    const char *acts[3] = {"MUAT PRESET", "SIMPAN SLOT", "UBAH NAMA"};
    for (uint8_t a = 0; a < 3; a++) {
      const int16_t ay = 83 + a * 25;
      sp.fillRoundRect(64, ay, TFT_WIDTH - 128, 21, 4, (a == presetAction_) ? pal.accent : pal.panel);
      sp.setTextDatum(middle_center);
      sp.setTextColor((a == presetAction_) ? pal.bg : pal.text);
      sp.drawString(acts[a], TFT_WIDTH / 2, ay + 10);
    }
  }
}

void UiTft::drawSetupMenuPage(LGFX_Sprite &sp, const ThemePalette &pal) {
  // Grid 2x2 super bersih & lega dengan judul besar mudah dibaca
  for (uint8_t i = 0; i < NUM_SETUP_ITEMS; i++) {
    const int16_t col = i % 2;
    const int16_t row = i / 2;
    const int16_t x   = 12 + col * 152;
    const int16_t y   = 38 + row * 90;
    const bool sel    = (i == setupCursor_);

    sp.fillRoundRect(x, y, 144, 78, 8, sel ? pal.accent : pal.panel);
    sp.drawRoundRect(x, y, 144, 78, 8, sel ? TFT_WHITE : pal.grid);
    if (!sel) {
      sp.fillRoundRect(x + 3, y + 12, 3, 54, 1, pal.accent);
    }

    sp.setTextDatum(middle_left);
    sp.setTextSize(2);
    sp.setTextColor(sel ? pal.bg : pal.text);
    sp.drawString(SETUP_LABELS[i], x + 14, y + 30);

    sp.setTextSize(1);
    sp.setTextColor(sel ? pal.bg : pal.textDim);
    sp.drawString(SETUP_SUBTITLES[i], x + 14, y + 54);
  }

  drawFooterHint(sp, pal, "Putar: Pilih  |  Klik: Buka  |  Tahan: Kembali");
}

void UiTft::drawSetupSubPage(LGFX_Sprite &sp, const ThemePalette &pal,
                             const audio_io::AudioTelemetry &audioTel) {
  if (setupSub_ == SetupSubmenu::PRESET) {
    drawPresetPage(sp, pal);
    return;
  }

  const int16_t y0 = HEADER_HEIGHT + 12;
  const char *footer = "Putar: Pilih  |  Klik: Ubah  |  Tahan: Kembali";

  switch (setupSub_) {
    case SetupSubmenu::KONEKSI: {
      const char *lbls[2] = {"MODE WIFI", "BLUETOOTH BLE"};
      const char *vals[2] = {
          WIFI_MODE_NAMES[static_cast<uint8_t>(uiCfg_.wifiMode)],
          uiCfg_.bleEnabled ? "AKTIF" : "OFF"};
      for (int i = 0; i < 2; i++) {
        const int16_t y = y0 + i * 42;
        const bool sel  = (subCursor_ == i);
        sp.fillRoundRect(12, y, TFT_WIDTH - 24, 34, 6, sel ? pal.accent : pal.panel);
        sp.drawRoundRect(12, y, TFT_WIDTH - 24, 34, 6, sel ? TFT_WHITE : pal.grid);
        sp.setTextSize(1);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel ? pal.bg : pal.text);
        sp.drawString(lbls[i], 22, y + 17);
        sp.setTextDatum(middle_right);
        sp.setTextColor(sel ? pal.bg : pal.accent2);
        sp.drawString(vals[i], TFT_WIDTH - 22, y + 17);
      }

      // Kartu info koneksi ringkas
      sp.fillRoundRect(12, y0 + 92, TFT_WIDTH - 24, 72, 6, pal.panel);
      sp.drawRoundRect(12, y0 + 92, TFT_WIDTH - 24, 72, 6, pal.grid);
      sp.setTextDatum(top_left);
      sp.setTextSize(1);
      if (uiCfg_.wifiMode == WifiMode::AU_WEB) {
        sp.fillCircle(24, y0 + 109, 3, pal.ok);
        sp.setTextColor(pal.accent);
        sp.drawString("WEB CONTROL AKTIF", 34, y0 + 105);
        sp.setTextColor(pal.text);
        sp.drawString("SSID : AU-WEB  |  Pass : 12345678", 22, y0 + 125);
        sp.setTextColor(pal.accent2);
        sp.drawString("URL  : http://192.168.4.1", 22, y0 + 143);
      } else if (uiCfg_.wifiMode == WifiMode::AU_OTA) {
        sp.fillCircle(24, y0 + 109, 3, pal.warn);
        sp.setTextColor(pal.warn);
        sp.drawString("PORTAL UPDATE OTA AKTIF", 34, y0 + 105);
        sp.setTextColor(pal.text);
        sp.drawString("SSID : AU-OTA  |  Pass : 12345678", 22, y0 + 125);
        sp.setTextColor(pal.accent2);
        sp.drawString("URL  : http://192.168.4.1", 22, y0 + 143);
      } else if (uiCfg_.bleEnabled) {
        sp.fillCircle(24, y0 + 109, 3, rgb565(90, 190, 255));
        sp.setTextColor(pal.accent2);
        sp.drawString("BLUETOOTH BLE AKTIF", 34, y0 + 105);
        sp.setTextColor(pal.text);
        sp.drawString("Nama : DSP-Audio-IF  |  PIN : 123456", 22, y0 + 125);
        sp.setTextColor(pal.textDim);
        sp.drawString("Hubungkan via Web App / nRF Connect", 22, y0 + 143);
      } else {
        sp.fillCircle(24, y0 + 109, 3, pal.textDim);
        sp.setTextColor(pal.text);
        sp.drawString("RADIO NONAKTIF", 34, y0 + 105);
        sp.setTextColor(pal.textDim);
        sp.drawString("Mode Studio Senyap (Bebas RF)", 22, y0 + 128);
      }
      break;
    }

    case SetupSubmenu::TAMPILAN: {
      const char *lbls[4] = {"TEMA WARNA", "PANEL LCD", "AUTO STANDBY", "WAKTU STANDBY"};
      char vals[4][24];
      snprintf(vals[0], sizeof(vals[0]), "%s", THEME_NAMES[static_cast<uint8_t>(uiCfg_.theme)]);
      snprintf(vals[1], sizeof(vals[1]), "%s", PANEL_MODE_NAMES[static_cast<uint8_t>(uiCfg_.panelMode)]);
      strcpy(vals[2], uiCfg_.standbyEnabled ? "AKTIF" : "NONAKTIF");
      snprintf(vals[3], sizeof(vals[3]), "%u dtk", uiCfg_.standbyTimeoutSec);

      for (int i = 0; i < 4; i++) {
        const int16_t y = y0 + i * 33;
        const bool sel  = (subCursor_ == i);
        const bool edit = (sel && subEditing_);
        sp.fillRoundRect(12, y, TFT_WIDTH - 24, 27, 5, edit ? pal.accent : (sel ? pal.headerBg : pal.panel));
        sp.drawRoundRect(12, y, TFT_WIDTH - 24, 27, 5, sel ? pal.accent : pal.grid);
        sp.setTextSize(1);
        sp.setTextDatum(middle_left);
        sp.setTextColor(edit ? pal.bg : pal.text);
        sp.drawString(lbls[i], 22, y + 13);
        sp.setTextDatum(middle_right);
        sp.setTextColor(edit ? pal.bg : pal.accent2);
        sp.drawString(vals[i], TFT_WIDTH - 22, y + 13);
      }

      // Strip pratinjau warna tema aktif
      const int16_t py = y0 + 140;
      sp.fillRoundRect(12,  py, 94, 20, 4, pal.accent);
      sp.fillRoundRect(113, py, 94, 20, 4, pal.accent2);
      sp.fillRoundRect(214, py, 94, 20, 4, pal.curve);
      sp.setTextDatum(middle_center);
      sp.setTextColor(TFT_BLACK);
      sp.drawString("UTAMA",  59, py + 10);
      sp.drawString("AKSEN", 160, py + 10);
      sp.drawString("KURVA", 261, py + 10);

      footer = subEditing_ ? "Putar: Ubah  |  Klik: Selesai"
                           : "Putar: Pilih  |  Klik: Ubah  |  Tahan: Kembali";
      break;
    }

    case SetupSubmenu::SISTEM: {
      const char *lbls[2] = {"KUNCI PANEL", "RESET PABRIK"};
      const char *vals[2] = {uiCfg_.panelLocked ? "TERKUNCI" : "KUNCI SEKARANG", "DEFAULT DSP"};
      for (int i = 0; i < 2; i++) {
        const int16_t y = y0 + i * 42;
        const bool sel  = (subCursor_ == i);
        sp.fillRoundRect(12, y, TFT_WIDTH - 24, 34, 6, sel ? pal.accent : pal.panel);
        sp.drawRoundRect(12, y, TFT_WIDTH - 24, 34, 6, sel ? TFT_WHITE : pal.grid);
        sp.setTextSize(1);
        sp.setTextDatum(middle_left);
        sp.setTextColor(sel ? pal.bg : pal.text);
        sp.drawString(lbls[i], 22, y + 17);
        sp.setTextDatum(middle_right);
        sp.setTextColor(sel ? pal.bg : (i == 1 ? pal.danger : pal.accent2));
        sp.drawString(vals[i], TFT_WIDTH - 22, y + 17);
      }

      // Kartu status perangkat ringkas
      sp.fillRoundRect(12, y0 + 92, TFT_WIDTH - 24, 72, 6, pal.panel);
      sp.drawRoundRect(12, y0 + 92, TFT_WIDTH - 24, 72, 6, pal.grid);
      sp.setTextSize(1);
      sp.setTextDatum(top_left);
      sp.setTextColor(pal.accent);
      sp.drawString("DSP Audio Interface  v2.0", 22, y0 + 104);

      char info1[48], info2[48];
      snprintf(info1, sizeof(info1), "Suhu Chip  : %.1f C  (Max %.1f C)",
               audioTel.chipTempC, audioTel.chipTempPeakC);
      snprintf(info2, sizeof(info2), "Beban DSP  : %.1f%%  |  48 kHz",
               audioTel.dspCpuLoadPct);
      sp.setTextColor(pal.text);
      sp.drawString(info1, 22, y0 + 124);
      sp.setTextColor(pal.textDim);
      sp.drawString(info2, 22, y0 + 142);
      break;
    }

    default:
      break;
  }

  drawFooterHint(sp, pal, footer);
}

void UiTft::drawStandbyScreen(LGFX_Sprite &sp, const ThemePalette &pal,
                              const audio_io::AudioTelemetry &audioTel) {
  if (standbyLogoBuf_ != nullptr) {
    sp.pushImage(0, 0, TFT_WIDTH, TFT_HEIGHT, standbyLogoBuf_);
  } else {
    renderFallbackLogo(sp, pal, true, static_cast<float>(millis()) * 0.001f);
  }
  char st[32];
  snprintf(st, sizeof(st), "USB %s  |  %.0fC", audioTel.usbMounted ? "AKTIF" : "SIAGA", audioTel.chipTempC);
  sp.setTextSize(1);
  sp.setTextDatum(bottom_right);
  sp.setTextColor(pal.textDim);
  sp.drawString(st, TFT_WIDTH - 14, TFT_HEIGHT - 12);
}

void UiTft::drawOverheatScreen(LGFX_Sprite &sp, float tempC) {
  const bool flash = ((millis() / 250) & 1u) != 0u;
  sp.fillScreen(flash ? rgb565(180, 18, 26) : rgb565(110, 12, 18));
  sp.drawRoundRect(10, 10, TFT_WIDTH - 20, TFT_HEIGHT - 20, 8, TFT_WHITE);

  sp.setTextDatum(middle_center);
  sp.setTextColor(TFT_WHITE);
  sp.setTextSize(2);
  sp.drawString("PERINGATAN SUHU", TFT_WIDTH / 2, 54);

  char tStr[20];
  snprintf(tStr, sizeof(tStr), "%.1f C", tempC);
  sp.setTextSize(4);
  sp.setTextColor(rgb565(255, 240, 80));
  sp.drawString(tStr, TFT_WIDTH / 2, 118);

  sp.setTextSize(1);
  sp.setTextColor(TFT_WHITE);
  sp.drawString("SUHU >= 75 C - PERIKSA VENTILASI", TFT_WIDTH / 2, 174);
  sp.drawString("Klik Encoder untuk abaikan sementara", TFT_WIDTH / 2, 200);
}

void UiTft::drawLockOverlay(LGFX_Sprite &sp, const ThemePalette &pal,
                            const ui_ctrl::InputEvent &ev) {
  const uint32_t nowMs = millis();
  const bool showHoldingUnlock = uiCfg_.panelLocked && ev.isHoldingBtn && (ev.holdDurationMs > 120);
  if (!showHoldingUnlock && nowMs >= lockBannerUntilMs_) return;

  sp.fillRoundRect(46, 86, TFT_WIDTH - 92, 66, 8, pal.bg);
  sp.drawRoundRect(46, 86, TFT_WIDTH - 92, 66, 8, pal.warn);

  sp.drawCircle(72, 110, 6, pal.warn);
  sp.fillRoundRect(63, 112, 18, 15, 3, pal.warn);

  sp.setTextSize(1);
  sp.setTextDatum(top_left);
  sp.setTextColor(pal.warn);
  sp.drawString("PANEL TERKUNCI", 92, 98);
  sp.setTextColor(pal.textDim);
  sp.drawString("Tahan 3 detik untuk buka", 92, 114);

  const float prog = showHoldingUnlock ? clampf(ev.holdDurationMs / 3000.0f, 0.0f, 1.0f) : 0.0f;
  sp.drawRect(92, 132, 164, 8, pal.grid);
  if (prog > 0.0f) {
    sp.fillRect(94, 134, static_cast<int16_t>(prog * 160.0f), 4, pal.warn);
  }
}

// ============================================================================
// 9. MAIN 30 FPS UPDATE & DOUBLE-BUFFER PUSH (CORE 0)
// ============================================================================
void UiTft::update(const ui_ctrl::InputEvent &ev,
                   const audio_io::AudioTelemetry &audioTel,
                   const dsp::DspMeters &dspMeters) {
  const uint32_t nowMs = millis();

  if (audioTel.chipTempC >= TEMP_WARN_THRESHOLD_C) {
    overheatActive_ = true;
    inStandby_      = false;
  } else if (audioTel.chipTempC < (TEMP_WARN_THRESHOLD_C - 3.0f)) {
    overheatActive_    = false;
    overheatDismissed_ = false;
  }

  handleInput(ev);

  if (uiCfg_.standbyEnabled && !inStandby_ && !overheatActive_) {
    const uint32_t idleSec = (nowMs - lastActivityMs_) / 1000u;
    if (idleSec >= uiCfg_.standbyTimeoutSec) {
      inStandby_ = true;
    }
  }

  LGFX_Sprite &sp = fb_[curFb_];
  const ThemePalette pal = resolveTheme(nowMs);
  const dsp::SystemConfig cfg = dsp_->getConfig();

  if (overheatActive_ && !overheatDismissed_) {
    drawOverheatScreen(sp, audioTel.chipTempC);
  } else if (inStandby_) {
    drawStandbyScreen(sp, pal, audioTel);
  } else {
    sp.fillScreen(pal.bg);

    if (!inDetailPage_) {
      drawHeader(sp, pal, audioTel, nullptr);
      drawLiveVuMeterPage(sp, pal, dspMeters, cfg);
    } else {
      switch (activeMenu_) {
        case TopMenuId::IN_CH:
          drawHeader(sp, pal, audioTel, "INPUT A / B");
          drawUnifiedInputPage(sp, pal, dspMeters, cfg);
          break;
        case TopMenuId::MATRIX:
          drawHeader(sp, pal, audioTel, "ROUTING MATRIX");
          drawMatrixPage(sp, pal, cfg);
          break;
        case TopMenuId::OUT_CH: {
          char hdr[28];
          snprintf(hdr, sizeof(hdr), "OUT %u  |  %s", outIdx_ + 1, outTab_ == 0 ? "EQ / XO" : "DYNAMICS");
          drawHeader(sp, pal, audioTel, hdr);
          drawOutputPage(sp, pal, dspMeters, cfg);
          break;
        }
        case TopMenuId::SETUP: {
          if (setupSub_ == SetupSubmenu::NONE) {
            drawHeader(sp, pal, audioTel, "SETUP");
            drawSetupMenuPage(sp, pal);
          } else {
            char hdr[28];
            snprintf(hdr, sizeof(hdr), "SETUP > %s", SETUP_LABELS[static_cast<uint8_t>(setupSub_) - 1]);
            drawHeader(sp, pal, audioTel, hdr);
            drawSetupSubPage(sp, pal, audioTel);
          }
          break;
        }
        case TopMenuId::METER:
        default:
          drawHeader(sp, pal, audioTel, nullptr);
          drawLiveVuMeterPage(sp, pal, dspMeters, cfg);
          break;
      }
    }

    drawLockOverlay(sp, pal, ev);

    if (nowMs < toastUntilMs_) {
      sp.fillRoundRect(64, TFT_HEIGHT - 38, TFT_WIDTH - 128, 22, 6, pal.accent);
      sp.drawRoundRect(64, TFT_HEIGHT - 38, TFT_WIDTH - 128, 22, 6, TFT_WHITE);
      sp.setTextSize(1);
      sp.setTextDatum(middle_center);
      sp.setTextColor(pal.bg);
      sp.drawString(toastMsg_, TFT_WIDTH / 2, TFT_HEIGHT - 27);
    }
  }

  lcd_.waitDMA();
  sp.pushSprite(&lcd_, 0, 0);
  curFb_ ^= 1;
}

} // namespace ui_tft
