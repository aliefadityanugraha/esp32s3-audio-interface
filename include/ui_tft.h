#pragma once

/**
 * @file ui_tft.h
 * @brief UI Layar TFT ILI9341 (320x240 Landscape, LovyanGFX)
 *        Double-Buffered PSRAM Sprites (30 FPS, Core 0) — Desain Studio Minimalis
 *
 * Arsitektur Navigasi 5-Tab Tanpa Scroll (Semua Terlihat Sekaligus):
 *  1. METER  : Live VU Meter 6-kanal (IN A/B | OUT 1-4) + Kontrol Master Volume/Mute cepat
 *  2. INPUT  : Kontrol terpadu Direct Input IN A & IN B (Live VU, Gain, Mute, Phase)
 *  3. MATRIX : Routing Matrix 4x4 langsung (OUT 1-4 x IN A / IN B / USB L / USB R)
 *  4. OUT    : DSP Output OUT 1-4 (Kurva XOVER + 9-Band PEQ & Kurva Gate / Limiter + Delay)
 *  5. SETUP  : 4 Kartu Utama (PRESET, KONEKSI WiFi/BLE, TAMPILAN Tema/LCD/Standby, SISTEM)
 */

#include <Arduino.h>
#include <LovyanGFX.hpp>
#include <cstdint>

#include "audio_config.h"
#include "audio_io.h"
#include "dsp_engine.h"
#include "input_ctrl.h"
#include "pin_config.h"

namespace ui_tft {

constexpr int16_t  TFT_WIDTH          = 320;
constexpr int16_t  TFT_HEIGHT         = 240;
constexpr int16_t  HEADER_HEIGHT      = 24;
constexpr uint8_t  NUM_TOP_MENU_ITEMS = 5;
constexpr uint8_t  NUM_SETUP_ITEMS    = 4;
constexpr uint8_t  NUM_PRESET_SLOTS   = 8;
constexpr uint32_t UI_CFG_MAGIC       = 0x55494346; // "UICF"

// ============================================================================
// 1. ENUM TEMA, PANEL COLOR MODE, WIFI MODE, DAN NAVIGASI HALAMAN
// ============================================================================
enum class UiTheme : uint8_t {
  RED         = 0,
  ICE_LUXURY  = 1,
  UNGU_EMAS   = 2,
  MERAH_BIRU  = 3,
  MERAH_PUTIH = 4,
  BIRU_KUNING = 5,
  RGB_PELANGI = 6
};

enum class PanelColorMode : uint8_t {
  NORMAL           = 0, // Normal RGB
  NEGATIF          = 1, // Hardware Display Invert
  TUKAR_RB         = 2, // Hardware RGB/BGR Swap
  NEGATIF_TUKAR_RB = 3  // Hardware Invert + RGB/BGR Swap
};

enum class WifiMode : uint8_t {
  OFF    = 0,
  AU_WEB = 1, // Web Control Console
  AU_OTA = 2  // Firmware & Web OTA Portal
};

enum class TopMenuId : uint8_t {
  METER  = 0,
  IN_CH  = 1,
  MATRIX = 2,
  OUT_CH = 3,
  SETUP  = 4
};

enum class SetupSubmenu : uint8_t {
  NONE     = 0,
  PRESET   = 1,
  KONEKSI  = 2,
  TAMPILAN = 3,
  SISTEM   = 4
};

// Struktur Pengaturan UI yang disimpan permanen ke LittleFS (`/ui_settings.bin`)
struct UiConfig {
  uint32_t       magic;
  UiTheme        theme;
  PanelColorMode panelMode;
  uint16_t       standbyTimeoutSec; // 5 - 600 detik
  bool           standbyEnabled;
  bool           panelLocked;
  WifiMode       wifiMode;
  bool           bleEnabled;
  uint8_t        brightness;        // 20 - 255
};

// Palet RGB565 hasil resolusi tema aktif (diperbarui tiap frame untuk RGB PELANGI)
struct ThemePalette {
  uint16_t bg;
  uint16_t panel;
  uint16_t headerBg;
  uint16_t accent;
  uint16_t accent2;
  uint16_t text;
  uint16_t textDim;
  uint16_t grid;
  uint16_t curveFill;
  uint16_t curve;
  uint16_t ok;
  uint16_t warn;
  uint16_t danger;
};

// ============================================================================
// 2. DRIVER LOVYANGFX ILI9341 (240x320 SPI2_HOST)
// ============================================================================
class LgfxIli9341 : public lgfx::LGFX_Device {
public:
  LgfxIli9341();
  void applyPanelColorMode(PanelColorMode mode);

private:
  lgfx::Panel_ILI9341 panel_;
  lgfx::Bus_SPI       bus_;
};

// ============================================================================
// 3. KELAS UTAMA UI TFT ENGINE (CORE 0 - 30 FPS PSRAM DOUBLE BUFFERING)
// ============================================================================
class UiTft {
public:
  UiTft() = default;
  ~UiTft();

  bool begin(audio_io::AudioIO &audioIO,
             dsp::DspEngine &dspEngine,
             ui_ctrl::InputController &inputCtrl);

  void showBootScreen(uint32_t durationMs = 2000);

  void update(const ui_ctrl::InputEvent &ev,
              const audio_io::AudioTelemetry &audioTel,
              const dsp::DspMeters &dspMeters);

  UiConfig getUiConfig() const { return uiCfg_; }
  void applyUiConfig(const UiConfig &cfg, bool persist = true);
  void setRadioUiState(WifiMode wifiMode, bool bleEnabled);

  bool isOnHomeScreen() const;

  // Manajemen Preset LittleFS (8 slot: `/presets/slot_0.bin` .. `slot_7.bin`)
  bool savePresetSlot(uint8_t slot, const char *name = nullptr);
  bool loadPresetSlot(uint8_t slot);
  bool deletePresetSlot(uint8_t slot);
  bool renamePresetSlot(uint8_t slot, const char *newName);
  void refreshPresetNames();
  void reloadLogos();

private:
  void loadUiConfig();
  void saveUiConfig();

  bool loadRawImagePsram(const char *path, uint16_t *&dstBuf, size_t &dstPixels);
  void renderFallbackLogo(LGFX_Sprite &sp, const ThemePalette &pal,
                          bool isStandby, float progressOrPulse);

  ThemePalette resolveTheme(uint32_t nowMs) const;
  static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

  // Input handlers
  void handleInput(const ui_ctrl::InputEvent &ev);
  void handleHomeInput(const ui_ctrl::InputEvent &ev);
  void handleInputPageInput(const ui_ctrl::InputEvent &ev);
  void handleMatrixInput(const ui_ctrl::InputEvent &ev);
  void handleOutputInput(const ui_ctrl::InputEvent &ev);
  void handlePresetInput(const ui_ctrl::InputEvent &ev);
  void handleSetupInput(const ui_ctrl::InputEvent &ev);

  // Renderers utama
  void drawHeader(LGFX_Sprite &sp, const ThemePalette &pal,
                  const audio_io::AudioTelemetry &audioTel,
                  const char *titleOverride = nullptr);
  void drawBottomTabBar(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawFooterHint(LGFX_Sprite &sp, const ThemePalette &pal, const char *hint);
  void drawLiveVuMeterPage(LGFX_Sprite &sp, const ThemePalette &pal,
                           const dsp::DspMeters &meters,
                           const dsp::SystemConfig &cfg);
  void drawUnifiedInputPage(LGFX_Sprite &sp, const ThemePalette &pal,
                            const dsp::DspMeters &meters,
                            const dsp::SystemConfig &cfg);
  void drawMatrixPage(LGFX_Sprite &sp, const ThemePalette &pal,
                      const dsp::SystemConfig &cfg);
  void drawOutputPage(LGFX_Sprite &sp, const ThemePalette &pal,
                      const dsp::DspMeters &meters,
                      const dsp::SystemConfig &cfg);
  void drawOutputFreqCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                           int outIdx, const dsp::SystemConfig &cfg);
  void drawOutputDynamicsCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                               int outIdx, const dsp::DspMeters &meters,
                               const dsp::SystemConfig &cfg);
  void drawPresetPage(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawSetupMenuPage(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawSetupSubPage(LGFX_Sprite &sp, const ThemePalette &pal,
                        const audio_io::AudioTelemetry &audioTel);
  void drawStandbyScreen(LGFX_Sprite &sp, const ThemePalette &pal,
                         const audio_io::AudioTelemetry &audioTel);
  void drawOverheatScreen(LGFX_Sprite &sp, float tempC);
  void drawLockOverlay(LGFX_Sprite &sp, const ThemePalette &pal,
                       const ui_ctrl::InputEvent &ev);

  float evalChannelMagDb(int outIdx, float freqHz,
                         const dsp::SystemConfig &cfg) const;

  // Referensi modul inti
  audio_io::AudioIO        *audioIO_   = nullptr;
  dsp::DspEngine           *dsp_       = nullptr;
  ui_ctrl::InputController *input_     = nullptr;

  // Hardware LCD & PSRAM Double-Buffer Sprites (Ping-Pong)
  LgfxIli9341 lcd_;
  LGFX_Sprite fb_[2];
  uint8_t     curFb_        = 0;
  bool        psramSprites_ = false;

  // Cache Logo LittleFS di PSRAM
  uint16_t   *bootLogoBuf_    = nullptr;
  size_t      bootLogoPixels_ = 0;
  uint16_t   *standbyLogoBuf_ = nullptr;
  size_t      standbyLogoPix_ = 0;

  // Konfigurasi UI & State Runtime
  UiConfig    uiCfg_             = {};
  uint32_t    lastActivityMs_    = 0;
  bool        inStandby_         = false;
  bool        overheatActive_    = false;
  bool        overheatDismissed_ = false;
  uint32_t    lockBannerUntilMs_ = 0;

  // State Navigasi Utama (5 Tab Tanpa Scroll)
  TopMenuId   activeMenu_    = TopMenuId::METER;
  bool        inDetailPage_  = false; // True: di dalam halaman / kontrol Master
  bool        masterEditing_ = false; // True saat mengedit Master Volume langsung di METER

  // State Halaman INPUT (Unified IN A & IN B: 6 kontrol)
  uint8_t     inCursor_  = 0;     // 0..2 = IN A (Gain,Mute,Phase), 3..5 = IN B (Gain,Mute,Phase)
  bool        inEditing_ = false;

  // State Halaman MATRIX (16 sel 4x4)
  uint8_t     matrixCursor_ = 0;

  // State Halaman OUTPUT (OUT 1-4)
  uint8_t     outIdx_          = 0; // 0..3 (OUT 1..OUT 4)
  uint8_t     outTab_          = 0; // 0 = EQ/XO, 1 = DYN (Gate/Lim)
  uint8_t     outParamIdx_     = 0;
  bool        outEditingParam_ = false;
  uint8_t     outActiveBand_   = 0; // 0..8 (PEQ B1..B9)

  // State Halaman PRESET & Rename Editor
  uint8_t     presetSlotCursor_ = 0;
  uint8_t     presetAction_     = 0; // 0=LOAD, 1=SAVE, 2=RENAME
  bool        presetActionMenu_ = false;
  bool        presetRenaming_   = false;
  uint8_t     renameCharPos_    = 0;
  char        renameBuf_[20]    = {0};
  char        presetNames_[NUM_PRESET_SLOTS][20] = {{0}};
  bool        presetExists_[NUM_PRESET_SLOTS]    = {false};
  uint32_t    toastUntilMs_     = 0;
  char        toastMsg_[36]     = {0};

  // State Halaman SETUP & Submenu (4 Kartu)
  uint8_t      setupCursor_ = 0;
  SetupSubmenu setupSub_    = SetupSubmenu::NONE;
  uint8_t      subCursor_   = 0;
  bool         subEditing_  = false;

  // State VU Meter Smooth & Peak Hold (6 kanal: 0..1=IN A/B, 2..5=OUT 1..4)
  float        vuSmoothDb_[6]   = {-96,-96,-96,-96,-96,-96};
  float        vuPeakHoldDb_[6] = {-96,-96,-96,-96,-96,-96};
  uint32_t     vuPeakHoldMs_[6] = {0};
  uint32_t     vuClipHoldMs_[6] = {0};
  float        grSmoothDb_[4]   = {0};

  // Cache kurva respon frekuensi (73 titik) agar Core 0 ringan
  static constexpr uint8_t CURVE_POINTS = 73;
  mutable float    curveCacheDb_[CURVE_POINTS] = {0};
  mutable uint32_t curveCacheHash_             = 0;

  void showToast(const char *msg, uint32_t durationMs = 1300);
};

} // namespace ui_tft
