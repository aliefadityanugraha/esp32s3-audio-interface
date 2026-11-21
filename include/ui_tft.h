#pragma once

/**
 * @file ui_tft.h
 * @brief Tahap 3: UI Layar TFT ST7789 2.0" (320x240 Landscape) berbasis LovyanGFX
 *        dengan Sprite Double-Buffering di PSRAM (30 FPS bebas flicker, Core 0).
 *
 * Fitur Utama:
 *  - 7 Tema Warna UI (RED, ICE LUXURY, UNGU-EMAS, MERAH-BIRU, MERAH-PUTIH,
 *    BIRU-KUNING, dan RGB PELANGI dinamis).
 *  - 4 Mode Warna Hardware Panel (Normal, Negatif, Tukar R-B, Negatif+Tukar R-B).
 *  - Boot Logo (`/logo_boot.bin`) & Standby Logo (`/logo_standby.bin`) dari LittleFS
 *    dengan fallback vektor + Timer Auto-Standby (5 - 600 detik).
 *  - Header Bar permanen (USB, BLE/WiFi, Master Volume, Suhu Chip) + Layar Peringatan
 *    Merah penuh jika suhu chip >= 75°C.
 *  - Menu Geser Horizontal "AUDIO INTERFACE MENU":
 *    METER, IN A, IN B, OUT 1, OUT 2, OUT 3, OUT 4, PRESET, SETUP.
 *  - Halaman LIVE VU METER 6 kanal (Peak-Hold, CLIP, LIMITER, GR).
 *  - Halaman Kanal (IN A/B & OUT 1-4): Grafik kurva gabungan XOVER + PEQ real-time
 *    di atas spektrum RTA live + Halaman kurva transfer Compressor/Gate/Limiter & GR.
 *  - Halaman SETUP: MATRIX, PRESET (Save/Load/Rename LittleFS), LEVEL, SINYAL,
 *    TEMA, WIFI (AU-WEB / AU-OTA), INPUT, dan SISTEM (Panel Lock, Standby, Info).
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
constexpr int16_t  HEADER_HEIGHT      = 26;
constexpr uint8_t  NUM_TOP_MENU_ITEMS = 9;
constexpr uint8_t  NUM_SETUP_ITEMS    = 8;
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
  NORMAL          = 0, // Normal RGB
  NEGATIF         = 1, // Hardware Display Invert
  TUKAR_RB        = 2, // Hardware RGB/BGR Swap
  NEGATIF_TUKAR_RB= 3  // Hardware Invert + RGB/BGR Swap
};

enum class WifiMode : uint8_t {
  OFF    = 0,
  AU_WEB = 1, // Web Control DLMS (Tahap 4)
  AU_OTA = 2  // Firmware OTA Update (Tahap 4)
};

enum class TopMenuId : uint8_t {
  METER  = 0,
  IN_A   = 1,
  IN_B   = 2,
  OUT_1  = 3,
  OUT_2  = 4,
  OUT_3  = 5,
  OUT_4  = 6,
  PRESET = 7,
  SETUP  = 8
};

enum class SetupSubmenu : uint8_t {
  NONE   = 0,
  MATRIX = 1,
  PRESET = 2,
  LEVEL  = 3,
  SINYAL = 4,
  TEMA   = 5,
  WIFI   = 6,
  INPUT  = 7,
  SISTEM = 8
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
  uint16_t rtaBar;
  uint16_t curve;
  uint16_t ok;
  uint16_t warn;
  uint16_t danger;
};

// ============================================================================
// 2. DRIVER LOVYANGFX ST7789 (240x320 SPI2_HOST)
// ============================================================================
class LgfxSt7789 : public lgfx::LGFX_Device {
public:
  LgfxSt7789();
  void applyPanelColorMode(PanelColorMode mode);

private:
  lgfx::Panel_ST7789 panel_;
  lgfx::Bus_SPI      bus_;
};

// ============================================================================
// 3. KELAS UTAMA UI TFT ENGINE (CORE 0 - 30 FPS PSRAM DOUBLE BUFFERING)
// ============================================================================
class UiTft {
public:
  UiTft() = default;
  ~UiTft();

  // Inisialisasi LCD ST7789, 2x Sprite PSRAM, muat pengaturan UI & cache logo LittleFS
  bool begin(audio_io::AudioIO &audioIO,
             dsp::DspEngine &dspEngine,
             ui_ctrl::InputController &inputCtrl);

  // Tampilkan Boot Logo (`/logo_boot.bin` atau vektor fallback) + animasi LED RGB
  void showBootScreen(uint32_t durationMs = 2200);

  // Proses input encoder, cek thermal & auto-standby, render 1 frame (30 FPS Core 0)
  void update(const ui_ctrl::InputEvent &ev,
              const audio_io::AudioTelemetry &audioTel,
              const dsp::DspMeters &dspMeters,
              const dsp::RtaSpectrumData &rtaData);

  // Akses konfigurasi UI & mode WiFi/OTA (untuk integrasi Tahap 4)
  UiConfig getUiConfig() const { return uiCfg_; }
  void applyUiConfig(const UiConfig &cfg, bool persist = true);

  // Status layar utama (memberi tahu InputController kapan HOLD_2S aktif)
  bool isOnHomeScreen() const;

  // Manajemen Preset LittleFS (8 slot: `/presets/slot_0.bin` .. `slot_7.bin`)
  bool savePresetSlot(uint8_t slot, const char *name = nullptr);
  bool loadPresetSlot(uint8_t slot);
  bool deletePresetSlot(uint8_t slot);
  bool renamePresetSlot(uint8_t slot, const char *newName);
  void refreshPresetNames();
  void reloadLogos();

private:
  // Persistence pengaturan UI
  void loadUiConfig();
  void saveUiConfig();

  // Muat file gambar raw RGB565 dari LittleFS ke buffer PSRAM
  bool loadRawImagePsram(const char *path, uint16_t *&dstBuf, size_t &dstPixels);
  void renderFallbackLogo(LGFX_Sprite &sp, const ThemePalette &pal,
                          bool isStandby, float progressOrPulse);

  // Resolusi tema warna aktif (termasuk RGB PELANGI dinamis)
  ThemePalette resolveTheme(uint32_t nowMs) const;
  static uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

  // Input handlers per halaman
  void handleInput(const ui_ctrl::InputEvent &ev);
  void handleHomeInput(const ui_ctrl::InputEvent &ev);
  void handleChannelInput(const ui_ctrl::InputEvent &ev);
  void handlePresetInput(const ui_ctrl::InputEvent &ev);
  void handleSetupInput(const ui_ctrl::InputEvent &ev);

  // Renderers utama
  void drawHeader(LGFX_Sprite &sp, const ThemePalette &pal,
                  const audio_io::AudioTelemetry &audioTel,
                  const dsp::SystemConfig &cfg);
  void drawHorizontalMenuBar(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawLiveVuMeterPage(LGFX_Sprite &sp, const ThemePalette &pal,
                           const dsp::DspMeters &meters,
                           const dsp::SystemConfig &cfg);
  void drawChannelPage(LGFX_Sprite &sp, const ThemePalette &pal,
                       const dsp::DspMeters &meters,
                       const dsp::RtaSpectrumData &rtaData,
                       const dsp::SystemConfig &cfg);
  void drawChannelFreqCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                            int chIdx, bool isOutput,
                            const dsp::RtaSpectrumData &rtaData,
                            const dsp::SystemConfig &cfg);
  void drawChannelDynamicsCurve(LGFX_Sprite &sp, const ThemePalette &pal,
                                int chIdx, bool isOutput,
                                const dsp::DspMeters &meters,
                                const dsp::SystemConfig &cfg);
  void drawPresetPage(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawSetupMenuPage(LGFX_Sprite &sp, const ThemePalette &pal);
  void drawSetupSubPage(LGFX_Sprite &sp, const ThemePalette &pal,
                        const audio_io::AudioTelemetry &audioTel,
                        const dsp::DspMeters &meters,
                        const dsp::SystemConfig &cfg);
  void drawStandbyScreen(LGFX_Sprite &sp, const ThemePalette &pal,
                         const audio_io::AudioTelemetry &audioTel);
  void drawOverheatScreen(LGFX_Sprite &sp, float tempC);
  void drawLockOverlay(LGFX_Sprite &sp, const ThemePalette &pal,
                       const ui_ctrl::InputEvent &ev);

  // Perhitungan respon frekuensi gabungan (XOVER HPF + LPF + 9-Band PEQ) untuk kurva live
  float evalChannelMagDb(int chIdx, bool isOutput, float freqHz,
                         const dsp::SystemConfig &cfg) const;

  // Referensi modul inti
  audio_io::AudioIO        *audioIO_   = nullptr;
  dsp::DspEngine           *dsp_       = nullptr;
  ui_ctrl::InputController *input_     = nullptr;

  // Hardware LCD & PSRAM Double-Buffer Sprites (Ping-Pong)
  LgfxSt7789  lcd_;
  LGFX_Sprite fb_[2];
  uint8_t     curFb_       = 0;
  bool        psramSprites_= false;

  // Cache Logo LittleFS di PSRAM
  uint16_t   *bootLogoBuf_    = nullptr;
  size_t      bootLogoPixels_ = 0;
  uint16_t   *standbyLogoBuf_ = nullptr;
  size_t      standbyLogoPix_ = 0;

  // Konfigurasi UI & State Runtime
  UiConfig    uiCfg_ = {};
  uint32_t    lastActivityMs_     = 0;
  bool        inStandby_          = false;
  bool        overheatActive_     = false;
  bool        overheatDismissed_  = false;
  uint32_t    lockBannerUntilMs_  = 0;

  // State Navigasi Menu
  TopMenuId    activeMenu_      = TopMenuId::METER;
  bool         inDetailPage_    = false; // False: navigasi strip horizontal; True: di dalam halaman
  float        menuScrollX_     = 0.0f;

  // State Halaman Kanal (IN A/B & OUT 1-4)
  uint8_t      chTab_           = 0;     // 0 = XOVER+PEQ+RTA, 1 = DYNAMICS (Comp/Gate/Lim)
  uint8_t      chParamIdx_      = 0;
  bool         chEditingParam_  = false;
  uint8_t      chActivePeqBand_ = 0;     // 0..8 (Band 1..9)

  // State Halaman PRESET & Rename Editor
  uint8_t      presetSlotCursor_ = 0;
  uint8_t      presetAction_     = 0;    // 0=LOAD, 1=SAVE, 2=RENAME
  bool         presetActionMenu_ = false;
  bool         presetRenaming_   = false;
  uint8_t      renameCharPos_    = 0;
  char         renameBuf_[20]    = {0};
  char         presetNames_[NUM_PRESET_SLOTS][20] = {{0}};
  bool         presetExists_[NUM_PRESET_SLOTS]    = {false};
  uint32_t     toastUntilMs_     = 0;
  char         toastMsg_[36]     = {0};

  // State Halaman SETUP & Submenu
  uint8_t      setupCursor_     = 0;
  SetupSubmenu setupSub_        = SetupSubmenu::NONE;
  uint8_t      subCursor_       = 0;
  bool         subEditing_      = false;

  // State VU Meter Smooth & Peak Hold (6 kanal: 0..1=IN A/B, 2..5=OUT 1..4)
  float        vuSmoothDb_[6]     = {-96,-96,-96,-96,-96,-96};
  float        vuPeakHoldDb_[6]   = {-96,-96,-96,-96,-96,-96};
  uint32_t     vuPeakHoldMs_[6]   = {0};
  uint32_t     vuClipHoldMs_[6]   = {0};
  float        grSmoothDb_[6]     = {0};

  // Cache kurva respon frekuensi (73 titik) agar Core 0 tidak hitung ulang tiap 33ms
  static constexpr uint8_t CURVE_POINTS = 73;
  mutable float    curveCacheDb_[CURVE_POINTS] = {0};
  mutable uint32_t curveCacheHash_             = 0;

  void showToast(const char *msg, uint32_t durationMs = 1400);
};

} // namespace ui_tft
