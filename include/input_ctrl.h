#pragma once

/**
 * @file input_ctrl.h
 * @brief Tahap 3: Kontrol Fisik Rotary Encoder (GPIO 16, 17, 18), Panel Lock,
 *        dan Kontrol LED RGB WS2812 Bawaan (GPIO 48) di Core 0.
 *
 * Aturan Interaksi Encoder:
 *  - Putar          : Navigasi menu / ubah nilai (dengan akselerasi kecepatan putar)
 *  - Klik (<350 ms) : Pilih / konfirmasi / toggle edit
 *  - Tahan singkat (<1 detik) : Kembali (BACK / keluar edit)
 *  - Tahan 2 detik (dari layar utama) : Langsung masuk edit kanal aktif
 *  - Panel Lock     : Abaikan seluruh input kecuali tahan terus >= 3 detik untuk membuka kunci
 */

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <cstdint>

#include "audio_io.h"
#include "dsp_engine.h"
#include "pin_config.h"

namespace ui_ctrl {

enum class ButtonEvent : uint8_t {
  NONE           = 0,
  CLICK          = 1, // Klik singkat (< 350 ms): Pilih / Edit
  SHORT_HOLD     = 2, // Tahan singkat (< 1 detik): Kembali (Back)
  HOLD_2S        = 3, // Tahan 2 detik di layar utama: Quick-Edit kanal aktif
  UNLOCK_3S      = 4, // Tahan 3 detik saat Panel Lock: Buka Kunci
  LOCKED_BLOCKED = 5  // Input diblokir karena Panel Lock aktif (tampilkan ikon gembok)
};

struct InputEvent {
  int16_t     delta;          // Langkah putaran (+CW / -CCW), 0 jika terkunci
  int16_t     accelDelta;     // Langkah dengan akselerasi saat diputar cepat
  ButtonEvent button;         // Event tombol yang terjadi pada frame ini
  bool        anyActivity;    // True jika ada aktivitas fisik (untuk reset timer Auto-Standby)
  bool        panelLocked;    // Status Panel Lock terkini
  bool        isHoldingBtn;   // True saat tombol sedang ditekan
  uint32_t    holdDurationMs; // Lama penahanan tombol saat ini (ms)
};

class InputController {
public:
  InputController() = default;

  void begin();

  // Polling encoder & state machine tombol (dipanggil pada 30 FPS di Core 0)
  // `isHomeScreen`: true jika sedang di layar utama/menu utama agar HOLD_2S aktif
  InputEvent poll(bool isHomeScreen);

  // Panel Lock control
  void setPanelLocked(bool locked);
  bool isPanelLocked() const { return panelLocked_; }

  // LED Status WS2812 (GPIO 48)
  void setLedRgb(uint8_t r, uint8_t g, uint8_t b);
  void updateBootAnimation(uint32_t elapsedMs);
  void updateOperatingLed(const audio_io::AudioTelemetry &audioTel,
                          const dsp::DspMeters &dspMeters,
                          const dsp::SystemConfig &dspCfg,
                          bool panelLocked);

  // Helper konversi HSV (hue 0..359, sat 0..255, val 0..255) -> RGB888
  static void hsvToRgb(uint16_t hue, uint8_t sat, uint8_t val,
                       uint8_t &r, uint8_t &g, uint8_t &b);

private:
  ESP32Encoder encoder_;
  int64_t  lastEncCount_     = 0;
  uint32_t lastStepMs_       = 0;

  bool     panelLocked_      = false;
  bool     btnRawPrev_       = false;
  bool     btnStable_        = false;
  uint32_t btnDebounceMs_    = 0;
  uint32_t btnPressStartMs_  = 0;
  bool     btnHoldTriggered_ = false;
  uint32_t lastLedUpdateMs_  = 0;
};

} // namespace ui_ctrl
