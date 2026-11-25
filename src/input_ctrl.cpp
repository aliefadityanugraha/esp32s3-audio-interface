/**
 * @file input_ctrl.cpp
 * @brief Implementasi Kontrol Rotary Encoder, Panel Lock, & LED RGB Status (GPIO 48)
 */

#include "input_ctrl.h"

#include <cmath>

namespace ui_ctrl {

namespace {

constexpr uint32_t DEBOUNCE_MS         = 30;
constexpr uint32_t CLICK_MAX_MS        = 350;
constexpr uint32_t SHORT_HOLD_FIRE_MS  = 500;  // < 1 detik: langsung trigger BACK di submenu
constexpr uint32_t SHORT_HOLD_MAX_MS   = 1000; // < 1 detik: batas tahan singkat di home screen
constexpr uint32_t HOLD_2S_MS          = 2000; // 2 detik di layar utama: masuk edit kanal aktif
constexpr uint32_t UNLOCK_3S_MS        = 3000; // 3 detik saat terkunci: buka Panel Lock

} // namespace

void InputController::begin() {
  pinMode(PIN_ENC_SW,  INPUT_PULLUP);
  pinMode(PIN_ENC_DT,  INPUT_PULLUP);
  pinMode(PIN_ENC_CLK, INPUT_PULLUP);

  encoder_.attachHalfQuad(PIN_ENC_DT, PIN_ENC_CLK);
  encoder_.clearCount();
  lastEncCount_ = 0;
  lastStepMs_   = millis();

  btnRawPrev_       = (digitalRead(PIN_ENC_SW) == LOW);
  btnStable_        = btnRawPrev_;
  btnDebounceMs_    = millis();
  btnPressStartMs_  = 0;
  btnHoldTriggered_ = false;
  panelLocked_      = false;
}

void InputController::setPanelLocked(bool locked) {
  panelLocked_ = locked;
}

InputEvent InputController::poll(bool isHomeScreen) {
  InputEvent ev = {};
  ev.panelLocked = panelLocked_;

  const uint32_t nowMs = millis();

  // ---- 1. Pembacaan Rotary Encoder (Half-Quad: 2 pulsa = 1 detent) ----
  const int64_t curCount = encoder_.getCount() / 2;
  const int64_t rawDiff  = curCount - lastEncCount_;
  lastEncCount_ = curCount;

  if (rawDiff != 0) {
    ev.anyActivity = true;
    if (panelLocked_) {
      // Saat Panel Lock aktif: abaikan putaran dan kirim feedback terkunci
      ev.delta      = 0;
      ev.accelDelta = 0;
      ev.button     = ButtonEvent::LOCKED_BLOCKED;
    } else {
      const int16_t step = static_cast<int16_t>(rawDiff);
      ev.delta = step;

      // Akselerasi halus saat diputar cepat (< 45 ms antar klik) untuk Freq/Delay
      const uint32_t dt = nowMs - lastStepMs_;
      int16_t mult = 1;
      if (dt < 25)      mult = 8;
      else if (dt < 50) mult = 4;
      else if (dt < 85) mult = 2;
      ev.accelDelta = static_cast<int16_t>(step * mult);
      lastStepMs_   = nowMs;
    }
  }

  // ---- 2. Debounce & State Machine Tombol Encoder (Active LOW) ----
  const bool rawPressed = (digitalRead(PIN_ENC_SW) == LOW);
  if (rawPressed != btnRawPrev_) {
    btnRawPrev_    = rawPressed;
    btnDebounceMs_ = nowMs;
  }

  const bool prevStable = btnStable_;
  if ((nowMs - btnDebounceMs_) >= DEBOUNCE_MS) {
    btnStable_ = rawPressed;
  }

  if (btnStable_ && !prevStable) {
    // Rising edge: tombol baru ditekan
    btnPressStartMs_  = nowMs;
    btnHoldTriggered_ = false;
    ev.anyActivity    = true;
  } else if (btnStable_ && prevStable) {
    // Tombol sedang ditahan
    const uint32_t holdMs = nowMs - btnPressStartMs_;
    ev.isHoldingBtn   = true;
    ev.holdDurationMs = holdMs;
    ev.anyActivity    = true;

    if (panelLocked_) {
      // Mode Panel Lock: HANYA buka kunci jika ditahan >= 3 detik penuh
      if (!btnHoldTriggered_ && holdMs >= UNLOCK_3S_MS) {
        btnHoldTriggered_ = true;
        panelLocked_      = false;
        ev.panelLocked    = false;
        ev.button         = ButtonEvent::UNLOCK_3S;
      }
    } else {
      if (isHomeScreen) {
        // Di layar utama: tahan 2 detik -> langsung masuk edit kanal aktif
        if (!btnHoldTriggered_ && holdMs >= HOLD_2S_MS) {
          btnHoldTriggered_ = true;
          ev.button         = ButtonEvent::HOLD_2S;
        }
      } else {
        // Di dalam submenu / mode edit: tahan singkat (~500 ms, < 1 detik) -> BACK
        if (!btnHoldTriggered_ && holdMs >= SHORT_HOLD_FIRE_MS) {
          btnHoldTriggered_ = true;
          ev.button         = ButtonEvent::SHORT_HOLD;
        }
      }
    }
  } else if (!btnStable_ && prevStable) {
    // Falling edge: tombol dilepas
    const uint32_t pressDur = nowMs - btnPressStartMs_;
    ev.anyActivity = true;

    if (!btnHoldTriggered_) {
      if (panelLocked_) {
        ev.button = ButtonEvent::LOCKED_BLOCKED;
      } else if (pressDur < CLICK_MAX_MS) {
        ev.button = ButtonEvent::CLICK;
      } else if (pressDur < SHORT_HOLD_MAX_MS) {
        ev.button = ButtonEvent::SHORT_HOLD;
      }
    }
  }

  return ev;
}

// ============================================================================
// KONTROL LED RGB STATUS (WS2812 BAWAAN GPIO 48)
// ============================================================================
void InputController::setLedRgb(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    rgbLedWrite(PIN_LED_STATUS, r, g, b);
  #else
    neopixelWrite(PIN_LED_STATUS, r, g, b);
  #endif
#else
  neopixelWrite(PIN_LED_STATUS, r, g, b);
#endif
}

void InputController::hsvToRgb(uint16_t hue, uint8_t sat, uint8_t val,
                               uint8_t &r, uint8_t &g, uint8_t &b) {
  hue %= 360;
  const uint8_t region = static_cast<uint8_t>(hue / 60);
  const uint8_t rem    = static_cast<uint8_t>(((hue % 60) * 255) / 60);

  const uint8_t p = static_cast<uint8_t>((val * (255 - sat)) >> 8);
  const uint8_t q = static_cast<uint8_t>((val * (255 - ((sat * rem) >> 8))) >> 8);
  const uint8_t t = static_cast<uint8_t>((val * (255 - ((sat * (255 - rem)) >> 8))) >> 8);

  switch (region) {
    case 0:  r = val; g = t;   b = p;   break;
    case 1:  r = q;   g = val; b = p;   break;
    case 2:  r = p;   g = val; b = t;   break;
    case 3:  r = p;   g = q;   b = val; break;
    case 4:  r = t;   g = p;   b = val; break;
    default: r = val; g = p;   b = q;   break;
  }
}

void InputController::updateBootAnimation(uint32_t elapsedMs) {
  // Animasi pelangi halus + breathing brightness selama boot logo
  const uint16_t hue = static_cast<uint16_t>((elapsedMs / 5) % 360);
  const float wave   = 0.5f + 0.5f * sinf(static_cast<float>(elapsedMs) * 0.006f);
  const uint8_t val  = static_cast<uint8_t>(18.0f + wave * 62.0f);

  uint8_t r = 0, g = 0, b = 0;
  hsvToRgb(hue, 255, val, r, g, b);
  setLedRgb(r, g, b);
}

void InputController::updateOperatingLed(const audio_io::AudioTelemetry &audioTel,
                                         const dsp::DspMeters &dspMeters,
                                         const dsp::SystemConfig &dspCfg,
                                         bool panelLocked) {
  const uint32_t nowMs = millis();
  if (nowMs - lastLedUpdateMs_ < 33) return; // Sync ~30 Hz
  lastLedUpdateMs_ = nowMs;

  const bool fastBlink = ((nowMs / 150) & 1u) != 0u;
  const bool slowPulse = ((nowMs / 600) & 1u) != 0u;

  const bool anyClip = dspMeters.inClip[0] || dspMeters.inClip[1] ||
                       dspMeters.outClip[0] || dspMeters.outClip[1] ||
                       dspMeters.outClip[2] || dspMeters.outClip[3];

  const bool anyLimiter = dspMeters.outLimiterActive[0] || dspMeters.outLimiterActive[1] ||
                          dspMeters.outLimiterActive[2] || dspMeters.outLimiterActive[3];

  const bool allMuted = dspCfg.master.mute ||
                        (dspCfg.outputs[0].mute && dspCfg.outputs[1].mute &&
                         dspCfg.outputs[2].mute && dspCfg.outputs[3].mute);

  // Prioritas 1: Error I2S atau Suhu >= 75°C (Merah Terang Berkedip)
  if (!audioTel.i2sReady || audioTel.chipTempC >= TEMP_WARN_THRESHOLD_C) {
    setLedRgb(fastBlink ? 110 : 12, 0, 0);
    return;
  }

  // Prioritas 2: Indikator CLIP (Merah Terang Solid)
  if (anyClip) {
    setLedRgb(115, 0, 0);
    return;
  }

  // Prioritas 3: Indikator LIMITER Bekerja (Oranye/Kuning Terang)
  if (anyLimiter) {
    setLedRgb(95, 52, 0);
    return;
  }

  // Prioritas 4: Mode MUTE -> LED Redup
  if (allMuted) {
    setLedRgb(8, 0, 6);
    return;
  }

  // Prioritas 7: Panel Lock Indikasi Halus / Normal Operasional
  if (panelLocked) {
    setLedRgb(16, 12, 0);
  } else if (audioTel.usbMounted) {
    setLedRgb(0, audioTel.usbRxStreaming ? 24 : 14, audioTel.usbRxStreaming ? 18 : 0);
  } else {
    setLedRgb(0, 0, slowPulse ? 18 : 6);
  }
}

} // namespace ui_ctrl
