/**
 * @file main.cpp
 * @brief Entry Point Lengkap Tahap 1 + 2 + 3 + 4:
 *        ESP32-S3 N16R8 Hybrid USB Audio Interface (UAC2) + DLMS 2-In 4-Out
 *
 * Arsitektur Dual-Core FreeRTOS:
 *  - Core 1 (Prioritas 6): AudioDSP_Core1 -> I2S Dual-Port Phase-Sync + USB UAC2 + DLMS 2x4 DSP
 *  - Core 0 (Prioritas 2): UiTft_Core0    -> LovyanGFX ILI9341 30 FPS (2x PSRAM Sprite),
 *                                            Rotary Encoder + Panel Lock, LED RGB, RTA FFT, Suhu
 *  - Core 0 (Prioritas 1): Conn_Core0     -> Radio Manager (BLE NUS / AU-WEB / AU-OTA),
 *                                            Command Parser, & Stream Telemetry 80 ms
 */

#include <Arduino.h>
#include <LittleFS.h>

#include "audio_config.h"
#include "audio_io.h"
#include "connectivity.h"
#include "dsp_engine.h"
#include "input_ctrl.h"
#include "pin_config.h"
#include "ui_tft.h"

using namespace audio_io;
using namespace dsp;
using namespace ui_ctrl;
using namespace ui_tft;
using namespace conn;

// ============================================================================
// OBJEK GLOBAL SISTEM
// ============================================================================
static AudioIO             gAudioIO;
static DspEngine           gDspEngine;
static InputController     gInputCtrl;
static UiTft               gUiTft;
static ConnectivityManager gConnMgr;

static TaskHandle_t        gAudioTaskHandle = nullptr;
static volatile bool       gUiReady         = false;

// ============================================================================
// CORE 1: REAL-TIME AUDIO I/O & DLMS DSP TASK (PRIORITAS TERTINGGI)
// ============================================================================
static void audioTask(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    gAudioIO.processOneBlock();
  }
}

// ============================================================================
// CORE 0 (PRIO 2): UI TFT 30 FPS (PSRAM DOUBLE BUFFER), ENCODER, LED RGB, SUHU
// ============================================================================
static void uiTask(void *pvParameters) {
  (void)pvParameters;

  bool uiOk = gUiTft.begin(gAudioIO, gDspEngine, gInputCtrl);
  Serial.printf("[UI ] ILI9341 LovyanGFX + 2x PSRAM Sprite: %s\n", uiOk ? "OK" : "FAILED!");

  // Tampilkan Boot Logo (`/logo_boot.bin` atau fallback vektor) + animasi LED RGB
  gUiTft.showBootScreen(2200);
  gUiReady = true;

  constexpr uint32_t FRAME_PERIOD_MS = 33; // Target ~30 FPS
  constexpr uint32_t MIN_YIELD_MS    = 10; // Wajib yield >=10ms agar IDLE0 & connTask tidak pernah TWDT panic

  for (;;) {
    const uint32_t frameStartMs = millis();

    // 1. Pembacaan periodik sensor suhu internal ESP32-S3 (tiap 1 detik)
    gAudioIO.updateTemperature(TEMP_READ_INTERVAL_MS);

    // 2. Poll Rotary Encoder & State Machine Tombol (termasuk Panel Lock 3 detik)
    InputEvent ev = gInputCtrl.poll(gUiTft.isOnHomeScreen());

    // 3. Ambil snapshot telemetri Audio I/O & Meter DSP
    AudioTelemetry audioTel = gAudioIO.getTelemetry();
    DspMeters      dspMet   = gDspEngine.getMeters();
    SystemConfig   dspCfg   = gDspEngine.getConfig();

    // 4. Update LED RGB Status (GPIO 48)
    gInputCtrl.updateOperatingLed(audioTel, dspMet, dspCfg, gInputCtrl.isPanelLocked());

    // 5. Saat proses tulis OTA Flash sedang berlangsung, jeda render 30 FPS SPI/PSRAM
    //    agar Core 0 tidak memblokir IDLE0 (TWDT) dan bebas lonjakan arus/kontensi cache Flash.
    if (gConnMgr.isOtaFlashing()) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    // 6. Render 1 frame UI ILI9341 (PSRAM Double-Buffering)
    gUiTft.update(ev, audioTel, dspMet);

    // 7. Pastikan Core 0 selalu memberi jatah ke IDLE0 (mencegah TWDT 5s) & connTask
    const uint32_t elapsedMs = millis() - frameStartMs;
    const uint32_t sleepMs =
        (elapsedMs + MIN_YIELD_MS < FRAME_PERIOD_MS) ? (FRAME_PERIOD_MS - elapsedMs) : MIN_YIELD_MS;
    vTaskDelay(pdMS_TO_TICKS(sleepMs));
  }
}

// ============================================================================
// CORE 0 (PRIO 1): KONEKTIVITAS RADIO (BLE NUS / AU-WEB / AU-OTA) & TELEMETRY
// ============================================================================
static void connTask(void *pvParameters) {
  (void)pvParameters;

  // Tunggu UI selesai memuat config & boot screen agar startup rapi
  while (!gUiReady) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  gConnMgr.begin(gAudioIO, gDspEngine, gUiTft, gAudioTaskHandle);

  for (;;) {
    AudioTelemetry audioTel = gAudioIO.getTelemetry();
    DspMeters      dspMet   = gDspEngine.getMeters();

    gConnMgr.update(audioTel, dspMet);
    vTaskDelay(pdMS_TO_TICKS(10)); // Responsif untuk WebSocket, HTTP, & BLE NUS
  }
}

// ============================================================================
// ARDUINO SETUP & LOOP
// ============================================================================
void setup() {
  // 1. Inisialisasi USB Audio UAC2 PERTAMA KALI agar enumerasi host PC tidak timeout
  bool usbOk = gAudioIO.beginUSB(USB_AUDIO_MANUFACTURER, USB_AUDIO_PRODUCT);

  // 2. Serial Debug pada UART0 (CH340/CP2102)
  Serial.begin(115200);
  delay(120);
  Serial.println("\n================================================================");
  Serial.println(" ESP32-S3 N16R8 Hybrid USB Audio Interface + DLMS 2-In 4-Out");
  Serial.println(" Tahap 1-4: Audio I/O + DSP + ILI9341 UI + BLE/AU-WEB/AU-OTA");
  Serial.println("================================================================");

  // 3. Verifikasi Hardware Memori & Alasan Reset terakhir
  Serial.printf("[SYS] Reset Rsn  : %d (9=BROWNOUT, 1=POWERON, 3=SW)\n",
                static_cast<int>(esp_reset_reason()));
  Serial.printf("[SYS] CPU Freq   : %lu MHz\n", static_cast<unsigned long>(getCpuFrequencyMhz()));
  Serial.printf("[SYS] Flash Size : %.2f MB\n", ESP.getFlashChipSize() / (1024.0f * 1024.0f));
  if (psramFound()) {
    Serial.printf("[SYS] OPI PSRAM  : OK (Total %.2f MB, Free %.2f MB)\n",
                  ESP.getPsramSize() / (1024.0f * 1024.0f),
                  ESP.getFreePsram() / (1024.0f * 1024.0f));
  } else {
    Serial.println("[SYS] OPI PSRAM  : TIDAK TERDETEKSI! Periksa memory_type = qio_opi");
  }

  // 4. Mount LittleFS 8MB (Logo Boot/Standby, Preset DLMS, Web App /index.html)
  if (LittleFS.begin(true)) {
    Serial.printf("[FS ] LittleFS   : OK (Total %.2f MB, Used %.2f KB)\n",
                  LittleFS.totalBytes() / (1024.0f * 1024.0f),
                  LittleFS.usedBytes() / 1024.0f);
  } else {
    Serial.println("[FS ] LittleFS   : GAGAL mount!");
  }

  // 5. Inisialisasi Kontrol Fisik Encoder (GPIO 16, 17, 18) & LED Status (GPIO 48)
  gInputCtrl.begin();

  // 6. Inisialisasi DLMS DSP Engine (alokasi 4x300ms Delay Line di PSRAM)
  bool dspOk = gDspEngine.begin(static_cast<float>(AUDIO_SAMPLE_RATE));
  gAudioIO.setProcessCallback(&DspEngine::audioCallbackAdapter, &gDspEngine);
  Serial.printf("[DSP] DLMS 2x4 Engine Init: %s\n", dspOk ? "OK" : "FAILED!");

  // 7. Inisialisasi Sensor Suhu Internal ESP32-S3 (sebelum start I2S agar tidak memakan preload DMA)
  bool tempOk = gAudioIO.beginTempSensor();
  Serial.printf("[TMP] Internal Temp Sensor                     : %s\n", tempOk ? "OK" : "FAILED!");
  Serial.printf("[USB] UAC2 48kHz 16-bit 2x2 Init               : %s\n", usbOk ? "OK" : "FAILED!");

  // 8. Inisialisasi I2S Dual-Port 100% Sinkron Fase tepat sebelum audioTask jalan
  //    agar 4 buffer preload DMA langsung diteruskan Core 1 tanpa underrun startup
  bool i2sOk = gAudioIO.beginI2S();

  // 9. Jalankan Arsitektur Dual-Core FreeRTOS Tasks
  xTaskCreatePinnedToCore(audioTask, "AudioDSP_Core1", AUDIO_TASK_STACK_BYTES,
                          nullptr, AUDIO_TASK_PRIORITY, &gAudioTaskHandle, AUDIO_TASK_CORE);
  Serial.printf("[I2S] Dual-Port Sync (I2S0 Master + I2S1 Slave): %s\n", i2sOk ? "OK" : "FAILED!");

  xTaskCreatePinnedToCore(uiTask, "UiTft_Core0", 12288,
                          nullptr, 2, nullptr, SYS_TASK_CORE);

  xTaskCreatePinnedToCore(connTask, "Conn_Core0", 10240,
                          nullptr, 1, nullptr, SYS_TASK_CORE);
}

void loop() {
  // Jangan spam log Serial saat proses tulis OTA Flash sedang berlangsung
  if (gConnMgr.isOtaFlashing()) {
    vTaskDelay(pdMS_TO_TICKS(500));
    return;
  }

  // Log diagnosa audio/DSP ringkas tiap 3 detik pada UART0 untuk verifikasi live
  const AudioTelemetry tel = gAudioIO.getTelemetry();
  const DspMeters      met = gDspEngine.getMeters();
  const SystemConfig   cfg = gDspEngine.getConfig();
  Serial.printf(
      "[AUD] CPU:%.1f%% | USB:%d/R%d/T%d RB:%u(%.0f/%.0fdB) | IN:%.0f/%.0fdB | OUT:%.0f/%.0f/%.0f/%.0fdB | M:%s\n",
      tel.dspCpuLoadPct,
      tel.usbMounted ? 1 : 0,
      tel.usbRxStreaming ? 1 : 0,
      tel.usbTxStreaming ? 1 : 0,
      static_cast<unsigned>(tel.rbFillFrames),
      AudioIO::linearToDbfs(tel.peakUsbIn[0]),
      AudioIO::linearToDbfs(tel.peakUsbIn[1]),
      met.inPeakDb[0], met.inPeakDb[1],
      met.outPeakDb[0], met.outPeakDb[1], met.outPeakDb[2], met.outPeakDb[3],
      cfg.master.mute ? "MUTE" : "ON");
  vTaskDelay(pdMS_TO_TICKS(3000));
}
