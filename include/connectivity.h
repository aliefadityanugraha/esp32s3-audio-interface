#pragma once

/**
 * @file connectivity.h
 * @brief Tahap 4: Manajemen Radio Eksklusif (BLE NUS vs WiFi AU-WEB vs WiFi AU-OTA)
 *        berjalan di Core 0 tanpa mengganggu real-time DSP Core 1.
 *
 * Mode Radio:
 *  1. BLE_NUS (Default):
 *     - Nordic UART Service (NUS) dengan Nama Perangkat & PIN Pairing 6-digit di NVS.
 *     - WiFi mati total (`WIFI_OFF`).
 *  2. WIFI_WEB (`AU-WEB`):
 *     - BLE dimatikan & di-deinit penuh (`BLEDevice::deinit(true)`) untuk hemat RAM.
 *     - WiFi SoftAP "AU-WEB" (192.168.4.1), HTTP Server menyajikan `/index.html` LittleFS,
 *       dan WebSocket Server (port 81) untuk kontrol & stream telemetry live (50-100ms).
 *  3. WIFI_OTA (`AU-OTA`):
 *     - Audio output di-mute mulus & Task Audio DSP Core 1 di-suspend aman.
 *     - BLE di-deinit penuh.
 *     - WiFi SoftAP "AU-OTA" (192.168.4.1) menyajikan portal web untuk upload firmware
 *       `.bin` (`Update.h`) serta upload logo custom (`/logo_boot.bin`, `/logo_standby.bin`)
 *       ke LittleFS.
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <cstdint>

#include "audio_io.h"
#include "cmd_parser.h"
#include "dsp_engine.h"
#include "input_ctrl.h"
#include "ui_tft.h"

// Forward declarations Bluedroid BLE agar header tetap bersih
class BLEServer;
class BLECharacteristic;
class BLEServerCallbacks;
class BLECharacteristicCallbacks;
class BLESecurityCallbacks;

namespace conn {

constexpr uint16_t HTTP_SERVER_PORT   = 80;
constexpr uint16_t WS_SERVER_PORT     = 81;
constexpr const char *AP_SSID_WEB     = "AU-WEB";
constexpr const char *AP_SSID_OTA     = "AU-OTA";

enum class RadioMode : uint8_t {
  OFF      = 0,
  BLE_NUS  = 1,
  WIFI_WEB = 2,
  WIFI_OTA = 3
};

struct RadioConfig {
  char     bleName[24];
  uint32_t blePin;          // 6-digit PIN pairing (000000 - 999999)
  char     wifiWebPass[24]; // Password AP AU-WEB (minimal 8 karakter, "" = open)
  char     wifiOtaPass[24]; // Password AP AU-OTA
};

class ConnectivityManager {
public:
  ConnectivityManager() = default;
  ~ConnectivityManager();

  // Inisialisasi NVS radio, CommandParser, dan nyalakan mode default (BLE NUS)
  void begin(audio_io::AudioIO &audioIO,
             dsp::DspEngine &dspEngine,
             ui_tft::UiTft &uiTft,
             TaskHandle_t audioTaskHandle);

  // Dipanggil berkala dari task Core 0: sync mode radio dari UI, proses HTTP/WS/BLE,
  // serta kirim stream telemetri otomatis tiap 50-100 ms saat klien terhubung.
  void update(const audio_io::AudioTelemetry &audioTel,
              const dsp::DspMeters &dspMet,
              const dsp::RtaSpectrumData &rtaData);

  // Transisi mode radio secara eksplisit & aman
  void setRadioMode(RadioMode newMode);
  RadioMode getRadioMode() const { return activeMode_; }

  // Konfigurasi NVS Nama BLE & PIN 6-digit
  RadioConfig getRadioConfig() const { return radioCfg_; }
  void setBleCredentials(const char *name, uint32_t pin6Digits);

  // Status klien terhubung
  bool isBleClientConnected() const { return bleConnected_; }
  bool isWsClientConnected() const { return wsClientCount_ > 0; }

  // Callback internal dari BLE & WebSocket (dipanggil dari class callback friend/static)
  void onBleConnect();
  void onBleDisconnect();
  void onBleRxData(const uint8_t *data, size_t len);
  void onWsEvent(uint8_t clientNum, WStype_t type, uint8_t *payload, size_t length);

private:
  void loadRadioConfig();
  void saveRadioConfig();

  // Manajemen transisi radio
  void startBleNus();
  void stopBleNus();
  void startWifiWeb();
  void startWifiOta();
  void stopWifiAll();

  // Proteksi Audio DSP saat masuk/keluar mode OTA
  void suspendAudioForOta();
  void resumeAudioAfterOta();

  // Pengiriman data ber-chunk via BLE NUS Notify (aman untuk MTU 20..512 byte)
  void sendBleNotifyChunked(const char *data, size_t len);

  // Setup endpoint HTTP untuk AU-WEB dan AU-OTA
  void registerWebRoutes();
  void registerOtaRoutes();
  static const char *mimeTypeFromPath(const char *path);

  // Eksekusi perintah yang masuk dari antrean BLE di konteks task Core 0 (bukan ISR/BT task)
  void processQueuedBleRx();

  // Referensi modul inti
  audio_io::AudioIO *audioIO_         = nullptr;
  dsp::DspEngine    *dsp_             = nullptr;
  ui_tft::UiTft     *ui_              = nullptr;
  TaskHandle_t       audioTaskHandle_ = nullptr;

  CommandParser parser_;
  RadioConfig   radioCfg_   = {};
  RadioMode     activeMode_ = RadioMode::OFF;

  // Audio mute/suspend state saat OTA
  bool audioSuspendedForOta_ = false;
  bool preOtaMasterMute_     = false;

  // BLE NUS State & Callback ownership (bebas memory leak saat toggle BLE <-> WiFi)
  BLEServer                  *bleServer_   = nullptr;
  BLECharacteristic          *bleTxChar_   = nullptr;
  BLECharacteristic          *bleRxChar_   = nullptr;
  BLEServerCallbacks         *bleServerCb_ = nullptr;
  BLECharacteristicCallbacks *bleRxCb_     = nullptr;
  BLESecurityCallbacks       *bleSecCb_    = nullptr;
  volatile bool               bleConnected_= false;
  bool                        bleActive_   = false;

  // Antrean RX BLE -> Core 0 Task agar akses LittleFS/UI 100% thread-safe
  portMUX_TYPE bleRxMux_ = portMUX_INITIALIZER_UNLOCKED;
  uint8_t      bleRxQueue_[1024] = {0};
  size_t       bleRxLen_         = 0;
  bool         bleRxIsBinary_    = false;

  // WiFi HTTP + WebSocket Server
  WebServer        *httpServer_    = nullptr;
  WebSocketsServer *wsServer_      = nullptr;
  uint8_t           wsClientCount_ = 0;

  // State Upload Logo pada halaman AU-OTA
  File otaLogoFile_;
  char otaLogoFinalPath_[32] = {0};

  // Telemetry Stream Timer
  uint32_t lastTelemMs_  = 0;
  char     telemBuf_[640]= {0};
};

} // namespace conn
