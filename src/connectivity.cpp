/**
 * @file connectivity.cpp
 * @brief Implementasi Manajemen Radio (BLE NUS vs WiFi AU-WEB vs WiFi AU-OTA)
 * di Core 0
 */

#include "connectivity.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLESecurity.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <LittleFS.h>
#include <Update.h>
#include <cstring>
#include <esp_system.h>

namespace conn {

namespace {

// Nordic UART Service (NUS) Standard 128-bit UUIDs
constexpr const char *NUS_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *NUS_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *NUS_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr const char *NVS_RADIO_NS = "radio_cfg";

// Pointer instance singleton untuk jembatan callback C/BLE/WebSocket
ConnectivityManager *gConnInstance = nullptr;

// Halaman fallback jika `/index.html` belum diunggah ke LittleFS saat mode
// AU-WEB
static const char FALLBACK_WEB_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>DSP Audio Interface - AU-WEB</title>
<style>body{font-family:system-ui,sans-serif;background:#0b1220;color:#e8f1ff;margin:16px}
.box{background:#132038;border:1px solid #2b456e;border-radius:10px;padding:14px;margin-bottom:12px}
input,button{padding:8px 10px;border-radius:6px;border:1px solid #35588a;background:#0d172a;color:#fff}
button{background:#0ea5e9;border:none;cursor:pointer;font-weight:600}pre{background:#070c16;padding:10px;border-radius:6px;max-height:240px;overflow:auto;font-size:12px}</style></head>
<body><h2>DSP Audio Interface - AU-WEB</h2>
<div class="box"><p>File <code>/index.html</code> belum ditemukan di LittleFS. Gunakan konsol WebSocket cepat di bawah atau unggah Web App lewat mode <b>AU-OTA</b>.</p>
<input id="cmd" value="INFO" style="width:65%"> <button onclick="send()">Kirim</button> <button onclick="ws.send('DUMP')">DUMP</button></div>
<div class="box"><b>Live Telemetry:</b><pre id="tel">Menghubungkan WebSocket...</pre><b>Log Respons:</b><pre id="log"></pre></div>
<script>
const ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=()=>document.getElementById('tel').textContent='WebSocket Terhubung!';
ws.onmessage=(e)=>{if(e.data.startsWith('{"t":"M"'))document.getElementById('tel').textContent=e.data;
else document.getElementById('log').textContent=e.data+document.getElementById('log').textContent.slice(0,2000);};
function send(){ws.send(document.getElementById('cmd').value);}
</script></body></html>)HTML";

// Halaman Portal OTA & Upload Logo untuk mode AU-OTA (192.168.4.1)
static const char OTA_PORTAL_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="id"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AU-OTA Firmware & Logo Updater</title>
<style>body{font-family:system-ui,sans-serif;background:#120d16;color:#f5f0ff;margin:16px;max-width:560px}
.card{background:#1f1628;border:1px solid #4d2e68;border-radius:10px;padding:16px;margin-bottom:14px}
h2{color:#ffcb2b;margin-top:0}h3{margin:0 0 10px 0;color:#c084fc}
input[type=file]{display:block;margin:8px 0 12px 0;color:#ddd}
button{background:#ff3b5c;color:#fff;border:none;padding:9px 16px;border-radius:6px;font-weight:700;cursor:pointer}
button.alt{background:#3b82f6}progress{width:100%;height:14px;margin-top:8px}
.note{font-size:12px;color:#b8a5cc}</style></head>
<body><h2>AU-OTA Portal - DSP Audio Interface</h2>
<p class="note">Audio DSP dihentikan aman (muted) selama mode AU-OTA aktif.</p>
<div class="card"><h3>1. Update Firmware (.bin)</h3>
<input type="file" id="fwFile" accept=".bin">
<button onclick="upload('/ota/firmware','fwFile','fwProg','fwSt')">Upload Firmware</button>
<progress id="fwProg" value="0" max="100"></progress><div id="fwSt" class="note"></div></div>

<div class="card"><h3>2. Upload Boot Logo (/logo_boot.bin)</h3>
<p class="note">Format Raw RGB565 320x240 (153.600 byte)</p>
<input type="file" id="bootFile" accept=".bin">
<button class="alt" onclick="upload('/ota/logo?target=boot','bootFile','bProg','bSt')">Upload Boot Logo</button>
<progress id="bProg" value="0" max="100"></progress><div id="bSt" class="note"></div></div>

<div class="card"><h3>3. Upload Standby Logo (/logo_standby.bin)</h3>
<input type="file" id="stbyFile" accept=".bin">
<button class="alt" onclick="upload('/ota/logo?target=standby','stbyFile','sProg','sSt')">Upload Standby Logo</button>
<progress id="sProg" value="0" max="100"></progress><div id="sSt" class="note"></div></div>

<div class="card"><h3>4. Upload Web App (/index.html)</h3>
<input type="file" id="webFile" accept=".html,.htm">
<button class="alt" onclick="upload('/ota/webapp','webFile','wProg','wSt')">Upload index.html</button>
<progress id="wProg" value="0" max="100"></progress><div id="wSt" class="note"></div></div>

<div class="card"><button onclick="fetch('/ota/reboot',{method:'POST'}).then(()=>alert('Rebooting...'))">Reboot Perangkat</button></div>
<script>
let busy=false;
function upload(url,fid,pid,sid){
  if(busy){alert('Upload lain masih berlangsung!');return;}
  const f=document.getElementById(fid).files[0]; if(!f){alert('Pilih file dulu!');return;}
  busy=true;
  const st=document.getElementById(sid), pr=document.getElementById(pid);
  pr.value=0; st.textContent='Mengunggah 0% ('+f.size+' byte)... Jangan matikan daya.';
  const sep=url.includes('?')?'&':'?';
  const fd=new FormData(); fd.append('file',f);
  const xhr=new XMLHttpRequest(); xhr.open('POST',url+sep+'size='+f.size,true);
  xhr.upload.onprogress=(e)=>{
    if(e.lengthComputable){
      const pct=Math.round((e.loaded/e.total)*100);
      pr.value=pct;
      st.textContent=(pct<100)?('Mengunggah '+pct+'% ('+e.loaded+'/'+e.total+' byte)...'):'Memverifikasi Flash...';
    }
  };
  xhr.onload=()=>{busy=false;st.textContent=xhr.responseText||('HTTP '+xhr.status);};
  xhr.onerror=()=>{busy=false;st.textContent='Upload gagal / koneksi terputus!';};
  xhr.onabort=()=>{busy=false;st.textContent='Upload dibatalkan!';};
  xhr.send(fd);
}
</script></body></html>)HTML";

// ============================================================================
// CALLBACK KELAS BLE (BLUEDROID)
// ============================================================================
class DlmsBleServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    (void)pServer;
    if (gConnInstance)
      gConnInstance->onBleConnect();
  }
  void onDisconnect(BLEServer *pServer) override {
    if (gConnInstance)
      gConnInstance->onBleDisconnect();
    if (pServer)
      pServer->getAdvertising()->start();
  }
};

class DlmsBleRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    if (!pChar || !gConnInstance)
      return;
    auto val = pChar->getValue();
    if (val.length() > 0) {
      gConnInstance->onBleRxData(reinterpret_cast<const uint8_t *>(val.c_str()),
                                 val.length());
    }
  }
};

class DlmsBleSecurityCallbacks : public BLESecurityCallbacks {
public:
  explicit DlmsBleSecurityCallbacks(uint32_t pin) : pin_(pin) {}

  uint32_t onPassKeyRequest() override { return pin_; }
  void onPassKeyNotify(uint32_t pass_key) override {
    Serial.printf("[BLE] PIN Pairing: %06lu\n",
                  static_cast<unsigned long>(pass_key));
  }
  bool onConfirmPIN(uint32_t pass_key) override { return (pass_key == pin_); }
  bool onSecurityRequest() override { return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) override {
    Serial.printf("[BLE] Pairing Auth: %s\n", cmpl.success ? "OK" : "FAILED");
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc *desc, int status) override {
    (void)desc;
    Serial.printf("[BLE] Pairing Auth: %s (rc=%d)\n",
                  status == 0 ? "OK" : "FAILED", status);
  }
#endif

private:
  uint32_t pin_;
};

} // namespace

// ============================================================================
// 1. LIFECYCLE & NVS KONFIGURASI RADIO
// ============================================================================
ConnectivityManager::~ConnectivityManager() {
  stopWifiAll();
  stopBleNus();
  if (gConnInstance == this)
    gConnInstance = nullptr;
}

void ConnectivityManager::loadRadioConfig() {
  Preferences prefs;
  prefs.begin(NVS_RADIO_NS, true);
  String name = prefs.getString("ble_name", "DSP-Audio-IF");
  uint32_t pin = prefs.getUInt("ble_pin", 123456u);
  String webPass = prefs.getString("web_pass", "12345678");
  String otaPass = prefs.getString("ota_pass", "12345678");
  prefs.end();

  if (pin > 999999u)
    pin = 123456u;
  strncpy(radioCfg_.bleName, name.c_str(), sizeof(radioCfg_.bleName) - 1);
  radioCfg_.bleName[sizeof(radioCfg_.bleName) - 1] = '\0';
  radioCfg_.blePin = pin;
  strncpy(radioCfg_.wifiWebPass, webPass.c_str(),
          sizeof(radioCfg_.wifiWebPass) - 1);
  strncpy(radioCfg_.wifiOtaPass, otaPass.c_str(),
          sizeof(radioCfg_.wifiOtaPass) - 1);
}

void ConnectivityManager::saveRadioConfig() {
  Preferences prefs;
  prefs.begin(NVS_RADIO_NS, false);
  prefs.putString("ble_name", radioCfg_.bleName);
  prefs.putUInt("ble_pin", radioCfg_.blePin);
  prefs.putString("web_pass", radioCfg_.wifiWebPass);
  prefs.putString("ota_pass", radioCfg_.wifiOtaPass);
  prefs.end();
}

void ConnectivityManager::setBleCredentials(const char *name,
                                            uint32_t pin6Digits) {
  if (name && name[0] != '\0') {
    strncpy(radioCfg_.bleName, name, sizeof(radioCfg_.bleName) - 1);
    radioCfg_.bleName[sizeof(radioCfg_.bleName) - 1] = '\0';
  }
  radioCfg_.blePin = pin6Digits % 1000000u;
  saveRadioConfig();

  // Restart BLE jika sedang aktif agar Nama/PIN baru langsung berlaku
  if (activeMode_ == RadioMode::BLE_NUS) {
    stopBleNus();
    startBleNus();
  }
}

void ConnectivityManager::begin(audio_io::AudioIO &audioIO,
                                dsp::DspEngine &dspEngine, ui_tft::UiTft &uiTft,
                                TaskHandle_t audioTaskHandle) {
  audioIO_ = &audioIO;
  dsp_ = &dspEngine;
  ui_ = &uiTft;
  audioTaskHandle_ = audioTaskHandle;
  gConnInstance = this;

  loadRadioConfig();
  parser_.begin(audioIO, dspEngine, uiTft);

  // Hubungkan perintah BLENAME & BLEPIN dari parser ke NVS ConnectivityManager
  parser_.setRadioConfigCallback([this](const char *key, const char *val,
                                        char *resp, size_t sz) -> bool {
    if (strcmp(key, "BLENAME") == 0) {
      setBleCredentials(val, radioCfg_.blePin);
      snprintf(resp, sz, "{\"ok\":true,\"cmd\":\"BLENAME\",\"name\":\"%s\"}\n",
               radioCfg_.bleName);
      return true;
    }
    if (strcmp(key, "BLEPIN") == 0) {
      uint32_t pin =
          static_cast<uint32_t>(strtoul(val, nullptr, 10)) % 1000000u;
      setBleCredentials(radioCfg_.bleName, pin);
      snprintf(resp, sz, "{\"ok\":true,\"cmd\":\"BLEPIN\",\"pin\":\"%06lu\"}\n",
               static_cast<unsigned long>(radioCfg_.blePin));
      return true;
    }
    return false;
  });

  // Pastikan WiFi mati saat boot, lalu nyalakan sesuai konfigurasi UI (Default:
  // BLE NUS)
  WiFi.mode(WIFI_OFF);

  // Proteksi anti-bootloop Brownout (BOD): bila restart sebelumnya dipicu drop
  // tegangan 3.3V saat kalibrasi RF BLE/WiFi (umum di kabel USB tipis/hub),
  // tahan radio tetap OFF agar Audio + DLMS + LCD tetap stabil tanpa restart
  // berulang. Pengguna tetap dapat menyalakan BLE/WiFi manual lewat SETUP >
  // WIFI.
  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println(
        "[RADIO] PERINGATAN: Reset sebelumnya = BROWNOUT (drop tegangan 3.3V). "
        "Radio BLE/WiFi ditahan OFF untuk mencegah bootloop! "
        "Gunakan port USB langsung / adaptor 5V 2A, lalu aktifkan via SETUP > "
        "WIFI.");
    if (ui_) {
      ui_->setRadioUiState(ui_tft::WifiMode::OFF,
                           false); // Aman tanpa sentuh SPI & non-persisten
    }
    setRadioMode(RadioMode::OFF);
    return;
  }

  // Beri jeda 300ms agar arus puncak LCD/LED pasca-boot mereda sebelum
  // kalibrasi RF
  vTaskDelay(pdMS_TO_TICKS(300));

  const ui_tft::UiConfig ucfg = ui_->getUiConfig();
  if (ucfg.wifiMode == ui_tft::WifiMode::AU_WEB) {
    setRadioMode(RadioMode::WIFI_WEB);
  } else if (ucfg.wifiMode == ui_tft::WifiMode::AU_OTA) {
    setRadioMode(RadioMode::WIFI_OTA);
  } else if (ucfg.bleEnabled) {
    setRadioMode(RadioMode::BLE_NUS);
  } else {
    setRadioMode(RadioMode::OFF);
  }
}

// ============================================================================
// 2. MANAJEMEN RADIO EKSKLUSIF (BLE NUS <-> AU-WEB <-> AU-OTA)
// ============================================================================
void ConnectivityManager::setRadioMode(RadioMode newMode) {
  if (newMode == activeMode_)
    return;

  Serial.printf("[RADIO] Transisi mode: %u -> %u\n",
                static_cast<unsigned>(activeMode_),
                static_cast<unsigned>(newMode));

  parser_.abortLogoTransfer();

  // Jika keluar dari mode OTA -> pulihkan task Audio DSP dengan aman
  if (activeMode_ == RadioMode::WIFI_OTA && newMode != RadioMode::WIFI_OTA) {
    resumeAudioAfterOta();
  }

  // Matikan radio sebelumnya sepenuhnya sebelum menyalakan radio baru
  if (activeMode_ == RadioMode::BLE_NUS) {
    stopBleNus();
  } else if (activeMode_ == RadioMode::WIFI_WEB ||
             activeMode_ == RadioMode::WIFI_OTA) {
    stopWifiAll();
  }

  activeMode_ = newMode;

  switch (newMode) {
  case RadioMode::BLE_NUS:
    startBleNus();
    break;
  case RadioMode::WIFI_WEB:
    startWifiWeb();
    break;
  case RadioMode::WIFI_OTA:
    suspendAudioForOta();
    startWifiOta();
    break;
  case RadioMode::OFF:
  default:
    WiFi.mode(WIFI_OFF);
    break;
  }
}

void ConnectivityManager::startBleNus() {
  if (bleActive_)
    return;

  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(50));

  BLEDevice::init(radioCfg_.bleName);
  // Kurangi lonjakan arus puncak RF kalibrasi (+3 dBm cukup untuk jangkauan
  // ~10-15m)
  BLEDevice::setPower(ESP_PWR_LVL_P3);
  BLEDevice::setMTU(247);

  // Keamanan Pairing PIN 6-Digit Statis (tersimpan di NVS, kompatibel Bluedroid
  // & NimBLE)
#if defined(CONFIG_BLUEDROID_ENABLED)
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT_MITM);
#endif
  delete bleSecCb_;
  bleSecCb_ = new DlmsBleSecurityCallbacks(radioCfg_.blePin);
  BLEDevice::setSecurityCallbacks(bleSecCb_);

  BLESecurity::setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);
  BLESecurity::setCapability(ESP_IO_CAP_OUT);
  BLESecurity::setKeySize(16);
  BLESecurity::setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLESecurity::setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  BLESecurity::setPassKey(true, radioCfg_.blePin);

  bleServer_ = BLEDevice::createServer();
  delete bleServerCb_;
  bleServerCb_ = new DlmsBleServerCallbacks();
  bleServer_->setCallbacks(bleServerCb_);

  BLEService *nus = bleServer_->createService(NUS_SERVICE_UUID);

  bleTxChar_ = nus->createCharacteristic(NUS_TX_CHAR_UUID,
                                         BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  bleTxChar_->addDescriptor(new BLE2902());
#endif

  bleRxChar_ = nus->createCharacteristic(
      NUS_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  delete bleRxCb_;
  bleRxCb_ = new DlmsBleRxCallbacks();
  bleRxChar_->setCallbacks(bleRxCb_);

  nus->start();

  // Paket Advertising Utama: Flags + Nama Lengkap (agar langsung terlihat saat
  // scan pasif & aktif) Paket Scan Response    : 128-bit NUS Service UUID
  // (ditemukan Web Bluetooth / nRF Connect)
  BLEAdvertisementData advData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setName(radioCfg_.bleName);

  BLEAdvertisementData scanRespData;
  scanRespData.setCompleteServices(BLEUUID(NUS_SERVICE_UUID));

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setScanResponse(true);
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanRespData);
  const bool advOk = adv->start();

  bleConnected_ = false;
  bleActive_ = true;
  Serial.printf(
      "[BLE ] NUS Aktif (%s) | Nama: %s | PIN: %06lu | FreeHeap: %lu\n",
      advOk ? "ADV OK" : "ADV FAIL", radioCfg_.bleName,
      static_cast<unsigned long>(radioCfg_.blePin),
      static_cast<unsigned long>(ESP.getFreeHeap()));
}

void ConnectivityManager::stopBleNus() {
  if (!bleActive_)
    return;
  bleConnected_ = false;
  bleActive_ = false;

  // Deinit stack BLE TANPA melepas memori controller permanen
  // (release_memory=false) agar BLE tetap bisa dinyalakan kembali setelah
  // pindah dari mode AU-WEB / AU-OTA / OFF!
  BLEDevice::deinit(false);
  bleServer_ = nullptr;
  bleTxChar_ = nullptr;
  bleRxChar_ = nullptr;
  delete bleServerCb_;
  bleServerCb_ = nullptr;
  delete bleRxCb_;
  bleRxCb_ = nullptr;
  delete bleSecCb_;
  bleSecCb_ = nullptr;
  vTaskDelay(pdMS_TO_TICKS(60));
  Serial.printf("[BLE ] Deinit penuh selesai | FreeHeap: %lu\n",
                static_cast<unsigned long>(ESP.getFreeHeap()));
}

void ConnectivityManager::suspendAudioForOta() {
  if (audioSuspendedForOta_)
    return;
  if (dsp_) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    preOtaMasterMute_ = cfg.master.mute;
    cfg.master.mute = true;
    dsp_->applyConfig(cfg);
    // Beri waktu ~35 ms agar smoothing gain mencapai 0.0 (bebas bunyi "pop")
    vTaskDelay(pdMS_TO_TICKS(35));
  }
  if (audioTaskHandle_ != nullptr) {
    vTaskSuspend(audioTaskHandle_);
  }
  audioSuspendedForOta_ = true;
  Serial.println("[OTA ] Audio DSP Core 1 di-mute & di-suspend aman.");
}

void ConnectivityManager::resumeAudioAfterOta() {
  if (!audioSuspendedForOta_)
    return;
  if (audioTaskHandle_ != nullptr) {
    vTaskResume(audioTaskHandle_);
  }
  if (dsp_) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    cfg.master.mute = preOtaMasterMute_;
    dsp_->applyConfig(cfg);
  }
  audioSuspendedForOta_ = false;
  Serial.println("[OTA ] Audio DSP Core 1 dilanjutkan kembali.");
}

const char *ConnectivityManager::mimeTypeFromPath(const char *path) {
  if (strstr(path, ".html") || strstr(path, ".htm"))
    return "text/html";
  if (strstr(path, ".css"))
    return "text/css";
  if (strstr(path, ".js"))
    return "application/javascript";
  if (strstr(path, ".json"))
    return "application/json";
  if (strstr(path, ".svg"))
    return "image/svg+xml";
  if (strstr(path, ".png"))
    return "image/png";
  if (strstr(path, ".ico"))
    return "image/x-icon";
  return "application/octet-stream";
}

void ConnectivityManager::startWifiWeb() {
  stopBleNus();

  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(
      WIFI_POWER_15dBm); // Batasi lonjakan arus RF AP agar bebas brownout
  const char *pass =
      (strlen(radioCfg_.wifiWebPass) >= 8) ? radioCfg_.wifiWebPass : nullptr;
  WiFi.softAP(AP_SSID_WEB, pass);
  vTaskDelay(pdMS_TO_TICKS(80));

  httpServer_ = new WebServer(HTTP_SERVER_PORT);
  registerWebRoutes();
  httpServer_->begin();

  wsServer_ = new WebSocketsServer(WS_SERVER_PORT);
  wsServer_->begin();
  wsServer_->onEvent(
      [](uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
        if (gConnInstance)
          gConnInstance->onWsEvent(num, type, payload, len);
      });
  wsClientCount_ = 0;

  Serial.printf("[WIFI] Mode AU-WEB Aktif | SSID: %s | IP: %s | WS Port: %u\n",
                AP_SSID_WEB, WiFi.softAPIP().toString().c_str(),
                WS_SERVER_PORT);
}

void ConnectivityManager::registerWebRoutes() {
  if (!httpServer_)
    return;

  httpServer_->on("/", HTTP_GET, [this]() {
    if (LittleFS.exists("/index.html")) {
      File f = LittleFS.open("/index.html", "r");
      if (f) {
        httpServer_->streamFile(f, "text/html");
        f.close();
        return;
      }
    }
    httpServer_->send_P(200, "text/html", FALLBACK_WEB_HTML);
  });

  // Sajikan file statis lain dari LittleFS (CSS/JS/JSON/Logo)
  httpServer_->onNotFound([this]() {
    String uri = httpServer_->uri();
    if (LittleFS.exists(uri)) {
      File f = LittleFS.open(uri, "r");
      if (f) {
        httpServer_->streamFile(f, mimeTypeFromPath(uri.c_str()));
        f.close();
        return;
      }
    }
    httpServer_->send(404, "text/plain", "404 Not Found");
  });
}

void ConnectivityManager::startWifiOta() {
  stopBleNus();

  WiFi.mode(WIFI_AP);
  // Gunakan daya TX 11 dBm untuk OTA (jarak dekat) agar lonjakan arus RF +
  // erase blok 64KB SPI Flash tidak memicu Brownout Reset (3.3V drop).
  WiFi.setTxPower(WIFI_POWER_11dBm);
  const char *pass =
      (strlen(radioCfg_.wifiOtaPass) >= 8) ? radioCfg_.wifiOtaPass : nullptr;
  WiFi.softAP(AP_SSID_OTA, pass);
  vTaskDelay(pdMS_TO_TICKS(80));

  otaFlashing_     = false;
  otaFwFinishedOk_ = false;
  httpServer_ = new WebServer(HTTP_SERVER_PORT);
  registerOtaRoutes();
  httpServer_->begin();

  Serial.printf("[OTA ] Mode AU-OTA Aktif | SSID: %s | IP: %s | TX: 11dBm\n",
                AP_SSID_OTA, WiFi.softAPIP().toString().c_str());
}

void ConnectivityManager::registerOtaRoutes() {
  if (!httpServer_)
    return;

  auto yieldForWdt = [this]() {
    const uint32_t now = millis();
    // Beri ruang ke IDLE0 minimal tiap 20 ms agar Task Watchdog (5s) di Core 0
    // tidak pernah panic saat WebServer membaca stream multipart 1.5 MB nonstop.
    if (now - otaLastYieldMs_ >= 20) {
      otaLastYieldMs_ = now;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  };

  httpServer_->on("/", HTTP_GET, [this]() {
    httpServer_->send_P(200, "text/html", OTA_PORTAL_HTML);
  });

  // 1. Upload Firmware `.bin` menggunakan Update.h (OTA 2x3MB)
  httpServer_->on(
      "/ota/firmware", HTTP_POST,
      [this]() {
        otaFlashing_ = false;
        const bool ok = otaFwFinishedOk_ && !Update.hasError();
        String msg = ok ? String("Firmware OTA SUKSES! Rebooting...")
                        : (String("Firmware OTA GAGAL: ") + Update.errorString());
        httpServer_->send(ok ? 200 : 500, "text/plain", msg);
        if (ok) {
          vTaskDelay(pdMS_TO_TICKS(500));
          ESP.restart();
        }
      },
      [this, yieldForWdt]() {
        HTTPUpload &up = httpServer_->upload();
        if (up.status == UPLOAD_FILE_START) {
          otaFlashing_     = true;
          otaFwFinishedOk_ = false;
          otaLastYieldMs_  = millis();
          otaLastLogBytes_ = 0;

          // Jika ada sisa sesi Update sebelumnya yang sempat terputus, bersihkan dulu
          if (Update.isRunning()) {
            Update.abort();
          }

          size_t expectedSize = UPDATE_SIZE_UNKNOWN;
          if (httpServer_->hasArg("size")) {
            const unsigned long parsed =
                strtoul(httpServer_->arg("size").c_str(), nullptr, 10);
            if (parsed > 0) {
              expectedSize = static_cast<size_t>(parsed);
            }
          }

          Serial.printf("[OTA ] Mulai upload firmware: %s (size=%u)\n",
                        up.filename.c_str(),
                        static_cast<unsigned>(expectedSize));
          vTaskDelay(pdMS_TO_TICKS(2));
          if (!Update.begin(expectedSize, U_FLASH)) {
            Update.printError(Serial);
            otaFlashing_ = false;
          }
        } else if (up.status == UPLOAD_FILE_WRITE) {
          if (Update.isRunning() && up.currentSize > 0) {
            const uint32_t t0 = millis();
            if (Update.write(up.buf, up.currentSize) != up.currentSize) {
              Update.printError(Serial);
            }
            const uint32_t writeMs = millis() - t0;
            // Jika baru saja melewati erase blok 64KB (>15ms), wajib langsung yield
            if (writeMs >= 15) {
              otaLastYieldMs_ = millis();
              vTaskDelay(pdMS_TO_TICKS(2));
            } else {
              yieldForWdt();
            }

            const uint32_t written = up.totalSize + up.currentSize;
            if (written - otaLastLogBytes_ >= 131072u) {
              otaLastLogBytes_ = written;
              Serial.printf("[OTA ] Progres tulis: %u KB\n",
                            static_cast<unsigned>(written / 1024u));
            }
          } else {
            yieldForWdt();
          }
        } else if (up.status == UPLOAD_FILE_END) {
          vTaskDelay(pdMS_TO_TICKS(2));
          if (Update.isRunning() && Update.end(true)) {
            otaFwFinishedOk_ = true;
            Serial.printf("[OTA ] Firmware selesai & terverifikasi (%u bytes)\n",
                          static_cast<unsigned>(up.totalSize));
          } else {
            otaFwFinishedOk_ = false;
            Update.printError(Serial);
            if (Update.isRunning()) {
              Update.abort();
            }
          }
          otaFlashing_ = false;
        } else if (up.status == UPLOAD_FILE_ABORTED) {
          Serial.println("[OTA ] Upload firmware dibatalkan/terputus -> Update.abort()");
          if (Update.isRunning()) {
            Update.abort();
          }
          otaFwFinishedOk_ = false;
          otaFlashing_     = false;
        }
      });

  // 2. Upload Logo Custom ke LittleFS (`/logo_boot.bin` atau
  // `/logo_standby.bin`)
  httpServer_->on(
      "/ota/logo", HTTP_POST,
      [this]() {
        otaFlashing_ = false;
        httpServer_->send(200, "text/plain",
                          "Logo berhasil disimpan ke LittleFS!");
      },
      [this, yieldForWdt]() {
        HTTPUpload &up = httpServer_->upload();
        if (up.status == UPLOAD_FILE_START) {
          otaFlashing_    = true;
          otaLastYieldMs_ = millis();
          const String target = httpServer_->arg("target");
          snprintf(otaLogoFinalPath_, sizeof(otaLogoFinalPath_), "%s",
                   (target == "standby") ? "/logo_standby.bin"
                                         : "/logo_boot.bin");
          if (otaLogoFile_)
            otaLogoFile_.close();
          otaLogoFile_ = LittleFS.open("/logo_ota.tmp", "w");
        } else if (up.status == UPLOAD_FILE_WRITE) {
          if (otaLogoFile_)
            otaLogoFile_.write(up.buf, up.currentSize);
          yieldForWdt();
        } else if (up.status == UPLOAD_FILE_END) {
          if (otaLogoFile_) {
            otaLogoFile_.close();
            LittleFS.remove(otaLogoFinalPath_);
            LittleFS.rename("/logo_ota.tmp", otaLogoFinalPath_);
            if (ui_)
              ui_->reloadLogos();
            Serial.printf("[OTA ] Logo disimpan: %s (%u bytes)\n",
                          otaLogoFinalPath_,
                          static_cast<unsigned>(up.totalSize));
          }
          otaFlashing_ = false;
        } else if (up.status == UPLOAD_FILE_ABORTED) {
          if (otaLogoFile_) {
            otaLogoFile_.close();
          }
          LittleFS.remove("/logo_ota.tmp");
          otaFlashing_ = false;
        }
      });

  // 3. Upload Web App `/index.html` ke LittleFS
  httpServer_->on(
      "/ota/webapp", HTTP_POST,
      [this]() {
        otaFlashing_ = false;
        httpServer_->send(200, "text/plain",
                          "Web App /index.html berhasil disimpan!");
      },
      [this, yieldForWdt]() {
        HTTPUpload &up = httpServer_->upload();
        if (up.status == UPLOAD_FILE_START) {
          otaFlashing_    = true;
          otaLastYieldMs_ = millis();
          if (otaLogoFile_)
            otaLogoFile_.close();
          otaLogoFile_ = LittleFS.open("/index.html.tmp", "w");
        } else if (up.status == UPLOAD_FILE_WRITE) {
          if (otaLogoFile_)
            otaLogoFile_.write(up.buf, up.currentSize);
          yieldForWdt();
        } else if (up.status == UPLOAD_FILE_END) {
          if (otaLogoFile_) {
            otaLogoFile_.close();
            LittleFS.remove("/index.html");
            LittleFS.rename("/index.html.tmp", "/index.html");
            Serial.printf("[OTA ] Web App /index.html disimpan (%u bytes)\n",
                          static_cast<unsigned>(up.totalSize));
          }
          otaFlashing_ = false;
        } else if (up.status == UPLOAD_FILE_ABORTED) {
          if (otaLogoFile_) {
            otaLogoFile_.close();
          }
          LittleFS.remove("/index.html.tmp");
          otaFlashing_ = false;
        }
      });

  // 4. Reboot manual
  httpServer_->on("/ota/reboot", HTTP_POST, [this]() {
    httpServer_->send(200, "text/plain", "Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(250));
    ESP.restart();
  });
}

void ConnectivityManager::stopWifiAll() {
  if (Update.isRunning()) {
    Update.abort();
  }
  if (otaLogoFile_) {
    otaLogoFile_.close();
  }
  otaFlashing_     = false;
  otaFwFinishedOk_ = false;
  if (wsServer_) {
    wsServer_->close();
    delete wsServer_;
    wsServer_ = nullptr;
  }
  if (httpServer_) {
    httpServer_->stop();
    delete httpServer_;
    httpServer_ = nullptr;
  }
  wsClientCount_ = 0;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(50));
}

// ============================================================================
// 3. HANDLER EVENT BLE NUS & WEBSOCKET
// ============================================================================
void ConnectivityManager::onBleConnect() {
  bleConnected_ = true;
  Serial.println("[BLE ] Klien terhubung.");
}

void ConnectivityManager::onBleDisconnect() {
  bleConnected_ = false;
  parser_.abortLogoTransfer();
  Serial.println("[BLE ] Klien terputus, kembali advertising.");
}

void ConnectivityManager::onBleRxData(const uint8_t *data, size_t len) {
  if (!data || len == 0)
    return;

  portENTER_CRITICAL(&bleRxMux_);
  const size_t avail = sizeof(bleRxQueue_) - bleRxLen_ - 1;
  const size_t n = (len > avail) ? avail : len;
  if (n > 0) {
    memcpy(bleRxQueue_ + bleRxLen_, data, n);
    bleRxLen_ += n;
    bleRxQueue_[bleRxLen_] = '\0';
    bleRxIsBinary_ = parser_.isLogoTransferActive();
  }
  portEXIT_CRITICAL(&bleRxMux_);
}

void ConnectivityManager::sendBleNotifyChunked(const char *data, size_t len) {
  if (!bleActive_ || !bleConnected_ || !bleTxChar_ || !data || len == 0)
    return;

  // Potong paket agar aman untuk MTU BLE (maks 200 byte per notify)
  constexpr size_t CHUNK_SZ = 200;
  size_t off = 0;
  while (off < len && bleConnected_) {
    const size_t n = ((len - off) > CHUNK_SZ) ? CHUNK_SZ : (len - off);
    bleTxChar_->setValue(
        reinterpret_cast<uint8_t *>(const_cast<char *>(data + off)), n);
    bleTxChar_->notify();
    off += n;
    if (off < len) {
      vTaskDelay(pdMS_TO_TICKS(4));
    }
  }
}

void ConnectivityManager::processQueuedBleRx() {
  uint8_t localBuf[1024];
  size_t localLen = 0;
  bool isBin = false;

  portENTER_CRITICAL(&bleRxMux_);
  if (bleRxLen_ > 0) {
    localLen = bleRxLen_;
    memcpy(localBuf, bleRxQueue_, localLen + 1);
    bleRxLen_ = 0;
    isBin = bleRxIsBinary_;
  }
  portEXIT_CRITICAL(&bleRxMux_);

  if (localLen == 0)
    return;

  auto bleReply = [this](const char *d, size_t l) {
    sendBleNotifyChunked(d, l);
  };

  // Jika sedang dalam sesi transfer LOGO: cek apakah teks kontrol
  // (`LOGO_DATA`/`LOGO_ABORT`) atau raw binary
  if (isBin && parser_.isLogoTransferActive()) {
    const char *asTxt = reinterpret_cast<const char *>(localBuf);
    if (strncasecmp(asTxt, "LOGO_DATA ", 10) == 0 ||
        strncasecmp(asTxt, "LOGO_ABORT", 10) == 0 ||
        strncasecmp(asTxt, "B64 ", 4) == 0) {
      parser_.handleTextCommand(asTxt, bleReply);
    } else {
      parser_.handleBinaryChunk(localBuf, localLen, bleReply);
    }
    return;
  }

  // Pecah per baris (\n) bila klien mengirim beberapa perintah sekaligus
  char *savePtr = nullptr;
  char *line = strtok_r(reinterpret_cast<char *>(localBuf), "\r\n", &savePtr);
  while (line) {
    parser_.handleTextCommand(line, bleReply);
    line = strtok_r(nullptr, "\r\n", &savePtr);
  }
}

void ConnectivityManager::onWsEvent(uint8_t num, WStype_t type,
                                    uint8_t *payload, size_t len) {
  switch (type) {
  case WStype_CONNECTED:
    wsClientCount_++;
    Serial.printf("[WS  ] Klien #%u terhubung (total: %u)\n", num,
                  wsClientCount_);
    break;

  case WStype_DISCONNECTED:
    if (wsClientCount_ > 0)
      wsClientCount_--;
    if (wsClientCount_ == 0)
      parser_.abortLogoTransfer();
    Serial.printf("[WS  ] Klien #%u terputus (total: %u)\n", num,
                  wsClientCount_);
    break;

  case WStype_TEXT:
    if (payload && len > 0) {
      parser_.handleTextCommand(reinterpret_cast<const char *>(payload),
                                [this, num](const char *d, size_t l) {
                                  if (wsServer_)
                                    wsServer_->sendTXT(num, d, l);
                                });
    }
    break;

  case WStype_BIN:
    if (payload && len > 0) {
      parser_.handleBinaryChunk(payload, len,
                                [this, num](const char *d, size_t l) {
                                  if (wsServer_)
                                    wsServer_->sendTXT(num, d, l);
                                });
    }
    break;

  default:
    break;
  }
}

// ============================================================================
// 4. UPDATE LOOP CORE 0 (SYNC RADIO UI + HTTP/WS/BLE + TELEMETRY 80 MS)
// ============================================================================
void ConnectivityManager::update(const audio_io::AudioTelemetry &audioTel,
                                 const dsp::DspMeters &dspMet) {
  // 1. Sinkronkan perubahan mode radio dari menu LCD `SETUP > WIFI`
  if (ui_) {
    const ui_tft::UiConfig ucfg = ui_->getUiConfig();
    RadioMode desired = RadioMode::OFF;
    if (ucfg.wifiMode == ui_tft::WifiMode::AU_WEB) {
      desired = RadioMode::WIFI_WEB;
    } else if (ucfg.wifiMode == ui_tft::WifiMode::AU_OTA) {
      desired = RadioMode::WIFI_OTA;
    } else if (ucfg.bleEnabled) {
      desired = RadioMode::BLE_NUS;
    }
    if (desired != activeMode_) {
      setRadioMode(desired);
    }
  }

  // 2. Proses I/O sesuai mode radio aktif
  if (activeMode_ == RadioMode::BLE_NUS) {
    processQueuedBleRx();
  } else if (activeMode_ == RadioMode::WIFI_WEB) {
    if (httpServer_)
      httpServer_->handleClient();
    if (wsServer_)
      wsServer_->loop();
  } else if (activeMode_ == RadioMode::WIFI_OTA) {
    if (httpServer_)
      httpServer_->handleClient();
    return; // Jangan kirim stream telemetri saat OTA sedang berlangsung
  }

  // 3. Stream Telemetry Otomatis (~50-100 ms, default 80 ms) saat klien
  // terhubung
  const uint16_t intervalMs = parser_.getTelemetryIntervalMs();
  const bool hasClient =
      (activeMode_ == RadioMode::BLE_NUS && bleConnected_) ||
      (activeMode_ == RadioMode::WIFI_WEB && wsClientCount_ > 0);

  if (hasClient && intervalMs > 0 && !parser_.isLogoTransferActive()) {
    const uint32_t nowMs = millis();
    if (nowMs - lastTelemMs_ >= intervalMs) {
      lastTelemMs_ = nowMs;
      const size_t n = parser_.buildTelemetryJson(telemBuf_, sizeof(telemBuf_),
                                                  audioTel, dspMet);
      if (n > 0) {
        if (activeMode_ == RadioMode::WIFI_WEB && wsServer_) {
          wsServer_->broadcastTXT(telemBuf_, n);
        } else if (activeMode_ == RadioMode::BLE_NUS && bleConnected_) {
          sendBleNotifyChunked(telemBuf_, n);
        }
      }
    }
  }
}

} // namespace conn
