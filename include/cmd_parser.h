#pragma once

/**
 * @file cmd_parser.h
 * @brief Tahap 4: Protokol Command Parser Ringan (Teks & JSON) untuk BLE NUS & WebSocket
 *
 * Perintah yang Didukung:
 *  - SET <param> <val>          : Ubah parameter Gain/PEQ/Xover/Comp/Gate/Delay/Matrix/Gen/Master
 *  - GET <param>                : Baca nilai parameter aktif
 *  - DUMP                       : Ekspor seluruh konfigurasi aktif dalam JSON ringkas
 *  - PRESET <LOAD|SAVE|DEL> <n> : Kelola slot preset 0..7 di LittleFS (+ RENAME)
 *  - THEME <0-6>                : Pilih tema warna UI ST7789
 *  - COLORMODE <0-3>            : Pilih mode warna hardware panel ST7789
 *  - STANDBY <detik>            : Timer auto-standby (0=OFF, 5..600 detik)
 *  - COPY <src_ch> <dst_ch|ALL> : Salin pengaturan kanal (IN1/IN2/OUT1..OUT4 -> dst/ALL)
 *  - LOCK <0|1>                 : Aktifkan / nonaktifkan Panel Lock
 *  - MASTER <dB>                : Atur Master Volume (-60..0 dB) atau MASTER MUTE <0|1>
 *  - LINK <pair_mode>           : Link stereo (0=NONE, 1=IN, 2=OUT12, 4=OUT34, 6=OUT, 7=ALL)
 *  - INFO                       : Info versi firmware, suhu chip, heap/PSRAM, LittleFS, radio
 *  - RESTART                    : Mute aman lalu reboot ESP32-S3
 *  - LOGO <BOOT|STANDBY> <size> : Mulai transfer logo RGB565 (chunk raw binary atau base64)
 *  - BLENAME <nama> / BLEPIN <6-digit> : Ubah identitas pairing BLE (tersimpan di NVS)
 *  - TELEM <ms>                 : Interval stream telemetri otomatis (0=OFF, 50..500 ms)
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "audio_io.h"
#include "dsp_engine.h"
#include "input_ctrl.h"
#include "ui_tft.h"

namespace conn {

constexpr const char *FW_VERSION_STR   = "1.4.0-DLMS2x4";
constexpr size_t      MAX_LOGO_BYTES   = 160000; // 320x240 RGB565 = 153600 byte + margin
constexpr uint8_t     RTA_STREAM_BANDS = 32;     // 32-band RTA telemetry

// Bitmask Stereo Channel Linking
enum LinkMask : uint8_t {
  LINK_NONE  = 0x00,
  LINK_IN12  = 0x01, // IN 1 <-> IN 2
  LINK_OUT12 = 0x02, // OUT 1 <-> OUT 2
  LINK_OUT34 = 0x04, // OUT 3 <-> OUT 4
  LINK_OUT   = 0x06, // OUT 1-2 & OUT 3-4
  LINK_ALL   = 0x07  // Semua pasangan stereo
};

// Callback opsional untuk perintah radio/NVS (BLENAME / BLEPIN / Radio Info)
using RadioConfigCallback = std::function<bool(const char *key, const char *val,
                                               char *respBuf, size_t respSize)>;

// Callback pengirim respons (diimplementasikan oleh BLE NUS atau WebSocket)
using ReplyCallback = std::function<void(const char *data, size_t len)>;

class CommandParser {
public:
  CommandParser() = default;

  void begin(audio_io::AudioIO &audioIO,
             dsp::DspEngine &dspEngine,
             ui_tft::UiTft &uiTft);

  void setRadioConfigCallback(RadioConfigCallback cb) { radioCfgCb_ = cb; }

  // Proses 1 perintah teks (atau JSON ringan `{"cmd":"..."}`) dan kirim balasan via `reply`
  void handleTextCommand(const char *rawLine, const ReplyCallback &reply);

  // Proses chunk data RAW binary saat sesi `LOGO <BOOT|STANDBY> <size>` sedang aktif
  // Mengembalikan true jika paket dikonsumsi oleh penerima logo
  bool handleBinaryChunk(const uint8_t *data, size_t len, const ReplyCallback &reply);

  // Status sesi transfer logo
  bool isLogoTransferActive() const { return logoActive_; }
  void abortLogoTransfer();

  // Status Stereo Link & Telemetry Interval
  uint8_t  getLinkMask() const { return linkMask_; }
  void     setLinkMask(uint8_t mask) { linkMask_ = (mask & LINK_ALL); }
  uint16_t getTelemetryIntervalMs() const { return telemIntervalMs_; }

  // Bangun paket JSON Telemetri live (6-ch VU, RMS, GR, Clip, Limiter, 32-band RTA)
  size_t buildTelemetryJson(char *outBuf, size_t outSize,
                            const audio_io::AudioTelemetry &audioTel,
                            const dsp::DspMeters &dspMet,
                            const dsp::RtaSpectrumData &rtaData) const;

private:
  // Handler perintah utama
  void cmdSet(const char *param, const char *valStr, const ReplyCallback &reply);
  void cmdGet(const char *param, const ReplyCallback &reply);
  void cmdDump(const ReplyCallback &reply);
  void cmdPreset(const char *action, const char *arg1, const char *arg2, const ReplyCallback &reply);
  void cmdCopy(const char *srcStr, const char *dstStr, const ReplyCallback &reply);
  void cmdInfo(const ReplyCallback &reply);
  void cmdLogoStart(const char *targetStr, size_t totalBytes, const ReplyCallback &reply);
  bool appendLogoBytes(const uint8_t *bytes, size_t len, const ReplyCallback &reply);
  void finishLogoTransfer(const ReplyCallback &reply);

  // Helper SET/GET parameter pada 1 kanal tertentu
  bool applyParamSingle(dsp::SystemConfig &cfg, const char *paramUpper, float fVal, const char *strVal);
  bool readParamSingle(const dsp::SystemConfig &cfg, const char *paramUpper, char *valOut, size_t valSize) const;

  // Normalisasi input JSON ringan `{"cmd":"SET","param":"...","val":...}` -> teks standar
  static bool normalizeJsonCommand(const char *in, char *out, size_t outSize);
  static size_t decodeBase64(const char *b64, uint8_t *out, size_t maxOut);
  static bool parseChannelId(const char *tok, dsp::ChannelId &outId);

  // Mirror parameter ke kanal pasangan jika Stereo LINK aktif
  static bool makeLinkedParamName(const char *paramUpper, uint8_t linkMask,
                                  char *linkedOut, size_t outSize);

  audio_io::AudioIO  *audioIO_ = nullptr;
  dsp::DspEngine     *dsp_     = nullptr;
  ui_tft::UiTft      *ui_      = nullptr;
  RadioConfigCallback radioCfgCb_;

  uint8_t  linkMask_        = LINK_NONE;
  uint16_t telemIntervalMs_ = 80; // Default 80 ms (~12.5 Hz dalam rentang 50-100 ms)

  // State transfer file Logo ke LittleFS
  bool     logoActive_      = false;
  bool     logoIsStandby_   = false;
  size_t   logoExpected_    = 0;
  size_t   logoReceived_    = 0;
  uint32_t logoLastChunkMs_ = 0;
  File     logoFile_;

  // Buffer dinamis di PSRAM untuk respons `DUMP` besar agar tidak boros stack/SRAM
  char    *dumpBuf_         = nullptr;
  size_t   dumpBufCap_      = 0;
};

} // namespace conn
