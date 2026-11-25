/**
 * @file cmd_parser.cpp
 * @brief Implementasi Protokol Command Parser (Teks & JSON Ringan) untuk BLE NUS & WebSocket
 */

#include "cmd_parser.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace conn {

namespace {

constexpr size_t DUMP_BUF_BYTES    = 16384; // 16 KB di PSRAM untuk DUMP penuh 6 kanal x 9 PEQ
constexpr uint32_t LOGO_TIMEOUT_MS = 15000;

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void toUpperCopy(const char *src, char *dst, size_t dstSize) {
  size_t i = 0;
  for (; i + 1 < dstSize && src[i] != '\0'; i++) {
    dst[i] = static_cast<char>(toupper(static_cast<unsigned char>(src[i])));
  }
  dst[i] = '\0';
}

// Ekstrak nilai string/angka sederhana untuk 1 key JSON ringan (tanpa ArduinoJson)
bool extractJsonField(const char *json, const char *key, char *out, size_t outSize) {
  char pat[32];
  snprintf(pat, sizeof(pat), "\"%s\"", key);
  const char *p = strstr(json, pat);
  if (!p) return false;
  p += strlen(pat);
  while (*p && (*p == ' ' || *p == '\t' || *p == ':')) p++;
  if (!*p) return false;

  size_t idx = 0;
  if (*p == '"') {
    p++;
    while (*p && *p != '"' && idx + 1 < outSize) {
      out[idx++] = *p++;
    }
  } else {
    while (*p && *p != ',' && *p != '}' && *p != ' ' && *p != '\r' && *p != '\n' && idx + 1 < outSize) {
      out[idx++] = *p++;
    }
  }
  out[idx] = '\0';
  return (idx > 0);
}

int parsePeqTypeToken(const char *s) {
  if (isdigit(static_cast<unsigned char>(s[0]))) return atoi(s);
  if (strcmp(s, "PK") == 0 || strcmp(s, "PEAKING") == 0) return 0;
  if (strcmp(s, "LS") == 0 || strcmp(s, "LOW_SHELF") == 0) return 1;
  if (strcmp(s, "HS") == 0 || strcmp(s, "HIGH_SHELF") == 0) return 2;
  if (strcmp(s, "LP") == 0 || strcmp(s, "LOW_PASS") == 0) return 3;
  if (strcmp(s, "HP") == 0 || strcmp(s, "HIGH_PASS") == 0) return 4;
  if (strcmp(s, "NT") == 0 || strcmp(s, "NOTCH") == 0) return 5;
  if (strcmp(s, "AP") == 0 || strcmp(s, "ALL_PASS") == 0) return 6;
  return 0;
}

int parseXoverTypeToken(const char *s) {
  if (isdigit(static_cast<unsigned char>(s[0]))) return atoi(s);
  if (strcmp(s, "LR") == 0 || strcmp(s, "LINKWITZ_RILEY") == 0) return 0;
  if (strcmp(s, "BW") == 0 || strcmp(s, "BUTTERWORTH") == 0) return 1;
  if (strcmp(s, "BES") == 0 || strcmp(s, "BESSEL") == 0) return 2;
  return 0;
}

int parseMatrixSrcToken(const char *s) {
  if (isdigit(static_cast<unsigned char>(s[0]))) return atoi(s);
  if (strcmp(s, "IN1") == 0 || strcmp(s, "INA") == 0) return 0;
  if (strcmp(s, "IN2") == 0 || strcmp(s, "INB") == 0) return 1;
  if (strcmp(s, "USBL") == 0 || strcmp(s, "USB1") == 0) return 2;
  if (strcmp(s, "USBR") == 0 || strcmp(s, "USB2") == 0) return 3;
  return -1;
}

} // namespace

// ============================================================================
// 1. INISIALISASI & UTILITAS PARSER
// ============================================================================
void CommandParser::begin(audio_io::AudioIO &audioIO,
                          dsp::DspEngine &dspEngine,
                          ui_tft::UiTft &uiTft) {
  audioIO_ = &audioIO;
  dsp_     = &dspEngine;
  ui_      = &uiTft;

  if (!dumpBuf_) {
#if defined(ESP32)
    dumpBuf_ = static_cast<char *>(ps_malloc(DUMP_BUF_BYTES));
    if (!dumpBuf_) dumpBuf_ = static_cast<char *>(malloc(DUMP_BUF_BYTES));
#else
    dumpBuf_ = static_cast<char *>(malloc(DUMP_BUF_BYTES));
#endif
    dumpBufCap_ = dumpBuf_ ? DUMP_BUF_BYTES : 0;
  }
}

bool CommandParser::normalizeJsonCommand(const char *in, char *out, size_t outSize) {
  // Lewati spasi awal; jika diawali '{', ubah JSON ringan menjadi format teks
  while (*in == ' ' || *in == '\t') in++;
  if (*in != '{') return false;

  char cmd[24] = "", p1[48] = "", p2[48] = "", p3[32] = "";
  if (!extractJsonField(in, "cmd", cmd, sizeof(cmd))) return false;

  // Pemetaan field JSON fleksibel (param/val, action/slot, src/dst, target/size, data)
  if (extractJsonField(in, "param", p1, sizeof(p1))) {
    if (extractJsonField(in, "val", p2, sizeof(p2))) {
      snprintf(out, outSize, "%s %s %s", cmd, p1, p2);
    } else {
      snprintf(out, outSize, "%s %s", cmd, p1);
    }
    return true;
  }
  if (extractJsonField(in, "action", p1, sizeof(p1))) {
    extractJsonField(in, "slot", p2, sizeof(p2));
    extractJsonField(in, "name", p3, sizeof(p3));
    if (p3[0]) snprintf(out, outSize, "%s %s %s %s", cmd, p1, p2, p3);
    else if (p2[0]) snprintf(out, outSize, "%s %s %s", cmd, p1, p2);
    else snprintf(out, outSize, "%s %s", cmd, p1);
    return true;
  }
  if (extractJsonField(in, "src", p1, sizeof(p1)) &&
      extractJsonField(in, "dst", p2, sizeof(p2))) {
    snprintf(out, outSize, "%s %s %s", cmd, p1, p2);
    return true;
  }
  if (extractJsonField(in, "target", p1, sizeof(p1)) &&
      extractJsonField(in, "size", p2, sizeof(p2))) {
    snprintf(out, outSize, "%s %s %s", cmd, p1, p2);
    return true;
  }
  if (extractJsonField(in, "data", p1, sizeof(p1))) {
    snprintf(out, outSize, "%s %s", cmd, p1);
    return true;
  }
  if (extractJsonField(in, "val", p1, sizeof(p1))) {
    snprintf(out, outSize, "%s %s", cmd, p1);
    return true;
  }
  snprintf(out, outSize, "%s", cmd);
  return true;
}

size_t CommandParser::decodeBase64(const char *b64, uint8_t *out, size_t maxOut) {
  auto valOf = [](char c) -> int8_t {
    if (c >= 'A' && c <= 'Z') return static_cast<int8_t>(c - 'A');
    if (c >= 'a' && c <= 'z') return static_cast<int8_t>(c - 'a' + 26);
    if (c >= '0' && c <= '9') return static_cast<int8_t>(c - '0' + 52);
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
  };

  size_t outLen = 0;
  uint32_t acc = 0;
  int bits = 0;

  for (size_t i = 0; b64[i] != '\0'; i++) {
    if (b64[i] == '=') break;
    int8_t v = valOf(b64[i]);
    if (v < 0) continue;
    acc = (acc << 6) | static_cast<uint32_t>(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (outLen < maxOut) {
        out[outLen++] = static_cast<uint8_t>((acc >> bits) & 0xFFu);
      }
    }
  }
  return outLen;
}

bool CommandParser::parseChannelId(const char *tok, dsp::ChannelId &outId) {
  char u[16];
  toUpperCopy(tok, u, sizeof(u));
  if (strcmp(u, "IN1") == 0 || strcmp(u, "INA") == 0) { outId = dsp::ChannelId::IN1; return true; }
  if (strcmp(u, "IN2") == 0 || strcmp(u, "INB") == 0) { outId = dsp::ChannelId::IN2; return true; }
  if (strcmp(u, "OUT1") == 0 || strcmp(u, "O1") == 0) { outId = dsp::ChannelId::OUT1; return true; }
  if (strcmp(u, "OUT2") == 0 || strcmp(u, "O2") == 0) { outId = dsp::ChannelId::OUT2; return true; }
  if (strcmp(u, "OUT3") == 0 || strcmp(u, "O3") == 0) { outId = dsp::ChannelId::OUT3; return true; }
  if (strcmp(u, "OUT4") == 0 || strcmp(u, "O4") == 0) { outId = dsp::ChannelId::OUT4; return true; }
  if (strcmp(u, "ALL") == 0)                          { outId = dsp::ChannelId::ALL_OUTPUTS; return true; }
  return false;
}

bool CommandParser::makeLinkedParamName(const char *p, uint8_t linkMask,
                                        char *linkedOut, size_t outSize) {
  if (linkMask == LINK_NONE) return false;

  if ((linkMask & LINK_IN12) != 0) {
    if (strncmp(p, "IN1.", 4) == 0 || strncmp(p, "INA.", 4) == 0) {
      snprintf(linkedOut, outSize, "IN2.%s", p + 4); return true;
    }
    if (strncmp(p, "IN2.", 4) == 0 || strncmp(p, "INB.", 4) == 0) {
      snprintf(linkedOut, outSize, "IN1.%s", p + 4); return true;
    }
  }
  if ((linkMask & LINK_OUT12) != 0) {
    if (strncmp(p, "OUT1.", 5) == 0) { snprintf(linkedOut, outSize, "OUT2.%s", p + 5); return true; }
    if (strncmp(p, "OUT2.", 5) == 0) { snprintf(linkedOut, outSize, "OUT1.%s", p + 5); return true; }
  }
  if ((linkMask & LINK_OUT34) != 0) {
    if (strncmp(p, "OUT3.", 5) == 0) { snprintf(linkedOut, outSize, "OUT4.%s", p + 5); return true; }
    if (strncmp(p, "OUT4.", 5) == 0) { snprintf(linkedOut, outSize, "OUT3.%s", p + 5); return true; }
  }
  return false;
}

// ============================================================================
// 2. DISPATCHER PERINTAH UTAMA (TEKS / JSON)
// ============================================================================
void CommandParser::handleTextCommand(const char *rawLine, const ReplyCallback &reply) {
  if (!rawLine || !dsp_ || !ui_) return;

  // Timeout otomatis untuk sesi transfer logo yang terbengkalai
  if (logoActive_ && (millis() - logoLastChunkMs_ > LOGO_TIMEOUT_MS)) {
    abortLogoTransfer();
  }

  // Salin & trim whitespace/newline
  char line[384];
  if (!normalizeJsonCommand(rawLine, line, sizeof(line))) {
    strncpy(line, rawLine, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
  }
  size_t len = strlen(line);
  while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ')) {
    line[--len] = '\0';
  }
  if (len == 0) return;

  // Jika sedang dalam sesi transfer LOGO dan menerima chunk base64 `LOGO_DATA <b64>` atau `LOGO_ABORT`
  if (logoActive_) {
    if (strncasecmp(line, "LOGO_ABORT", 10) == 0) {
      abortLogoTransfer();
      if (reply) reply("{\"ok\":true,\"cmd\":\"LOGO_ABORT\"}\n", 31);
      return;
    }
    const char *b64Ptr = nullptr;
    if (strncasecmp(line, "LOGO_DATA ", 10) == 0) {
      b64Ptr = line + 10;
    } else if (strncasecmp(line, "B64 ", 4) == 0) {
      b64Ptr = line + 4;
    }
    if (b64Ptr) {
      uint8_t decoded[256];
      const size_t decLen = decodeBase64(b64Ptr, decoded, sizeof(decoded));
      appendLogoBytes(decoded, decLen, reply);
      return;
    }
  }

  // Tokenisasi hingga 4 argumen
  char *tok[4] = {nullptr, nullptr, nullptr, nullptr};
  int argc = 0;
  char *savePtr = nullptr;
  char *t = strtok_r(line, " \t", &savePtr);
  while (t && argc < 4) {
    tok[argc++] = t;
    t = strtok_r(nullptr, " \t", &savePtr);
  }
  if (argc == 0) return;

  char cmd[24];
  toUpperCopy(tok[0], cmd, sizeof(cmd));

  char resp[220];

  if (strcmp(cmd, "SET") == 0 && argc >= 3) {
    cmdSet(tok[1], tok[2], reply);
  } else if (strcmp(cmd, "GET") == 0 && argc >= 2) {
    cmdGet(tok[1], reply);
  } else if (strcmp(cmd, "DUMP") == 0) {
    cmdDump(reply);
  } else if (strcmp(cmd, "PRESET") == 0 && argc >= 3) {
    cmdPreset(tok[1], tok[2], (argc >= 4) ? tok[3] : nullptr, reply);
  } else if (strcmp(cmd, "THEME") == 0 && argc >= 2) {
    const int th = constrain(atoi(tok[1]), 0, 6);
    ui_tft::UiConfig ucfg = ui_->getUiConfig();
    ucfg.theme = static_cast<ui_tft::UiTheme>(th);
    ui_->applyUiConfig(ucfg, true);
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"THEME\",\"val\":%d}\n", th);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "COLORMODE") == 0 && argc >= 2) {
    const int cm = constrain(atoi(tok[1]), 0, 3);
    ui_tft::UiConfig ucfg = ui_->getUiConfig();
    ucfg.panelMode = static_cast<ui_tft::PanelColorMode>(cm);
    ui_->applyUiConfig(ucfg, true);
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"COLORMODE\",\"val\":%d}\n", cm);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "STANDBY") == 0 && argc >= 2) {
    const int sec = atoi(tok[1]);
    ui_tft::UiConfig ucfg = ui_->getUiConfig();
    if (sec <= 0) {
      ucfg.standbyEnabled = false;
    } else {
      ucfg.standbyEnabled    = true;
      ucfg.standbyTimeoutSec = static_cast<uint16_t>(constrain(sec, 5, 600));
    }
    ui_->applyUiConfig(ucfg, true);
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"STANDBY\",\"en\":%d,\"sec\":%u}\n",
                     ucfg.standbyEnabled ? 1 : 0, ucfg.standbyTimeoutSec);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "COPY") == 0 && argc >= 3) {
    cmdCopy(tok[1], tok[2], reply);
  } else if (strcmp(cmd, "LOCK") == 0 && argc >= 2) {
    const bool locked = (atoi(tok[1]) != 0);
    ui_tft::UiConfig ucfg = ui_->getUiConfig();
    ucfg.panelLocked = locked;
    ui_->applyUiConfig(ucfg, true);
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"LOCK\",\"val\":%d}\n", locked ? 1 : 0);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "MASTER") == 0 && argc >= 2) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    if (strcasecmp(tok[1], "MUTE") == 0 && argc >= 3) {
      cfg.master.mute = (atoi(tok[2]) != 0);
    } else {
      cfg.master.volumeDb = clampf(static_cast<float>(atof(tok[1])), -60.0f, 0.0f);
    }
    dsp_->applyConfig(cfg);
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"MASTER\",\"vol\":%.1f,\"mute\":%d}\n",
                     cfg.master.volumeDb, cfg.master.mute ? 1 : 0);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "LINK") == 0 && argc >= 2) {
    char m[16];
    toUpperCopy(tok[1], m, sizeof(m));
    if (isdigit(static_cast<unsigned char>(m[0]))) {
      linkMask_ = static_cast<uint8_t>(atoi(m)) & LINK_ALL;
    } else if (strcmp(m, "NONE") == 0 || strcmp(m, "OFF") == 0) {
      linkMask_ = LINK_NONE;
    } else if (strcmp(m, "IN") == 0 || strcmp(m, "IN12") == 0) {
      linkMask_ = LINK_IN12;
    } else if (strcmp(m, "OUT12") == 0) {
      linkMask_ = LINK_OUT12;
    } else if (strcmp(m, "OUT34") == 0) {
      linkMask_ = LINK_OUT34;
    } else if (strcmp(m, "OUT") == 0) {
      linkMask_ = LINK_OUT;
    } else if (strcmp(m, "ALL") == 0) {
      linkMask_ = LINK_ALL;
    }
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"LINK\",\"mask\":%u}\n", linkMask_);
    if (reply) reply(resp, n);
  } else if (strcmp(cmd, "INFO") == 0) {
    cmdInfo(reply);
  } else if (strcmp(cmd, "RESTART") == 0 || strcmp(cmd, "REBOOT") == 0) {
    dsp::SystemConfig cfg = dsp_->getConfig();
    cfg.master.mute = true;
    dsp_->applyConfig(cfg);
    const char *msg = "{\"ok\":true,\"cmd\":\"RESTART\"}\n";
    if (reply) reply(msg, strlen(msg));
    vTaskDelay(pdMS_TO_TICKS(180));
    ESP.restart();
  } else if (strcmp(cmd, "LOGO") == 0 && argc >= 3) {
    cmdLogoStart(tok[1], static_cast<size_t>(strtoul(tok[2], nullptr, 10)), reply);
  } else if ((strcmp(cmd, "BLENAME") == 0 || strcmp(cmd, "BLEPIN") == 0) && argc >= 2) {
    if (radioCfgCb_ && radioCfgCb_(cmd, tok[1], resp, sizeof(resp))) {
      if (reply) reply(resp, strlen(resp));
    } else {
      const char *err = "{\"ok\":false,\"err\":\"RADIO_CFG_UNAVAILABLE\"}\n";
      if (reply) reply(err, strlen(err));
    }
  } else if (strcmp(cmd, "TELEM") == 0 && argc >= 2) {
    const int ms = atoi(tok[1]);
    telemIntervalMs_ = (ms <= 0) ? 0 : static_cast<uint16_t>(constrain(ms, 50, 500));
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"TELEM\",\"ms\":%u}\n", telemIntervalMs_);
    if (reply) reply(resp, n);
  } else {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"err\":\"UNKNOWN_CMD\",\"cmd\":\"%s\"}\n", cmd);
    if (reply) reply(resp, n);
  }
}

// ============================================================================
// 3. SET & GET PARAMETER LENGKAP (SELURUH PARAMETER DSP + STEREO LINK)
// ============================================================================
bool CommandParser::applyParamSingle(dsp::SystemConfig &cfg, const char *p,
                                     float fVal, const char *sVal) {
  // ---- A. INPUT STAGE (IN1 / INA / IN2 / INB — Direct Clean Input) ----
  if (strncmp(p, "IN1.", 4) == 0 || strncmp(p, "INA.", 4) == 0 ||
      strncmp(p, "IN2.", 4) == 0 || strncmp(p, "INB.", 4) == 0) {
    const int ch = (p[2] == '2' || p[2] == 'B') ? 1 : 0;
    const char *sub = p + 4;
    auto &in = cfg.inputs[ch];

    if (strcmp(sub, "GAIN") == 0)  { in.gainDb = clampf(fVal, -12.0f, 12.0f); return true; }
    if (strcmp(sub, "MUTE") == 0)  { in.mute = (fVal != 0.0f); return true; }
    if (strcmp(sub, "PHASE") == 0) { in.phaseInvert = (fVal != 0.0f); return true; }
    return false;
  }

  // ---- B. OUTPUT STAGE (OUT1..OUT4) ----
  if (strncmp(p, "OUT", 3) == 0 && p[3] >= '1' && p[3] <= '4' && p[4] == '.') {
    const int ch = p[3] - '1';
    const char *sub = p + 5;
    auto &out = cfg.outputs[ch];

    if (strcmp(sub, "GAIN") == 0)       { out.gainDb = clampf(fVal, -12.0f, 12.0f); return true; }
    if (strcmp(sub, "MUTE") == 0)       { out.mute = (fVal != 0.0f); return true; }
    if (strcmp(sub, "PHASE") == 0)      { out.phaseInvert = (fVal != 0.0f); return true; }
    if (strcmp(sub, "BYPASS") == 0)     { out.bypassDsp = (fVal != 0.0f); return true; }
    if (strcmp(sub, "DELAY") == 0)      { out.delayMs = clampf(fVal, 0.0f, dsp::DSP_MAX_DELAY_MS); return true; }
    if (strcmp(sub, "CLIP") == 0)       { out.antiClipMode = (fVal != 0.0f) ? dsp::AntiClipMode::HARD_CLIP : dsp::AntiClipMode::SOFT_TANH; return true; }

    auto applyXover = [&](dsp::XoverFilterConfig &xo, const char *fld) -> bool {
      if (strcmp(fld, "EN") == 0)    { xo.enabled = (fVal != 0.0f); return true; }
      if (strcmp(fld, "TYPE") == 0)  { xo.type = static_cast<dsp::XoverType>(constrain(parseXoverTypeToken(sVal), 0, 2)); return true; }
      if (strcmp(fld, "FREQ") == 0)  { xo.freqHz = dsp::DspEngine::quantizeFreqHalfHz(fVal); return true; }
      if (strcmp(fld, "SLOPE") == 0) {
        int sl = static_cast<int>(fVal);
        xo.slope = (sl <= 12) ? dsp::XoverSlope::SLOPE_12DB
                 : (sl >= 48 ? dsp::XoverSlope::SLOPE_48DB : dsp::XoverSlope::SLOPE_24DB);
        return true;
      }
      return false;
    };
    if (strncmp(sub, "HPF.", 4) == 0) return applyXover(out.hpf, sub + 4);
    if (strncmp(sub, "LPF.", 4) == 0) return applyXover(out.lpf, sub + 4);

    if (strcmp(sub, "GATE.EN") == 0)    { out.gate.enabled = (fVal != 0.0f); return true; }
    if (strcmp(sub, "GATE.THR") == 0)   { out.gate.thresholdDb = clampf(fVal, -90.0f, 0.0f); return true; }
    if (strcmp(sub, "GATE.ATK") == 0)   { out.gate.attackMs = clampf(fVal, 0.1f, 100.0f); return true; }
    if (strcmp(sub, "GATE.REL") == 0)   { out.gate.releaseMs = clampf(fVal, 10.0f, 2000.0f); return true; }
    if (strcmp(sub, "GATE.HOLD") == 0)  { out.gate.holdMs = clampf(fVal, 0.0f, 500.0f); return true; }
    if (strcmp(sub, "GATE.RANGE") == 0) { out.gate.rangeDb = clampf(fVal, -80.0f, 0.0f); return true; }

    if (strcmp(sub, "LIM.EN") == 0)     { out.limiter.enabled = (fVal != 0.0f); return true; }
    if (strcmp(sub, "LIM.THR") == 0)    { out.limiter.thresholdDb = clampf(fVal, -30.0f, 0.0f); return true; }
    if (strcmp(sub, "LIM.ATK") == 0)    { out.limiter.attackMs = clampf(fVal, 0.05f, 50.0f); return true; }
    if (strcmp(sub, "LIM.REL") == 0)    { out.limiter.releaseMs = clampf(fVal, 10.0f, 2000.0f); return true; }

    if (strncmp(sub, "PEQ", 3) == 0 && sub[3] >= '1' && sub[3] <= '9' && sub[4] == '.') {
      auto &b = out.peq[sub[3] - '1'];
      const char *fld = sub + 5;
      if (strcmp(fld, "EN") == 0)   { b.enabled = (fVal != 0.0f); return true; }
      if (strcmp(fld, "TYPE") == 0) { b.type = static_cast<dsp::PeqFilterType>(constrain(parsePeqTypeToken(sVal), 0, 6)); return true; }
      if (strcmp(fld, "FREQ") == 0) { b.freqHz = dsp::DspEngine::quantizeFreqHalfHz(fVal); return true; }
      if (strcmp(fld, "GAIN") == 0) { b.gainDb = clampf(fVal, -15.0f, 15.0f); return true; }
      if (strcmp(fld, "Q") == 0)    { b.q = clampf(fVal, 0.4f, 12.0f); return true; }
    }
    return false;
  }

  // ---- C. ROUTING MATRIX (`MATRIX.OUT1.IN1` atau `MATRIX.1.0`) ----
  if (strncmp(p, "MATRIX.", 7) == 0) {
    const char *sub = p + 7;
    int outIdx = -1;
    const char *dot = strchr(sub, '.');
    if (dot) {
      if (strncmp(sub, "OUT", 3) == 0 && sub[3] >= '1' && sub[3] <= '4') outIdx = sub[3] - '1';
      else if (sub[0] >= '1' && sub[0] <= '4') outIdx = sub[0] - '1';
      const int srcIdx = parseMatrixSrcToken(dot + 1);
      if (outIdx >= 0 && outIdx < 4 && srcIdx >= 0 && srcIdx < 4) {
        cfg.matrix.mixGain[outIdx][srcIdx] = clampf(fVal, 0.0f, 1.0f);
        return true;
      }
    }
    return false;
  }

  // ---- D. MASTER ----
  if (strcmp(p, "MASTER.VOL") == 0)  { cfg.master.volumeDb = clampf(fVal, -60.0f, 0.0f); return true; }
  if (strcmp(p, "MASTER.MUTE") == 0) { cfg.master.mute = (fVal != 0.0f); return true; }

  return false;
}

bool CommandParser::readParamSingle(const dsp::SystemConfig &cfg, const char *p,
                                    char *out, size_t sz) const {
  if (strncmp(p, "IN1.", 4) == 0 || strncmp(p, "INA.", 4) == 0 ||
      strncmp(p, "IN2.", 4) == 0 || strncmp(p, "INB.", 4) == 0) {
    const int ch = (p[2] == '2' || p[2] == 'B') ? 1 : 0;
    const char *sub = p + 4;
    const auto &in = cfg.inputs[ch];
    if (strcmp(sub, "GAIN") == 0)  { snprintf(out, sz, "%.2f", in.gainDb); return true; }
    if (strcmp(sub, "MUTE") == 0)  { snprintf(out, sz, "%d", in.mute ? 1 : 0); return true; }
    if (strcmp(sub, "PHASE") == 0) { snprintf(out, sz, "%d", in.phaseInvert ? 1 : 0); return true; }
    return false;
  }

  if (strncmp(p, "OUT", 3) == 0 && p[3] >= '1' && p[3] <= '4' && p[4] == '.') {
    const int ch = p[3] - '1';
    const char *sub = p + 5;
    const auto &o = cfg.outputs[ch];
    if (strcmp(sub, "GAIN") == 0)   { snprintf(out, sz, "%.2f", o.gainDb); return true; }
    if (strcmp(sub, "MUTE") == 0)   { snprintf(out, sz, "%d", o.mute ? 1 : 0); return true; }
    if (strcmp(sub, "PHASE") == 0)  { snprintf(out, sz, "%d", o.phaseInvert ? 1 : 0); return true; }
    if (strcmp(sub, "BYPASS") == 0) { snprintf(out, sz, "%d", o.bypassDsp ? 1 : 0); return true; }
    if (strcmp(sub, "DELAY") == 0)  { snprintf(out, sz, "%.2f", o.delayMs); return true; }
    if (strcmp(sub, "CLIP") == 0)   { snprintf(out, sz, "%u", static_cast<unsigned>(o.antiClipMode)); return true; }
    if (strcmp(sub, "HPF.EN") == 0)    { snprintf(out, sz, "%d", o.hpf.enabled ? 1 : 0); return true; }
    if (strcmp(sub, "HPF.TYPE") == 0)  { snprintf(out, sz, "%u", static_cast<unsigned>(o.hpf.type)); return true; }
    if (strcmp(sub, "HPF.SLOPE") == 0) { snprintf(out, sz, "%u", static_cast<unsigned>(o.hpf.slope)); return true; }
    if (strcmp(sub, "HPF.FREQ") == 0)  { snprintf(out, sz, "%.1f", o.hpf.freqHz); return true; }
    if (strcmp(sub, "LPF.EN") == 0)    { snprintf(out, sz, "%d", o.lpf.enabled ? 1 : 0); return true; }
    if (strcmp(sub, "LPF.TYPE") == 0)  { snprintf(out, sz, "%u", static_cast<unsigned>(o.lpf.type)); return true; }
    if (strcmp(sub, "LPF.SLOPE") == 0) { snprintf(out, sz, "%u", static_cast<unsigned>(o.lpf.slope)); return true; }
    if (strcmp(sub, "LPF.FREQ") == 0)  { snprintf(out, sz, "%.1f", o.lpf.freqHz); return true; }
    if (strcmp(sub, "GATE.EN") == 0)   { snprintf(out, sz, "%d", o.gate.enabled ? 1 : 0); return true; }
    if (strcmp(sub, "GATE.THR") == 0)  { snprintf(out, sz, "%.1f", o.gate.thresholdDb); return true; }
    if (strcmp(sub, "LIM.EN") == 0)    { snprintf(out, sz, "%d", o.limiter.enabled ? 1 : 0); return true; }
    if (strcmp(sub, "LIM.THR") == 0)   { snprintf(out, sz, "%.1f", o.limiter.thresholdDb); return true; }
    if (strcmp(sub, "LIM.ATK") == 0)   { snprintf(out, sz, "%.2f", o.limiter.attackMs); return true; }
    if (strcmp(sub, "LIM.REL") == 0)   { snprintf(out, sz, "%.1f", o.limiter.releaseMs); return true; }
    if (strncmp(sub, "PEQ", 3) == 0 && sub[3] >= '1' && sub[3] <= '9' && sub[4] == '.') {
      const auto &b = o.peq[sub[3] - '1'];
      const char *fld = sub + 5;
      if (strcmp(fld, "EN") == 0)   { snprintf(out, sz, "%d", b.enabled ? 1 : 0); return true; }
      if (strcmp(fld, "TYPE") == 0) { snprintf(out, sz, "%u", static_cast<unsigned>(b.type)); return true; }
      if (strcmp(fld, "FREQ") == 0) { snprintf(out, sz, "%.1f", b.freqHz); return true; }
      if (strcmp(fld, "GAIN") == 0) { snprintf(out, sz, "%.2f", b.gainDb); return true; }
      if (strcmp(fld, "Q") == 0)    { snprintf(out, sz, "%.2f", b.q); return true; }
    }
    return false;
  }

  if (strncmp(p, "MATRIX.", 7) == 0) {
    const char *sub = p + 7;
    const char *dot = strchr(sub, '.');
    if (dot) {
      int o = (strncmp(sub, "OUT", 3) == 0) ? (sub[3] - '1') : (sub[0] - '1');
      int s = parseMatrixSrcToken(dot + 1);
      if (o >= 0 && o < 4 && s >= 0 && s < 4) { snprintf(out, sz, "%.2f", cfg.matrix.mixGain[o][s]); return true; }
    }
    return false;
  }

  if (strcmp(p, "MASTER.VOL") == 0) { snprintf(out, sz, "%.1f", cfg.master.volumeDb); return true; }
  if (strcmp(p, "MASTER.MUTE") == 0){ snprintf(out, sz, "%d", cfg.master.mute ? 1 : 0); return true; }

  return false;
}

void CommandParser::cmdSet(const char *param, const char *valStr, const ReplyCallback &reply) {
  char pUp[48], vUp[32];
  toUpperCopy(param, pUp, sizeof(pUp));
  toUpperCopy(valStr, vUp, sizeof(vUp));
  const float fVal = static_cast<float>(atof(vUp));

  dsp::SystemConfig cfg = dsp_->getConfig();
  const bool ok = applyParamSingle(cfg, pUp, fVal, vUp);

  char resp[192];
  if (!ok) {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"cmd\":\"SET\",\"err\":\"BAD_PARAM\",\"param\":\"%s\"}\n", pUp);
    if (reply) reply(resp, n);
    return;
  }

  // Terapkan juga ke pasangan stereo bila LINK aktif
  char linkedP[48];
  const bool mirrored = makeLinkedParamName(pUp, linkMask_, linkedP, sizeof(linkedP));
  if (mirrored) {
    applyParamSingle(cfg, linkedP, fVal, vUp);
  }

  dsp_->applyConfig(cfg);

  int n = snprintf(resp, sizeof(resp),
                   "{\"ok\":true,\"cmd\":\"SET\",\"param\":\"%s\",\"val\":\"%s\",\"linked\":%d}\n",
                   pUp, vUp, mirrored ? 1 : 0);
  if (reply) reply(resp, n);
}

void CommandParser::cmdGet(const char *param, const ReplyCallback &reply) {
  char pUp[48], valBuf[32];
  toUpperCopy(param, pUp, sizeof(pUp));

  const dsp::SystemConfig cfg = dsp_->getConfig();
  char resp[160];
  if (readParamSingle(cfg, pUp, valBuf, sizeof(valBuf))) {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":true,\"cmd\":\"GET\",\"param\":\"%s\",\"val\":%s}\n", pUp, valBuf);
    if (reply) reply(resp, n);
  } else {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"cmd\":\"GET\",\"err\":\"BAD_PARAM\",\"param\":\"%s\"}\n", pUp);
    if (reply) reply(resp, n);
  }
}

// ============================================================================
// 4. DUMP KONFIGURASI LENGKAP (JSON RINGKAS UNTUK SINKRONISASI WEB APP / BLE)
// ============================================================================
void CommandParser::cmdDump(const ReplyCallback &reply) {
  if (!dumpBuf_ || dumpBufCap_ == 0 || !reply) return;

  const dsp::SystemConfig cfg = dsp_->getConfig();
  const ui_tft::UiConfig  ucfg = ui_->getUiConfig();

  char *b = dumpBuf_;
  size_t pos = 0;
  const size_t cap = dumpBufCap_;
  auto app = [&](const char *fmt, auto... args) {
    if (pos + 2 < cap) {
      int w = snprintf(b + pos, cap - pos, fmt, args...);
      if (w > 0) pos += static_cast<size_t>(w);
      if (pos >= cap) pos = cap - 1;
    }
  };
  auto appStr = [&](const char *s) {
    const size_t l = strlen(s);
    if (pos + l + 1 < cap) {
      memcpy(b + pos, s, l);
      pos += l;
      b[pos] = '\0';
    }
  };

  app("{\"type\":\"DUMP\",\"preset\":\"%s\",\"link\":%u,\"master\":{\"vol\":%.1f,\"mute\":%d},",
      cfg.presetName, linkMask_, cfg.master.volumeDb, cfg.master.mute ? 1 : 0);

  app("\"ui\":{\"theme\":%u,\"colorMode\":%u,\"standbyEn\":%d,\"standbySec\":%u,\"lock\":%d,\"wifi\":%u},",
      static_cast<unsigned>(ucfg.theme), static_cast<unsigned>(ucfg.panelMode),
      ucfg.standbyEnabled ? 1 : 0, ucfg.standbyTimeoutSec,
      ucfg.panelLocked ? 1 : 0, static_cast<unsigned>(ucfg.wifiMode));

  // Matrix 4x4
  appStr("\"matrix\":[");
  for (int o = 0; o < 4; o++) {
    app("%s[%.2f,%.2f,%.2f,%.2f]", o ? "," : "",
        cfg.matrix.mixGain[o][0], cfg.matrix.mixGain[o][1],
        cfg.matrix.mixGain[o][2], cfg.matrix.mixGain[o][3]);
  }
  appStr("],");

  // Input Stage (IN A / IN B — Direct Clean Input)
  appStr("\"in\":[");
  for (int ch = 0; ch < 2; ch++) {
    const auto &in = cfg.inputs[ch];
    app("%s{\"g\":%.1f,\"m\":%d,\"ph\":%d}",
        ch ? "," : "", in.gainDb, in.mute ? 1 : 0, in.phaseInvert ? 1 : 0);
  }
  appStr("],\"out\":[");

  // Output Stage (OUT 1-4)
  for (int ch = 0; ch < 4; ch++) {
    const auto &o = cfg.outputs[ch];
    app("%s{\"g\":%.1f,\"m\":%d,\"ph\":%d,\"byp\":%d,\"dly\":%.1f,\"clip\":%u,"
        "\"hpf\":[%d,%u,%u,%.1f],\"lpf\":[%d,%u,%u,%.1f],"
        "\"gate\":[%d,%.1f,%.1f,%.0f],\"lim\":[%d,%.1f,%.2f,%.0f],\"peq\":[",
        ch ? "," : "", o.gainDb, o.mute ? 1 : 0, o.phaseInvert ? 1 : 0,
        o.bypassDsp ? 1 : 0, o.delayMs, static_cast<unsigned>(o.antiClipMode),
        o.hpf.enabled ? 1 : 0, static_cast<unsigned>(o.hpf.type), static_cast<unsigned>(o.hpf.slope), o.hpf.freqHz,
        o.lpf.enabled ? 1 : 0, static_cast<unsigned>(o.lpf.type), static_cast<unsigned>(o.lpf.slope), o.lpf.freqHz,
        o.gate.enabled ? 1 : 0, o.gate.thresholdDb, o.gate.attackMs, o.gate.releaseMs,
        o.limiter.enabled ? 1 : 0, o.limiter.thresholdDb, o.limiter.attackMs, o.limiter.releaseMs);
    for (int k = 0; k < dsp::DSP_PEQ_BANDS; k++) {
      const auto &q = o.peq[k];
      app("%s[%d,%u,%.1f,%.1f,%.1f]", k ? "," : "",
          q.enabled ? 1 : 0, static_cast<unsigned>(q.type), q.freqHz, q.gainDb, q.q);
    }
    appStr("]}");
  }
  appStr("]}\n");

  reply(dumpBuf_, pos);
}

// ============================================================================
// 5. PRESET, COPY, INFO, & LOGO UPLOAD (BINARY / BASE64)
// ============================================================================
void CommandParser::cmdPreset(const char *action, const char *arg1, const char *arg2,
                              const ReplyCallback &reply) {
  char act[16];
  toUpperCopy(action, act, sizeof(act));
  const int slot = atoi(arg1);
  bool ok = false;

  if (slot >= 0 && slot < ui_tft::NUM_PRESET_SLOTS) {
    if (strcmp(act, "LOAD") == 0) {
      ok = ui_->loadPresetSlot(static_cast<uint8_t>(slot));
    } else if (strcmp(act, "SAVE") == 0) {
      ok = ui_->savePresetSlot(static_cast<uint8_t>(slot), arg2);
    } else if (strcmp(act, "DEL") == 0 || strcmp(act, "DELETE") == 0) {
      ok = ui_->deletePresetSlot(static_cast<uint8_t>(slot));
    } else if (strcmp(act, "RENAME") == 0 && arg2) {
      ok = ui_->renamePresetSlot(static_cast<uint8_t>(slot), arg2);
    }
  }

  char resp[128];
  int n = snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"PRESET\",\"act\":\"%s\",\"slot\":%d}\n",
                   ok ? "true" : "false", act, slot);
  if (reply) reply(resp, n);
}

void CommandParser::cmdCopy(const char *srcStr, const char *dstStr, const ReplyCallback &reply) {
  dsp::ChannelId src, dst;
  bool ok = parseChannelId(srcStr, src) && parseChannelId(dstStr, dst) &&
            dsp_->copyChannelSettings(src, dst);
  char resp[128];
  int n = snprintf(resp, sizeof(resp), "{\"ok\":%s,\"cmd\":\"COPY\",\"src\":\"%s\",\"dst\":\"%s\"}\n",
                   ok ? "true" : "false", srcStr, dstStr);
  if (reply) reply(resp, n);
}

void CommandParser::cmdInfo(const ReplyCallback &reply) {
  const audio_io::AudioTelemetry tel = audioIO_->getTelemetry();
  const ui_tft::UiConfig ucfg = ui_->getUiConfig();

  char resp[384];
  int n = snprintf(
      resp, sizeof(resp),
      "{\"ok\":true,\"cmd\":\"INFO\",\"fw\":\"%s\",\"build\":\"%s\","
      "\"cpuMhz\":%lu,\"tempC\":%.1f,\"tempPeakC\":%.1f,\"dspCpuPct\":%.1f,"
      "\"freeHeap\":%lu,\"freePsram\":%lu,\"fsTotal\":%lu,\"fsUsed\":%lu,"
      "\"usb\":%d,\"wifiMode\":%u,\"ble\":%d,\"lock\":%d}\n",
      FW_VERSION_STR, __DATE__,
      static_cast<unsigned long>(getCpuFrequencyMhz()),
      tel.chipTempC, tel.chipTempPeakC, tel.dspCpuLoadPct,
      static_cast<unsigned long>(ESP.getFreeHeap()),
      static_cast<unsigned long>(ESP.getFreePsram()),
      static_cast<unsigned long>(LittleFS.totalBytes()),
      static_cast<unsigned long>(LittleFS.usedBytes()),
      tel.usbMounted ? 1 : 0,
      static_cast<unsigned>(ucfg.wifiMode),
      ucfg.bleEnabled ? 1 : 0,
      ucfg.panelLocked ? 1 : 0);
  if (reply) reply(resp, n);
}

void CommandParser::abortLogoTransfer() {
  if (logoFile_) {
    logoFile_.close();
  }
  LittleFS.remove("/logo_upload.tmp");
  logoActive_   = false;
  logoExpected_ = 0;
  logoReceived_ = 0;
}

void CommandParser::cmdLogoStart(const char *targetStr, size_t totalBytes,
                                 const ReplyCallback &reply) {
  abortLogoTransfer();

  char tUp[16];
  toUpperCopy(targetStr, tUp, sizeof(tUp));
  const bool isBoot    = (strcmp(tUp, "BOOT") == 0);
  const bool isStandby = (strcmp(tUp, "STANDBY") == 0);

  char resp[144];
  if ((!isBoot && !isStandby) || totalBytes == 0 || totalBytes > MAX_LOGO_BYTES) {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"cmd\":\"LOGO\",\"err\":\"INVALID_ARGS\"}\n");
    if (reply) reply(resp, n);
    return;
  }

  logoFile_ = LittleFS.open("/logo_upload.tmp", "w");
  if (!logoFile_) {
    int n = snprintf(resp, sizeof(resp), "{\"ok\":false,\"cmd\":\"LOGO\",\"err\":\"FS_OPEN_FAIL\"}\n");
    if (reply) reply(resp, n);
    return;
  }

  logoActive_      = true;
  logoIsStandby_   = isStandby;
  logoExpected_    = totalBytes;
  logoReceived_    = 0;
  logoLastChunkMs_ = millis();

  int n = snprintf(resp, sizeof(resp),
                   "{\"ok\":true,\"cmd\":\"LOGO\",\"status\":\"READY\",\"target\":\"%s\",\"size\":%u}\n",
                   tUp, static_cast<unsigned>(totalBytes));
  if (reply) reply(resp, n);
}

bool CommandParser::handleBinaryChunk(const uint8_t *data, size_t len,
                                      const ReplyCallback &reply) {
  if (!logoActive_ || !data || len == 0) return false;
  return appendLogoBytes(data, len, reply);
}

bool CommandParser::appendLogoBytes(const uint8_t *bytes, size_t len,
                                    const ReplyCallback &reply) {
  if (!logoActive_ || !logoFile_) return false;

  const size_t remain  = logoExpected_ - logoReceived_;
  const size_t toWrite = (len > remain) ? remain : len;
  if (toWrite > 0) {
    const size_t written = logoFile_.write(bytes, toWrite);
    logoReceived_ += written;
    logoLastChunkMs_ = millis();
  }

  if (logoReceived_ >= logoExpected_) {
    finishLogoTransfer(reply);
  }
  return true;
}

void CommandParser::finishLogoTransfer(const ReplyCallback &reply) {
  if (logoFile_) logoFile_.close();
  const char *finalPath = logoIsStandby_ ? "/logo_standby.bin" : "/logo_boot.bin";
  LittleFS.remove(finalPath);
  const bool renamed = LittleFS.rename("/logo_upload.tmp", finalPath);
  const size_t saved = logoReceived_;

  logoActive_   = false;
  logoExpected_ = 0;
  logoReceived_ = 0;

  if (renamed && ui_) {
    ui_->reloadLogos();
  }

  char resp[144];
  int n = snprintf(resp, sizeof(resp),
                   "{\"ok\":%s,\"cmd\":\"LOGO\",\"status\":\"DONE\",\"path\":\"%s\",\"bytes\":%u}\n",
                   renamed ? "true" : "false", finalPath, static_cast<unsigned>(saved));
  if (reply) reply(resp, n);
}

// ============================================================================
// 6. TELEMETRY STREAM JSON (6-CH VU, RMS, OUTPUT GR, CLIP, LIMITER, GATE)
// ============================================================================
size_t CommandParser::buildTelemetryJson(char *outBuf, size_t outSize,
                                         const audio_io::AudioTelemetry &audioTel,
                                         const dsp::DspMeters &m) const {
  if (!outBuf || outSize < 128) return 0;

  const uint8_t clipMask =
      (m.inClip[0]  ? 0x01u : 0u) | (m.inClip[1]  ? 0x02u : 0u) |
      (m.outClip[0] ? 0x04u : 0u) | (m.outClip[1] ? 0x08u : 0u) |
      (m.outClip[2] ? 0x10u : 0u) | (m.outClip[3] ? 0x20u : 0u);

  const uint8_t limMask =
      (m.outLimiterActive[0] ? 0x01u : 0u) | (m.outLimiterActive[1] ? 0x02u : 0u) |
      (m.outLimiterActive[2] ? 0x04u : 0u) | (m.outLimiterActive[3] ? 0x08u : 0u);

  const uint8_t gateMask =
      (m.outGateClosed[0] ? 0x04u : 0u) | (m.outGateClosed[1] ? 0x08u : 0u) |
      (m.outGateClosed[2] ? 0x10u : 0u) | (m.outGateClosed[3] ? 0x20u : 0u);

  const int pos = snprintf(
      outBuf, outSize,
      "{\"t\":\"M\",\"vu\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
      "\"rms\":[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f],"
      "\"gr\":[0.0,0.0,%.1f,%.1f,%.1f,%.1f],"
      "\"clip\":%u,\"lim\":%u,\"gate\":%u,\"temp\":%.1f,\"cpu\":%.1f}\n",
      m.inPeakDb[0], m.inPeakDb[1], m.outPeakDb[0], m.outPeakDb[1], m.outPeakDb[2], m.outPeakDb[3],
      m.inRmsDb[0],  m.inRmsDb[1],  m.outRmsDb[0],  m.outRmsDb[1],  m.outRmsDb[2],  m.outRmsDb[3],
      m.outLimiterGrDb[0], m.outLimiterGrDb[1], m.outLimiterGrDb[2], m.outLimiterGrDb[3],
      clipMask, limMask, gateMask, audioTel.chipTempC, audioTel.dspCpuLoadPct);

  if (pos < 0 || static_cast<size_t>(pos) >= outSize) return 0;
  return static_cast<size_t>(pos);
}

} // namespace conn
