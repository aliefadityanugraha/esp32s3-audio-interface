/**
 * @file dsp_engine.cpp
 * @brief Implementasi Real-Time 32-Bit Float DSP Audio Interface 2-In 4-Out
 *        Input Stage bersih (Gain/Mute/Phase) + Output Stage penuh (OUT 1..4)
 */

#include "dsp_engine.h"

#include <cmath>
#include <cstring>

#if defined(ESP32) && !defined(NATIVE_TEST)
  #include "esp_heap_caps.h"
#endif

namespace dsp {

namespace {

inline float flushSubnormal(float v) {
  if (!std::isfinite(v) || fabsf(v) < 1.0e-20f) return 0.0f;
  return v;
}

constexpr double PI_D        = 3.14159265358979323846;
constexpr float  DB_FLOOR    = -96.0f;
constexpr float  LIN_FLOOR   = 1.5848932e-5f; // 10^(-96/20)
constexpr float  GAIN_SMOOTH = 0.0025f;       // ~8.3 ms ramp @ 48 kHz
constexpr float  CLIP_THRESH = 0.995f;        // -0.04 dBFS

inline float fastLog2f(float x) {
  if (x <= 1.0e-12f) return -40.0f;
  uint32_t u = 0;
  memcpy(&u, &x, sizeof(u));
  const int32_t e = static_cast<int32_t>((u >> 23) & 0xFFu) - 127;
  uint32_t mu = (u & 0x007FFFFFu) | 0x3F800000u;
  float m = 1.0f;
  memcpy(&m, &mu, sizeof(m));
  const float f = m - 1.0f;
  const float p = f * (1.44269504f + f * (-0.72134752f + f * (0.48089835f +
                       f * (-0.28089835f + 0.07865248f * f))));
  return static_cast<float>(e) + p;
}

inline float fastLinearToDb(float lin) {
  if (lin <= LIN_FLOOR) return DB_FLOOR;
  return 6.02059991f * fastLog2f(lin);
}

inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

inline float msToOnePoleCoeff(float timeMs, float sampleRate) {
  if (timeMs <= 0.01f) return 1.0f;
  const float samples = (timeMs * 0.001f) * sampleRate;
  return 1.0f - expf(-1.0f / samples);
}

// ============================================================================
// TABEL POLE CROSSOVER EKSAK (BUTTERWORTH, LINKWITZ-RILEY, BESSEL -3 dB NORM)
// ============================================================================
struct XoverSectionSpec {
  float freqScale;
  float q;
};

constexpr XoverSectionSpec BW_12[1] = {{1.0f, 0.70710678f}};
constexpr XoverSectionSpec BW_24[2] = {{1.0f, 0.54119610f}, {1.0f, 1.30656296f}};
constexpr XoverSectionSpec BW_48[4] = {
    {1.0f, 0.50979558f}, {1.0f, 0.60134489f},
    {1.0f, 0.89997622f}, {1.0f, 2.56291545f}};

constexpr XoverSectionSpec LR_12[1] = {{1.0f, 0.50000000f}};
constexpr XoverSectionSpec LR_24[2] = {{1.0f, 0.70710678f}, {1.0f, 0.70710678f}};
constexpr XoverSectionSpec LR_48[4] = {
    {1.0f, 0.54119610f}, {1.0f, 1.30656296f},
    {1.0f, 0.54119610f}, {1.0f, 1.30656296f}};

constexpr XoverSectionSpec BES_12[1] = {{1.27201965f, 0.57735027f}};
constexpr XoverSectionSpec BES_24[2] = {
    {1.43017156f, 0.52193458f}, {1.60335752f, 0.80553828f}};
constexpr XoverSectionSpec BES_48[4] = {
    {1.77846591f, 0.50599107f}, {1.83209260f, 0.55960916f},
    {1.95319576f, 0.71085207f}, {2.18872623f, 1.22566943f}};

constexpr float DEFAULT_PEQ_FREQS[DSP_PEQ_BANDS] = {
    40.0f, 80.0f, 160.0f, 315.0f, 630.0f, 1250.0f, 2500.0f, 5000.0f, 10000.0f};

void designSecondOrderFilter(BiquadState &bq, float fcHz, float q,
                             bool isHighPass, float sampleRate) {
  const double fs = static_cast<double>(sampleRate);
  const double fc = clampf(fcHz, 10.0f, sampleRate * 0.475f);
  const double w0 = 2.0 * PI_D * fc / fs;
  const double cosW0 = cos(w0);
  const double sinW0 = sin(w0);
  const double alpha = sinW0 / (2.0 * static_cast<double>(q));

  double b0 = 1.0, b1 = 0.0, b2 = 0.0;
  const double a0 = 1.0 + alpha;
  const double a1 = -2.0 * cosW0;
  const double a2 = 1.0 - alpha;

  if (isHighPass) {
    b0 = (1.0 + cosW0) * 0.5;
    b1 = -(1.0 + cosW0);
    b2 = (1.0 + cosW0) * 0.5;
  } else {
    b0 = (1.0 - cosW0) * 0.5;
    b1 = (1.0 - cosW0);
    b2 = (1.0 - cosW0) * 0.5;
  }

  const double invA0 = 1.0 / a0;
  bq.b0 = static_cast<float>(b0 * invA0);
  bq.b1 = static_cast<float>(b1 * invA0);
  bq.b2 = static_cast<float>(b2 * invA0);
  bq.a1 = static_cast<float>(a1 * invA0);
  bq.a2 = static_cast<float>(a2 * invA0);
  bq.active = true;
}

} // namespace

// ============================================================================
// 1. CROSSOVER CHAIN (LR / BUTTERWORTH / BESSEL @ 12, 24, 48 dB/OCT)
// ============================================================================
void CrossoverChain::reset() {
  for (uint8_t i = 0; i < DSP_MAX_XOVER_STAGES; i++) {
    stages[i].reset();
  }
}

void CrossoverChain::configure(const XoverFilterConfig &cfg, bool isHighPass,
                               float sampleRate) {
  if (!cfg.enabled) {
    if (active) {
      reset();
    }
    active = false;
    numStages = 0;
    return;
  }

  const XoverSectionSpec *specs = BW_24;
  uint8_t count = 2;

  switch (cfg.type) {
    case XoverType::LINKWITZ_RILEY:
      if (cfg.slope == XoverSlope::SLOPE_12DB) { specs = LR_12; count = 1; }
      else if (cfg.slope == XoverSlope::SLOPE_48DB) { specs = LR_48; count = 4; }
      else { specs = LR_24; count = 2; }
      break;

    case XoverType::BESSEL:
      if (cfg.slope == XoverSlope::SLOPE_12DB) { specs = BES_12; count = 1; }
      else if (cfg.slope == XoverSlope::SLOPE_48DB) { specs = BES_48; count = 4; }
      else { specs = BES_24; count = 2; }
      break;

    case XoverType::BUTTERWORTH:
    default:
      if (cfg.slope == XoverSlope::SLOPE_12DB) { specs = BW_12; count = 1; }
      else if (cfg.slope == XoverSlope::SLOPE_48DB) { specs = BW_48; count = 4; }
      else { specs = BW_24; count = 2; }
      break;
  }

  const float baseFc = clampf(cfg.freqHz, 16.0f, 20000.0f);
  for (uint8_t i = 0; i < count; i++) {
    const float secFc = isHighPass ? (baseFc / specs[i].freqScale)
                                   : (baseFc * specs[i].freqScale);
    designSecondOrderFilter(stages[i], secFc, specs[i].q, isHighPass, sampleRate);
  }
  for (uint8_t i = count; i < DSP_MAX_XOVER_STAGES; i++) {
    stages[i].active = false;
    stages[i].reset();
  }

  numStages = count;
  active = true;
}

// ============================================================================
// 2. OUTPUT NOISE GATE
// ============================================================================
void NoiseGateState::reset() {
  envLin = 0.0f;
  gainLin = 1.0f;
  holdCounter = 0;
  gateOpen = true;
}

void NoiseGateState::configure(const NoiseGateConfig &cfg, float sampleRate) {
  enabled     = cfg.enabled;
  threshLin   = DspEngine::dbToLinear(clampf(cfg.thresholdDb, -90.0f, 0.0f));
  rangeLin    = DspEngine::dbToLinear(clampf(cfg.rangeDb, -80.0f, 0.0f));
  atkCoeff    = msToOnePoleCoeff(clampf(cfg.attackMs, 0.1f, 100.0f), sampleRate);
  relCoeff    = msToOnePoleCoeff(clampf(cfg.releaseMs, 10.0f, 2000.0f), sampleRate);
  holdSamples = static_cast<uint32_t>((clampf(cfg.holdMs, 0.0f, 500.0f) * 0.001f) * sampleRate);
}

float NoiseGateState::process(float x, float antiDenormalOffset) {
  if (!enabled) {
    gateOpen = true;
    return x;
  }

  const float absX = fabsf(x) + fabsf(antiDenormalOffset);
  if (absX > envLin) {
    envLin += (absX - envLin) * 0.04f;
  } else {
    envLin += (absX - envLin) * 0.0008f;
  }
  if (envLin < 1.0e-20f) envLin = 0.0f;

  const float openThresh  = threshLin;
  const float closeThresh = threshLin * 0.8414f; // -1.5 dB hysteresis

  if (envLin >= openThresh) {
    gateOpen = true;
    holdCounter = holdSamples;
  } else if (envLin < closeThresh) {
    if (holdCounter > 0) {
      holdCounter--;
    } else {
      gateOpen = false;
    }
  }

  const float targetGain = gateOpen ? 1.0f : rangeLin;
  const float coeff = (targetGain > gainLin) ? atkCoeff : relCoeff;
  gainLin += (targetGain - gainLin) * coeff;
  if (gainLin < 1.0e-20f) gainLin = 0.0f;

  return x * gainLin;
}

// ============================================================================
// 3. OUTPUT PEAK LIMITER
// ============================================================================
void PeakLimiterState::reset() {
  envLin  = 0.0f;
  gainLin = 1.0f;
  grDb    = 0.0f;
}

void PeakLimiterState::configure(const PeakLimiterConfig &cfg, float sampleRate) {
  enabled   = cfg.enabled;
  threshLin = DspEngine::dbToLinear(clampf(cfg.thresholdDb, -30.0f, 0.0f));
  atkCoeff  = msToOnePoleCoeff(clampf(cfg.attackMs, 0.05f, 50.0f), sampleRate);
  relCoeff  = msToOnePoleCoeff(clampf(cfg.releaseMs, 10.0f, 2000.0f), sampleRate);
}

float PeakLimiterState::process(float x, float antiDenormalOffset) {
  if (!enabled) {
    grDb = 0.0f;
    return x;
  }

  const float absX = fabsf(x) + fabsf(antiDenormalOffset);
  if (absX > envLin) {
    envLin += (absX - envLin) * atkCoeff;
  } else {
    envLin += (absX - envLin) * relCoeff;
  }
  if (envLin < 1.0e-20f) envLin = 0.0f;

  gainLin = (envLin > threshLin && envLin > 1.0e-9f) ? (threshLin / envLin) : 1.0f;
  if (gainLin < 1.0e-5f) gainLin = 1.0e-5f;

  grDb = (gainLin < 0.9995f) ? fastLinearToDb(gainLin) : 0.0f;
  return x * gainLin;
}

// ============================================================================
// 4. DELAY LINE HINGGA 300 MS DI PSRAM
// ============================================================================
bool DelayLinePsram::allocate(float maxDelayMs, float sampleRate) {
  release();
  capacity = static_cast<size_t>(ceilf((maxDelayMs * 0.001f) * sampleRate)) + 8;
  const size_t bytes = capacity * sizeof(float);

#if defined(ESP32) && !defined(NATIVE_TEST)
  buffer = static_cast<float *>(ps_malloc(bytes));
  if (buffer != nullptr) {
    inPsram = true;
  } else {
    buffer = static_cast<float *>(malloc(bytes));
    inPsram = false;
  }
#else
  buffer = static_cast<float *>(malloc(bytes));
  inPsram = false;
#endif

  if (buffer == nullptr) {
    capacity = 0;
    return false;
  }

  clear();
  return true;
}

void DelayLinePsram::release() {
  if (buffer != nullptr) {
    free(buffer);
    buffer = nullptr;
  }
  capacity = 0;
  writeIdx = 0;
  targetDelay = 0.0f;
  currentDelay = 0.0f;
  inPsram = false;
}

void DelayLinePsram::clear() {
  if (buffer != nullptr && capacity > 0) {
    memset(buffer, 0, capacity * sizeof(float));
  }
  writeIdx = 0;
}

void DelayLinePsram::setDelayMs(float delayMs, float sampleRate, bool snapImmediate) {
  const float clampedMs = clampf(delayMs, 0.0f, DSP_MAX_DELAY_MS);
  float samples = static_cast<float>((static_cast<double>(clampedMs) * 0.001) *
                                     static_cast<double>(sampleRate));
  if (capacity > 4 && samples > static_cast<float>(capacity - 4)) {
    samples = static_cast<float>(capacity - 4);
  }
  targetDelay = samples;
  if (snapImmediate) {
    currentDelay = targetDelay;
  }
}

float DelayLinePsram::process(float x) {
  if (buffer == nullptr || capacity == 0) {
    return x;
  }

  buffer[writeIdx] = x;

  const float diff = targetDelay - currentDelay;
  if (fabsf(diff) > 0.01f) {
    currentDelay += diff * 0.002f;
  } else {
    currentDelay = targetDelay;
  }

  float y = x;
  if (currentDelay > 1.0e-4f) {
    const float dFloor = floorf(currentDelay);
    const float frac   = currentDelay - dFloor;
    size_t dInt        = static_cast<size_t>(dFloor);
    if (dInt >= capacity) dInt = capacity - 1;

    size_t idx0 = (writeIdx >= dInt) ? (writeIdx - dInt) : (writeIdx + capacity - dInt);
    if (frac < 1.0e-4f) {
      y = buffer[idx0];
    } else {
      size_t idx1 = (idx0 == 0) ? (capacity - 1) : (idx0 - 1);
      y = buffer[idx0] + frac * (buffer[idx1] - buffer[idx0]);
    }
  }

  writeIdx++;
  if (writeIdx >= capacity) writeIdx = 0;
  return y;
}

// ============================================================================
// 5. DSP ENGINE: LIFECYCLE, DEFAULT CONFIG & HELPER
// ============================================================================
DspEngine::~DspEngine() {
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outDelay_[ch].release();
  }
}

float DspEngine::quantizeFreqHalfHz(float freqHz) {
  const float clamped = clampf(freqHz, 16.0f, 20000.0f);
  return roundf(clamped * 2.0f) * 0.5f;
}

float DspEngine::dbToLinear(float db) {
  if (db <= DB_FLOOR) return 0.0f;
  return powf(10.0f, db * 0.05f);
}

float DspEngine::linearToDb(float lin) {
  if (lin <= LIN_FLOOR) return DB_FLOOR;
  return 20.0f * log10f(lin);
}

void DspEngine::loadDefaultConfig(SystemConfig &cfg) {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic   = DSP_CONFIG_MAGIC;
  cfg.version = DSP_CONFIG_VERSION;
  strncpy(cfg.presetName, "Default 2x4", sizeof(cfg.presetName) - 1);

  // 1. Input Stage Bersih (IN A & IN B: Gain, Mute, Phase saja)
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    cfg.inputs[ch].gainDb      = 0.0f;
    cfg.inputs[ch].mute        = false;
    cfg.inputs[ch].phaseInvert = false;
  }

  // 2. Routing Matrix Default
  memset(cfg.matrix.mixGain, 0, sizeof(cfg.matrix.mixGain));
  cfg.matrix.mixGain[0][SRC_IN1]   = 1.0f;
  cfg.matrix.mixGain[0][SRC_USB_L] = 1.0f;

  cfg.matrix.mixGain[1][SRC_IN2]   = 1.0f;
  cfg.matrix.mixGain[1][SRC_USB_R] = 1.0f;

  cfg.matrix.mixGain[2][SRC_IN1]   = 1.0f;
  cfg.matrix.mixGain[3][SRC_IN2]   = 1.0f;

  // 3. Output Stage Penuh (OUT 1..4)
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    auto &out = cfg.outputs[ch];
    out.gainDb      = 0.0f;
    out.mute        = false;
    out.phaseInvert = false;
    out.bypassDsp   = false;

    out.hpf.enabled = false;
    out.hpf.type    = XoverType::LINKWITZ_RILEY;
    out.hpf.slope   = XoverSlope::SLOPE_24DB;
    out.hpf.freqHz  = 80.0f;

    out.lpf.enabled = false;
    out.lpf.type    = XoverType::LINKWITZ_RILEY;
    out.lpf.slope   = XoverSlope::SLOPE_24DB;
    out.lpf.freqHz  = 18000.0f;

    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      out.peq[b].enabled = true;
      out.peq[b].type    = PeqFilterType::PEAKING;
      out.peq[b].freqHz  = DEFAULT_PEQ_FREQS[b];
      out.peq[b].gainDb  = 0.0f;
      out.peq[b].q       = 1.4f;
    }

    out.delayMs = 0.0f;

    out.gate.enabled     = true;
    out.gate.thresholdDb = -55.0f;
    out.gate.attackMs    = 2.0f;
    out.gate.releaseMs   = 250.0f;
    out.gate.holdMs      = 120.0f;
    out.gate.rangeDb     = -80.0f;

    out.limiter.enabled     = true;
    out.limiter.thresholdDb = -1.0f;
    out.limiter.attackMs    = 0.5f;
    out.limiter.releaseMs   = 100.0f;
    out.antiClipMode        = AntiClipMode::SOFT_TANH;
  }

  // 4. Master Volume
  cfg.master.volumeDb = 0.0f;
  cfg.master.mute     = false;
}

bool DspEngine::begin(float sampleRate) {
  sampleRate_ = sampleRate;
  loadDefaultConfig(pendingCfg_);
  activeCfg_ = pendingCfg_;

  bool allPsram = true;
  bool allocOk  = true;
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    if (!outDelay_[ch].allocate(DSP_MAX_DELAY_MS, sampleRate_)) {
      allocOk = false;
    }
    if (!outDelay_[ch].inPsram) {
      allPsram = false;
    }
  }

  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outHpf_[ch].reset();
    outLpf_[ch].reset();
    outGate_[ch].reset();
    outLimiter_[ch].reset();
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) outPeq_[ch][b].reset();
  }

  portENTER_CRITICAL(&meterMux_);
  memset(&meters_, 0, sizeof(meters_));
  for (uint8_t i = 0; i < DSP_NUM_INPUTS; i++) {
    meters_.inPeakDb[i] = DB_FLOOR;
    meters_.inRmsDb[i]  = DB_FLOOR;
  }
  for (uint8_t i = 0; i < DSP_NUM_OUTPUTS; i++) {
    meters_.outPeakDb[i] = DB_FLOOR;
    meters_.outRmsDb[i]  = DB_FLOOR;
  }
  meters_.delayAllocatedInPsram = allPsram;
  portEXIT_CRITICAL(&meterMux_);

  configDirty_   = true;
  forceFullSync_ = true;
  syncPendingParameters();

  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) inGainSmooth_[ch] = inGainTarget_[ch];
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outGainSmooth_[ch] = outGainTarget_[ch];
    for (uint8_t s = 0; s < DSP_MATRIX_SOURCES; s++) {
      matrixGainSmooth_[ch][s] = activeCfg_.matrix.mixGain[ch][s];
    }
  }
  masterGainSmooth_ = masterGainTarget_;

  initialized_ = allocOk;
  return allocOk;
}

void DspEngine::applyConfig(const SystemConfig &newCfg) {
  portENTER_CRITICAL(&configMux_);
  pendingCfg_ = newCfg;
  configDirty_ = true;
  portEXIT_CRITICAL(&configMux_);
}

SystemConfig DspEngine::getConfig() const {
  SystemConfig copy = {};
  portENTER_CRITICAL(&configMux_);
  copy = pendingCfg_;
  portEXIT_CRITICAL(&configMux_);
  return copy;
}

void DspEngine::snapAndResetStates() {
  configDirty_   = true;
  forceFullSync_ = true;
  syncPendingParameters();
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    inGainSmooth_[ch] = inGainTarget_[ch];
  }
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outGainSmooth_[ch] = outGainTarget_[ch];
    for (uint8_t s = 0; s < DSP_MATRIX_SOURCES; s++) {
      matrixGainSmooth_[ch][s] = activeCfg_.matrix.mixGain[ch][s];
    }
    outHpf_[ch].reset();
    outLpf_[ch].reset();
    outGate_[ch].reset();
    outLimiter_[ch].reset();
    outDelay_[ch].clear();
    outDelay_[ch].setDelayMs(activeCfg_.outputs[ch].delayMs, sampleRate_, true);
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) outPeq_[ch][b].reset();
  }
  masterGainSmooth_ = masterGainTarget_;
}

bool DspEngine::copyChannelSettings(ChannelId src, ChannelId dst) {
  if (src == dst) return true;

  portENTER_CRITICAL(&configMux_);
  bool ok = false;

  const int8_t s = static_cast<int8_t>(src);
  const int8_t d = static_cast<int8_t>(dst);

  if ((s == 0 || s == 1) && (d == 0 || d == 1)) {
    pendingCfg_.inputs[d] = pendingCfg_.inputs[s];
    configDirty_ = true;
    ok = true;
  } else if (s >= 2 && s <= 5) {
    const uint8_t srcOut = static_cast<uint8_t>(s - 2);
    if (dst == ChannelId::ALL_OUTPUTS) {
      for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
        if (ch != srcOut) {
          pendingCfg_.outputs[ch] = pendingCfg_.outputs[srcOut];
        }
      }
      configDirty_ = true;
      ok = true;
    } else if (d >= 2 && d <= 5) {
      const uint8_t dstOut = static_cast<uint8_t>(d - 2);
      pendingCfg_.outputs[dstOut] = pendingCfg_.outputs[srcOut];
      configDirty_ = true;
      ok = true;
    }
  }

  portEXIT_CRITICAL(&configMux_);
  return ok;
}

// ============================================================================
// 6. DESAIN KOEFISIEN BIQUAD PEQ (OUTPUT 1..4)
// ============================================================================
void DspEngine::designPeqBiquad(BiquadState &bq, const PeqBandConfig &cfg,
                                float sampleRate) {
  if (!cfg.enabled) {
    if (bq.active) bq.reset();
    bq.active = false;
    return;
  }

  const float gainDb = clampf(cfg.gainDb, -15.0f, 15.0f);
  const bool isGainFilter = (cfg.type == PeqFilterType::PEAKING ||
                             cfg.type == PeqFilterType::LOW_SHELF ||
                             cfg.type == PeqFilterType::HIGH_SHELF);
  if (isGainFilter && fabsf(gainDb) < 0.05f) {
    if (bq.active) bq.reset();
    bq.active = false;
    return;
  }

  const double fs    = static_cast<double>(sampleRate);
  const double fc    = static_cast<double>(quantizeFreqHalfHz(cfg.freqHz));
  const double Q     = static_cast<double>(clampf(cfg.q, 0.4f, 12.0f));
  const double A     = pow(10.0, static_cast<double>(gainDb) / 40.0);
  const double w0    = 2.0 * PI_D * fc / fs;
  const double cosW0 = cos(w0);
  const double sinW0 = sin(w0);
  const double alpha = sinW0 / (2.0 * Q);

  double b0 = 1.0, b1 = 0.0, b2 = 0.0;
  double a0 = 1.0, a1 = 0.0, a2 = 0.0;

  switch (cfg.type) {
    case PeqFilterType::PEAKING:
      b0 = 1.0 + alpha * A; b1 = -2.0 * cosW0; b2 = 1.0 - alpha * A;
      a0 = 1.0 + alpha / A; a1 = -2.0 * cosW0; a2 = 1.0 - alpha / A;
      break;

    case PeqFilterType::LOW_SHELF: {
      // Formula RBJ Q-form: selalu riil, mulus, dan bebas NaN untuk seluruh Q (0.4..12.0)
      const double twoSqrtAAlpha = 2.0 * sqrt(A) * alpha;
      b0 = A * ((A + 1.0) - (A - 1.0) * cosW0 + twoSqrtAAlpha);
      b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cosW0);
      b2 = A * ((A + 1.0) - (A - 1.0) * cosW0 - twoSqrtAAlpha);
      a0 = (A + 1.0) + (A - 1.0) * cosW0 + twoSqrtAAlpha;
      a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cosW0);
      a2 = (A + 1.0) + (A - 1.0) * cosW0 - twoSqrtAAlpha;
      break;
    }

    case PeqFilterType::HIGH_SHELF: {
      const double twoSqrtAAlpha = 2.0 * sqrt(A) * alpha;
      b0 = A * ((A + 1.0) + (A - 1.0) * cosW0 + twoSqrtAAlpha);
      b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cosW0);
      b2 = A * ((A + 1.0) + (A - 1.0) * cosW0 - twoSqrtAAlpha);
      a0 = (A + 1.0) - (A - 1.0) * cosW0 + twoSqrtAAlpha;
      a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cosW0);
      a2 = (A + 1.0) - (A - 1.0) * cosW0 - twoSqrtAAlpha;
      break;
    }

    case PeqFilterType::LOW_PASS:
      b0 = (1.0 - cosW0) * 0.5; b1 = (1.0 - cosW0); b2 = b0;
      a0 = 1.0 + alpha; a1 = -2.0 * cosW0; a2 = 1.0 - alpha;
      break;

    case PeqFilterType::HIGH_PASS:
      b0 = (1.0 + cosW0) * 0.5; b1 = -(1.0 + cosW0); b2 = b0;
      a0 = 1.0 + alpha; a1 = -2.0 * cosW0; a2 = 1.0 - alpha;
      break;

    case PeqFilterType::NOTCH:
      b0 = 1.0; b1 = -2.0 * cosW0; b2 = 1.0;
      a0 = 1.0 + alpha; a1 = -2.0 * cosW0; a2 = 1.0 - alpha;
      break;

    case PeqFilterType::ALL_PASS:
      b0 = 1.0 - alpha; b1 = -2.0 * cosW0; b2 = 1.0 + alpha;
      a0 = 1.0 + alpha; a1 = -2.0 * cosW0; a2 = 1.0 - alpha;
      break;
  }

  if (fabs(a0) < 1.0e-12) {
    bq.active = false;
    return;
  }

  const double invA0 = 1.0 / a0;
  bq.b0 = static_cast<float>(b0 * invA0);
  bq.b1 = static_cast<float>(b1 * invA0);
  bq.b2 = static_cast<float>(b2 * invA0);
  bq.a1 = static_cast<float>(a1 * invA0);
  bq.a2 = static_cast<float>(a2 * invA0);
  bq.active = true;
}

// ============================================================================
// 7. SINKRONISASI PARAMETER ATOMIC PADA BATAS BLOK (CORE 1)
// ============================================================================
void DspEngine::syncPendingParameters() {
  if (!configDirty_) return;

  const SystemConfig prevCfg = activeCfg_;
  const bool full = forceFullSync_;

  portENTER_CRITICAL(&configMux_);
  activeCfg_   = pendingCfg_;
  configDirty_ = false;
  portEXIT_CRITICAL(&configMux_);

  forceFullSync_ = false;

  // 1. Input Stage ringan (Gain / Mute / Phase saja)
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    const auto &in = activeCfg_.inputs[ch];
    if (full || memcmp(&in, &prevCfg.inputs[ch], sizeof(in)) != 0) {
      const float linGain = in.mute ? 0.0f : dbToLinear(clampf(in.gainDb, -12.0f, 12.0f));
      inGainTarget_[ch] = in.phaseInvert ? -linGain : linGain;
    }
  }

  // 2. Output Stage penuh (OUT 1..4) — Delta-Sync agar putar knob/slider
  //    TIDAK menghitung ulang semua 44 biquad double-precision di Core 1!
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    const auto &out  = activeCfg_.outputs[ch];
    const auto &pOut = prevCfg.outputs[ch];

    if (full || out.mute != pOut.mute || out.gainDb != pOut.gainDb ||
        out.phaseInvert != pOut.phaseInvert) {
      const float linGain = out.mute ? 0.0f : dbToLinear(clampf(out.gainDb, -12.0f, 12.0f));
      outGainTarget_[ch] = out.phaseInvert ? -linGain : linGain;
    }

    if (full || memcmp(&out.hpf, &pOut.hpf, sizeof(out.hpf)) != 0) {
      outHpf_[ch].configure(out.hpf, true, sampleRate_);
    }
    if (full || memcmp(&out.lpf, &pOut.lpf, sizeof(out.lpf)) != 0) {
      outLpf_[ch].configure(out.lpf, false, sampleRate_);
    }
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      if (full || memcmp(&out.peq[b], &pOut.peq[b], sizeof(out.peq[b])) != 0) {
        designPeqBiquad(outPeq_[ch][b], out.peq[b], sampleRate_);
      }
    }
    if (full || out.delayMs != pOut.delayMs) {
      outDelay_[ch].setDelayMs(out.delayMs, sampleRate_);
    }
    if (full || memcmp(&out.gate, &pOut.gate, sizeof(out.gate)) != 0) {
      outGate_[ch].configure(out.gate, sampleRate_);
    }
    if (full || memcmp(&out.limiter, &pOut.limiter, sizeof(out.limiter)) != 0) {
      outLimiter_[ch].configure(out.limiter, sampleRate_);
    }
  }

  // 3. Master Volume
  if (full || memcmp(&activeCfg_.master, &prevCfg.master, sizeof(activeCfg_.master)) != 0) {
    const float mVol = clampf(activeCfg_.master.volumeDb, -60.0f, 0.0f);
    masterGainTarget_ = (activeCfg_.master.mute || mVol <= -59.9f) ? 0.0f : dbToLinear(mVol);
  }
}

// ============================================================================
// 8. SOFT / HARD ANTI-CLIP LIMITER
// ============================================================================
float DspEngine::applyAntiClip(float x, AntiClipMode mode) {
  if (!std::isfinite(x)) return 0.0f;

  if (mode == AntiClipMode::HARD_CLIP) {
    return clampf(x, -1.0f, 1.0f);
  }

  constexpr float KNEE = 0.90f;
  constexpr float HEADROOM = 1.0f - KNEE;
  const float ax = fabsf(x);
  if (ax <= KNEE) return x;

  const float over = (ax - KNEE) / HEADROOM;
  const float u = (over > 3.0f) ? 3.0f : over;
  const float u2 = u * u;
  const float sat = (u * (27.0f + u2)) / (27.0f + 9.0f * u2);
  const float mag = KNEE + HEADROOM * sat;
  return (x >= 0.0f) ? mag : -mag;
}

// ============================================================================
// 9. PIPELINE PEMROSESAN REAL-TIME CORE 1 (128 FRAME PER BLOK)
// ============================================================================
void DspEngine::audioCallbackAdapter(audio_io::AudioBlock &block, void *userCtx) {
  if (userCtx != nullptr) {
    static_cast<DspEngine *>(userCtx)->processBlock(block);
  }
}

void DspEngine::processBlock(audio_io::AudioBlock &block) {
  if (!initialized_) return;

  syncPendingParameters();

  const size_t nFrames = block.frames;
  const float invN = 1.0f / static_cast<float>(nFrames);

  float inPk[DSP_NUM_INPUTS]    = {0.0f, 0.0f};
  float inSum2[DSP_NUM_INPUTS]  = {0.0f, 0.0f};
  bool  inClip[DSP_NUM_INPUTS]  = {false, false};

  float outPk[DSP_NUM_OUTPUTS]   = {0.0f, 0.0f, 0.0f, 0.0f};
  float outSum2[DSP_NUM_OUTPUTS] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool  outClip[DSP_NUM_OUTPUTS] = {false, false, false, false};

  // ---- TAHAP 1: INPUT STAGE BERSIH (GAIN / MUTE / PHASE SAJA) ----
  for (size_t i = 0; i < nFrames; i++) {
    for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
      inGainSmooth_[ch] += (inGainTarget_[ch] - inGainSmooth_[ch]) * GAIN_SMOOTH;

      float x = flushSubnormal(block.adcIn[ch][i] * inGainSmooth_[ch]);
      inProcBuf_[ch][i] = x;

      // Jalur USB Record ke PC (langsung bersih dari Input Stage)
      block.usbOut[ch][i] = flushSubnormal(clampf(x, -1.0f, 1.0f));

      const float ax = fabsf(x);
      if (ax > inPk[ch]) inPk[ch] = ax;
      if (ax >= CLIP_THRESH) inClip[ch] = true;
      inSum2[ch] += x * x;
    }
  }

  // ---- TAHAP 2 & 3: ROUTING MATRIX 4x4 -> OUTPUT STAGE PENUH (OUT 1..4) -> MASTER ----
  for (size_t i = 0; i < nFrames; i++) {
    antiDenormalOffset_ = -antiDenormalOffset_;
    const float dn = antiDenormalOffset_;

    masterGainSmooth_ += (masterGainTarget_ - masterGainSmooth_) * GAIN_SMOOTH;

    const float src[DSP_MATRIX_SOURCES] = {
        inProcBuf_[0][i],  // SRC_IN1
        inProcBuf_[1][i],  // SRC_IN2
        block.usbIn[0][i], // SRC_USB_L
        block.usbIn[1][i]  // SRC_USB_R
    };

    for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
      const float *wt = activeCfg_.matrix.mixGain[ch];
      float *ws = matrixGainSmooth_[ch];
      ws[SRC_IN1]   += (wt[SRC_IN1]   - ws[SRC_IN1])   * GAIN_SMOOTH;
      ws[SRC_IN2]   += (wt[SRC_IN2]   - ws[SRC_IN2])   * GAIN_SMOOTH;
      ws[SRC_USB_L] += (wt[SRC_USB_L] - ws[SRC_USB_L]) * GAIN_SMOOTH;
      ws[SRC_USB_R] += (wt[SRC_USB_R] - ws[SRC_USB_R]) * GAIN_SMOOTH;

      float y = ws[SRC_IN1]   * src[SRC_IN1]   +
                ws[SRC_IN2]   * src[SRC_IN2]   +
                ws[SRC_USB_L] * src[SRC_USB_L] +
                ws[SRC_USB_R] * src[SRC_USB_R];

      outGainSmooth_[ch] += (outGainTarget_[ch] - outGainSmooth_[ch]) * GAIN_SMOOTH;
      y *= outGainSmooth_[ch];

      if (!activeCfg_.outputs[ch].bypassDsp) {
        y = outHpf_[ch].process(y, dn);
        y = outLpf_[ch].process(y, dn);

        for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
          y = outPeq_[ch][b].process(y, dn);
        }

        y = outDelay_[ch].process(y);
        y = outGate_[ch].process(y, dn);
        y = outLimiter_[ch].process(y, dn);
      }

      y *= masterGainSmooth_;

      const float preClipAbs = fabsf(y);
      if (preClipAbs >= CLIP_THRESH) {
        outClip[ch] = true;
      }
      y = flushSubnormal(applyAntiClip(y, activeCfg_.outputs[ch].antiClipMode));

      block.dacOut[ch][i] = y;

      const float ay = fabsf(y);
      if (ay > outPk[ch]) outPk[ch] = ay;
      outSum2[ch] += y * y;
    }
  }

  // ---- UPDATE METER SNAPSHOT ----
  portENTER_CRITICAL(&meterMux_);
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    const float rms = sqrtf(inSum2[ch] * invN);
    meters_.inPeakLin[ch] = inPk[ch];
    meters_.inRmsLin[ch]  = rms;
    meters_.inPeakDb[ch]  = fastLinearToDb(inPk[ch]);
    meters_.inRmsDb[ch]   = fastLinearToDb(rms);
    meters_.inClip[ch]    = inClip[ch];
  }
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    const float rms = sqrtf(outSum2[ch] * invN);
    meters_.outPeakLin[ch]       = outPk[ch];
    meters_.outRmsLin[ch]        = rms;
    meters_.outPeakDb[ch]        = fastLinearToDb(outPk[ch]);
    meters_.outRmsDb[ch]         = fastLinearToDb(rms);
    meters_.outLimiterGrDb[ch]   = outLimiter_[ch].grDb;
    meters_.outClip[ch]          = outClip[ch];
    meters_.outGateClosed[ch]    = outGate_[ch].enabled && !outGate_[ch].gateOpen;
    meters_.outLimiterActive[ch] = outLimiter_[ch].enabled && (outLimiter_[ch].grDb < -0.1f);
  }
  portEXIT_CRITICAL(&meterMux_);
}

DspMeters DspEngine::getMeters() const {
  DspMeters copy = {};
  portENTER_CRITICAL(&meterMux_);
  copy = meters_;
  portEXIT_CRITICAL(&meterMux_);
  return copy;
}

} // namespace dsp
