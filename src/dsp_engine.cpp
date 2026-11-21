/**
 * @file dsp_engine.cpp
 * @brief Implementasi Tahap 2: Real-Time 32-Bit Float DLMS 2-In 4-Out DSP Engine
 *        Berjalan di Core 1 (ESP32-S3 FPU 240 MHz) dengan proteksi anti-denormal.
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

constexpr float  PI_F          = 3.14159265358979323846f;
constexpr double PI_D          = 3.14159265358979323846;
constexpr float  DB_FLOOR      = -96.0f;
constexpr float  LIN_FLOOR     = 1.5848932e-5f; // 10^(-96/20)
constexpr float  GAIN_SMOOTH   = 0.0025f;       // ~8.3 ms ramp @ 48 kHz (bebas klik/zipper)
constexpr float  CLIP_THRESH   = 0.995f;        // -0.04 dBFS deteksi clip

// ============================================================================
// MATEMATIKA CEPAT (FAST LOG2 / EXP2 AKURASI < 0.01 dB UNTUK DYNAMICS CORE 1)
// ============================================================================
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

inline float fastExp2f(float x) {
  if (x < -126.0f) return 0.0f;
  if (x > 126.0f)  x = 126.0f;
  const float kf = floorf(x);
  const int32_t k = static_cast<int32_t>(kf);
  const float f = x - kf;
  const float p = 1.0f + f * (0.69314718f + f * (0.24022651f +
                         f * (0.05550411f + 0.00961813f * f)));
  uint32_t eu = static_cast<uint32_t>(k + 127) << 23;
  float scale = 1.0f;
  memcpy(&scale, &eu, sizeof(scale));
  return p * scale;
}

inline float fastLinearToDb(float lin) {
  if (lin <= LIN_FLOOR) return DB_FLOOR;
  return 6.02059991f * fastLog2f(lin);
}

inline float fastDbToLinear(float db) {
  if (db <= DB_FLOOR) return 0.0f;
  return fastExp2f(db * 0.166096405f); // 1 / (20 * log10(2))
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
  float freqScale; // LPF: fc * freqScale; HPF: fc / freqScale
  float q;
};

// Butterworth (maximally flat, -3 dB @ fc)
constexpr XoverSectionSpec BW_12[1] = {{1.0f, 0.70710678f}};
constexpr XoverSectionSpec BW_24[2] = {{1.0f, 0.54119610f}, {1.0f, 1.30656296f}};
constexpr XoverSectionSpec BW_48[4] = {
    {1.0f, 0.50979558f}, {1.0f, 0.60134489f},
    {1.0f, 0.89997622f}, {1.0f, 2.56291545f}};

// Linkwitz-Riley (squared Butterworth, -6 dB @ fc, flat acoustic sum)
constexpr XoverSectionSpec LR_12[1] = {{1.0f, 0.50000000f}};
constexpr XoverSectionSpec LR_24[2] = {{1.0f, 0.70710678f}, {1.0f, 0.70710678f}};
constexpr XoverSectionSpec LR_48[4] = {
    {1.0f, 0.54119610f}, {1.0f, 1.30656296f},
    {1.0f, 0.54119610f}, {1.0f, 1.30656296f}};

// Bessel (maximally flat group delay, ternormalisasi tepat -3.01 dB @ fc)
constexpr XoverSectionSpec BES_12[1] = {{1.27201965f, 0.57735027f}};
constexpr XoverSectionSpec BES_24[2] = {
    {1.43017156f, 0.52193458f}, {1.60335752f, 0.80553828f}};
constexpr XoverSectionSpec BES_48[4] = {
    {1.77846591f, 0.50599107f}, {1.83209260f, 0.55960916f},
    {1.95319576f, 0.71085207f}, {2.18872623f, 1.22566943f}};

// Frekuensi tengah standar 9-band PEQ (default flat 0 dB)
constexpr float DEFAULT_PEQ_FREQS[DSP_PEQ_BANDS] = {
    40.0f, 80.0f, 160.0f, 315.0f, 630.0f, 1250.0f, 2500.0f, 5000.0f, 10000.0f};

// 31-Band ISO 1/3-Octave Center Frequencies untuk tampilan RTA
constexpr float ISO_RTA_CENTERS[DSP_RTA_ISO_BANDS] = {
    20.0f,   25.0f,   31.5f,   40.0f,   50.0f,   63.0f,   80.0f,   100.0f,
    125.0f,  160.0f,  200.0f,  250.0f,  315.0f,  400.0f,  500.0f,  630.0f,
    800.0f,  1000.0f, 1250.0f, 1600.0f, 2000.0f, 2500.0f, 3150.0f, 4000.0f,
    5000.0f, 6300.0f, 8000.0f, 10000.0f, 12500.0f, 16000.0f, 20000.0f};

// Desain 1 Biquad 2nd-order LPF / HPF dengan kalkulasi presisi double
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
// 2. NOISE GATE (THRESHOLD, ATTACK, RELEASE, HOLD, RANGE)
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

  // Detektor peak envelope cepat (attack ~0.5 ms, release ~25 ms)
  const float absX = fabsf(x) + fabsf(antiDenormalOffset);
  if (absX > envLin) {
    envLin += (absX - envLin) * 0.04f;
  } else {
    envLin += (absX - envLin) * 0.0008f;
  }
  if (envLin < 1.0e-20f) envLin = 0.0f;

  // Hysteresis ±1.5 dB agar gate tidak berkedip di sekitar threshold
  const float openThresh  = threshLin;
  const float closeThresh = threshLin * 0.8414f; // -1.5 dB di bawah threshold

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
// 3. COMPRESSOR (SOFT KNEE 0-12 dB, MAKEUP 0-18 dB, REAL-TIME GR)
// ============================================================================
void CompressorState::reset() {
  grDb  = 0.0f;
}

void CompressorState::configure(const CompressorConfig &cfg, float sampleRate) {
  enabled   = cfg.enabled;
  threshDb  = clampf(cfg.thresholdDb, -60.0f, 0.0f);
  ratio     = clampf(cfg.ratio, 1.0f, 20.0f);
  kneeDb    = clampf(cfg.kneeDb, 0.0f, 12.0f);
  makeupLin = DspEngine::dbToLinear(clampf(cfg.makeupGainDb, 0.0f, 18.0f));
  atkCoeff  = msToOnePoleCoeff(clampf(cfg.attackMs, 0.1f, 200.0f), sampleRate);
  relCoeff  = msToOnePoleCoeff(clampf(cfg.releaseMs, 10.0f, 2000.0f), sampleRate);
}

float CompressorState::process(float x, float antiDenormalOffset) {
  if (!enabled) {
    grDb = 0.0f;
    return x;
  }

  const float absX = fabsf(x) + fabsf(antiDenormalOffset);
  const float xDb  = fastLinearToDb(absX);

  // Kalkulasi kurva kompresi statis dengan Soft Knee kuadratik (Giannoulis 2012)
  const float overDb = xDb - threshDb;
  const float halfKnee = kneeDb * 0.5f;
  float targetGrDb = 0.0f;

  if (kneeDb > 0.05f && fabsf(overDb) <= halfKnee) {
    // Wilayah transisi Soft Knee (halus & musikal)
    const float t = overDb + halfKnee;
    targetGrDb = ((1.0f / ratio) - 1.0f) * (t * t) / (2.0f * kneeDb);
  } else if (overDb > halfKnee) {
    // Di atas knee: kompresi penuh sesuai ratio
    targetGrDb = ((1.0f / ratio) - 1.0f) * overDb;
  } else {
    targetGrDb = 0.0f;
  }

  // Ballistics smoothing pada domain dB (Attack saat GR makin negatif, Release saat pulih)
  const float coeff = (targetGrDb < grDb) ? atkCoeff : relCoeff;
  grDb += (targetGrDb - grDb) * coeff;
  if (fabsf(grDb) < 1.0e-6f) grDb = 0.0f;

  const float gainLin = fastDbToLinear(grDb) * makeupLin;
  return x * gainLin;
}

// ============================================================================
// 4. PEAK LIMITER (THRESHOLD, ATTACK, RELEASE, REAL-TIME GR)
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

  // Terapkan rasio gain langsung dari envelope (single-stage -> attack akurat & hemat CPU)
  gainLin = (envLin > threshLin && envLin > 1.0e-9f) ? (threshLin / envLin) : 1.0f;
  if (gainLin < 1.0e-5f) gainLin = 1.0e-5f;

  grDb = (gainLin < 0.9995f) ? fastLinearToDb(gainLin) : 0.0f;
  return x * gainLin;
}

// ============================================================================
// 5. DELAY LINE HINGGA 300 MS DI PSRAM (ps_malloc + INTERPOLASI ANTI-ZIPPER)
// ============================================================================
bool DelayLinePsram::allocate(float maxDelayMs, float sampleRate) {
  release();
  // Tambah margin 8 sampel untuk interpolasi linear aman
  capacity = static_cast<size_t>(ceilf((maxDelayMs * 0.001f) * sampleRate)) + 8;
  const size_t bytes = capacity * sizeof(float);

#if defined(ESP32) && !defined(NATIVE_TEST)
  buffer = static_cast<float *>(ps_malloc(bytes));
  if (buffer != nullptr) {
    inPsram = true;
  } else {
    // Fallback aman jika PSRAM tidak aktif (tetap tidak crash)
    buffer = static_cast<float *>(malloc(bytes));
    inPsram = false;
  }
#else
  // Mode NATIVE_TEST di PC menggunakan malloc() standar
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
  // Presisi double agar 300.0 ms * 48000 Hz tepat 14400.0 sampel tanpa galat float 0.001f
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

  // Tulis sampel baru ke ring buffer PSRAM
  buffer[writeIdx] = x;

  // Slew halus perubahan waktu delay (mencegah suara klik saat diputar live)
  const float diff = targetDelay - currentDelay;
  if (fabsf(diff) > 0.01f) {
    currentDelay += diff * 0.002f;
  } else {
    currentDelay = targetDelay;
  }

  float y = x;
  if (currentDelay >= 0.5f) {
    const float dFloor = floorf(currentDelay);
    const float frac   = currentDelay - dFloor;
    const size_t dInt  = static_cast<size_t>(dFloor);

    // Jalur cepat integer-delay (1x baca PSRAM saja saat steady-state & delay bulat)
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
// 6. INTERNAL NOISE GENERATOR (WHITE & PINK NOISE + AUTO-OFF 30 DETIK)
// ============================================================================
void NoiseGeneratorState::reset() {
  b0 = b1 = b2 = b3 = b4 = b5 = b6 = 0.0f;
  remainingSamples = 0;
  fadeGain = 0.0f;
  activeType = GeneratorType::OFF;
}

void NoiseGeneratorState::trigger(const SignalGeneratorConfig &cfg, float sampleRate) {
  activeType = cfg.type;
  levelLin   = DspEngine::dbToLinear(clampf(cfg.levelDb, -60.0f, -6.0f));

  if (cfg.type == GeneratorType::OFF) {
    remainingSamples = 0;
  } else {
    const uint32_t sec = (cfg.autoOffSeconds == 0) ? 30u : static_cast<uint32_t>(cfg.autoOffSeconds);
    remainingSamples = sec * static_cast<uint32_t>(sampleRate);
  }
}

float NoiseGeneratorState::nextSample(float antiDenormalOffset) {
  const bool running = (activeType != GeneratorType::OFF) && (remainingSamples > 0);
  if (running) {
    remainingSamples--;
    if (remainingSamples == 0) {
      activeType = GeneratorType::OFF;
    }
  }

  // Smooth fade-in/fade-out (~10 ms) agar start/stop noise tidak berbunyi "pop"
  const float targetFade = running ? 1.0f : 0.0f;
  fadeGain += (targetFade - fadeGain) * 0.002f;
  if (!running && fadeGain < 1.0e-5f) {
    fadeGain = 0.0f;
    return 0.0f;
  }

  // Xorshift32 PRNG ultra-cepat -> White Noise [-1.0, +1.0]
  uint32_t s = prngState;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  prngState = s;

  const float white = (static_cast<int32_t>(s) * (1.0f / 2147483648.0f)) + antiDenormalOffset;

  float out = white;
  if (activeType == GeneratorType::PINK_NOISE) {
    // Paul Kellet 7-pole Pink Noise filter (-3 dB/oct, ±0.05 dB @ 20Hz-20kHz)
    b0 = 0.99886f * b0 + white * 0.0555179f;
    b1 = 0.99332f * b1 + white * 0.0750759f;
    b2 = 0.96900f * b2 + white * 0.1538520f;
    b3 = 0.86650f * b3 + white * 0.3104856f;
    b4 = 0.55000f * b4 + white * 0.5329522f;
    b5 = -0.7616f * b5 - white * 0.0168980f;
    out = (b0 + b1 + b2 + b3 + b4 + b5 + b6 + white * 0.5362f) * 0.14f;
    b6 = white * 0.115926f;

    if (fabsf(b0) < 1.0e-20f) b0 = 0.0f;
    if (fabsf(b1) < 1.0e-20f) b1 = 0.0f;
    if (fabsf(b2) < 1.0e-20f) b2 = 0.0f;
    if (fabsf(b3) < 1.0e-20f) b3 = 0.0f;
    if (fabsf(b4) < 1.0e-20f) b4 = 0.0f;
    if (fabsf(b5) < 1.0e-20f) b5 = 0.0f;
    if (fabsf(b6) < 1.0e-20f) b6 = 0.0f;
  }

  return flushSubnormal(out * levelLin * fadeGain);
}

// ============================================================================
// 7. DSP ENGINE: LIFECYCLE, DEFAULT CONFIG & HELPER
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
  strncpy(cfg.presetName, "Default Hybrid 2x4", sizeof(cfg.presetName) - 1);

  // ---- 1. Default INPUT STAGE (IN 1 & IN 2) ----
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    auto &in = cfg.inputs[ch];
    in.gainDb      = 0.0f;
    in.mute        = false;
    in.phaseInvert = false;

    // Input Noise Gate (siap pakai, default off agar sinyal lemah langsung terdengar)
    in.gate.enabled     = false;
    in.gate.thresholdDb = -60.0f;
    in.gate.attackMs    = 2.0f;
    in.gate.releaseMs   = 120.0f;
    in.gate.holdMs      = 20.0f;
    in.gate.rangeDb     = -80.0f;

    // 9-Band PEQ Input (default flat 0 dB -> bypass 0 CPU)
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      in.peq[b].enabled = true;
      in.peq[b].type    = PeqFilterType::PEAKING;
      in.peq[b].freqHz  = DEFAULT_PEQ_FREQS[b];
      in.peq[b].gainDb  = 0.0f;
      in.peq[b].q       = 1.4f;
    }

    // Input Compressor (default off, parameter standar pro-audio siap diaktifkan)
    in.comp.enabled      = false;
    in.comp.thresholdDb  = -18.0f;
    in.comp.ratio        = 4.0f;
    in.comp.attackMs     = 10.0f;
    in.comp.releaseMs    = 150.0f;
    in.comp.kneeDb       = 3.0f;
    in.comp.makeupGainDb = 0.0f;
  }

  // ---- 2. Default SIGNAL GENERATOR ----
  cfg.generator.type           = GeneratorType::OFF;
  cfg.generator.levelDb        = -20.0f;
  cfg.generator.destMask       = 0x0F; // Semua OUT 1-4
  cfg.generator.autoOffSeconds = 30;   // Auto-off 30 detik

  // ---- 3. Default ROUTING MATRIX ----
  // OUT 1-2: Mix dari signal monitor IN 1-2 + USB Audio Output dari PC
  // OUT 3-4: Direct/Bypass dari IN 1-2 (dapat diubah bebas lewat matriks)
  memset(cfg.matrix.mixGain, 0, sizeof(cfg.matrix.mixGain));
  cfg.matrix.mixGain[0][SRC_IN1]   = 1.0f;
  cfg.matrix.mixGain[0][SRC_USB_L] = 1.0f;

  cfg.matrix.mixGain[1][SRC_IN2]   = 1.0f;
  cfg.matrix.mixGain[1][SRC_USB_R] = 1.0f;

  cfg.matrix.mixGain[2][SRC_IN1]   = 1.0f;
  cfg.matrix.mixGain[3][SRC_IN2]   = 1.0f;

  cfg.matrix.usbRecPostInputDsp    = true;

  // ---- 4. Default OUTPUT STAGE (OUT 1..4) ----
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    auto &out = cfg.outputs[ch];
    out.gainDb      = 0.0f;
    out.mute        = false;
    out.phaseInvert = false;
    out.bypassDsp   = false;

    // Crossover HPF & LPF (default off/full-range, siap diaktifkan ke LR24/BW24/BES)
    out.hpf.enabled = false;
    out.hpf.type    = XoverType::LINKWITZ_RILEY;
    out.hpf.slope   = XoverSlope::SLOPE_24DB;
    out.hpf.freqHz  = 80.0f;

    out.lpf.enabled = false;
    out.lpf.type    = XoverType::LINKWITZ_RILEY;
    out.lpf.slope   = XoverSlope::SLOPE_24DB;
    out.lpf.freqHz  = 18000.0f;

    // 9-Band PEQ Output (16 Hz - 20 kHz, resolusi 0.5 Hz, ±15 dB, Q 0.4 - 12.0)
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      out.peq[b].enabled = true;
      out.peq[b].type    = PeqFilterType::PEAKING;
      out.peq[b].freqHz  = DEFAULT_PEQ_FREQS[b];
      out.peq[b].gainDb  = 0.0f;
      out.peq[b].q       = 1.4f;
    }

    out.delayMs = 0.0f;

    // Output Noise Gate: Default Auto-Gate -50 dB (sesuai spesifikasi)
    out.gate.enabled     = true;
    out.gate.thresholdDb = -50.0f;
    out.gate.attackMs    = 2.0f;
    out.gate.releaseMs   = 150.0f;
    out.gate.holdMs      = 25.0f;
    out.gate.rangeDb     = -80.0f;

    // Peak Limiter & Anti-Clip
    out.limiter.enabled     = true;
    out.limiter.thresholdDb = -1.0f;
    out.limiter.attackMs    = 0.5f;
    out.limiter.releaseMs   = 100.0f;
    out.antiClipMode        = AntiClipMode::SOFT_TANH;
  }

  // ---- 5. Default MASTER & RTA ----
  cfg.master.volumeDb   = 0.0f; // 0 dB unity
  cfg.master.mute       = false;
  cfg.master.rtaSource  = RtaSource::OUT12_MIX;
  cfg.master.rtaFftSize = 512;
}

bool DspEngine::begin(float sampleRate) {
  sampleRate_ = sampleRate;
  loadDefaultConfig(pendingCfg_);
  activeCfg_ = pendingCfg_;

  // Alokasi 4x Delay Line hingga 300 ms di PSRAM (ps_malloc)
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

  // Reset seluruh state DSP
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    inGate_[ch].reset();
    inComp_[ch].reset();
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) inPeq_[ch][b].reset();
  }
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outHpf_[ch].reset();
    outLpf_[ch].reset();
    outGate_[ch].reset();
    outLimiter_[ch].reset();
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) outPeq_[ch][b].reset();
  }
  noiseGen_.reset();

  // Bangun tabel Hann Window untuk RTA FFT
  buildRtaLookupTables(activeCfg_.master.rtaFftSize);

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

  configDirty_ = true;
  syncPendingParameters();

  // Inisialisasi smooth gain langsung di target boot agar tidak fade lambat saat menyala
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) inGainSmooth_[ch] = inGainTarget_[ch];
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) outGainSmooth_[ch] = outGainTarget_[ch];
  masterGainSmooth_ = masterGainTarget_;

  initialized_ = allocOk;
  return allocOk;
}

void DspEngine::applyConfig(const SystemConfig &newCfg) {
  portENTER_CRITICAL(&configMux_);
  const bool genChanged =
      (newCfg.generator.type != pendingCfg_.generator.type) ||
      (newCfg.generator.levelDb != pendingCfg_.generator.levelDb) ||
      (newCfg.generator.autoOffSeconds != pendingCfg_.generator.autoOffSeconds);
  pendingCfg_ = newCfg;
  configDirty_ = true;
  if (genChanged) {
    generatorDirty_ = true;
  }
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
  configDirty_ = true;
  syncPendingParameters();
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    inGainSmooth_[ch] = inGainTarget_[ch];
    inGate_[ch].reset();
    inComp_[ch].reset();
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) inPeq_[ch][b].reset();
  }
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    outGainSmooth_[ch] = outGainTarget_[ch];
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

void DspEngine::setGenerator(GeneratorType type, float levelDb,
                             uint8_t destMask, uint16_t autoOffSec) {
  portENTER_CRITICAL(&configMux_);
  pendingCfg_.generator.type           = type;
  pendingCfg_.generator.levelDb        = clampf(levelDb, -60.0f, -6.0f);
  pendingCfg_.generator.destMask       = destMask;
  pendingCfg_.generator.autoOffSeconds = (autoOffSec == 0) ? 30 : autoOffSec;
  configDirty_    = true;
  generatorDirty_ = true;
  portEXIT_CRITICAL(&configMux_);
}

void DspEngine::stopGenerator() {
  setGenerator(GeneratorType::OFF, pendingCfg_.generator.levelDb,
               pendingCfg_.generator.destMask, pendingCfg_.generator.autoOffSeconds);
}

bool DspEngine::copyChannelSettings(ChannelId src, ChannelId dst) {
  if (src == dst) return true;

  portENTER_CRITICAL(&configMux_);
  bool ok = false;

  const int8_t s = static_cast<int8_t>(src);
  const int8_t d = static_cast<int8_t>(dst);

  // Salin antar kanal INPUT (IN1 <-> IN2)
  if ((s == 0 || s == 1) && (d == 0 || d == 1)) {
    pendingCfg_.inputs[d] = pendingCfg_.inputs[s];
    configDirty_ = true;
    ok = true;
  }
  // Salin dari 1 kanal OUTPUT (OUT1..OUT4) ke 1 kanal OUTPUT lain atau ke SEMUA OUTPUT
  else if (s >= 2 && s <= 5) {
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

void DspEngine::setRtaConfig(RtaSource source, uint16_t fftSize) {
  const uint16_t validSize = (fftSize <= 256) ? 256 : 512;
  portENTER_CRITICAL(&configMux_);
  pendingCfg_.master.rtaSource  = source;
  pendingCfg_.master.rtaFftSize = validSize;
  configDirty_ = true;
  portEXIT_CRITICAL(&configMux_);

  buildRtaLookupTables(validSize);
}

// ============================================================================
// 8. DESAIN KOEFISIEN BIQUAD PEQ (PRESISI DOUBLE, RESOLUSI 0.5 Hz)
// ============================================================================
void DspEngine::designPeqBiquad(BiquadState &bq, const PeqBandConfig &cfg,
                                float sampleRate) {
  if (!cfg.enabled) {
    bq.active = false;
    return;
  }

  const float gainDb = clampf(cfg.gainDb, -15.0f, 15.0f);
  const bool isGainFilter = (cfg.type == PeqFilterType::PEAKING ||
                             cfg.type == PeqFilterType::LOW_SHELF ||
                             cfg.type == PeqFilterType::HIGH_SHELF);
  if (isGainFilter && fabsf(gainDb) < 0.05f) {
    // 0 dB bypass otomatis -> 0 beban CPU
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
      b0 = 1.0 + alpha * A;
      b1 = -2.0 * cosW0;
      b2 = 1.0 - alpha * A;
      a0 = 1.0 + alpha / A;
      a1 = -2.0 * cosW0;
      a2 = 1.0 - alpha / A;
      break;

    case PeqFilterType::LOW_SHELF: {
      const double alphaS = sinW0 / 2.0 * sqrt((A + 1.0 / A) * (1.0 / Q - 1.0) + 2.0);
      const double twoSqrtAAlpha = (std::isfinite(alphaS) ? 2.0 * sqrt(A) * alphaS
                                                          : 2.0 * sqrt(A) * (sinW0 / 1.41421356));
      b0 = A * ((A + 1.0) - (A - 1.0) * cosW0 + twoSqrtAAlpha);
      b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cosW0);
      b2 = A * ((A + 1.0) - (A - 1.0) * cosW0 - twoSqrtAAlpha);
      a0 = (A + 1.0) + (A - 1.0) * cosW0 + twoSqrtAAlpha;
      a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cosW0);
      a2 = (A + 1.0) + (A - 1.0) * cosW0 - twoSqrtAAlpha;
      break;
    }

    case PeqFilterType::HIGH_SHELF: {
      const double alphaS = sinW0 / 2.0 * sqrt((A + 1.0 / A) * (1.0 / Q - 1.0) + 2.0);
      const double twoSqrtAAlpha = (std::isfinite(alphaS) ? 2.0 * sqrt(A) * alphaS
                                                          : 2.0 * sqrt(A) * (sinW0 / 1.41421356));
      b0 = A * ((A + 1.0) + (A - 1.0) * cosW0 + twoSqrtAAlpha);
      b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cosW0);
      b2 = A * ((A + 1.0) + (A - 1.0) * cosW0 - twoSqrtAAlpha);
      a0 = (A + 1.0) - (A - 1.0) * cosW0 + twoSqrtAAlpha;
      a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cosW0);
      a2 = (A + 1.0) - (A - 1.0) * cosW0 - twoSqrtAAlpha;
      break;
    }

    case PeqFilterType::LOW_PASS:
      b0 = (1.0 - cosW0) * 0.5;
      b1 = (1.0 - cosW0);
      b2 = (1.0 - cosW0) * 0.5;
      a0 = 1.0 + alpha;
      a1 = -2.0 * cosW0;
      a2 = 1.0 - alpha;
      break;

    case PeqFilterType::HIGH_PASS:
      b0 = (1.0 + cosW0) * 0.5;
      b1 = -(1.0 + cosW0);
      b2 = (1.0 + cosW0) * 0.5;
      a0 = 1.0 + alpha;
      a1 = -2.0 * cosW0;
      a2 = 1.0 - alpha;
      break;

    case PeqFilterType::NOTCH:
      b0 = 1.0;
      b1 = -2.0 * cosW0;
      b2 = 1.0;
      a0 = 1.0 + alpha;
      a1 = -2.0 * cosW0;
      a2 = 1.0 - alpha;
      break;

    case PeqFilterType::ALL_PASS:
      b0 = 1.0 - alpha;
      b1 = -2.0 * cosW0;
      b2 = 1.0 + alpha;
      a0 = 1.0 + alpha;
      a1 = -2.0 * cosW0;
      a2 = 1.0 - alpha;
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
// 9. SINKRONISASI PARAMETER ATOMIC PADA BATAS BLOK (CORE 1)
// ============================================================================
void DspEngine::syncPendingParameters() {
  if (!configDirty_) return;

  bool retriggerGen = false;
  portENTER_CRITICAL(&configMux_);
  activeCfg_    = pendingCfg_;
  configDirty_  = false;
  retriggerGen  = generatorDirty_;
  generatorDirty_ = false;
  portEXIT_CRITICAL(&configMux_);

  // 1. Update Input Stage (IN 1 & IN 2)
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    const auto &in = activeCfg_.inputs[ch];
    const float linGain = in.mute ? 0.0f : dbToLinear(clampf(in.gainDb, -12.0f, 12.0f));
    inGainTarget_[ch] = in.phaseInvert ? -linGain : linGain;

    inGate_[ch].configure(in.gate, sampleRate_);
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      designPeqBiquad(inPeq_[ch][b], in.peq[b], sampleRate_);
    }
    inComp_[ch].configure(in.comp, sampleRate_);
  }

  // 2. Update Signal Generator (hanya re-trigger timer 30s bila berubah)
  if (retriggerGen) {
    noiseGen_.trigger(activeCfg_.generator, sampleRate_);
  }

  // 3. Update Output Stage (OUT 1..4)
  for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
    const auto &out = activeCfg_.outputs[ch];
    const float linGain = out.mute ? 0.0f : dbToLinear(clampf(out.gainDb, -12.0f, 12.0f));
    outGainTarget_[ch] = out.phaseInvert ? -linGain : linGain;

    outHpf_[ch].configure(out.hpf, true,  sampleRate_);
    outLpf_[ch].configure(out.lpf, false, sampleRate_);
    for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
      designPeqBiquad(outPeq_[ch][b], out.peq[b], sampleRate_);
    }
    outDelay_[ch].setDelayMs(out.delayMs, sampleRate_);
    outGate_[ch].configure(out.gate, sampleRate_);
    outLimiter_[ch].configure(out.limiter, sampleRate_);
  }

  // 4. Update Master Volume (0 dB s/d -60 dB; <= -59.9 dB atau mute -> 0.0)
  const float mVol = clampf(activeCfg_.master.volumeDb, -60.0f, 0.0f);
  masterGainTarget_ = (activeCfg_.master.mute || mVol <= -59.9f) ? 0.0f : dbToLinear(mVol);
}

// ============================================================================
// 10. SOFT / HARD ANTI-CLIP LIMITER SEBELUM KONVERSI KE I2S INTEGER
// ============================================================================
float DspEngine::applyAntiClip(float x, AntiClipMode mode) {
  if (!std::isfinite(x)) return 0.0f;

  if (mode == AntiClipMode::HARD_CLIP) {
    return clampf(x, -1.0f, 1.0f);
  }

  // SOFT_TANH: 100% linear hingga ±0.90 (-0.9 dBFS), lalu lengkung lembut menuju ±0.999
  constexpr float KNEE = 0.90f;
  constexpr float HEADROOM = 1.0f - KNEE; // 0.10
  const float ax = fabsf(x);
  if (ax <= KNEE) {
    return x;
  }
  const float over = (ax - KNEE) / HEADROOM;
  // Aproksimasi Pade tanh cepat: tanh(u) ≈ u * (27 + u^2) / (27 + 9*u^2)
  const float u = (over > 3.0f) ? 3.0f : over;
  const float u2 = u * u;
  const float sat = (u * (27.0f + u2)) / (27.0f + 9.0f * u2);
  const float mag = KNEE + HEADROOM * sat;
  return (x >= 0.0f) ? mag : -mag;
}

// ============================================================================
// 11. PIPELINE PEMROSESAN REAL-TIME CORE 1 (128 FRAME PER BLOK)
// ============================================================================
void DspEngine::audioCallbackAdapter(audio_io::AudioBlock &block, void *userCtx) {
  if (userCtx != nullptr) {
    static_cast<DspEngine *>(userCtx)->processBlock(block);
  }
}

void DspEngine::processBlock(audio_io::AudioBlock &block) {
  if (!initialized_) return;

  // 0. Terapkan perubahan konfigurasi dari Core 0 secara atomic di awal blok
  syncPendingParameters();

  const size_t nFrames = block.frames;
  const float invN = 1.0f / static_cast<float>(nFrames);

  // Akumulator meter Peak & RMS blok ini
  float inPk[DSP_NUM_INPUTS]   = {0.0f, 0.0f};
  float inSum2[DSP_NUM_INPUTS] = {0.0f, 0.0f};
  bool  inClip[DSP_NUM_INPUTS] = {false, false};

  float outPk[DSP_NUM_OUTPUTS]   = {0.0f, 0.0f, 0.0f, 0.0f};
  float outSum2[DSP_NUM_OUTPUTS] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool  outClip[DSP_NUM_OUTPUTS] = {false, false, false, false};

  // ==========================================================================
  // TAHAP 1: INPUT STAGE (IN 1 & IN 2)
  // Gain/Mute/Phase -> Noise Gate -> 9-Band PEQ -> Compressor
  // ==========================================================================
  for (size_t i = 0; i < nFrames; i++) {
    // Balik tanda offset anti-denormal setiap sampel (±1e-25f) agar rata-rata DC = 0
    antiDenormalOffset_ = -antiDenormalOffset_;
    const float dn = antiDenormalOffset_;

    for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
      inGainSmooth_[ch] += (inGainTarget_[ch] - inGainSmooth_[ch]) * GAIN_SMOOTH;

      float x = block.adcIn[ch][i] * inGainSmooth_[ch];
      x = inGate_[ch].process(x, dn);

      for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
        x = inPeq_[ch][b].process(x, dn);
      }

      x = inComp_[ch].process(x, dn);

      x = flushSubnormal(x);
      inProcBuf_[ch][i] = x;

      // Jalur USB Record ke PC (post-DSP atau raw ADC sesuai matrix.usbRecPostInputDsp)
      block.usbOut[ch][i] = flushSubnormal(clampf(
          activeCfg_.matrix.usbRecPostInputDsp ? x : block.adcIn[ch][i],
          -1.0f, 1.0f));

      // Metering Input
      const float ax = fabsf(x);
      if (ax > inPk[ch]) inPk[ch] = ax;
      if (ax >= CLIP_THRESH) inClip[ch] = true;
      inSum2[ch] += x * x;
    }
  }

  // ==========================================================================
  // TAHAP 2 & 3 & 4: GENERATOR + ROUTING MATRIX -> OUTPUT STAGE -> MASTER
  // ==========================================================================
  const uint8_t genMask = activeCfg_.generator.destMask;
  const RtaSource rtaSrc = activeCfg_.master.rtaSource;

  for (size_t i = 0; i < nFrames; i++) {
    antiDenormalOffset_ = -antiDenormalOffset_;
    const float dn = antiDenormalOffset_;

    // Master volume smoothing per sampel (0 dB s/d -60 dB)
    masterGainSmooth_ += (masterGainTarget_ - masterGainSmooth_) * GAIN_SMOOTH;

    // 4 Sumber Routing Matrix pada sampel ke-i
    const float src[DSP_MATRIX_SOURCES] = {
        inProcBuf_[0][i], // SRC_IN1
        inProcBuf_[1][i], // SRC_IN2
        block.usbIn[0][i],// SRC_USB_L
        block.usbIn[1][i] // SRC_USB_R
    };

    // Signal Generator internal (White/Pink Noise + Auto-Off 30 detik)
    const float genSample = noiseGen_.nextSample(dn);

    // Proses 4 Kanal Output Independen (OUT 1, OUT 2, OUT 3, OUT 4)
    for (uint8_t ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
      // Routing Matrix Sum
      const float *w = activeCfg_.matrix.mixGain[ch];
      float y = w[SRC_IN1]   * src[SRC_IN1]   +
                w[SRC_IN2]   * src[SRC_IN2]   +
                w[SRC_USB_L] * src[SRC_USB_L] +
                w[SRC_USB_R] * src[SRC_USB_R];

      if ((genMask & (1u << ch)) != 0u) {
        y += genSample;
      }

      // Output Gain / Mute / Phase Invert
      outGainSmooth_[ch] += (outGainTarget_[ch] - outGainSmooth_[ch]) * GAIN_SMOOTH;
      y *= outGainSmooth_[ch];

      if (!activeCfg_.outputs[ch].bypassDsp) {
        // Crossover HPF & LPF (LR / BW / Bessel @ 12/24/48 dB/oct)
        y = outHpf_[ch].process(y, dn);
        y = outLpf_[ch].process(y, dn);

        // 9-Band Output PEQ
        for (uint8_t b = 0; b < DSP_PEQ_BANDS; b++) {
          y = outPeq_[ch][b].process(y, dn);
        }

        // Delay Line hingga 300 ms di PSRAM
        y = outDelay_[ch].process(y);

        // Output Noise Gate (default -50 dB)
        y = outGate_[ch].process(y, dn);

        // Peak Limiter
        y = outLimiter_[ch].process(y, dn);
      }

      // Master Volume (0 dB s/d -60 dB)
      y *= masterGainSmooth_;

      // Hard / Soft Anti-Clip Limiter sebelum konversi ke integer I2S
      const float preClipAbs = fabsf(y);
      if (preClipAbs >= CLIP_THRESH) {
        outClip[ch] = true;
      }
      y = flushSubnormal(applyAntiClip(y, activeCfg_.outputs[ch].antiClipMode));

      block.dacOut[ch][i] = y;

      // Metering Output (Peak & RMS)
      const float ay = fabsf(y);
      if (ay > outPk[ch]) outPk[ch] = ay;
      outSum2[ch] += y * y;
    }

    // Tangkap 1 sampel ke buffer RTA (sangat ringan di Core 1; FFT dihitung Core 0)
    float rtaSample = 0.0f;
    switch (rtaSrc) {
      case RtaSource::IN1:       rtaSample = inProcBuf_[0][i]; break;
      case RtaSource::IN2:       rtaSample = inProcBuf_[1][i]; break;
      case RtaSource::OUT1:      rtaSample = block.dacOut[0][i]; break;
      case RtaSource::OUT2:      rtaSample = block.dacOut[1][i]; break;
      case RtaSource::OUT3:      rtaSample = block.dacOut[2][i]; break;
      case RtaSource::OUT4:      rtaSample = block.dacOut[3][i]; break;
      case RtaSource::IN_MIX:    rtaSample = 0.5f * (inProcBuf_[0][i] + inProcBuf_[1][i]); break;
      case RtaSource::OUT12_MIX:
      default:                   rtaSample = 0.5f * (block.dacOut[0][i] + block.dacOut[1][i]); break;
    }

    rtaCaptureBuf_[rtaWritePos_++] = rtaSample;
    if (rtaWritePos_ >= rtaFftSize_) {
      rtaWritePos_ = 0;
      portENTER_CRITICAL(&rtaMux_);
      if (!rtaFrameReady_) {
        memcpy(rtaReadyBuf_, rtaCaptureBuf_, rtaFftSize_ * sizeof(float));
        rtaFrameReady_ = true;
      }
      portEXIT_CRITICAL(&rtaMux_);
    }
  }

  // ==========================================================================
  // UPDATE SNAPSHOT TELEMETRI METER SECARA ATOMIC
  // ==========================================================================
  const bool genAct = (noiseGen_.activeType != GeneratorType::OFF) &&
                      (noiseGen_.remainingSamples > 0);
  const uint16_t genRemSec = genAct
      ? static_cast<uint16_t>((noiseGen_.remainingSamples + static_cast<uint32_t>(sampleRate_) - 1u) /
                              static_cast<uint32_t>(sampleRate_))
      : 0u;

  portENTER_CRITICAL(&meterMux_);
  for (uint8_t ch = 0; ch < DSP_NUM_INPUTS; ch++) {
    const float rms = sqrtf(inSum2[ch] * invN);
    meters_.inPeakLin[ch]    = inPk[ch];
    meters_.inRmsLin[ch]     = rms;
    meters_.inPeakDb[ch]     = fastLinearToDb(inPk[ch]);
    meters_.inRmsDb[ch]      = fastLinearToDb(rms);
    meters_.inCompGrDb[ch]   = inComp_[ch].grDb;
    meters_.inClip[ch]       = inClip[ch];
    meters_.inGateClosed[ch] = inGate_[ch].enabled && !inGate_[ch].gateOpen;
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
  meters_.generatorActive       = genAct;
  meters_.generatorRemainingSec = genRemSec;
  portEXIT_CRITICAL(&meterMux_);
}

DspMeters DspEngine::getMeters() const {
  DspMeters copy = {};
  portENTER_CRITICAL(&meterMux_);
  copy = meters_;
  portEXIT_CRITICAL(&meterMux_);
  return copy;
}

// ============================================================================
// 12. LIGHTWEIGHT 256 / 512-POINT RADIX-2 FFT & 31-BAND ISO RTA (CORE 0)
// ============================================================================
void DspEngine::buildRtaLookupTables(uint16_t fftSize) {
  const uint16_t n = (fftSize <= 256) ? 256 : 512;

  portENTER_CRITICAL(&rtaMux_);
  rtaFftSize_    = n;
  rtaWritePos_   = 0;
  rtaFrameReady_ = false;
  portEXIT_CRITICAL(&rtaMux_);

  const float denom = static_cast<float>(n - 1);
  for (uint16_t i = 0; i < n; i++) {
    // Hann Window: 0.5 * (1 - cos(2*pi*i / (N-1)))
    rtaHannWindow_[i] = 0.5f * (1.0f - cosf((2.0f * PI_F * static_cast<float>(i)) / denom));
  }
  for (uint8_t b = 0; b < DSP_RTA_ISO_BANDS; b++) {
    rtaBandSmooth_[b] = DB_FLOOR;
  }
}

void DspEngine::runRadix2Fft(float *real, float *imag, uint16_t n) const {
  // 1. Bit-reversal permutation
  uint16_t j = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (i < j) {
      float tr = real[i]; real[i] = real[j]; real[j] = tr;
      float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
    }
    uint16_t m = n >> 1;
    while (m >= 1 && j >= m) {
      j -= m;
      m >>= 1;
    }
    j += m;
  }

  // 2. Iterative Cooley-Tukey butterflies
  for (uint16_t len = 2; len <= n; len <<= 1) {
    const float ang = -2.0f * PI_F / static_cast<float>(len);
    const float wLenR = cosf(ang);
    const float wLenI = sinf(ang);
    const uint16_t halfLen = len >> 1;

    for (uint16_t i = 0; i < n; i += len) {
      float wR = 1.0f;
      float wI = 0.0f;
      for (uint16_t k = 0; k < halfLen; k++) {
        const uint16_t idxA = i + k;
        const uint16_t idxB = idxA + halfLen;

        const float uR = real[idxA];
        const float uI = imag[idxA];
        const float vR = real[idxB] * wR - imag[idxB] * wI;
        const float vI = real[idxB] * wI + imag[idxB] * wR;

        real[idxA] = uR + vR;
        imag[idxA] = uI + vI;
        real[idxB] = uR - vR;
        imag[idxB] = uI - vI;

        const float nextWR = wR * wLenR - wI * wLenI;
        wI = wR * wLenI + wI * wLenR;
        wR = nextWR;
      }
    }
  }
}

bool DspEngine::computeRtaSpectrum(RtaSpectrumData &outSpectrum) {
  uint16_t n = 0;
  bool hasData = false;

  portENTER_CRITICAL(&rtaMux_);
  if (rtaFrameReady_) {
    n = rtaFftSize_;
    memcpy(rtaFftReal_, rtaReadyBuf_, n * sizeof(float));
    rtaFrameReady_ = false;
    hasData = true;
  }
  portEXIT_CRITICAL(&rtaMux_);

  if (!hasData || n == 0) {
    return false;
  }

  // Aplikasikan Hann Window dan nolkan komponen imajiner
  for (uint16_t i = 0; i < n; i++) {
    rtaFftReal_[i] *= rtaHannWindow_[i];
    rtaFftImag_[i] = 0.0f;
  }

  runRadix2Fft(rtaFftReal_, rtaFftImag_, n);

  const uint16_t numBins = n >> 1;
  // Kompensasi faktor skala FFT + coherent gain Hann Window (0.5) => 4 / N
  const float normFactor = 4.0f / static_cast<float>(n);
  const float freqStep   = sampleRate_ / static_cast<float>(n);

  outSpectrum.fftSize       = n;
  outSpectrum.numBins       = numBins;
  outSpectrum.binFreqStepHz = freqStep;
  outSpectrum.valid         = true;

  for (uint16_t k = 0; k < numBins; k++) {
    const float mag = sqrtf(rtaFftReal_[k] * rtaFftReal_[k] +
                            rtaFftImag_[k] * rtaFftImag_[k]) * normFactor;
    outSpectrum.binDb[k] = fastLinearToDb(mag);
  }

  // Agregasi ke 31-Band ISO 1/3-Octave (20 Hz - 20 kHz) untuk UI ST7789
  constexpr float HALF_BAND_FACTOR = 1.12246205f; // 2^(1/6)
  for (uint8_t b = 0; b < DSP_RTA_ISO_BANDS; b++) {
    const float fc  = ISO_RTA_CENTERS[b];
    const float fLo = fc / HALF_BAND_FACTOR;
    const float fHi = fc * HALF_BAND_FACTOR;

    int32_t kLo = static_cast<int32_t>(floorf(fLo / freqStep));
    int32_t kHi = static_cast<int32_t>(ceilf(fHi / freqStep));
    if (kLo < 1) kLo = 1;
    if (kHi >= numBins) kHi = numBins - 1;
    if (kHi < kLo) kHi = kLo;

    float maxDb = DB_FLOOR;
    for (int32_t k = kLo; k <= kHi; k++) {
      if (outSpectrum.binDb[k] > maxDb) {
        maxDb = outSpectrum.binDb[k];
      }
    }

    // Attack cepat, decay halus agar grafik RTA enak dilihat di LCD ST7789
    if (maxDb > rtaBandSmooth_[b]) {
      rtaBandSmooth_[b] = (rtaBandSmooth_[b] * 0.35f) + (maxDb * 0.65f);
    } else {
      rtaBandSmooth_[b] = (rtaBandSmooth_[b] * 0.82f) + (maxDb * 0.18f);
    }
    outSpectrum.bandDb[b] = rtaBandSmooth_[b];
  }

  return true;
}

} // namespace dsp
