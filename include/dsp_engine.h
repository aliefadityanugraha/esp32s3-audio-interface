#pragma once

/**
 * @file dsp_engine.h
 * @brief Real-Time 32-Bit Float DSP Audio Interface 2-In 4-Out Engine (Core 1)
 *
 * Alur Sinyal (Signal Chain Ringan & Bersih):
 *   1. INPUT STAGE (IN A & IN B):
 *      Direct Clean Input: Gain (-12..+12 dB) -> Mute -> Phase (0/180°) -> USB Record & Matrix
 *      (Tanpa EQ / Compressor / Gate di sisi Input agar latensi & beban CPU minimal)
 *   2. ROUTING MATRIX 4x4:
 *      4 Sumber (IN A, IN B, USB L, USB R) -> 4 Output (OUT 1..4)
 *   3. OUTPUT STAGE (OUT 1..4):
 *      Gain (-12..+12 dB) -> Mute -> Phase (0/180°)
 *      -> Crossover HPF & LPF (LR / Butterworth / Bessel: 12, 24, 48 dB/oct)
 *      -> 9-Band PEQ (16 Hz - 20 kHz, resolusi 0.5 Hz, ±15 dB, Q 0.4 - 12.0)
 *      -> Delay Line hingga 300 ms di PSRAM (ps_malloc)
 *      -> Output Noise Gate (default -50 dB) -> Peak Limiter -> Hard/Soft Anti-Clip
 *   4. MASTER & METERING:
 *      Master Volume (0 dB s/d -60 dB smoothing) -> Peak & RMS 6-Kanal + Clip/Limiter Flags
 */

#ifdef NATIVE_TEST
  #include <cmath>
  #include <cstddef>
  #include <cstdint>
  #include <cstdlib>
  #include "audio_config.h"

  namespace audio_io {
  struct AudioBlock {
    float adcIn[AUDIO_IN_CHANNELS][AUDIO_BLOCK_FRAMES];
    float usbIn[AUDIO_USB_CHANNELS][AUDIO_BLOCK_FRAMES];
    float usbOut[AUDIO_USB_CHANNELS][AUDIO_BLOCK_FRAMES];
    float dacOut[AUDIO_OUT_CHANNELS][AUDIO_BLOCK_FRAMES];
    size_t frames;
    bool usbPlaybackActive;
  };
  } // namespace audio_io

  using portMUX_TYPE = int;
  #ifndef portMUX_INITIALIZER_UNLOCKED
    #define portMUX_INITIALIZER_UNLOCKED 0
  #endif
  #ifndef portENTER_CRITICAL
    #define portENTER_CRITICAL(mux) ((void)(mux))
    #define portEXIT_CRITICAL(mux)  ((void)(mux))
  #endif
#else
  #include <Arduino.h>
  #include <cstddef>
  #include <cstdint>
  #include "audio_config.h"
  #include "audio_io.h"
#endif

namespace dsp {

// ============================================================================
// 1. KONSTANTA ARSITEKTUR DSP 2-IN 4-OUT
// ============================================================================
constexpr uint8_t  DSP_NUM_INPUTS        = 2;
constexpr uint8_t  DSP_NUM_OUTPUTS       = 4;
constexpr uint8_t  DSP_TOTAL_CHANNELS    = DSP_NUM_INPUTS + DSP_NUM_OUTPUTS; // 6 ch
constexpr uint8_t  DSP_MATRIX_SOURCES    = 4;  // IN1, IN2, USB_L, USB_R
constexpr uint8_t  DSP_PEQ_BANDS         = 9;
constexpr uint8_t  DSP_MAX_XOVER_STAGES  = 4;  // 48 dB/oct = 8th-order = 4 biquads
constexpr float    DSP_MAX_DELAY_MS      = 300.0f;

constexpr uint32_t DSP_CONFIG_MAGIC      = 0x444C4D53; // "DLMS"
constexpr uint16_t DSP_CONFIG_VERSION    = 0x0200;     // v2.0 (Clean Input + Output-Only DSP)

enum MatrixSource : uint8_t {
  SRC_IN1   = 0,
  SRC_IN2   = 1,
  SRC_USB_L = 2,
  SRC_USB_R = 3
};

enum class ChannelId : int8_t {
  IN1         = 0,
  IN2         = 1,
  OUT1        = 2,
  OUT2        = 3,
  OUT3        = 4,
  OUT4        = 5,
  ALL_OUTPUTS = -1
};

// ============================================================================
// 2. ENUM & STRUKTUR PARAMETER KONFIGURASI (POD - SIAP SERIALISASI LITTLEFS)
// ============================================================================
enum class PeqFilterType : uint8_t {
  PEAKING    = 0,
  LOW_SHELF  = 1,
  HIGH_SHELF = 2,
  LOW_PASS   = 3,
  HIGH_PASS  = 4,
  NOTCH      = 5,
  ALL_PASS   = 6
};

struct PeqBandConfig {
  bool          enabled;
  PeqFilterType type;
  float         freqHz;  // 16.0 - 20000.0 Hz (resolusi 0.5 Hz)
  float         gainDb;  // -15.0 - +15.0 dB
  float         q;       // 0.4 - 12.0
};

struct NoiseGateConfig {
  bool  enabled;
  float thresholdDb;     // -90.0 - 0.0 dB (Output default: -50.0 dB)
  float attackMs;        // 0.1 - 100.0 ms
  float releaseMs;       // 10.0 - 2000.0 ms
  float holdMs;          // 0.0 - 500.0 ms
  float rangeDb;         // -80.0 - 0.0 dB
};

enum class XoverType : uint8_t {
  LINKWITZ_RILEY = 0,
  BUTTERWORTH    = 1,
  BESSEL         = 2
};

enum class XoverSlope : uint8_t {
  SLOPE_12DB = 12,
  SLOPE_24DB = 24,
  SLOPE_48DB = 48
};

struct XoverFilterConfig {
  bool       enabled;
  XoverType  type;
  XoverSlope slope;
  float      freqHz;     // 16.0 - 20000.0 Hz
};

struct PeakLimiterConfig {
  bool  enabled;
  float thresholdDb;     // -30.0 - 0.0 dB
  float attackMs;        // 0.05 - 50.0 ms
  float releaseMs;       // 10.0 - 2000.0 ms
};

enum class AntiClipMode : uint8_t {
  SOFT_TANH = 0,
  HARD_CLIP = 1
};

struct RoutingMatrixConfig {
  float mixGain[DSP_NUM_OUTPUTS][DSP_MATRIX_SOURCES];
};

// Input bersih tanpa chain EQ/Comp/Gate (hanya Gain, Mute, Phase)
struct InputChannelConfig {
  float gainDb;      // -12.0 - +12.0 dB
  bool  mute;
  bool  phaseInvert; // false = 0°, true = 180°
};

// Output Stage penuh (OUT 1..4): Gain/Mute/Phase + Xover + 9-PEQ + Delay + Gate + Limiter + Anti-Clip
struct OutputChannelConfig {
  float             gainDb;      // -12.0 - +12.0 dB
  bool              mute;
  bool              phaseInvert; // false = 0°, true = 180°
  XoverFilterConfig hpf;
  XoverFilterConfig lpf;
  PeqBandConfig     peq[DSP_PEQ_BANDS];
  float             delayMs;     // 0.0 - 300.0 ms
  NoiseGateConfig   gate;        // Default: enabled=true, threshold=-50 dB
  PeakLimiterConfig limiter;
  AntiClipMode      antiClipMode;
  bool              bypassDsp;
};

struct MasterConfig {
  float volumeDb; // -60.0 - 0.0 dB (smooth ramp)
  bool  mute;
};

struct SystemConfig {
  uint32_t            magic;
  uint16_t            version;
  char                presetName[32];
  InputChannelConfig  inputs[DSP_NUM_INPUTS];
  RoutingMatrixConfig matrix;
  OutputChannelConfig outputs[DSP_NUM_OUTPUTS];
  MasterConfig        master;
};

// ============================================================================
// 3. TELEMETRI REAL-TIME DSP (DIBACA CORE 0: UI ILI9341 / WEB / SERIAL)
// ============================================================================
struct DspMeters {
  float inPeakLin[DSP_NUM_INPUTS];
  float inRmsLin[DSP_NUM_INPUTS];
  float inPeakDb[DSP_NUM_INPUTS];
  float inRmsDb[DSP_NUM_INPUTS];

  float outPeakLin[DSP_NUM_OUTPUTS];
  float outRmsLin[DSP_NUM_OUTPUTS];
  float outPeakDb[DSP_NUM_OUTPUTS];
  float outRmsDb[DSP_NUM_OUTPUTS];

  float outLimiterGrDb[DSP_NUM_OUTPUTS];

  bool inClip[DSP_NUM_INPUTS];
  bool outClip[DSP_NUM_OUTPUTS];
  bool outGateClosed[DSP_NUM_OUTPUTS];
  bool outLimiterActive[DSP_NUM_OUTPUTS];

  bool delayAllocatedInPsram;
};

// ============================================================================
// 4. PRIMITIF DSP INTERNAL (OUTPUT STAGE: BIQUAD, XOVER, GATE, LIMITER, DELAY)
// ============================================================================
struct BiquadState {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float s1 = 0.0f, s2 = 0.0f;
  bool  active = false;

  inline void reset() { s1 = 0.0f; s2 = 0.0f; }

  inline float process(float x, float antiDenormalOffset) {
    if (!active) return x;
    const float xn = x + antiDenormalOffset;
    float y        = b0 * xn + s1;
    s1 = b1 * xn - a1 * y + s2;
    s2 = b2 * xn - a2 * y;

    if (!std::isfinite(y) || !std::isfinite(s1) || !std::isfinite(s2)) {
      s1 = 0.0f;
      s2 = 0.0f;
      return 0.0f;
    }
    if (fabsf(s1) < 1.0e-20f) s1 = 0.0f;
    if (fabsf(s2) < 1.0e-20f) s2 = 0.0f;
    if (fabsf(y)  < 1.0e-20f) y  = 0.0f;
    return y;
  }
};

struct CrossoverChain {
  BiquadState stages[DSP_MAX_XOVER_STAGES];
  uint8_t     numStages = 0;
  bool        active = false;

  void reset();
  void configure(const XoverFilterConfig &cfg, bool isHighPass, float sampleRate);
  inline float process(float x, float antiDenormalOffset) {
    if (!active) return x;
    for (uint8_t s = 0; s < numStages; s++) {
      x = stages[s].process(x, antiDenormalOffset);
    }
    return x;
  }
};

struct NoiseGateState {
  float envLin     = 0.0f;
  float gainLin    = 1.0f;
  float threshLin  = 0.003162f; // -50 dB
  float rangeLin   = 0.0001f;   // -80 dB
  float atkCoeff   = 0.01f;
  float relCoeff   = 0.001f;
  uint32_t holdSamples = 0;
  uint32_t holdCounter = 0;
  bool  gateOpen   = true;
  bool  enabled    = false;

  void reset();
  void configure(const NoiseGateConfig &cfg, float sampleRate);
  float process(float x, float antiDenormalOffset);
};

struct PeakLimiterState {
  float envLin      = 0.0f;
  float gainLin     = 1.0f;
  float grDb        = 0.0f;
  float threshLin   = 0.89125f; // -1 dBFS
  float atkCoeff    = 0.1f;
  float relCoeff    = 0.001f;
  bool  enabled     = false;

  void reset();
  void configure(const PeakLimiterConfig &cfg, float sampleRate);
  float process(float x, float antiDenormalOffset);
};

struct DelayLinePsram {
  float   *buffer       = nullptr;
  size_t   capacity     = 0;
  size_t   writeIdx     = 0;
  float    targetDelay  = 0.0f;
  float    currentDelay = 0.0f;
  bool     inPsram      = false;

  bool allocate(float maxDelayMs, float sampleRate);
  void release();
  void clear();
  void setDelayMs(float delayMs, float sampleRate, bool snapImmediate = false);
  float process(float x);
};

// ============================================================================
// 5. KELAS UTAMA DSP ENGINE (CORE 1 REAL-TIME)
// ============================================================================
class DspEngine {
public:
  DspEngine() = default;
  ~DspEngine();

  bool begin(float sampleRate = static_cast<float>(AUDIO_SAMPLE_RATE));
  static void loadDefaultConfig(SystemConfig &cfg);

  void applyConfig(const SystemConfig &newCfg);
  SystemConfig getConfig() const;
  void snapAndResetStates();

  bool copyChannelSettings(ChannelId src, ChannelId dst);

  static void audioCallbackAdapter(audio_io::AudioBlock &block, void *userCtx);
  void processBlock(audio_io::AudioBlock &block);
  DspMeters getMeters() const;

  static float quantizeFreqHalfHz(float freqHz);
  static float dbToLinear(float db);
  static float linearToDb(float lin);

private:
  void syncPendingParameters();
  static void designPeqBiquad(BiquadState &bq, const PeqBandConfig &cfg, float sampleRate);
  static float applyAntiClip(float x, AntiClipMode mode);

  float sampleRate_ = static_cast<float>(AUDIO_SAMPLE_RATE);
  bool  initialized_ = false;

  mutable portMUX_TYPE configMux_ = portMUX_INITIALIZER_UNLOCKED;
  SystemConfig pendingCfg_ = {};
  SystemConfig activeCfg_  = {};
  volatile bool configDirty_   = false;
  bool          forceFullSync_ = true;

  // Input Stage ringan (Gain/Mute/Phase smoothing saja)
  float inGainTarget_[DSP_NUM_INPUTS] = {1.0f, 1.0f};
  float inGainSmooth_[DSP_NUM_INPUTS] = {1.0f, 1.0f};

  // Matrix Routing Gain Smoothing (clickless saat toggle routing)
  float matrixGainSmooth_[DSP_NUM_OUTPUTS][DSP_MATRIX_SOURCES] = {{0}};

  // Output Stage penuh (OUT 1..4)
  float            outGainTarget_[DSP_NUM_OUTPUTS] = {1.0f, 1.0f, 1.0f, 1.0f};
  float            outGainSmooth_[DSP_NUM_OUTPUTS] = {1.0f, 1.0f, 1.0f, 1.0f};
  CrossoverChain   outHpf_[DSP_NUM_OUTPUTS];
  CrossoverChain   outLpf_[DSP_NUM_OUTPUTS];
  BiquadState      outPeq_[DSP_NUM_OUTPUTS][DSP_PEQ_BANDS];
  DelayLinePsram   outDelay_[DSP_NUM_OUTPUTS];
  NoiseGateState   outGate_[DSP_NUM_OUTPUTS];
  PeakLimiterState outLimiter_[DSP_NUM_OUTPUTS];

  float masterGainTarget_ = 1.0f;
  float masterGainSmooth_ = 1.0f;
  float antiDenormalOffset_ = 1.0e-25f;

  alignas(16) float inProcBuf_[DSP_NUM_INPUTS][AUDIO_BLOCK_FRAMES] = {{0}};

  mutable portMUX_TYPE meterMux_ = portMUX_INITIALIZER_UNLOCKED;
  DspMeters meters_ = {};
};

} // namespace dsp
