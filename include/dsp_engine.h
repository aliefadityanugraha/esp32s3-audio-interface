#pragma once

/**
 * @file dsp_engine.h
 * @brief Tahap 2: Real-Time 32-Bit Float DLMS 2-In 4-Out DSP Engine (Core 1)
 *
 * Alur Sinyal (Signal Chain):
 *   1. INPUT STAGE (IN 1 & IN 2):
 *      Gain (-12..+12 dB) -> Mute -> Phase (0/180°) -> Noise Gate
 *      -> 9-Band PEQ (Biquad IIR) -> Compressor (Knee 0-12 dB, Makeup 0-18 dB, GR)
 *   2. SIGNAL GENERATOR & ROUTING MATRIX:
 *      White/Pink Noise Generator (auto-off 30 detik) + Matrix 4-Source -> 4-Output
 *      (Default OUT 1-2: Mix IN 1-2 + USB PC; Default OUT 3-4: Direct IN 1-2)
 *   3. OUTPUT STAGE (OUT 1..4):
 *      Gain (-12..+12 dB) -> Mute -> Phase (0/180°)
 *      -> Crossover HPF & LPF (LR / Butterworth / Bessel: 12, 24, 48 dB/oct)
 *      -> 9-Band PEQ (16 Hz - 20 kHz, resolusi 0.5 Hz, ±15 dB, Q 0.4 - 12.0)
 *      -> Delay Line hingga 300 ms di PSRAM (ps_malloc)
 *      -> Output Noise Gate (default -50 dB) -> Peak Limiter -> Hard/Soft Anti-Clip
 *   4. MASTER & METERING:
 *      Master Volume (0 dB s/d -60 dB smoothing) -> Peak & RMS 6-Kanal
 *      -> Clip/Limiter Flags -> Lightweight 256/512-Point FFT RTA Spectrum
 *
 * Proteksi Anti-Denormal:
 *   Alternating micro-offset (±1e-25f) + Flush-To-Zero (< 1e-20f) pada seluruh
 *   state Biquad & Envelope Follower agar tidak muncul bunyi "tek" saat hening.
 */

#ifdef NATIVE_TEST
  #include <cmath>
  #include <cstddef>
  #include <cstdint>
  #include <cstdlib>
  #include "audio_config.h"

  // Minimal AudioBlock & critical-section stubs untuk pengujian murni di PC
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
// 1. KONSTANTA ARSITEKTUR DSP DLMS 2-IN 4-OUT
// ============================================================================
constexpr uint8_t  DSP_NUM_INPUTS        = 2;
constexpr uint8_t  DSP_NUM_OUTPUTS       = 4;
constexpr uint8_t  DSP_TOTAL_CHANNELS    = DSP_NUM_INPUTS + DSP_NUM_OUTPUTS; // 6 ch
constexpr uint8_t  DSP_MATRIX_SOURCES    = 4;  // IN1, IN2, USB_L, USB_R
constexpr uint8_t  DSP_PEQ_BANDS         = 9;
constexpr uint8_t  DSP_MAX_XOVER_STAGES  = 4;  // 48 dB/oct = 8th-order = 4 biquads
constexpr float    DSP_MAX_DELAY_MS      = 300.0f;
constexpr uint16_t DSP_RTA_MAX_FFT_SIZE  = 512;
constexpr uint8_t  DSP_RTA_ISO_BANDS     = 31; // 20 Hz - 20 kHz 1/3-Octave RTA

constexpr uint32_t DSP_CONFIG_MAGIC      = 0x444C4D53; // "DLMS"
constexpr uint16_t DSP_CONFIG_VERSION    = 0x0100;     // v1.0

// Indeks sumber pada Routing Matrix
enum MatrixSource : uint8_t {
  SRC_IN1   = 0,
  SRC_IN2   = 1,
  SRC_USB_L = 2,
  SRC_USB_R = 3
};

// ID Kanal terpadu untuk fungsi helper copyChannelSettings()
enum class ChannelId : int8_t {
  IN1         = 0,
  IN2         = 1,
  OUT1        = 2,
  OUT2        = 3,
  OUT3        = 4,
  OUT4        = 5,
  ALL_OUTPUTS = -1 // Target khusus untuk menyalin pengaturan 1 Output ke semua OUT 1..4
};

// Sumber tap sinyal untuk RTA FFT Spectrum
enum class RtaSource : uint8_t {
  IN1       = 0,
  IN2       = 1,
  OUT1      = 2,
  OUT2      = 3,
  OUT3      = 4,
  OUT4      = 5,
  IN_MIX    = 6, // (IN1 + IN2) * 0.5
  OUT12_MIX = 7  // (OUT1 + OUT2) * 0.5
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
  float holdMs;          // 0.0 - 500.0 ms (anti-chatter)
  float rangeDb;         // -80.0 - 0.0 dB (kedalaman atenuasi saat gate tertutup)
};

struct CompressorConfig {
  bool  enabled;
  float thresholdDb;     // -60.0 - 0.0 dB
  float ratio;           // 1.0 - 20.0 (1:1 s/d 20:1)
  float attackMs;        // 0.1 - 200.0 ms
  float releaseMs;       // 10.0 - 2000.0 ms
  float kneeDb;          // 0.0 - 12.0 dB (soft knee)
  float makeupGainDb;    // 0.0 - 18.0 dB
};

enum class XoverType : uint8_t {
  LINKWITZ_RILEY = 0,
  BUTTERWORTH    = 1,
  BESSEL         = 2
};

enum class XoverSlope : uint8_t {
  SLOPE_12DB = 12,       // 2nd-order (1 biquad)
  SLOPE_24DB = 24,       // 4th-order (2 biquads)
  SLOPE_48DB = 48        // 8th-order (4 biquads)
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
  SOFT_TANH = 0,         // Saturasi lembut (soft-clip) sebelum full-scale
  HARD_CLIP = 1          // Clamping tegas di ±1.0 full-scale
};

enum class GeneratorType : uint8_t {
  OFF         = 0,
  WHITE_NOISE = 1,
  PINK_NOISE  = 2
};

struct SignalGeneratorConfig {
  GeneratorType type;
  float         levelDb;        // -60.0 - -6.0 dB (default aman: -20.0 dB)
  uint8_t       destMask;       // Bit 0..3 = OUT 1..4 (default 0x0F = semua output)
  uint16_t      autoOffSeconds; // Default 30 detik
};

struct RoutingMatrixConfig {
  // mixGain[outIdx][srcIdx]: bobot linear 0.0 .. 1.0 (0.0 = terputus)
  // Default:
  //   OUT 1 = IN1 (1.0) + USB_L (1.0)
  //   OUT 2 = IN2 (1.0) + USB_R (1.0)
  //   OUT 3 = IN1 (1.0)  [Direct/Bypass dari IN 1]
  //   OUT 4 = IN2 (1.0)  [Direct/Bypass dari IN 2]
  float mixGain[DSP_NUM_OUTPUTS][DSP_MATRIX_SOURCES];

  // Jika true: sinyal USB Record ke PC diambil SETELAH Input Stage DSP (Gate/PEQ/Comp)
  // Jika false: sinyal ADC mentah (hanya DC blocker)
  bool usbRecPostInputDsp;
};

struct InputChannelConfig {
  float            gainDb;      // -12.0 - +12.0 dB
  bool             mute;
  bool             phaseInvert; // false = 0°, true = 180°
  NoiseGateConfig  gate;
  PeqBandConfig    peq[DSP_PEQ_BANDS];
  CompressorConfig comp;
};

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
  bool              bypassDsp;   // True: bypass Xover/PEQ/Delay/Gate (tetap aman di Anti-Clip)
};

struct MasterConfig {
  float     volumeDb;            // -60.0 - 0.0 dB (smooth ramp)
  bool      mute;
  RtaSource rtaSource;
  uint16_t  rtaFftSize;          // 256 atau 512
};

// Struktur konfigurasi lengkap sistem (siap disimpan/dimuat JSON LittleFS / NVS)
struct SystemConfig {
  uint32_t              magic;
  uint16_t              version;
  char                  presetName[32];
  InputChannelConfig    inputs[DSP_NUM_INPUTS];
  SignalGeneratorConfig generator;
  RoutingMatrixConfig   matrix;
  OutputChannelConfig   outputs[DSP_NUM_OUTPUTS];
  MasterConfig          master;
};

// ============================================================================
// 3. TELEMETRI REAL-TIME DSP (DIBACA CORE 0: UI ST7789 / WEB / SERIAL)
// ============================================================================
struct DspMeters {
  // Peak & RMS dalam satuan linear [0..1] dan dBFS [-96..0]
  float inPeakLin[DSP_NUM_INPUTS];
  float inRmsLin[DSP_NUM_INPUTS];
  float inPeakDb[DSP_NUM_INPUTS];
  float inRmsDb[DSP_NUM_INPUTS];

  float outPeakLin[DSP_NUM_OUTPUTS];
  float outRmsLin[DSP_NUM_OUTPUTS];
  float outPeakDb[DSP_NUM_OUTPUTS];
  float outRmsDb[DSP_NUM_OUTPUTS];

  // Gain Reduction real-time (<= 0.0 dB)
  float inCompGrDb[DSP_NUM_INPUTS];
  float outLimiterGrDb[DSP_NUM_OUTPUTS];

  // Indikator status real-time
  bool inClip[DSP_NUM_INPUTS];
  bool outClip[DSP_NUM_OUTPUTS];
  bool inGateClosed[DSP_NUM_INPUTS];
  bool outGateClosed[DSP_NUM_OUTPUTS];
  bool outLimiterActive[DSP_NUM_OUTPUTS];

  // Status Signal Generator & Delay PSRAM
  bool     generatorActive;
  uint16_t generatorRemainingSec;
  bool     delayAllocatedInPsram;
};

struct RtaSpectrumData {
  uint16_t fftSize;                            // 256 atau 512
  uint16_t numBins;                            // fftSize / 2
  float    binFreqStepHz;                      // 48000 / fftSize (93.75 Hz @ 512)
  float    binDb[DSP_RTA_MAX_FFT_SIZE / 2];    // Spektrum mentah per bin (dBFS)
  float    bandDb[DSP_RTA_ISO_BANDS];          // 31-band ISO 1/3-Octave (20 Hz - 20 kHz)
  bool     valid;
};

// ============================================================================
// 4. PRIMITIF DSP INTERNAL (BIQUAD, GATE, COMP, LIMITER, DELAY, NOISE GEN)
// ============================================================================
struct BiquadState {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float s1 = 0.0f, s2 = 0.0f;
  bool  active = false;

  inline void reset() { s1 = 0.0f; s2 = 0.0f; }

  // Direct Form II Transposed + Proteksi Anti-Denormal (offset mikro + FTZ)
  inline float process(float x, float antiDenormalOffset) {
    if (!active) return x;
    const float xn = x + antiDenormalOffset;
    float y        = b0 * xn + s1;
    s1 = b1 * xn - a1 * y + s2;
    s2 = b2 * xn - a2 * y;

    // Flush-to-zero pada state & output untuk mencegah subnormal/denormal & bunyi "tek"
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
  uint32_t holdSamples  = 0;
  uint32_t holdCounter  = 0;
  bool  gateOpen   = true;
  bool  enabled    = false;

  void reset();
  void configure(const NoiseGateConfig &cfg, float sampleRate);
  float process(float x, float antiDenormalOffset);
};

struct CompressorState {
  float grDb        = 0.0f;     // Gain reduction terkini (<= 0 dB)
  float threshDb    = -18.0f;
  float ratio       = 4.0f;
  float kneeDb      = 3.0f;
  float makeupLin   = 1.0f;
  float atkCoeff    = 0.01f;
  float relCoeff    = 0.001f;
  bool  enabled     = false;

  void reset();
  void configure(const CompressorConfig &cfg, float sampleRate);
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
  float    targetDelay  = 0.0f; // dalam sampel
  float    currentDelay = 0.0f; // smoothed delay (anti-zipper)
  bool     inPsram      = false;

  bool allocate(float maxDelayMs, float sampleRate);
  void release();
  void clear();
  void setDelayMs(float delayMs, float sampleRate, bool snapImmediate = false);
  float process(float x);
};

struct NoiseGeneratorState {
  uint32_t prngState = 0xA5C3F197u;
  // Paul Kellet 7-pole Pink Noise filter states
  float b0 = 0.0f, b1 = 0.0f, b2 = 0.0f, b3 = 0.0f, b4 = 0.0f, b5 = 0.0f, b6 = 0.0f;
  uint32_t remainingSamples = 0;
  float    levelLin         = 0.1f;
  float    fadeGain         = 0.0f;
  GeneratorType activeType  = GeneratorType::OFF;

  void reset();
  void trigger(const SignalGeneratorConfig &cfg, float sampleRate);
  float nextSample(float antiDenormalOffset);
};

// ============================================================================
// 5. KELAS UTAMA DSP ENGINE (CORE 1 REAL-TIME + CORE 0 RTA/CONTROL API)
// ============================================================================
class DspEngine {
public:
  DspEngine() = default;
  ~DspEngine();

  // Inisialisasi engine, default config, alokasi 4x300ms Delay Line di PSRAM, & tabel FFT
  bool begin(float sampleRate = static_cast<float>(AUDIO_SAMPLE_RATE));

  // Mengisi struct SystemConfig dengan nilai default pabrik DLMS 2-In 4-Out
  static void loadDefaultConfig(SystemConfig &cfg);

  // Terapkan konfigurasi baru secara thread-safe (dipanggil dari Core 0 UI/Web/Storage)
  void applyConfig(const SystemConfig &newCfg);
  SystemConfig getConfig() const;

  // Snap instan seluruh smoother (gain/delay) & reset state filter (berguna untuk reset/unit test)
  void snapAndResetStates();

  // Trigger / Matikan Signal Generator (otomatis reset timer auto-off 30 detik)
  void setGenerator(GeneratorType type, float levelDb = -20.0f,
                    uint8_t destMask = 0x0F, uint16_t autoOffSec = 30);
  void stopGenerator();

  // Helper copy pengaturan kanal:
  //  - IN1 <-> IN2
  //  - OUTx -> OUTy atau OUTx -> ChannelId::ALL_OUTPUTS (salin ke semua OUT 1..4)
  bool copyChannelSettings(ChannelId src, ChannelId dst);

  // Atur ukuran FFT RTA (256 atau 512) dan sumber tap RTA
  void setRtaConfig(RtaSource source, uint16_t fftSize = 512);

  // Callback statis untuk didaftarkan ke AudioIO::setProcessCallback()
  static void audioCallbackAdapter(audio_io::AudioBlock &block, void *userCtx);

  // Fungsi pemrosesan real-time per blok (128 frame) di Core 1
  void processBlock(audio_io::AudioBlock &block);

  // Ambil snapshot meter Peak/RMS/GR/Clip/Limiter secara aman dari Core 0
  DspMeters getMeters() const;

  // Hitung spektrum FFT 256/512-point & 31-band ISO RTA di Core 0 (non-blocking Core 1)
  // Mengembalikan true jika ada frame FFT baru yang selesai dihitung
  bool computeRtaSpectrum(RtaSpectrumData &outSpectrum);

  // Utilitas konversi matematika cepat & aman
  static float quantizeFreqHalfHz(float freqHz);
  static float dbToLinear(float db);
  static float linearToDb(float lin);

private:
  // Hitung ulang seluruh koefisien filter/dynamics pada batas blok di Core 1
  void syncPendingParameters();

  // Desain koefisien Biquad IIR (Audio EQ Cookbook, kalkulasi presisi double)
  static void designPeqBiquad(BiquadState &bq, const PeqBandConfig &cfg, float sampleRate);

  // Soft/Hard Anti-Clip Limiter sebelum konversi ke integer I2S
  static float applyAntiClip(float x, AntiClipMode mode);

  // Radix-2 Cooley-Tukey FFT in-place ringan (256 atau 512-point)
  void runRadix2Fft(float *real, float *imag, uint16_t n) const;
  void buildRtaLookupTables(uint16_t fftSize);

  float sampleRate_ = static_cast<float>(AUDIO_SAMPLE_RATE);
  bool  initialized_ = false;

  // Double-buffered configuration (Core 0 tulis pendingCfg_, Core 1 baca activeCfg_)
  mutable portMUX_TYPE configMux_ = portMUX_INITIALIZER_UNLOCKED;
  SystemConfig pendingCfg_ = {};
  SystemConfig activeCfg_  = {};
  volatile bool configDirty_    = false;
  volatile bool generatorDirty_ = false;

  // State Input Stage (IN 1 & IN 2)
  float           inGainTarget_[DSP_NUM_INPUTS] = {1.0f, 1.0f};
  float           inGainSmooth_[DSP_NUM_INPUTS] = {1.0f, 1.0f};
  NoiseGateState  inGate_[DSP_NUM_INPUTS];
  BiquadState     inPeq_[DSP_NUM_INPUTS][DSP_PEQ_BANDS];
  CompressorState inComp_[DSP_NUM_INPUTS];

  // State Signal Generator
  NoiseGeneratorState noiseGen_;

  // State Output Stage (OUT 1..4)
  float            outGainTarget_[DSP_NUM_OUTPUTS] = {1.0f, 1.0f, 1.0f, 1.0f};
  float            outGainSmooth_[DSP_NUM_OUTPUTS] = {1.0f, 1.0f, 1.0f, 1.0f};
  CrossoverChain   outHpf_[DSP_NUM_OUTPUTS];
  CrossoverChain   outLpf_[DSP_NUM_OUTPUTS];
  BiquadState      outPeq_[DSP_NUM_OUTPUTS][DSP_PEQ_BANDS];
  DelayLinePsram   outDelay_[DSP_NUM_OUTPUTS];
  NoiseGateState   outGate_[DSP_NUM_OUTPUTS];
  PeakLimiterState outLimiter_[DSP_NUM_OUTPUTS];

  // State Master Volume
  float masterGainTarget_ = 1.0f;
  float masterGainSmooth_ = 1.0f;

  // Anti-Denormal alternating sign (±1e-25f)
  float antiDenormalOffset_ = 1.0e-25f;

  // Buffer kerja internal per blok (statis di dalam objek -> 0 heap alloc di Core 1)
  alignas(16) float inProcBuf_[DSP_NUM_INPUTS][AUDIO_BLOCK_FRAMES] = {{0}};

  // Telemetri Meter
  mutable portMUX_TYPE meterMux_ = portMUX_INITIALIZER_UNLOCKED;
  DspMeters meters_ = {};

  // RTA Capture & FFT Buffers (Core 1 isi capture, Core 0 proses FFT)
  mutable portMUX_TYPE rtaMux_ = portMUX_INITIALIZER_UNLOCKED;
  uint16_t rtaFftSize_         = 512;
  uint16_t rtaWritePos_        = 0;
  bool     rtaFrameReady_      = false;
  float    rtaCaptureBuf_[DSP_RTA_MAX_FFT_SIZE] = {0};
  float    rtaReadyBuf_[DSP_RTA_MAX_FFT_SIZE]   = {0};
  float    rtaHannWindow_[DSP_RTA_MAX_FFT_SIZE] = {0};
  float    rtaFftReal_[DSP_RTA_MAX_FFT_SIZE]    = {0};
  float    rtaFftImag_[DSP_RTA_MAX_FFT_SIZE]    = {0};
  float    rtaBandSmooth_[DSP_RTA_ISO_BANDS]    = {0};
};

} // namespace dsp
