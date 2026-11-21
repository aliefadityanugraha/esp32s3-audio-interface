/**
 * @file test_dsp_pure.cpp
 * @brief Pure Native PC Unit Tests (Unity Framework) untuk DspEngine (DLMS 2-In 4-Out)
 *        Menjalankan seluruh verifikasi matematika DSP tanpa hardware ESP32-S3, ADC, maupun DAC.
 *
 * Cara menjalankan di PlatformIO:
 *   pio test -e native
 */

#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

#include "dsp_engine.h"

using namespace dsp;
using namespace audio_io;

namespace {

constexpr float SAMPLE_RATE = static_cast<float>(AUDIO_SAMPLE_RATE); // 48000.0f
constexpr double PI_D       = 3.14159265358979323846;

static DspEngine gDsp;

// ============================================================================
// GENERATOR SINYAL UJI SINTETIS (IMPULSE & SINE WAVE)
// ============================================================================

// Menghasilkan array Impulse: 1.0f pada sampel ke-0, sisanya 0.0f
std::vector<float> genImpulse(size_t totalSamples) {
  std::vector<float> buf(totalSamples, 0.0f);
  if (totalSamples > 0) buf[0] = 1.0f;
  return buf;
}

// Menghasilkan array Sine Wave pada frekuensi `freqHz` dengan amplitudo `ampDbFs` (mis. -6 atau +6 dBFS)
std::vector<float> genSine(size_t totalSamples, float freqHz, float ampDbFs) {
  std::vector<float> buf(totalSamples, 0.0f);
  const float amp = powf(10.0f, ampDbFs / 20.0f);
  const double inc = 2.0 * PI_D * static_cast<double>(freqHz) / static_cast<double>(SAMPLE_RATE);
  for (size_t i = 0; i < totalSamples; i++) {
    buf[i] = amp * static_cast<float>(sin(inc * static_cast<double>(i)));
  }
  return buf;
}

// Evaluasi DFT eksak pada 1 frekuensi `freqHz` dari respon impulse h[n] -> Magnitudo (dB) & Fasa (derajat)
struct FreqPoint {
  float magDb;
  float phaseDeg;
};

FreqPoint evalImpulseDft(const std::vector<float> &h, float freqHz) {
  const double w = 2.0 * PI_D * static_cast<double>(freqHz) / static_cast<double>(SAMPLE_RATE);
  double re = 0.0, im = 0.0;
  for (size_t n = 0; n < h.size(); n++) {
    const double ang = w * static_cast<double>(n);
    re += static_cast<double>(h[n]) * cos(ang);
    im -= static_cast<double>(h[n]) * sin(ang);
  }
  const double mag = sqrt(re * re + im * im);
  FreqPoint fp;
  fp.magDb    = static_cast<float>(20.0 * log10(mag + 1.0e-12));
  fp.phaseDeg = static_cast<float>(atan2(im, re) * (180.0 / PI_D));
  return fp;
}

// Jalankan array input melalui DspEngine dalam blok 128 sampel dan kumpulkan 1 kanal output
std::vector<float> runMonoAdcThroughOut(DspEngine &eng,
                                        const std::vector<float> &inSignal,
                                        uint8_t adcCh, uint8_t outCh) {
  const size_t nBlocks = (inSignal.size() + AUDIO_BLOCK_FRAMES - 1) / AUDIO_BLOCK_FRAMES;
  std::vector<float> out(nBlocks * AUDIO_BLOCK_FRAMES, 0.0f);
  AudioBlock blk = {};
  blk.frames = AUDIO_BLOCK_FRAMES;
  blk.usbPlaybackActive = true;

  for (size_t b = 0; b < nBlocks; b++) {
    memset(blk.adcIn, 0, sizeof(blk.adcIn));
    memset(blk.usbIn, 0, sizeof(blk.usbIn));
    for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
      const size_t idx = b * AUDIO_BLOCK_FRAMES + i;
      if (idx < inSignal.size()) {
        blk.adcIn[adcCh][i] = inSignal[idx];
      }
    }
    eng.processBlock(blk);
    for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
      out[b * AUDIO_BLOCK_FRAMES + i] = blk.dacOut[outCh][i];
    }
  }
  return out;
}

// Helper: konfgurasi bersih tanpa gate/limiter/anti-clip pada semua output untuk uji linearitas filter
SystemConfig makeLinearFilterConfig() {
  SystemConfig cfg = {};
  DspEngine::loadDefaultConfig(cfg);
  for (int o = 0; o < DSP_NUM_OUTPUTS; o++) {
    cfg.outputs[o].gate.enabled    = false;
    cfg.outputs[o].limiter.enabled = false;
    cfg.outputs[o].antiClipMode    = AntiClipMode::HARD_CLIP;
  }
  return cfg;
}

float measurePeakDb(const std::vector<float> &sig, size_t startIdx, size_t endIdx) {
  float pk = 0.0f;
  for (size_t i = startIdx; i < endIdx && i < sig.size(); i++) {
    float a = fabsf(sig[i]);
    if (a > pk) pk = a;
  }
  return (pk <= 1.0e-6f) ? -120.0f : (20.0f * log10f(pk));
}

} // namespace

void setUp(void) {
  TEST_ASSERT_TRUE(gDsp.begin(SAMPLE_RATE));
}

void tearDown(void) {}

// ============================================================================
// TEST 1: IMPULSE RESPONSE — 9-BAND PEQ (MAGNITUDO & FASA)
// ============================================================================
void test_peq_9band_impulse_response(void) {
  SystemConfig cfg = makeLinearFilterConfig();
  // Gunakan OUT 1 dengan gain -12 dB supaya puncak impulse PEQ (+6 dB) tidak clip di ±1.0
  cfg.outputs[0].gainDb = -6.0f;
  // Aktifkan beberapa band dari 9-Band PEQ pada OUT 1
  cfg.outputs[0].peq[2] = {true, PeqFilterType::PEAKING, 250.0f,  -6.0f, 1.4f};
  cfg.outputs[0].peq[5] = {true, PeqFilterType::PEAKING, 1000.0f, +6.0f, 1.4f};
  cfg.outputs[0].peq[7] = {true, PeqFilterType::PEAKING, 4000.0f, +3.0f, 2.0f};
  gDsp.applyConfig(cfg);
  gDsp.snapAndResetStates();

  auto imp = genImpulse(4096);
  auto res = runMonoAdcThroughOut(gDsp, imp, 0, 0);

  // Kompensasi -6 dB headroom agar diukur relatif terhadap 0 dB
  const FreqPoint f250  = evalImpulseDft(res, 250.0f);
  const FreqPoint f1000 = evalImpulseDft(res, 1000.0f);
  const FreqPoint f4000 = evalImpulseDft(res, 4000.0f);

  TEST_ASSERT_FLOAT_WITHIN(0.35f, -6.0f, f250.magDb  + 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.35f, +6.0f, f1000.magDb + 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.35f, +3.0f, f4000.magDb + 6.0f);

  // Di titik puncak Peaking EQ terisolasi (1 kHz), fasa sangat dekat 0°
  TEST_ASSERT_FLOAT_WITHIN(12.0f, 0.0f, f1000.phaseDeg);
}

// ============================================================================
// TEST 2: IMPULSE RESPONSE — CROSSOVER LR, BUTTERWORTH, BESSEL (12/24/48 dB/OCT)
// ============================================================================
void test_crossover_types_and_slopes_impulse(void) {
  const XoverSlope slopes[3] = {
      XoverSlope::SLOPE_12DB, XoverSlope::SLOPE_24DB, XoverSlope::SLOPE_48DB};

  // A. Linkwitz-Riley (12, 24, 48 dB/oct): tepat -6.02 dB @ fc (1 kHz)
  for (int s = 0; s < 3; s++) {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].lpf = {true, XoverType::LINKWITZ_RILEY, slopes[s], 1000.0f};
    cfg.outputs[1].hpf = {true, XoverType::LINKWITZ_RILEY, slopes[s], 1000.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    auto imp  = genImpulse(4096);
    auto outL = runMonoAdcThroughOut(gDsp, imp, 0, 0);
    gDsp.snapAndResetStates();
    auto outH = runMonoAdcThroughOut(gDsp, imp, 1, 1);

    const FreqPoint lpFc = evalImpulseDft(outL, 1000.0f);
    const FreqPoint hpFc = evalImpulseDft(outH, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.35f, -6.02f, lpFc.magDb);
    TEST_ASSERT_FLOAT_WITHIN(0.35f, -6.02f, hpFc.magDb);

    // Verifikasi LR24 & LR48 memiliki penjumlahan fasa datar (0 dB flat sum @ fc!)
    if (slopes[s] != XoverSlope::SLOPE_12DB) {
      std::vector<float> sum(outL.size(), 0.0f);
      for (size_t i = 0; i < sum.size(); i++) sum[i] = outL[i] + outH[i];
      const FreqPoint sumFc = evalImpulseDft(sum, 1000.0f);
      TEST_ASSERT_FLOAT_WITHIN(0.35f, 0.0f, sumFc.magDb);
    }
  }

  // B. Butterworth (12, 24, 48 dB/oct): tepat -3.01 dB @ fc & redaman 1 oktaf di atas fc
  for (int s = 0; s < 3; s++) {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].lpf = {true, XoverType::BUTTERWORTH, slopes[s], 1000.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    auto imp = genImpulse(4096);
    auto out = runMonoAdcThroughOut(gDsp, imp, 0, 0);

    const FreqPoint atFc  = evalImpulseDft(out, 1000.0f);
    const FreqPoint at2Fc = evalImpulseDft(out, 2000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.35f, -3.01f, atFc.magDb);

    // Redaman 1 oktaf di atas fc pada Butterworth order N: -10*log10(1 + 2^(2N))
    const float expected2Fc = (slopes[s] == XoverSlope::SLOPE_12DB) ? -12.3f
                            : (slopes[s] == XoverSlope::SLOPE_24DB ? -24.1f : -48.2f);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, expected2Fc, at2Fc.magDb);
  }

  // C. Bessel (12, 24, 48 dB/oct): ternormalisasi tepat -3.01 dB @ fc (1 kHz)
  for (int s = 0; s < 3; s++) {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].lpf = {true, XoverType::BESSEL, slopes[s], 1000.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    auto imp = genImpulse(4096);
    auto out = runMonoAdcThroughOut(gDsp, imp, 0, 0);

    const FreqPoint atFc = evalImpulseDft(out, 1000.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.45f, -3.01f, atFc.magDb);
  }
}

// ============================================================================
// TEST 3: SINE 1 kHz (-6 dBFS & +6 dBFS) — COMPRESSOR, NOISE GATE, PEAK LIMITER
// ============================================================================
void test_dynamics_compressor_gate_limiter(void) {
  // ---- A. Input Compressor: Threshold -18 dB, Ratio 4:1, Hard Knee vs Soft Knee ----
  {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.inputs[0].comp = {true, -18.0f, 4.0f, 1.0f, 80.0f, 0.0f, 0.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    auto sineNeg6 = genSine(4800, 1000.0f, -6.0f); // 100 ms @ -6 dBFS (12 dB di atas thr)
    auto outNeg6  = runMonoAdcThroughOut(gDsp, sineNeg6, 0, 0);
    const DspMeters mNeg6 = gDsp.getMeters();
    // Di -6 dBFS (12 dB di atas -18 dB, ratio 4:1): GR teoritis ~ -9 dB
    TEST_ASSERT_FLOAT_WITHIN(2.0f, -9.0f, mNeg6.inCompGrDb[0]);

    // Uji +6 dBFS (24 dB di atas -18 dB, ratio 4:1): GR harus jauh lebih dalam (~ -18 dB)
    auto sinePos6 = genSine(4800, 1000.0f, +6.0f);
    runMonoAdcThroughOut(gDsp, sinePos6, 0, 0);
    const DspMeters mPos6 = gDsp.getMeters();
    TEST_ASSERT_TRUE(mPos6.inCompGrDb[0] < mNeg6.inCompGrDb[0] - 5.0f);

    // Uji Soft Knee (12 dB) tepat di threshold (-18 dBFS): GR harus lembut (< -0.2 dB dan > -4 dB)
    cfg.inputs[0].comp.kneeDb = 12.0f;
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();
    auto sineAtThr = genSine(4800, 1000.0f, -18.0f);
    runMonoAdcThroughOut(gDsp, sineAtThr, 0, 0);
    const DspMeters mKnee = gDsp.getMeters();
    TEST_ASSERT_TRUE(mKnee.inCompGrDb[0] < -0.2f && mKnee.inCompGrDb[0] > -4.0f);
  }

  // ---- B. Output Noise Gate (default -50 dB) ----
  {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].gate = {true, -50.0f, 1.0f, 40.0f, 5.0f, -80.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    // Sinyal di bawah threshold (-65 dBFS) -> Gate harus menutup & meredam output
    auto quietSine = genSine(9600, 1000.0f, -65.0f);
    auto outQuiet  = runMonoAdcThroughOut(gDsp, quietSine, 0, 0);
    const DspMeters mQ = gDsp.getMeters();
    TEST_ASSERT_TRUE(mQ.outGateClosed[0]);
    TEST_ASSERT_TRUE(measurePeakDb(outQuiet, 7000, 9600) < -90.0f);

    // Sinyal di atas threshold (-6 dBFS) -> Gate harus membuka penuh (0 dB loss)
    auto loudSine = genSine(4800, 1000.0f, -6.0f);
    auto outLoud  = runMonoAdcThroughOut(gDsp, loudSine, 0, 0);
    const DspMeters mL = gDsp.getMeters();
    TEST_ASSERT_FALSE(mL.outGateClosed[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, -6.0f, measurePeakDb(outLoud, 2400, 4800));
  }

  // ---- C. Output Peak Limiter (-1 dBFS) dengan Sine -6 dBFS & +6 dBFS ----
  {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].limiter = {true, -1.0f, 0.2f, 100.0f};
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    // -6 dBFS di bawah threshold -1 dBFS -> limiter tidak aktif (GR == 0 dB)
    auto sineNeg6 = genSine(4800, 1000.0f, -6.0f);
    auto outNeg6  = runMonoAdcThroughOut(gDsp, sineNeg6, 0, 0);
    const DspMeters mN = gDsp.getMeters();
    TEST_ASSERT_FALSE(mN.outLimiterActive[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.4f, -6.0f, measurePeakDb(outNeg6, 2400, 4800));

    // +6 dBFS (amplitudo 2.0f) -> limiter wajib aktif & menahan puncak di sekitar -1 dBFS
    auto sinePos6 = genSine(4800, 1000.0f, +6.0f);
    auto outPos6  = runMonoAdcThroughOut(gDsp, sinePos6, 0, 0);
    const DspMeters mP = gDsp.getMeters();
    TEST_ASSERT_TRUE(mP.outLimiterActive[0]);
    TEST_ASSERT_TRUE(mP.outLimiterGrDb[0] < -4.0f);
    TEST_ASSERT_FLOAT_WITHIN(1.2f, -1.0f, measurePeakDb(outPos6, 2400, 4800));
  }
}

// ============================================================================
// TEST 4: ROUTING MATRIX DEFAULT & COPY CHANNEL SETTINGS (OUT_1 -> ALL)
// ============================================================================
void test_routing_matrix_and_copy_channel(void) {
  // ---- A. Verifikasi Default Routing Matrix ----
  // OUT 1-2: Mix IN 1-2 + USB RX; OUT 3-4: Direct/Bypass hanya dari IN 1-2
  SystemConfig cfg = makeLinearFilterConfig();
  gDsp.applyConfig(cfg);
  gDsp.snapAndResetStates();

  AudioBlock blk = {};
  blk.frames = AUDIO_BLOCK_FRAMES;
  blk.usbPlaybackActive = true;
  for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
    blk.adcIn[0][i] = 0.25f; // IN 1
    blk.adcIn[1][i] = 0.15f; // IN 2
    blk.usbIn[0][i] = 0.40f; // USB L
    blk.usbIn[1][i] = 0.30f; // USB R
  }
  gDsp.processBlock(blk);

  const size_t k = AUDIO_BLOCK_FRAMES - 1;
  TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.65f, blk.dacOut[0][k]); // OUT 1 = 0.25 + 0.40
  TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.45f, blk.dacOut[1][k]); // OUT 2 = 0.15 + 0.30
  TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.25f, blk.dacOut[2][k]); // OUT 3 = 0.25 (Direct IN 1)
  TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 0.15f, blk.dacOut[3][k]); // OUT 4 = 0.15 (Direct IN 2)

  // ---- B. Verifikasi copyChannelSettings(OUT1 -> ALL_OUTPUTS) 100% Identik ----
  cfg = gDsp.getConfig();
  cfg.outputs[0].gainDb      = 3.5f;
  cfg.outputs[0].phaseInvert = true;
  cfg.outputs[0].hpf         = {true, XoverType::LINKWITZ_RILEY, XoverSlope::SLOPE_48DB, 95.5f};
  cfg.outputs[0].lpf         = {true, XoverType::BESSEL,         XoverSlope::SLOPE_24DB, 14500.0f};
  cfg.outputs[0].peq[4]      = {true, PeqFilterType::PEAKING,    630.5f, -4.5f, 3.2f};
  cfg.outputs[0].delayMs     = 123.5f;
  cfg.outputs[0].limiter     = {true, -3.5f, 0.4f, 180.0f};
  cfg.outputs[0].antiClipMode= AntiClipMode::HARD_CLIP;
  gDsp.applyConfig(cfg);

  TEST_ASSERT_TRUE(gDsp.copyChannelSettings(ChannelId::OUT1, ChannelId::ALL_OUTPUTS));
  const SystemConfig after = gDsp.getConfig();

  for (int ch = 1; ch < DSP_NUM_OUTPUTS; ch++) {
    TEST_ASSERT_EQUAL_MEMORY(&after.outputs[0], &after.outputs[ch], sizeof(OutputChannelConfig));
  }
}

// ============================================================================
// TEST 5: DELAY LINE HINGGA 300 MS @ 48 kHz = 14.400 SAMPEL (IMPULSE EKSAK)
// ============================================================================
void test_delay_line_300ms_exact_sample_index(void) {
  // Uji 2 titik: 10.0 ms (480 sampel) dan maksimum 300.0 ms (14.400 sampel)
  const float testMs[2]      = {10.0f, 300.0f};
  const size_t expectedIdx[2]= {480,   14400};

  for (int t = 0; t < 2; t++) {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[2].delayMs = testMs[t];
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    const size_t totalSamples = expectedIdx[t] + 512;
    auto imp = genImpulse(totalSamples);
    auto out = runMonoAdcThroughOut(gDsp, imp, 0, 2);

    // Cari indeks puncak impulse pada output
    size_t peakIdx = 0;
    float  peakVal = 0.0f;
    for (size_t i = 0; i < out.size(); i++) {
      if (fabsf(out[i]) > peakVal) {
        peakVal = fabsf(out[i]);
        peakIdx = i;
      }
    }

    TEST_ASSERT_EQUAL_UINT32(static_cast<uint32_t>(expectedIdx[t]), static_cast<uint32_t>(peakIdx));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, 1.0f, peakVal);

    // Pastikan tidak ada kebocoran pulsa sebelum indeks delay
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, out[0]);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, out[expectedIdx[t] - 1]);
  }
}

// ============================================================================
// TEST 6: ANTI-DENORMAL (0 NaN / Inf / Subnormal) & AUTO-OFF GENERATOR 30 DETIK
// ============================================================================
void test_anti_denormal_and_generator_auto_off_30s(void) {
  // ---- A. Anti-Denormal saat input diputus dari sinyal keras ke 0.0f ----
  {
    SystemConfig cfg = makeLinearFilterConfig();
    cfg.outputs[0].hpf = {true, XoverType::LINKWITZ_RILEY, XoverSlope::SLOPE_48DB, 40.0f};
    cfg.outputs[0].lpf = {true, XoverType::BUTTERWORTH,    XoverSlope::SLOPE_48DB, 12000.0f};
    for (int b = 0; b < DSP_PEQ_BANDS; b++) {
      cfg.outputs[0].peq[b].gainDb = (b % 2 == 0) ? 4.0f : -3.0f;
      cfg.outputs[0].peq[b].q      = 6.0f; // Q tinggi agar ekor IIR panjang
    }
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();

    AudioBlock blk = {};
    blk.frames = AUDIO_BLOCK_FRAMES;
    // 10 blok sinyal keras lalu 200 blok hening total (25.600 sampel)
    for (int b = 0; b < 210; b++) {
      for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
        blk.adcIn[0][i] = (b < 10) ? 0.8f * sinf(0.13f * static_cast<float>(b * 128 + i)) : 0.0f;
        blk.adcIn[1][i] = 0.0f;
        blk.usbIn[0][i] = blk.usbIn[1][i] = 0.0f;
      }
      gDsp.processBlock(blk);
      for (int ch = 0; ch < DSP_NUM_OUTPUTS; ch++) {
        for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
          const float v = blk.dacOut[ch][i];
          TEST_ASSERT_TRUE(std::isfinite(v));
          TEST_ASSERT_TRUE(std::fpclassify(v) != FP_SUBNORMAL);
        }
      }
    }
  }

  // ---- B. White Noise & Pink Noise Auto-Off tepat setelah 1.440.000 sampel (30 detik) ----
  const GeneratorType types[2] = {GeneratorType::WHITE_NOISE, GeneratorType::PINK_NOISE};
  for (int t = 0; t < 2; t++) {
    SystemConfig cfg = makeLinearFilterConfig();
    gDsp.applyConfig(cfg);
    gDsp.snapAndResetStates();
    gDsp.setGenerator(types[t], -18.0f, 0x0F, 30); // 30 detik = 1.440.000 sampel @ 48 kHz

    AudioBlock blk = {};
    blk.frames = AUDIO_BLOCK_FRAMES;
    memset(blk.adcIn, 0, sizeof(blk.adcIn));
    memset(blk.usbIn, 0, sizeof(blk.usbIn));

    constexpr size_t TARGET_SAMPLES = 30u * AUDIO_SAMPLE_RATE; // 1.440.000 sampel
    constexpr size_t TOTAL_BLOCKS   = TARGET_SAMPLES / AUDIO_BLOCK_FRAMES; // tepat 11.250 blok

    float midPeak = 0.0f;
    for (size_t b = 0; b < TOTAL_BLOCKS; b++) {
      gDsp.processBlock(blk);
      if (b == 50) {
        TEST_ASSERT_TRUE(gDsp.getMeters().generatorActive);
        for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
          if (fabsf(blk.dacOut[0][i]) > midPeak) midPeak = fabsf(blk.dacOut[0][i]);
        }
      }
    }

    // Harus menghasilkan energi audio selama aktif, lalu tepat mati setelah 1.440.000 sampel
    TEST_ASSERT_TRUE(midPeak > 0.01f);
    TEST_ASSERT_FALSE(gDsp.getMeters().generatorActive);
    TEST_ASSERT_EQUAL_UINT16(0, gDsp.getMeters().generatorRemainingSec);

    // Jalankan beberapa blok setelah 30 detik -> fade-out mulus hingga tepat 0.0f
    for (int b = 0; b < 60; b++) {
      gDsp.processBlock(blk);
    }
    for (size_t i = 0; i < AUDIO_BLOCK_FRAMES; i++) {
      TEST_ASSERT_FLOAT_WITHIN(1.0e-5f, 0.0f, blk.dacOut[0][i]);
    }
  }
}

// ============================================================================
// UNITY TEST RUNNER ENTRY POINT
// ============================================================================
int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_peq_9band_impulse_response);
  RUN_TEST(test_crossover_types_and_slopes_impulse);
  RUN_TEST(test_dynamics_compressor_gate_limiter);
  RUN_TEST(test_routing_matrix_and_copy_channel);
  RUN_TEST(test_delay_line_300ms_exact_sample_index);
  RUN_TEST(test_anti_denormal_and_generator_auto_off_30s);
  return UNITY_END();
}
