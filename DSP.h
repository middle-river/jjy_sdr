/*
  Signal processing library.
  2021-09-12  T. Nakagawa
*/

#ifndef DSP_H_
#define DSP_H_

#pragma GCC optimize ("Ofast")

#include <algorithm>
#include <cmath>

#define USE_UNOPTIMIZED_CODE 0

#ifndef ARDUINO_ARCH_STM32
static inline int32_t __SSAT(int32_t val, uint32_t sat) { const int32_t lim = 1 << (sat - 1); return (val < -lim) ? -lim : (val >= lim) ? (lim - 1) : val; }
#endif

/*
  Make a sin/cos table (Numerically Controlled Oscillator).
  <tbl0/1>: int16_t[2 * size]
*/
static inline int nco(int samp, int freq, int size, int16_t *tbl0, int16_t *tbl1) {
  int f = (int)((float)freq * size / samp + 0.5);
  for (int i = 0; i < size; i++) {
    const float rad = i * 2.0f * (float)M_PI * f / size;
    tbl0[i] = __SSAT((int32_t)(sinf(rad) * 32768.0f), 16);
    tbl1[i] = __SSAT((int32_t)(cosf(rad) * 32768.0f), 16);
  }
  return (f * samp / size);
}

/*
  DC offset removal.
  <inp>: int16_t[size], <out>: int16_t[size]
*/
static inline void offset(uint32_t &ofst, int size, const int16_t *inp, int16_t *out) {
  const int16_t bias = (int16_t)(ofst >> 16);
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    const int16_t inpv = *inp++;
    sum += inpv;
    *out++ = inpv - bias;
  }
  ofst = (uint32_t)((uint64_t)ofst * (65536 - 2048) / 65536 + (uint64_t)sum * 2048 / size);
}

/*
  Mixer and CIC decimator (3-stage, 1/32).
  <tbl>: int16_t[size], <state>: int32_t[6], <inp>: int16_t[size], <out>: int16_t[size / 32]
*/
static inline void decimator_cic(const int16_t *tbl, int32_t *state, int size, const int16_t *inp, int16_t *out) {
#if USE_UNOPTIMIZED_CODE
  int32_t *intg = state;
  int32_t *comb = state + 3;
  for (int i = 0; i < size; i++) {
    // Mixer.
    const int32_t sgnl = (tbl[i] * (inp[i] << 4)) >> 15;
    // CIC decimator.
    for (int j = 0; j < 3; j++) intg[j] += (j == 0) ? sgnl : intg[j - 1];
    if (i % 32 == 31) {
      int32_t accm[3];
      for (int j = 0; j < 3; j++) {
        const int32_t y = (j == 0) ? intg[2] : accm[j - 1];
        accm[j] = y - comb[j];
        comb[j] = y;
      }
      out[i / 32] = __SSAT(accm[2] >> 15, 16);
    }
  }
#else
  int32_t intg0 = state[0], intg1 = state[1], intg2 = state[2];
  int32_t comb0 = state[3], comb1 = state[4], comb2 = state[5];
  for (int i = 0; i < size / 32; i++) {
    for (int j = 0; j < 32; j++) {
      const int32_t sgnl = ((*tbl++) * (*inp++)) >> 11;
      intg0 += sgnl; intg1 += intg0; intg2 += intg1;
    }
    const int32_t accm0 = intg2 - comb0; comb0 = intg2;
    const int32_t accm1 = accm0 - comb1; comb1 = accm0;
    const int32_t accm2 = accm1 - comb2; comb2 = accm1;
    *out++ = __SSAT(accm2 >> 15, 16);
  }
  state[0] = intg0; state[1] = intg1; state[2] = intg2;
  state[3] = comb0; state[4] = comb1; state[5] = comb2;
#endif
}

/*
  FIR decimator (32-tap, 1/8).
  <coeff>: int16_t[32] (q0.15), <state>: int16_t[32], <inp>: int16_t[size], <out>: int16_t[size / 8]
*/
static inline void decimator_fir(const int16_t *coeff, int16_t *state, int size, const int16_t *inp, int16_t *out) {
#if USE_UNOPTIMIZED_CODE
  for (int i = 0; i < size; i++) {
    for (int j = 31; j >= 1; j--) state[j] = state[j - 1];
    state[0] = inp[i];
    if (i % 8 == 7) {
      int32_t acc = 0;
      for (int j = 0; j < 32; j++) acc += coeff[j] * state[j];
      out[i / 8] = __SSAT(acc >> 15, 16);
    }
  }
#else
  int16_t stt[32];
  for (int i = 0; i < 32; i++) stt[i] = state[i];
  for (int i = 0; i < size / 32; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 8; k++) stt[j * 8 + k] = *inp++;
      int32_t acc = 0;	// q16.15.
      for (int k = 0; k < 16; k++) acc += coeff[k] * (stt[(j * 8 + 7 - k + 32) % 32] + stt[(j * 8 + 8 + k) % 32]);
      *out++ = __SSAT(acc >> 15, 16);
    }
  }
  for (int i = 0; i < 32; i++) state[i] = stt[i];
#endif
}

/*
  Hilbert transform with 31-tap FIR.
  <coeff>: int16_t[31] (q0.15), <state>: int16_t[65], <inp0/1>: int16_t[size], <out>: int16_t[size], <size> is assumed to be less than 19.
*/
static inline void hilbert(const int16_t *coeff, int16_t *state, int size, const int16_t *inp0, const int16_t *inp1, int16_t *out) {
#if USE_UNOPTIMIZED_CODE
  int16_t *s0 = state;	// int16_t[31]
  int16_t *s1 = state + 31;	// int16_t[16]
  for (int i = 0; i < size; i++) {
    for (int j = 30; j >= 1; j--) s0[j] = s0[j - 1];
    for (int j = 15; j >= 1; j--) s1[j] = s1[j - 1];
    s0[0] = inp0[i];
    s1[0] = inp1[i];
    int32_t acc = 0;
    for (int j = 0; j < 31; j++) acc += coeff[j] * s0[j];
    out[i] = __SSAT((acc >> 15) - s1[31 / 2], 16);
  }
#else
  int16_t *s0 = state;	// int16_t[32]
  int16_t *s1 = state + 32;	// int16_t[32]
  int16_t &ptr = state[64];
  for (int i = 0; i < size; i++) s1[(ptr + 15 + i) % 32] = *inp1++;
  for (int i = 0; i < size; i++) {
    s0[ptr] = *inp0++;
    int32_t acc = 0;
    for (int k = 0; k < 15; k += 2) acc += coeff[k] * (s0[(ptr - k + 32) % 32] - s0[(ptr + 2 + k) % 32]);
    *out++ = __SSAT((acc >> 15) - s1[ptr], 16);
    ptr++; if (ptr >= 32) ptr = 0;
  }
#endif
}

/*
  IIR filter (3-stage biquad).
  <coeff>: int16_t[6] (q1.14), <state>: int16_t[12], <inp>: int16_t[size], <out>: int16_t[size]
*/
static inline void bpf_iir(const int16_t *coeff, int16_t *state, int size, const int16_t *inp, int16_t *out) {
  for (int i = 0; i < size; i++) {
    int16_t inpv = *inp++;
    for (int j = 0; j < 3; j++) {
      int16_t *stt = state + j * 4;
      const int16_t outv = __SSAT((coeff[0] * inpv + coeff[1] * stt[0] + coeff[2] * stt[1] - coeff[4] * stt[2] - coeff[5] * stt[3]) >> 14, 16);
      stt[1] = stt[0]; stt[0] = inpv;
      stt[3] = stt[2]; stt[2] = outv;
      inpv = outv;
    }
    *out++ = inpv;
  }
}

/*
  Automatic gain control.
  <config>: int[6], <state>: int[2], <inp>: int16_t[size], <out>: int16_t[size]
*/
static inline void agc(const int *config, int *state, int size, const int16_t *inp, int16_t *out) {
  const int &agc_period = config[0];
  const int &attack_time = config[1];
  const int &hold_time = config[2];
  const int &release_time = config[3];
  const int &max_gain = config[4];
  const int &out_level = config[5];
  int &gain = state[0];
  int &hold = state[1];

  for (int i = 0; i < size; i++) {
    const int g = std::max(std::min(out_level / (std::abs(inp[i]) + 1), max_gain), 1);
    if (g < gain) {	// Attack.
      gain = std::max(gain - max_gain * agc_period / attack_time - 1, g);
      hold = hold_time / agc_period;
    } else if (hold > 0) {	// Hold.
      hold--;
    } else {	// Release.
      gain = std::min(gain + max_gain * agc_period / release_time + 1, max_gain);
    }
    out[i] = __SSAT(inp[i] * gain, 16);
  }
}

/*
  FIR interpolator (64-tap, x8).
  <coeff>: int16_t[64] (q0.15), <state>: int16_t[32], <inp>: int16_t[size], <out>: int16_t[size * 8]
*/
static inline void interpolator(const int16_t *coeff, int16_t *state, int size, const int16_t *inp, int16_t *out) {
#if USE_UNOPTIMIZED_CODE
  for (int i = 0; i < size; i++) {
    for (int j = 0; j < 8; j++) {
      for (int k = 63; k >= 1; k--) state[k] = state[k - 1];
      state[0] = (j == 0) ? inp[i] : 0;
      int32_t acc = 0;	// q16.15.
      for (int k = 0; k < 64; k++) acc += coeff[k] * state[k];
      out[i * 8 + j] = __SSAT((8 * acc) >> 15, 16);
    }
  }
#else
  int16_t stt[8];
  for (int i = 0; i < 8; i++) stt[i] = state[i * 8];
  for (int i = 0; i < size / 8; i++) {
    for (int j = 0; j < 8; j++) {
      stt[j] = *inp++;
      for (int k = 0; k < 8; k++) {
        int32_t acc = 0;	// q16.15.
        for (int l = 0; l < 8; l++) acc += coeff[l * 8 + k] * stt[(j - l + 8) % 8];
        *out++ = __SSAT((8 * acc) >> 15, 16);
      }
    }
  }
  for (int i = 0; i < 8; i++) state[i * 8] = stt[i];
#endif
}

#endif
