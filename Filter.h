/*
  Coefficients for filters.
  2021-09-12  T. Nakagawa
*/

#ifndef FILTER_H_
#define FILTER_H_

// Coefficients for the decimation FIR filter (tap=32, factor=8, q0.15).
const int16_t fir_coeff[32] = {
  -10, -35, -75, -132, -197, -244, -231, -112,
  152, 582, 1166, 1861, 2589, 3256, 3768, 4045,
  4045, 3768, 3256, 2589, 1861, 1166, 582, 152,
  -112, -231, -244, -197, -132, -75, -35, -10,
};

// Coefficients for the Hilbert FIR filter (tap=31, q0.15).
const int16_t hlb_coeff[31] = {
  671, 0, 699, 0, 1069, 0, 1597, 0,
  2390, 0, 3735, 0, 6685, 0, 20769, 0,
  -20769, 0, -6685, 0, -3735, 0, -2390, 0,
  -1597, 0, -1069, 0, -699, 0, -671,
};

// Coefficients for the band-pass IIR filter 0 (order=1, fc=937.5, bw=50, fs=3906.25, q1.14).
const int16_t iir_coeff0[6] = {
  633, 0, -633, 16384, -1979, 15116,
};

// Coefficients for the band-pass IIR filter 1 (order=1, fc=918, bw=50, fs=3906.25, q1.14).
const int16_t iir_coeff1[6] = {
  633, 0, -633, 16384, -2965, 15116,
};

// Coefficients for the interpolation FIR filter (tap=64, factor=8, q0.15).
const int16_t itp_coeff[64] = {
  -5, -15, -26, -36, -43, -44, -36, -15,
  18, 65, 117, 165, 195, 195, 152, 62,
  -72, -239, -413, -563, -650, -637, -494, -202,
  238, 810, 1474, 2175, 2849, 3428, 3853, 4078,
  4078, 3853, 3428, 2849, 2175, 1474, 810, 238,
  -202, -494, -637, -650, -563, -413, -239, -72,
  62, 152, 195, 195, 165, 117, 65, 18,
  -15, -36, -44, -43, -36, -26, -15, -5,
};

#endif
