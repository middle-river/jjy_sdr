#!/usr/bin/python3
# Calculate coefficients of filters for JJY-SDR.
# 2021-09-12  T. Nakagawa

import scipy.signal

def float_to_q15(x):
  return int(min(max(x * 32768.0, -32768), 32767))


# Decimation FIR filter.

T = 32	# Number of taps.
L = 8	# Decimation factor.

fir = scipy.signal.firwin(T, 1.0 / L)

print('// Coefficients for the decimation FIR filter (tap=%d, factor=%d, q0.15).' % (T, L))
print('const int16_t fir_coeff[%d] = {' % T)
for i in range(0, T, 8):
  print('  %s,' % ', '.join(['%d' % float_to_q15(v) for v in fir[i:i + 8]]))
print('};')
print('')


# Hilbert FIR filter.

T = 31	# Number of taps.

hlb = scipy.signal.remez(T, [0.03, 0.47], [1.0], type='hilbert')

print('// Coefficients for the Hilbert FIR filter (tap=%d, q0.15).' % T)
print('const int16_t hlb_coeff[%d] = {' % T)
for i in range(0, T, 8):
  print('  %s,' % ', '.join(['%d' % float_to_q15(v) for v in hlb[i:i + 8]]))
print('};')
print('')


# Band-pass IIR filter 0.

O = 1		# Order of the filter.
C = 937.5	# Tone frequency.
D = 50.0	# Filter width.
S = 3906.25	# Sampling rate.

iir0 = scipy.signal.iirfilter(O, [C - D / 2, C + D / 2], fs=S, btype='bandpass')

print('// Coefficients for the band-pass IIR filter 0 (order=%d, fc=%g, bw=%g, fs=%g, q1.14).' % (O, C, D, S))
print('const int16_t iir_coeff0[%d] = {' % (2 * (2 * O + 1)))
print('  %s,' % ', '.join(['%d' % float_to_q15(v / 2.0) for v in iir0[0]]) +
      ' %s,' % ', '.join(['%d' % float_to_q15(v / 2.0) for v in iir0[1]]))
print('};')
print('')


# Band-pass IIR filter 1.

O = 1		# Order of the filter.
C = 918.0	# Tone frequency.
D = 50.0	# Filter width.
S = 3906.25	# Sampling rate.

iir1 = scipy.signal.iirfilter(O, [C - D / 2, C + D / 2], fs=S, btype='bandpass')

print('// Coefficients for the band-pass IIR filter 1 (order=%d, fc=%g, bw=%g, fs=%g, q1.14).' % (O, C, D, S))
print('const int16_t iir_coeff1[%d] = {' % (2 * (2 * O + 1)))
print('  %s,' % ', '.join(['%d' % float_to_q15(v / 2.0) for v in iir1[0]]) +
      ' %s,' % ', '.join(['%d' % float_to_q15(v / 2.0) for v in iir1[1]]))
print('};')
print('')


# Interpolation FIR filter.

T = 64	# Number of taps.
L = 8	# Interpolation factor.

itp = scipy.signal.firwin(T, 1.0 / L)

print('// Coefficients for the interpolation FIR filter (tap=%d, factor=%d, q0.15).' % (T, L))
print('const int16_t itp_coeff[%d] = {' % T)
for i in range(0, T, 8):
  print('  %s,' % ', '.join(['%d' % float_to_q15(v) for v in itp[i:i + 8]]))
print('};')
print('')
