/*
  Software Defined Radio for JJY using Blue Pill.
  2021-09-12  T. Nakagawa
*/

#include "DSP.h"
#include "Filter.h"
#include "Peripheral.h"

constexpr int PIN_SW = PB2;							// Input pin for switching the radio stations.
constexpr int CIC_DCM_FACTOR = 32;						// CIC decimation factor.
constexpr int FIR_DCM_FACTOR = 8;						// FIR decimation factor.
constexpr int ITP_FACTOR = 8;							// Interpolation factor.
constexpr int ADC_SAMPLES = 2048;						// ADC buffer size.
constexpr int CIC_SAMPLES = ADC_SAMPLES / CIC_DCM_FACTOR;			// Number of samples after CIC decimation.
constexpr int FIR_SAMPLES = CIC_SAMPLES / FIR_DCM_FACTOR;			// Number of samples after FIR decimation.
constexpr int PWM_SAMPLES = FIR_SAMPLES * ITP_FACTOR;				// Number of samples after interpolation.
constexpr int ADC_FREQ = 1000000;						// ADC sampling frequency.
constexpr int TUN_FREQ0 = 40000;						// Tuning frequency 0.
constexpr int TUN_FREQ1 = 60000;						// Tuning frequency 1.
constexpr int CWT_FREQ0 = 938;							// CW tone frequency 0.
constexpr int CWT_FREQ1 = 918;							// CW tone frequency 1.
constexpr int AGC_PERIOD = 1000000 * ADC_SAMPLES / ADC_FREQ / FIR_SAMPLES;	// Time period for an AGC sample [us].
constexpr int AGC_ATK_TIME = 10000;						// AGC attack time [us].
constexpr int AGC_HLD_TIME = 100000;						// AGC hold time [us].
constexpr int AGC_REL_TIME = 500000;						// AGC release time [us].
constexpr int AGC_MAX_GAIN = 1000;						// Maximum AGC gain.
constexpr int AGC_OUT_LEVEL = 20000;						// Output signal level for AGC.

static int16_t adc_buf[2 * ADC_SAMPLES];					// ADC buffer (double buffered).
static int16_t nco_tbl[2 * ADC_SAMPLES];					// NCO table (I/Q).
static int16_t wrk_buf[2 * CIC_SAMPLES];					// Work buffer (I/Q).
static int16_t pwm_buf[2 * PWM_SAMPLES];					// PWM buffer (double buffered).
static uint8_t com_buf[FIR_SAMPLES * 4];					// USART buffer.
static bool freq_sw;

static struct {
  uint32_t time_dcr;
  uint32_t time_cic;
  uint32_t time_fir;
  uint32_t time_com;
  uint32_t time_hlb;
  uint32_t time_bpf;
  uint32_t time_agc;
  uint32_t time_itp;
} stat;

const int16_t *iir_coeff;

// Signal processing handler which is called by ADC DMA interuption.
void dsp_handler() {
  static int16_t *adc_buf_wr = adc_buf, *adc_buf_rd = adc_buf + ADC_SAMPLES;
  static int16_t *pwm_buf_rd = pwm_buf, *pwm_buf_wr = pwm_buf + PWM_SAMPLES;
  std::swap(adc_buf_rd, adc_buf_wr);
  std::swap(pwm_buf_rd, pwm_buf_wr);
  uint32_t time_end = micros(), time_bgn = time_end;

  // DC offset removal.
  static uint32_t adc_ofst = (2048 << 16);	// offset * 2^16
  offset(adc_ofst, ADC_SAMPLES, adc_buf_rd, adc_buf_rd);
  time_end = micros(); stat.time_dcr = time_end - time_bgn; time_bgn = time_end;

  // Mixer and CIC decimator (1/32).
  static int32_t cic_state[2 * 6];
  decimator_cic(nco_tbl              , cic_state    , ADC_SAMPLES, adc_buf_rd, wrk_buf              );
  decimator_cic(nco_tbl + ADC_SAMPLES, cic_state + 6, ADC_SAMPLES, adc_buf_rd, wrk_buf + CIC_SAMPLES);
  time_end = micros(); stat.time_cic = time_end - time_bgn; time_bgn = time_end;

  // FIR decimator (1/8).
  static int16_t fir_state[2 * 32];
  decimator_fir(fir_coeff, fir_state     , CIC_SAMPLES, wrk_buf              , wrk_buf              );
  decimator_fir(fir_coeff, fir_state + 32, CIC_SAMPLES, wrk_buf + CIC_SAMPLES, wrk_buf + CIC_SAMPLES);
  time_end = micros(); stat.time_fir = time_end - time_bgn; time_bgn = time_end;

  // Serial output with the IQ12 format for Spectrum Lab.
  uint8_t *com_ptr = com_buf;
  for (int i = 0; i < FIR_SAMPLES; i++) {
    const uint32_t sig0 = (wrk_buf[i              ] + 32768) >> 4;
    const uint32_t sig1 = (wrk_buf[i + CIC_SAMPLES] + 32768) >> 4;
    *com_ptr++ = 0xff;
    *com_ptr++ = sig0 & 0xff;
    *com_ptr++ = sig1 & 0xff;
    *com_ptr++ = ((sig0 & 0xf00) >> 4) | (sig1 >> 8);
  }
  sendSerial(com_buf, FIR_SAMPLES * 4);
  time_end = micros(); stat.time_com = time_end - time_bgn; time_bgn = time_end;

  // Hilbert transform.
  static int16_t hlb_state[65];
  hilbert(hlb_coeff, hlb_state, FIR_SAMPLES, wrk_buf, wrk_buf + CIC_SAMPLES, wrk_buf);
  time_end = micros(); stat.time_hlb = time_end - time_bgn; time_bgn = time_end;

  // Band pass filter.
  static int16_t bpf_state[12];
  bpf_iir(iir_coeff, bpf_state, FIR_SAMPLES, wrk_buf, wrk_buf);
  time_end = micros(); stat.time_bpf = time_end - time_bgn; time_bgn = time_end;

  // Automatic gain control.
  static int agc_config[6] = {AGC_PERIOD, AGC_ATK_TIME, AGC_HLD_TIME, AGC_REL_TIME, AGC_MAX_GAIN, AGC_OUT_LEVEL};
  static int agc_state[2];
  agc(agc_config, agc_state, FIR_SAMPLES, wrk_buf, wrk_buf);
  time_end = micros(); stat.time_agc = time_end - time_bgn; time_bgn = time_end;

  // FIR interpolator (x8).
  static int16_t itp_state[64];
  interpolator(itp_coeff, itp_state, FIR_SAMPLES, wrk_buf, pwm_buf_wr);
  time_end = micros(); stat.time_itp = time_end - time_bgn; time_bgn = time_end;

  // Covnert the PWM signals.
  for (int i = 0; i < PWM_SAMPLES; i++) {
    pwm_buf_wr[i] = (pwm_buf_wr[i] >> 8) + 128;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_SW, INPUT);

  Serial.begin(115200);
  Serial.println("JJY-SDR");
  Serial.println("Clock=" + String(F_CPU) + "MHz.");

  // Make the NCO table.
  freq_sw = (digitalRead(PIN_SW) == HIGH);
  int freq;
  if (freq_sw) {
    freq = TUN_FREQ1 - CWT_FREQ1;
    iir_coeff = iir_coeff1;
  } else {
    freq = TUN_FREQ0 - CWT_FREQ0;
    iir_coeff = iir_coeff0;
  }
  const int nco_freq = nco(ADC_FREQ, freq, ADC_SAMPLES, nco_tbl, nco_tbl + ADC_SAMPLES);
  Serial.println("NCO=" + String(nco_freq) + "Hz.");

  initPeripherals();
  startPeripherals(2 * ADC_SAMPLES, adc_buf, dsp_handler, 2 * PWM_SAMPLES, pwm_buf);
}

void loop() {
  if (freq_sw) {
    digitalWrite(LED_BUILTIN, millis() & (1 << 9));	// Blink the LED at 1000/512Hz.
  } else {
    digitalWrite(LED_BUILTIN, millis() & (1 << 10));	// Blink the LED at 1000/1024Hz.
  }

  // Print the latency information.
  static uint32_t timer = 5000;
  if (millis() >= timer) {
    Serial.println("");
    Serial.print("DCR: "); Serial.println(stat.time_dcr);
    Serial.print("CIC: "); Serial.println(stat.time_cic);
    Serial.print("FIR: "); Serial.println(stat.time_fir);
    Serial.print("COM: "); Serial.println(stat.time_com);
    Serial.print("HLB: "); Serial.println(stat.time_hlb);
    Serial.print("BPF: "); Serial.println(stat.time_bpf);
    Serial.print("AGC: "); Serial.println(stat.time_agc);
    Serial.print("ITP: "); Serial.println(stat.time_itp);
    timer += 5000;
  }
}
