/*
  Peripheral control library.
  2021-09-12  T. Nakagawa
*/

#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_

#include <cstdint>

void initPeripherals();
void startPeripherals(int adc_buf_size, int16_t *adc_buf, void (*adc_handler)(), int pwm_buf_size, int16_t *pwm_buf);
void sendSerial(uint8_t *buf, int size);

#endif
