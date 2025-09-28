#ifndef FOC_H
#define FOC_H

#include "foc_math.h"
#include <stdint.h>

#define HALL_TICK 1.38889e-7f // 72Mhz 10 prescaler = 1/7.2Mhz

void openloop(float vbat);
void svpwm_q31(float v_alpha, float v_beta, float vbat);
void foc_update(float vbat, float ia, float ib, float ic, float theta);

void foc_init();

typedef struct {
  float curr_a;
  float curr_b;
  float curr_c;
  float vbat;
  float theta;
  float alpha;
  float beta;
  uint32_t va;
  uint32_t vb;
  uint32_t vc;
  uint32_t adc1;
  uint32_t adc2;
  uint32_t adc3;
} Logger;

extern Logger buffer;
extern Logger cache;

void initluts();
float fast_sin(float theta);
float fast_cos(float theta);

#endif // FOC_H