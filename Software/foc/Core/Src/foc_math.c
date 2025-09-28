#include "foc_math.h"
#include <math.h>
#include <gpio.h>

const float hall_to_rad[8] = {
    -1.0f,
    0.0f,
    2.094395f, // 120° in rad
    1.047198f, // 60° in rad
    4.188790f, // 240° in rad
    5.235988f, // 300° in rad
    3.141593f, // 180° in rad
    -1.0f
};

PI_Controller id_ctrl = {.kp = 2.0f, .ki = 200.0f, .integral = 0.0f, .out_min = -1.0f, .out_max = 1.0f};
PI_Controller iq_ctrl = {.kp = 2.0f, .ki = 200.0f, .integral = 0.0f, .out_min = -1.0f, .out_max = 1.0f};

inline float pi_run(PI_Controller *pi, float err, float dt)
{
  // Proportional part
  float P = pi->kp * err;

  // Predict integral contribution
  float I = pi->integral + pi->ki * err * dt;

  // Unclamped output
  float out = P + I;

  // Anti-windup: clamp output
  if (out > pi->out_max)
  {
    out = pi->out_max;
    // Only integrate if error would push output *back* into range
    if (err < 0)
      pi->integral = I;
  }
  else if (out < pi->out_min)
  {
    out = pi->out_min;
    if (err > 0)
      pi->integral = I;
  }
  else
  {
    // Within limits → accept integral update
    pi->integral = I;
  }

  return out;
}

inline void clarke_park(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq)
{
  // Clarke transform
  float i_alpha = ia;
  float i_beta = ONE_OVER_SQRT3 * ia + TWO_OVER_SQRT3 * ib;

  // Park transform
  *id = i_alpha * cos_theta + i_beta * sin_theta;
  *iq = i_beta * cos_theta - i_alpha * sin_theta;
}

inline void inverse_park(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta)
{
  *valpha = vd * cos_theta - vq * sin_theta;
  *vbeta = vd * sin_theta + vq * cos_theta;
}

float sin_lut[LUT_SIZE];
float cos_lut[LUT_SIZE];

void initluts()
{
  for (int i = 0; i < LUT_SIZE; i++)
  {
    float angle = i * LUT_STEP;
    sin_lut[i] = sinf(angle);
    cos_lut[i] = cosf(angle);
  }
}

/*
  Fast sine and cosine using lookup tables with linear interpolation.
  Input angle in radians, output in [-1, 1].
*/
inline float fast_sin(float theta)
{
  // Normalize theta to [0, 2*PI)
  while (theta < 0.0f)
  theta += TWO_PI;
  while (theta >= TWO_PI)
  theta -= TWO_PI;
  
  
  // Calculate index in LUT
  int index = (int)(theta * LUT_INV_STEP);
  if (index < 0)
  index += LUT_SIZE; // Handle negative angles
  if (index >= LUT_SIZE)
  index -= LUT_SIZE; // Wrap around
  
  // Return sine value from LUT
  return sin_lut[index];
}

/*
  Fast sine and cosine using lookup tables with linear interpolation.
  Input angle in radians, output in [-1, 1].
*/
inline float fast_cos(float theta)
{
  // Normalize theta to [0, 2*PI)
  while (theta < 0.0f)
    theta += TWO_PI;
  while (theta >= TWO_PI)
    theta -= TWO_PI;

  // Calculate index in LUT
  int index = (int)(theta * LUT_INV_STEP);
  if (index < 0)
    index += LUT_SIZE; // Handle negative angles
  if (index >= LUT_SIZE)
    index -= LUT_SIZE; // Wrap around

  // Return cosine value from LUT
  return cos_lut[index];
}
