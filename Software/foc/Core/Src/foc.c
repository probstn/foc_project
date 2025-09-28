#include "foc.h"
#include "tim.h"
#include "adc.h"
#include <math.h>
#include <stdio.h>
#include <arm_math.h>

#define Ts 0.0001f

void foc_init()
{
  initluts();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, htim1.Init.Period*3/4); // 50% duty cycle

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, htim1.Init.Period / 2); // 50% duty cycle

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, htim1.Init.Period / 2); // 50% duty cycle

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, htim1.Init.Period); // tim4 to trigger adc at middle of pwm on-time
  HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  HAL_TIM_Base_Start(&htim3);
}

inline void foc_update(float vbat, float ia, float ib, float ic, float theta)
{
  // === 1. Clarke-Park transform ===
  float sin_t = fast_sin(theta);
  float cos_t = fast_cos(theta);
  // DWT->CYCCNT = 0; // reset cycle counter
  // float sin_t = sin_lut[(int)(theta * LUT_INV_STEP) & (LUT_SIZE - 1)];
  // uint32_t time = DWT->CYCCNT;
  // printf("LUT: %lu cycles\n", time);
  // float cos_t = cos_lut[(int)(theta * LUT_INV_STEP) & (LUT_SIZE - 1)];
  float id, iq;
  
  clarke_park(ia, ib, sin_t, cos_t, &id, &iq);
  
  
  // === 2. Current references ===
  float id_ref = 0.0f; // No field weakening for now
  float iq_ref = 1.0f; // Torque-producing current [A] (tune per application)
  
  // === 3. PI current controllers (with anti-windup) ===
  float vd_norm = pi_run(&id_ctrl, id_ref - id, Ts);
  float vq_norm = pi_run(&iq_ctrl, iq_ref - iq, Ts);
  
  // === 4. Scale normalized outputs to real voltages ===
  // Each controller outputs [-1..1], scale to 0.5 * Vdc
  float vd = vd_norm * (vbat * ONE_OVER_SQRT3); // 1/sqrt(3) = max linear modulation
  float vq = vq_norm * (vbat * ONE_OVER_SQRT3); // 1/sqrt(3) = max linear modulation
  
  // === 5. Inverse Park transform ===
  float valpha, vbeta;
  inverse_park(vd, vq, sin_t, cos_t, &valpha, &vbeta);
  
  // === 6. Apply Space Vector PWM ===
  svpwm_q31(valpha, vbeta, vbat);
}

/**
 * @brief Generates open-loop voltage commands for a three-phase motor
 *        and applies Space Vector PWM (SVPWM) based on a fixed
 *        electrical speed.
 *
 * This function simulates an open-loop control of a motor by:
 * 1. Incrementing the rotor electrical angle based on a fixed
 *    electrical speed (omega).
 * 2. Generating alpha-beta voltage components as a rotating
 *    voltage vector (v_alpha, v_beta) scaled by v_ref and supply
 *    voltage.
 * 3. Passing the voltages to an SVPWM function to generate
 *    PWM signals for the inverter.
 *
 * @param vbat Supply voltage (V) of the motor inverter.
 */
inline void openloop_q31(q31_t vbat_q31)
{
    // --- Constants ---
    static q31_t theta_q31 = 0; 

    // omega = 200 Hz electrical * Ts (50 µs) / (2π)
    // Precomputed in float: 200*0.00005/(2*M_PI) ≈ 0.00159
    // Convert to Q31 once:
    static const q31_t omega_q31 = (q31_t)(0.00159f * 2147483648.0f);

    // Reference utilization (modulation index)
    static const q31_t v_ref_q31 = (q31_t)(0.03f * 2147483648.0f);

    // --- 1. Update angle (wraps automatically in Q31) ---
    theta_q31 += omega_q31;

    // --- 2. Generate sin/cos in Q31 ---
    q31_t sin_q31, cos_q31;
    arm_sin_cos_q31(theta_q31, &sin_q31, &cos_q31);

    // --- 3. Compute αβ voltages ---
    q31_t v_alpha, v_beta, tmp;

    // v_alpha = v_ref * vbat * cos(theta)
    arm_mult_q31(&v_ref_q31, &vbat_q31, &tmp, 1);     // tmp = v_ref * vbat
    arm_mult_q31(&tmp, &cos_q31, &v_alpha, 1);

    // v_beta = v_ref * vbat * sin(theta)
    arm_mult_q31(&tmp, &sin_q31, &v_beta, 1);

    // --- 4. SVPWM (expecting Q31 inputs now) ---
    svpwm_q31(v_alpha, v_beta, vbat_q31);
}

// Single-call SVPWM: writes 3-phase duties to TIM1 CH1..CH3
// v_alpha, v_beta in volts (clarke frame); vbus is DC link voltage in volts.
inline void svpwm_q31(q31_t v_alpha, q31_t v_beta, q31_t vbus)
{
  // LOGGER
  buffer.alpha = v_alpha;
  buffer.beta = v_beta;

  // Guard: avoid divide-by-zero and nonsense input
  if (vbus <= 1e-6f)
    return;

  // 1) αβ -> three phase (phase-to-neutral) references
  //    Va + Vb + Vc = 0 by construction.
  float Va = v_alpha;
  float Vb = (-0.5f * v_alpha) + (SQRT3_OVER_2 * v_beta);
  float Vc = (-0.5f * v_alpha) - (SQRT3_OVER_2 * v_beta);

  // 2) SVPWM common-mode (zero-sequence) injection:
  //    shift all three by the average of their max and min
  //    => maximizes linear modulation range and yields 7-segment centered pattern
  float Vmax = Va;
  if (Vb > Vmax)
    Vmax = Vb;
  if (Vc > Vmax)
    Vmax = Vc;
  float Vmin = Va;
  if (Vb < Vmin)
    Vmin = Vb;
  if (Vc < Vmin)
    Vmin = Vc;
  float Voffset = 0.5f * (Vmax + Vmin);

  // 3) Convert to duty ratios in [0..1]
  //    After offset: each (Vx - Voffset) is within [-Vbus/2 .. +Vbus/2] in linear region.
  //    Duty = 0.5 + (phase_ref / Vbus)
  float invVbus = 1.0f / vbus;
  float da = 0.5f + (Va - Voffset) * invVbus;
  float db = 0.5f + (Vb - Voffset) * invVbus;
  float dc = 0.5f + (Vc - Voffset) * invVbus;

  // 4) Clamp (handles numerical noise / slight overmodulation)
  if (da < 0.0f)
    da = 0.0f;
  else if (da > 1.0f)
    da = 1.0f;
  if (db < 0.0f)
    db = 0.0f;
  else if (db > 1.0f)
    db = 1.0f;
  if (dc < 0.0f)
    dc = 0.0f;
  else if (dc > 1.0f)
    dc = 1.0f;
  // 5) Write CCRs (works for edge- or center-aligned; your 50% example uses Period/2)
  uint32_t arr = htim1.Init.Period;
  uint32_t ccr1 = buffer.va = (uint32_t)(da * (float)arr + 0.5f);
  uint32_t ccr2 = buffer.vb = (uint32_t)(db * (float)arr + 0.5f);
  uint32_t ccr3 = buffer.vc = (uint32_t)(dc * (float)arr + 0.5f);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1); // Phase A
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2); // Phase B
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3); // Phase C
}
