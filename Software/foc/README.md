# Field Oriented Control (FOC) on STM32F103

This project demonstrates how to build a simple **Field Oriented Control (FOC)** system for BLDC/PMSM motors on the STM32F103 from scratch.
It covers PWM generation using Space Vector PWM (SVPWM), synchronized current measurement, and ADC triggering with STM32 timers.

---

## Chapter 1 – PWM Generation and Space Vector Modulation

### Center-Aligned PWM

FOC requires three-phase sinusoidal voltages that are modulated onto the DC bus. To achieve this efficiently, we use **center-aligned PWM mode** on TIM1:

* Counter counts **up** and then **down** within the period (`ARR`).
* PWM edges are symmetric, reducing harmonic distortion.
* TIM1 (advanced control timer) supports complementary outputs and deadtime insertion, which are needed for driving a 3-phase inverter.

Each channel (CH1, CH2, CH3) drives one motor phase (plus its complementary “N” output with deadtime).

### Space Vector PWM (SVPWM)

Instead of generating three sinewaves directly, SVPWM computes duty cycles based on the desired voltage vector `(v_alpha, v_beta)` in the stationary αβ-plane:

1. Transform `(v_alpha, v_beta)` → three phase voltages `(Va, Vb, Vc)`.
2. Add a common-mode offset so that `(Va, Vb, Vc)` fit into the DC bus range.
3. Normalize to `[0..1]` duty ratios `(da, db, dc)`.
4. Convert to CCR registers of TIM1:

   ```c
   CCR1 = da * ARR;
   CCR2 = db * ARR;
   CCR3 = dc * ARR;
   ```

This yields **symmetrical, maximally linear modulation** with better bus utilization than sinusoidal PWM.

See [`foc.c`](./foc.c) for implementation details.

---

## Chapter 2 – Current Measurement and Synchronized ADC Sampling

### Why Current Measurement?

FOC requires the **stator currents** (`ia, ib, ic`) to be measured every control cycle. These are transformed (Clarke + Park) into rotating frame components (`id, iq`), which represent flux and torque.
Accurate current sampling is critical — noise or wrong timing leads to torque ripple and instability.

### Why Sample in the Middle of the High-Side PWM Pulse?

When MOSFETs switch, **switching transients and ringing** appear at the start and end of each PWM cycle.
The quietest time to measure current is in the **middle of the conduction window** (when high-side or low-side MOSFET is fully on).

On STM32F103, we align ADC sampling to the **middle of the high PWM time** to minimize noise.

### Using TIM1 CH4 as ADC Trigger

* TIM1 CH1..CH3 are used for PWM outputs.
* TIM1 **CH4** is configured as “PWM mode no output” — it does not drive a pin.
* Instead, its compare event (`OC4REF`) is routed as a **TRGO** (trigger output).
* This TRGO signal triggers the ADC injected conversion.

⚠️ On STM32F103 the TRGO is **fixed to rising edge** of OCxREF. Unlike newer STM32s, you cannot select falling-edge or both-edges.

This means:

* The ADC trigger fires **when CNT == CCR4 and OC4REF goes active**.
* Because it’s rising-edge only, if you want the ADC to sample **at the PWM midpoint**, you must program `CCR4` to be **just before the midpoint** of the timer period (`ARR/2 - 1`).
* This ensures the ADC sample happens as close as possible to the center of the high pulse.

Code (see [`tim.c`](./tim.c)):

```c
sConfigOC.Pulse = (ARR/2) - 1;   // one tick before midpoint
HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
```

Thus, every PWM cycle generates **one ADC trigger near the current-measurement sweet spot**.

### ADC Configuration

In [`adc.c`](./adc.c), ADC1 is configured to run **injected conversions** triggered by TIM1 TRGO:

```c
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
```

* Injected group contains the **phase current channels**: IN0 (IA), IN2 (IB), IN4 (IC).
* Each PWM cycle, when TIM1 CH4 generates TRGO, ADC1 performs all three injected conversions automatically.
* Results are available in the **JDR registers**, ready for Clarke/Park transform in the FOC loop.

This guarantees that currents are **always measured at the same point in the PWM cycle**, ensuring deterministic control.

---