#ifndef FOC_MATH_H
#define FOC_MATH_H

#define ONE_OVER_SQRT3 0.57735026919f
#define TWO_OVER_SQRT3 1.15470053838f
#define SQRT3_OVER_2 0.8660254037844386f
#define PI 3.1415926f
#define HALF_PI 1.5707963f
#define TWO_PI (2.0f * PI)

#define LUT_SIZE 1024      // Number of samples in the table
#define LUT_STEP (TWO_PI / (float)LUT_SIZE) // Step size in radians
#define LUT_INV_STEP (1.0f / LUT_STEP) // Inverse step size for faster indexing

extern const float hall_to_rad[8];

extern float sin_lut[LUT_SIZE];
extern float cos_lut[LUT_SIZE];

typedef struct {
    float kp;        // Proportional gain
    float ki;        // Integral gain
    float integral;  // Integral state
    float out_min;   // Output lower limit
    float out_max;   // Output upper limit
} PI_Controller;

extern PI_Controller iq_ctrl;
extern PI_Controller id_ctrl;

float pi_run(PI_Controller *pi, float err, float dt);
void clarke_park(float ia, float ib, float sin_theta, float cos_theta, float *id, float *iq);
void inverse_park(float vd, float vq, float sin_theta, float cos_theta, float *valpha, float *vbeta);
void initluts();
float fast_sin(float theta);
float fast_cos(float theta);

#endif // FOC_MATH_H