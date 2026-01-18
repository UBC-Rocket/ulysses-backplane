#ifndef TMCS1108_H
#define TMCS1108_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * TMCS110x (TMCS1101 / TMCS1108 family) analog Hall current sensor helper
 *
 * Model (bidirectional):
 *   Vout = Voffset + (sensitivity_V_per_A) * I
 *
 * Typical Voffset is Vref/2 for bidirectional parts.
 * You should calibrate Voffset using TMCS1108_CalibrateZero().
 * ==========================================================================*/

/* Common sensitivity options from TMCS1101-Q1 datasheet family:
 * A1:  50 mV/A
 * A2: 100 mV/A
 * A3: 200 mV/A
 * A4: 400 mV/A
 *
 * If your TMCS1108 has a different sensitivity, use TMCS1108_SetSensitivity().
 */
typedef enum {
  TMCS1108_SENS_A1_50mV_PER_A  = 0,
  TMCS1108_SENS_A2_100mV_PER_A = 1,
  TMCS1108_SENS_A3_200mV_PER_A = 2,
  TMCS1108_SENS_A4_400mV_PER_A = 3,
  TMCS1108_SENS_CUSTOM         = 255
} tmcs1108_sens_t;

/* Simple first-order IIR filter:
 *   y = alpha*y + (1-alpha)*x
 * alpha in [0,1). Higher alpha = more smoothing.
 */
typedef struct {
  bool  enabled;
  float alpha;
  float y;
  bool  initialized;
} tmcs1108_iir_t;

typedef struct {
  /* ADC conversion parameters */
  float adc_vref;         /* ADC reference voltage (V), e.g., 3.3 */
  uint16_t adc_fullscale; /* e.g., 4095 for 12-bit */

  /* Sensor parameters */
  float voffset;          /* Vout at 0A (V). Calibrate this. */
  float sensitivity;      /* V/A */

  /* Derived / runtime */
  float current_A;        /* last computed current */
  float voltage_V;        /* last computed sensor voltage */

  /* Overcurrent detection */
  float oc_threshold_A;   /* absolute threshold in amps; <=0 disables */
  bool  oc_flag;

  /* Optional filter */
  tmcs1108_iir_t filter;
} TMCS1108;

/* ===== Configuration ===== */

/* Initialize struct with ADC params and a selected sensitivity option.
 * You still should call TMCS1108_CalibrateZero() at startup (no current flow).
 */
void TMCS1108_Init(TMCS1108 *s,
                   float adc_vref,
                   uint16_t adc_fullscale,
                   tmcs1108_sens_t sens_opt);

/* Set sensitivity directly (V/A). Example: 0.4f for 400mV/A */
void TMCS1108_SetSensitivity(TMCS1108 *s, float sensitivity_V_per_A);

/* Enable/disable IIR filter */
void TMCS1108_FilterEnable(TMCS1108 *s, bool enable, float alpha);

/* Set overcurrent threshold (absolute amps). Use <=0 to disable */
void TMCS1108_SetOvercurrentThreshold(TMCS1108 *s, float oc_threshold_A);

/* Calibrate zero-current offset using N ADC samples (averaged).
 * Pass in a callback that returns ADC raw code from your configured channel.
 */
typedef uint16_t (*tmcs1108_adc_read_fn_t)(void);
void TMCS1108_CalibrateZero(TMCS1108 *s,
                            tmcs1108_adc_read_fn_t adc_read,
                            uint16_t samples);

/* ===== Runtime update ===== */

/* Update from a single ADC code. Returns computed current (A). */
float TMCS1108_UpdateFromAdc(TMCS1108 *s, uint16_t adc_code);

/* Convenience getters */
static inline float TMCS1108_GetCurrentA(const TMCS1108 *s) { return s ? s->current_A : 0.0f; }
static inline bool  TMCS1108_GetOCFlag(const TMCS1108 *s)   { return s ? s->oc_flag : false; }

#ifdef __cplusplus
}
#endif

#endif /* TMCS1108_H */
