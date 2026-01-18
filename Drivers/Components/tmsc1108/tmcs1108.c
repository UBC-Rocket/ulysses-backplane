#include "tmcs1108.h"

/* Sensitivity lookup (V/A) for the common family options */
static float sens_lookup(tmcs1108_sens_t opt)
{
  switch (opt) {
    case TMCS1108_SENS_A1_50mV_PER_A:   return 0.050f;
    case TMCS1108_SENS_A2_100mV_PER_A:  return 0.100f;
    case TMCS1108_SENS_A3_200mV_PER_A:  return 0.200f;
    case TMCS1108_SENS_A4_400mV_PER_A:  return 0.400f;
    default:                            return 0.100f; /* safe default */
  }
}

void TMCS1108_Init(TMCS1108 *s,
                   float adc_vref,
                   uint16_t adc_fullscale,
                   tmcs1108_sens_t sens_opt)
{
  if (!s) return;

  s->adc_vref = adc_vref;
  s->adc_fullscale = adc_fullscale ? adc_fullscale : 4095u;

  s->sensitivity = sens_lookup(sens_opt);

  /* Start with a reasonable guess for bidirectional devices: Vref/2 */
  s->voffset = 0.5f * adc_vref;

  s->current_A = 0.0f;
  s->voltage_V = 0.0f;

  s->oc_threshold_A = -1.0f;
  s->oc_flag = false;

  s->filter.enabled = false;
  s->filter.alpha = 0.0f;
  s->filter.y = 0.0f;
  s->filter.initialized = false;
}

void TMCS1108_SetSensitivity(TMCS1108 *s, float sensitivity_V_per_A)
{
  if (!s) return;
  if (sensitivity_V_per_A > 0.0f) {
    s->sensitivity = sensitivity_V_per_A;
  }
}

void TMCS1108_FilterEnable(TMCS1108 *s, bool enable, float alpha)
{
  if (!s) return;

  s->filter.enabled = enable;

  /* clamp alpha into [0,1) */
  if (alpha < 0.0f) alpha = 0.0f;
  if (alpha >= 1.0f) alpha = 0.999f;

  s->filter.alpha = alpha;
  s->filter.initialized = false;
  s->filter.y = 0.0f;
}

void TMCS1108_SetOvercurrentThreshold(TMCS1108 *s, float oc_threshold_A)
{
  if (!s) return;
  s->oc_threshold_A = oc_threshold_A;
  s->oc_flag = false;
}

static float adc_to_volts(const TMCS1108 *s, uint16_t code)
{
  /* avoid div by zero */
  const float fs = (s && s->adc_fullscale) ? (float)s->adc_fullscale : 4095.0f;
  const float vref = (s) ? s->adc_vref : 3.3f;
  return (vref * (float)code) / fs;
}

void TMCS1108_CalibrateZero(TMCS1108 *s,
                            tmcs1108_adc_read_fn_t adc_read,
                            uint16_t samples)
{
  if (!s || !adc_read) return;
  if (samples == 0) samples = 1;

  uint32_t acc = 0;
  for (uint16_t i = 0; i < samples; i++) {
    acc += (uint32_t)adc_read();
  }

  uint16_t avg = (uint16_t)(acc / samples);
  s->voffset = adc_to_volts(s, avg);

  /* reset filter state so we don't blend old values */
  s->filter.initialized = false;
  s->filter.y = 0.0f;

  s->oc_flag = false;
}

float TMCS1108_UpdateFromAdc(TMCS1108 *s, uint16_t adc_code)
{
  if (!s) return 0.0f;

  /* Convert ADC code -> volts */
  float v = adc_to_volts(s, adc_code);

  /* Optional filter on voltage */
  if (s->filter.enabled) {
    if (!s->filter.initialized) {
      s->filter.y = v;
      s->filter.initialized = true;
    } else {
      s->filter.y = s->filter.alpha * s->filter.y + (1.0f - s->filter.alpha) * v;
    }
    v = s->filter.y;
  }

  s->voltage_V = v;

  /* Volts -> current (A) */
  if (s->sensitivity > 0.0f) {
    s->current_A = (v - s->voffset) / s->sensitivity;
  } else {
    s->current_A = 0.0f;
  }

  /* Overcurrent flagging (absolute magnitude) */
  s->oc_flag = false;
  if (s->oc_threshold_A > 0.0f) {
    float mag = (s->current_A >= 0.0f) ? s->current_A : -s->current_A;
    if (mag >= s->oc_threshold_A) {
      s->oc_flag = true;
    }
  }

  return s->current_A;
}
