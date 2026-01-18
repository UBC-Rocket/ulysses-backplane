#ifndef BQ76925_TYPES_H
#define BQ76925_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#ifndef BQ76925_MAX_CELLS
#define BQ76925_MAX_CELLS 6u
#endif

typedef enum {
  BQ76925_OK = 0,
  BQ76925_ERR_NULL = -1,
  BQ76925_ERR_I2C  = -2,
  BQ76925_ERR_PARAM = -3
} bq76925_status_t;

typedef enum {
  BQ76925_VCOUT_MODE_VSS = 0,
  BQ76925_VCOUT_MODE_VCELL = 1,     /* VCn selected by cell index */
  BQ76925_VCOUT_MODE_VREF_X_0P5 = 2,
  BQ76925_VCOUT_MODE_VREF_X_0P85 = 3
} bq76925_vcout_mode_t;

typedef enum {
  BQ76925_REF_1V5_GAIN_0P3 = 0,
  BQ76925_REF_3V0_GAIN_0P6 = 1
} bq76925_refsel_t;

typedef struct {
  bq76925_refsel_t ref_sel;     /* affects VREF + VCOUT gain + VIOUT range */
  bool enable_crc_write;        /* CRC_EN bit (optional for host writes) */
  bool enable_ref;              /* REF_EN */
  bool enable_vtb;              /* VTB_EN */
  bool enable_vc_amp;           /* VC_AMP_EN */
  bool enable_i_amp;            /* I_AMP_EN */
  bool enable_i_comp;           /* I_COMP_EN */
  bool disable_sleep;           /* SLEEP_DIS */
} bq76925_config_t;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t i2c_addr;            /* HAL expects 8-bit address (7-bit << 1) */
  bq76925_config_t cfg;

  /* Latched / decoded status */
  uint8_t status_raw;
  bool alert;
  bool crc_err;
  bool por;

  /* State for VCOUT muxing in an ADC-driven system */
  uint8_t current_cell_index;   /* 0..5 for VC1..VC6 */

  /* Optional: you can fill these from your ADC in main.c */
  uint16_t cell_adc_raw[BQ76925_MAX_CELLS];

  /* Optional UV flag if you want the driver to track it */
  bool uv_flag;
  uint16_t uv_threshold_adc;    /* threshold in ADC codes (since VCOUT is analog) */
} bq76925_t;

#endif /* BQ76925_TYPES_H */
