#ifndef BQ76925_H
#define BQ76925_H

#include "bq76925_types.h"
#include "bq76925_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== Basic I2C register access ===================== */
bq76925_status_t BQ76925_ReadReg(bq76925_t *dev, uint8_t reg, uint8_t *val);
bq76925_status_t BQ76925_WriteReg(bq76925_t *dev, uint8_t reg, uint8_t val);

/* ===================== High-level control ===================== */
bq76925_status_t BQ76925_Init(bq76925_t *dev);
bq76925_status_t BQ76925_UpdateStatus(bq76925_t *dev);

/* VCOUT muxing (used with your ADC in main.c) */
bq76925_status_t BQ76925_SetVcoutMode(bq76925_t *dev, bq76925_vcout_mode_t mode);
bq76925_status_t BQ76925_SelectCell(bq76925_t *dev, uint8_t cell_index); /* 0..5 => VC1..VC6 */
bq76925_status_t BQ76925_SelectTempInternal(bq76925_t *dev);
bq76925_status_t BQ76925_SelectHiZ(bq76925_t *dev);

/* Balancing */
bq76925_status_t BQ76925_SetBalanceMask(bq76925_t *dev, uint8_t bal_mask /* bits 0..5 */);
bq76925_status_t BQ76925_ClearBalancing(bq76925_t *dev);

/* Power control bits */
bq76925_status_t BQ76925_SetPowerCtl(bq76925_t *dev,
                                     bool ref_en,
                                     bool vtb_en,
                                     bool vc_amp_en,
                                     bool i_amp_en,
                                     bool i_comp_en);

/* Convenience: clear POR flag */
bq76925_status_t BQ76925_ClearPor(bq76925_t *dev);

/* Optional: simple undervoltage check on ADC codes (since VCOUT is analog) */
void BQ76925_SetUvThresholdAdc(bq76925_t *dev, uint16_t threshold_adc);
void BQ76925_UndervoltageCheck(bq76925_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* BQ76925_H */
