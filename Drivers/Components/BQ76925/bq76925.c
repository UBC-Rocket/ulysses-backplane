#include "bq76925.h"

/* ===== Internal helpers ===== */

static inline bq76925_status_t i2c_read_u8(bq76925_t *dev, uint8_t reg, uint8_t *val)
{
  if (!dev || !dev->hi2c || !val) return BQ76925_ERR_NULL;

  /* HAL expects 8-bit address (7-bit << 1). Keep dev->i2c_addr in that form. */
  HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(dev->hi2c,
                                         dev->i2c_addr,
                                         reg,
                                         I2C_MEMADD_SIZE_8BIT,
                                         val,
                                         1,
                                         HAL_MAX_DELAY);
  return (rc == HAL_OK) ? BQ76925_OK : BQ76925_ERR_I2C;
}

static inline bq76925_status_t i2c_write_u8(bq76925_t *dev, uint8_t reg, uint8_t val)
{
  if (!dev || !dev->hi2c) return BQ76925_ERR_NULL;

  HAL_StatusTypeDef rc = HAL_I2C_Mem_Write(dev->hi2c,
                                          dev->i2c_addr,
                                          reg,
                                          I2C_MEMADD_SIZE_8BIT,
                                          &val,
                                          1,
                                          HAL_MAX_DELAY);
  return (rc == HAL_OK) ? BQ76925_OK : BQ76925_ERR_I2C;
}

static inline uint8_t clamp_u3(uint8_t v) { return (uint8_t)(v & 0x7u); }

/* ===== Public API ===== */

bq76925_status_t BQ76925_ReadReg(bq76925_t *dev, uint8_t reg, uint8_t *val)
{
  return i2c_read_u8(dev, reg, val);
}

bq76925_status_t BQ76925_WriteReg(bq76925_t *dev, uint8_t reg, uint8_t val)
{
  return i2c_write_u8(dev, reg, val);
}

bq76925_status_t BQ76925_UpdateStatus(bq76925_t *dev)
{
  uint8_t s = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_STATUS, &s);
  if (rc != BQ76925_OK) return rc;

  dev->status_raw = s;
  dev->alert   = ((s & BQ76925_STATUS_ALERT_Msk) != 0u);
  dev->crc_err = ((s & BQ76925_STATUS_CRC_ERR_Msk) != 0u);
  dev->por     = ((s & BQ76925_STATUS_POR_Msk) != 0u);
  return BQ76925_OK;
}

bq76925_status_t BQ76925_ClearPor(bq76925_t *dev)
{
  if (!dev) return BQ76925_ERR_NULL;

  /* POR bit is R/W; clear by writing 0 into POR bit position (datasheet Table 5). */
  uint8_t s = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_STATUS, &s);
  if (rc != BQ76925_OK) return rc;

  s &= (uint8_t)~BQ76925_STATUS_POR_Msk;
  rc = i2c_write_u8(dev, BQ76925_REG_STATUS, s);
  if (rc != BQ76925_OK) return rc;

  return BQ76925_UpdateStatus(dev);
}

bq76925_status_t BQ76925_SetVcoutMode(bq76925_t *dev, bq76925_vcout_mode_t mode)
{
  if (!dev) return BQ76925_ERR_NULL;

  uint8_t cellctl = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_CELL_CTL, &cellctl);
  if (rc != BQ76925_OK) return rc;

  /* Keep D7..D6 = 0, preserve CELL_SEL unless changing mode away from VCn */
  cellctl &= (uint8_t)~BQ76925_CELLCTL_VCOUT_SEL_Msk;
  cellctl |= (uint8_t)((clamp_u3((uint8_t)mode) << BQ76925_CELLCTL_VCOUT_SEL_Pos) & BQ76925_CELLCTL_VCOUT_SEL_Msk);

  /* enforce D7..D6 = 0 */
  cellctl &= 0x3Fu;

  return i2c_write_u8(dev, BQ76925_REG_CELL_CTL, cellctl);
}

bq76925_status_t BQ76925_SelectCell(bq76925_t *dev, uint8_t cell_index)
{
  if (!dev) return BQ76925_ERR_NULL;
  if (cell_index >= BQ76925_MAX_CELLS) return BQ76925_ERR_PARAM;

  uint8_t cellctl = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_CELL_CTL, &cellctl);
  if (rc != BQ76925_OK) return rc;

  /* Set VCOUT_SEL = 01 (VCn), set CELL_SEL */
  cellctl &= (uint8_t)~(BQ76925_CELLCTL_VCOUT_SEL_Msk | BQ76925_CELLCTL_CELL_SEL_Msk);
  cellctl |= (uint8_t)((BQ76925_VCOUT_SEL_VCn << BQ76925_CELLCTL_VCOUT_SEL_Pos) & BQ76925_CELLCTL_VCOUT_SEL_Msk);
  cellctl |= (uint8_t)((clamp_u3(cell_index) << BQ76925_CELLCTL_CELL_SEL_Pos) & BQ76925_CELLCTL_CELL_SEL_Msk);

  /* enforce D7..D6 = 0 */
  cellctl &= 0x3Fu;

  rc = i2c_write_u8(dev, BQ76925_REG_CELL_CTL, cellctl);
  if (rc != BQ76925_OK) return rc;

  dev->current_cell_index = cell_index;

  /* Datasheet shows tVCOUT ~ 100 µs for VCn select -> VCOUT.
     HAL_Delay is ms granularity; your ADC acquisition pacing likely dominates anyway.
     If you want tighter timing, replace with a microsecond delay using DWT or a timer. */
  return BQ76925_OK;
}

bq76925_status_t BQ76925_SelectTempInternal(bq76925_t *dev)
{
  if (!dev) return BQ76925_ERR_NULL;

  uint8_t cellctl = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_CELL_CTL, &cellctl);
  if (rc != BQ76925_OK) return rc;

  cellctl &= (uint8_t)~(BQ76925_CELLCTL_VCOUT_SEL_Msk | BQ76925_CELLCTL_CELL_SEL_Msk);
  cellctl |= (uint8_t)((BQ76925_VCOUT_SEL_VCn << BQ76925_CELLCTL_VCOUT_SEL_Pos) & BQ76925_CELLCTL_VCOUT_SEL_Msk);
  cellctl |= (uint8_t)((BQ76925_CELLSEL_VTEMP_INT << BQ76925_CELLCTL_CELL_SEL_Pos) & BQ76925_CELLCTL_CELL_SEL_Msk);
  cellctl &= 0x3Fu;

  return i2c_write_u8(dev, BQ76925_REG_CELL_CTL, cellctl);
}

bq76925_status_t BQ76925_SelectHiZ(bq76925_t *dev)
{
  if (!dev) return BQ76925_ERR_NULL;

  uint8_t cellctl = 0;
  bq76925_status_t rc = i2c_read_u8(dev, BQ76925_REG_CELL_CTL, &cellctl);
  if (rc != BQ76925_OK) return rc;

  cellctl &= (uint8_t)~(BQ76925_CELLCTL_VCOUT_SEL_Msk | BQ76925_CELLCTL_CELL_SEL_Msk);
  cellctl |= (uint8_t)((BQ76925_VCOUT_SEL_VCn << BQ76925_CELLCTL_VCOUT_SEL_Pos) & BQ76925_CELLCTL_VCOUT_SEL_Msk);
  cellctl |= (uint8_t)((BQ76925_CELLSEL_HIZ << BQ76925_CELLCTL_CELL_SEL_Pos) & BQ76925_CELLCTL_CELL_SEL_Msk);
  cellctl &= 0x3Fu;

  return i2c_write_u8(dev, BQ76925_REG_CELL_CTL, cellctl);
}

bq76925_status_t BQ76925_SetBalanceMask(bq76925_t *dev, uint8_t bal_mask)
{
  if (!dev) return BQ76925_ERR_NULL;

  /* Only bits 0..5 are valid */
  uint8_t v = (uint8_t)(bal_mask & 0x3Fu);
  return i2c_write_u8(dev, BQ76925_REG_BAL_CTL, v);
}

bq76925_status_t BQ76925_ClearBalancing(bq76925_t *dev)
{
  if (!dev) return BQ76925_ERR_NULL;
  return i2c_write_u8(dev, BQ76925_REG_BAL_CTL, 0x00u);
}

bq76925_status_t BQ76925_SetPowerCtl(bq76925_t *dev,
                                     bool ref_en,
                                     bool vtb_en,
                                     bool vc_amp_en,
                                     bool i_amp_en,
                                     bool i_comp_en)
{
  if (!dev) return BQ76925_ERR_NULL;

  uint8_t p = 0;
  if (ref_en)    p |= (uint8_t)BQ76925_PWR_REF_EN_Msk;
  if (vtb_en)    p |= (uint8_t)BQ76925_PWR_VTB_EN_Msk;
  if (vc_amp_en) p |= (uint8_t)BQ76925_PWR_VC_AMP_EN_Msk;
  if (i_amp_en)  p |= (uint8_t)BQ76925_PWR_I_AMP_EN_Msk;
  if (i_comp_en) p |= (uint8_t)BQ76925_PWR_I_COMP_EN_Msk;

  /* keep sleep bits off by default; you can add explicit sleep control later */
  if (dev->cfg.disable_sleep) p |= (uint8_t)BQ76925_PWR_SLEEP_DIS_Msk;

  return i2c_write_u8(dev, BQ76925_REG_POWER_CTL, p);
}

bq76925_status_t BQ76925_Init(bq76925_t *dev)
{
  if (!dev || !dev->hi2c) return BQ76925_ERR_NULL;

  /* Defaults that make sense for bring-up on most packs */
  if (dev->cfg.ref_sel != BQ76925_REF_1V5_GAIN_0P3 &&
      dev->cfg.ref_sel != BQ76925_REF_3V0_GAIN_0P6) {
    dev->cfg.ref_sel = BQ76925_REF_3V0_GAIN_0P6; /* common for 3.0V ref, gain 0.6 */
  }

  /* 1) CONFIG_2: set REF_SEL and optional CRC_EN */
  uint8_t cfg2 = 0;
  if (dev->cfg.enable_crc_write) cfg2 |= (uint8_t)BQ76925_CFG2_CRC_EN_Msk;
  if (dev->cfg.ref_sel == BQ76925_REF_3V0_GAIN_0P6) cfg2 |= (uint8_t)BQ76925_CFG2_REF_SEL_Msk;

  bq76925_status_t rc = i2c_write_u8(dev, BQ76925_REG_CONFIG_2, cfg2);
  if (rc != BQ76925_OK) return rc;

  /* 2) POWER_CTL: enable blocks */
  /* If user didn’t set these, choose safe defaults */
  bool ref_en    = dev->cfg.enable_ref    ? true : true;  /* typically ON */
  bool vtb_en    = dev->cfg.enable_vtb    ? true : false; /* only if you use thermistor bias */
  bool vc_amp_en = dev->cfg.enable_vc_amp ? true : true;  /* needed for VCOUT cell measurements */
  bool i_amp_en  = dev->cfg.enable_i_amp  ? true : false; /* only if using VIOUT current amp */
  bool i_comp_en = dev->cfg.enable_i_comp ? true : false; /* only if using ALERT comparator */

  rc = BQ76925_SetPowerCtl(dev, ref_en, vtb_en, vc_amp_en, i_amp_en, i_comp_en);
  if (rc != BQ76925_OK) return rc;

  /* 3) Put VCOUT into VCn mode and select VC1 initially */
  rc = BQ76925_SelectCell(dev, 0u);
  if (rc != BQ76925_OK) return rc;

  /* 4) Read status and clear POR */
  rc = BQ76925_UpdateStatus(dev);
  if (rc != BQ76925_OK) return rc;

  if (dev->por) {
    (void)BQ76925_ClearPor(dev);
  }

  return BQ76925_OK;
}

void BQ76925_SetUvThresholdAdc(bq76925_t *dev, uint16_t threshold_adc)
{
  if (!dev) return;
  dev->uv_threshold_adc = threshold_adc;
}

void BQ76925_UndervoltageCheck(bq76925_t *dev)
{
  if (!dev) return;

  /* Since VCOUT is analog and you’re sampling via ADC, a simple UV check can be done in ADC codes.
     This is intentionally conservative and simple: any cell below threshold sets uv_flag. */
  bool uv = false;
  for (uint8_t i = 0; i < BQ76925_MAX_CELLS; i++) {
    if (dev->cell_adc_raw[i] != 0u && dev->cell_adc_raw[i] < dev->uv_threshold_adc) {
      uv = true;
      break;
    }
  }
  dev->uv_flag = uv;
}
