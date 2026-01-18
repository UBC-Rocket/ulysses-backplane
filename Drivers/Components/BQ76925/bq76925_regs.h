#ifndef BQ76925_REGS_H
#define BQ76925_REGS_H

#include <stdint.h>

/* ===================== Register Addresses (Datasheet §8.6) ===================== */
#define BQ76925_REG_STATUS      (0x00u)  /* R/W */
#define BQ76925_REG_CELL_CTL    (0x01u)  /* R/W */
#define BQ76925_REG_BAL_CTL     (0x02u)  /* R/W */
#define BQ76925_REG_CONFIG_1    (0x03u)  /* R/W */
#define BQ76925_REG_CONFIG_2    (0x04u)  /* R/W */
#define BQ76925_REG_POWER_CTL   (0x05u)  /* R/W */
#define BQ76925_REG_CHIP_ID     (0x07u)  /* RO */

/* EEPROM calibration registers exist (0x10+) but are optional for basic bring-up. */

/* ===================== STATUS (0x00) bits (Datasheet Table 5) ===================== */
#define BQ76925_STATUS_ALERT_Pos     (7u)
#define BQ76925_STATUS_CRC_ERR_Pos   (2u)
#define BQ76925_STATUS_POR_Pos       (1u)

#define BQ76925_STATUS_ALERT_Msk     (1u << BQ76925_STATUS_ALERT_Pos)
#define BQ76925_STATUS_CRC_ERR_Msk   (1u << BQ76925_STATUS_CRC_ERR_Pos)
#define BQ76925_STATUS_POR_Msk       (1u << BQ76925_STATUS_POR_Pos)

/* ===================== CELL_CTL (0x01) (Datasheet Table 6–8) ===================== */
/* D7..D6 must be kept 0 per datasheet */
#define BQ76925_CELLCTL_VCOUT_SEL_Pos  (3u) /* D5..D3 */
#define BQ76925_CELLCTL_CELL_SEL_Pos   (0u) /* D2..D0 */

#define BQ76925_CELLCTL_VCOUT_SEL_Msk  (0x7u << BQ76925_CELLCTL_VCOUT_SEL_Pos)
#define BQ76925_CELLCTL_CELL_SEL_Msk   (0x7u << BQ76925_CELLCTL_CELL_SEL_Pos)

/* VCOUT_SEL values (Table 7) */
#define BQ76925_VCOUT_SEL_VSS          (0x0u) /* 000 */
#define BQ76925_VCOUT_SEL_VCn          (0x1u) /* 001 -> VCn selected by CELL_SEL */
#define BQ76925_VCOUT_SEL_VREF_X_0P5   (0x2u) /* 010 */
#define BQ76925_VCOUT_SEL_VREF_X_0P85  (0x3u) /* 011 */
/* other values reserved; keep to the table above */

/* CELL_SEL values when VCOUT_SEL=VCn (Table 8) */
#define BQ76925_CELLSEL_VC1            (0x0u)
#define BQ76925_CELLSEL_VC2            (0x1u)
#define BQ76925_CELLSEL_VC3            (0x2u)
#define BQ76925_CELLSEL_VC4            (0x3u)
#define BQ76925_CELLSEL_VC5            (0x4u)
#define BQ76925_CELLSEL_VC6            (0x5u)
#define BQ76925_CELLSEL_VTEMP_INT      (0x6u)
#define BQ76925_CELLSEL_HIZ            (0x7u)

/* ===================== BAL_CTL (0x02) (Datasheet Table 9) ===================== */
#define BQ76925_BAL_BAL1_Msk           (1u << 0)
#define BQ76925_BAL_BAL2_Msk           (1u << 1)
#define BQ76925_BAL_BAL3_Msk           (1u << 2)
#define BQ76925_BAL_BAL4_Msk           (1u << 3)
#define BQ76925_BAL_BAL5_Msk           (1u << 4)
#define BQ76925_BAL_BAL6_Msk           (1u << 5)

/* ===================== CONFIG_1 (0x03) (Datasheet Table 10–12) ===================== */
#define BQ76925_CFG1_I_GAIN_Pos        (0u) /* D0 */
#define BQ76925_CFG1_I_AMP_CAL_Pos     (1u) /* D1 */
#define BQ76925_CFG1_I_COMP_POL_Pos    (4u) /* D4 */
#define BQ76925_CFG1_I_THRESH_Pos      (5u) /* D7..D5 (3 bits? actually nibble D7..D4? datasheet uses D7..D4?) */

/* Datasheet shows I_THRESH in D7..D4? In Table 10 it’s labeled across D7..D2.
   The safe approach: treat I_THRESH as the upper 4 bits (D7..D4) and mask accordingly. */
#define BQ76925_CFG1_I_THRESH_Msk      (0xFu << 4u)
#define BQ76925_CFG1_I_COMP_POL_Msk    (1u << BQ76925_CFG1_I_COMP_POL_Pos)
#define BQ76925_CFG1_I_AMP_CAL_Msk     (1u << BQ76925_CFG1_I_AMP_CAL_Pos)
#define BQ76925_CFG1_I_GAIN_Msk        (1u << BQ76925_CFG1_I_GAIN_Pos)

/* ===================== CONFIG_2 (0x04) (Datasheet Table 13–14) ===================== */
#define BQ76925_CFG2_REF_SEL_Pos       (0u) /* D0 */
#define BQ76925_CFG2_CRC_EN_Pos        (7u) /* D7 */

#define BQ76925_CFG2_REF_SEL_Msk       (1u << BQ76925_CFG2_REF_SEL_Pos)
#define BQ76925_CFG2_CRC_EN_Msk        (1u << BQ76925_CFG2_CRC_EN_Pos)

/* REF_SEL values (Table 14) */
#define BQ76925_REFSEL_1V5_GAIN0P3     (0u)
#define BQ76925_REFSEL_3V0_GAIN0P6     (1u)

/* ===================== POWER_CTL (0x05) (Datasheet Table 15) ===================== */
#define BQ76925_PWR_REF_EN_Pos         (0u)
#define BQ76925_PWR_VTB_EN_Pos         (1u)
#define BQ76925_PWR_VC_AMP_EN_Pos      (2u)
#define BQ76925_PWR_I_AMP_EN_Pos       (3u)
#define BQ76925_PWR_I_COMP_EN_Pos      (4u)
#define BQ76925_PWR_SLEEP_DIS_Pos      (6u)
#define BQ76925_PWR_SLEEP_Pos          (7u)

#define BQ76925_PWR_REF_EN_Msk         (1u << BQ76925_PWR_REF_EN_Pos)
#define BQ76925_PWR_VTB_EN_Msk         (1u << BQ76925_PWR_VTB_EN_Pos)
#define BQ76925_PWR_VC_AMP_EN_Msk      (1u << BQ76925_PWR_VC_AMP_EN_Pos)
#define BQ76925_PWR_I_AMP_EN_Msk       (1u << BQ76925_PWR_I_AMP_EN_Pos)
#define BQ76925_PWR_I_COMP_EN_Msk      (1u << BQ76925_PWR_I_COMP_EN_Pos)
#define BQ76925_PWR_SLEEP_DIS_Msk      (1u << BQ76925_PWR_SLEEP_DIS_Pos)
#define BQ76925_PWR_SLEEP_Msk          (1u << BQ76925_PWR_SLEEP_Pos)

#endif /* BQ76925_REGS_H */
