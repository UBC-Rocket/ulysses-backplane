#ifndef BQ_I2C_H
#define BQ_I2C_H

#include <stdint.h>
#include "stm32g0xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal platform API used by the BQ76925 driver.
 * dev_addr is the HAL "8-bit address" form (7-bit << 1).
 */

HAL_StatusTypeDef bq_i2c_read_u8(I2C_HandleTypeDef *hi2c,
                                uint16_t dev_addr,
                                uint8_t reg,
                                uint8_t *val);

HAL_StatusTypeDef bq_i2c_write_u8(I2C_HandleTypeDef *hi2c,
                                 uint16_t dev_addr,
                                 uint8_t reg,
                                 uint8_t val);

HAL_StatusTypeDef bq_i2c_read(I2C_HandleTypeDef *hi2c,
                              uint16_t dev_addr,
                              uint8_t reg,
                              uint8_t *buf,
                              uint16_t len);

HAL_StatusTypeDef bq_i2c_write(I2C_HandleTypeDef *hi2c,
                               uint16_t dev_addr,
                               uint8_t reg,
                               const uint8_t *buf,
                               uint16_t len);

/* Optional helpers */
HAL_StatusTypeDef bq_i2c_is_ready(I2C_HandleTypeDef *hi2c,
                                 uint16_t dev_addr,
                                 uint32_t trials,
                                 uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* BQ_I2C_H */
