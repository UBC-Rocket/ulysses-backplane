#include "bq_i2c.h"

HAL_StatusTypeDef bq_i2c_read_u8(I2C_HandleTypeDef *hi2c,
                                uint16_t dev_addr,
                                uint8_t reg,
                                uint8_t *val)
{
  if (hi2c == NULL || val == NULL) {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(hi2c,
                          dev_addr,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          val,
                          1,
                          HAL_MAX_DELAY);
}

HAL_StatusTypeDef bq_i2c_write_u8(I2C_HandleTypeDef *hi2c,
                                 uint16_t dev_addr,
                                 uint8_t reg,
                                 uint8_t val)
{
  if (hi2c == NULL) {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Write(hi2c,
                           dev_addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &val,
                           1,
                           HAL_MAX_DELAY);
}

HAL_StatusTypeDef bq_i2c_read(I2C_HandleTypeDef *hi2c,
                              uint16_t dev_addr,
                              uint8_t reg,
                              uint8_t *buf,
                              uint16_t len)
{
  if (hi2c == NULL || buf == NULL || len == 0) {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(hi2c,
                          dev_addr,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          len,
                          HAL_MAX_DELAY);
}

HAL_StatusTypeDef bq_i2c_write(I2C_HandleTypeDef *hi2c,
                               uint16_t dev_addr,
                               uint8_t reg,
                               const uint8_t *buf,
                               uint16_t len)
{
  if (hi2c == NULL || buf == NULL || len == 0) {
    return HAL_ERROR;
  }

  /* HAL expects non-const pointer */
  return HAL_I2C_Mem_Write(hi2c,
                           dev_addr,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           (uint8_t*)buf,
                           len,
                           HAL_MAX_DELAY);
}

HAL_StatusTypeDef bq_i2c_is_ready(I2C_HandleTypeDef *hi2c,
                                 uint16_t dev_addr,
                                 uint32_t trials,
                                 uint32_t timeout_ms)
{
  if (hi2c == NULL || trials == 0) {
    return HAL_ERROR;
  }

  return HAL_I2C_IsDeviceReady(hi2c,
                               dev_addr,
                               trials,
                               timeout_ms);
}
