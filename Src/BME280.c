/*
 * BME280.c
 *
 *  Created on: 2023-01-20
 *      Author: Alexander Frolov
 */


#include "../../BME280/Inc/BME280.h"

#include <string.h>

/**
 * @brief Read BME280 data register(s) into buffer
 * @param hbmp: BME280 handle pointer
 * @param reg: Register statrting address
 * @param buffer: Pointer to buffer
 * @param size: Number of bytes to read sequentially
 * @return HAL status
 */
HAL_StatusTypeDef __BME280_ReadRegister(BME280_HandleTypeDef *hbmp, uint8_t reg, uint8_t *buffer, const size_t size) {
  HAL_StatusTypeDef result;
  if ((result = HAL_I2C_Master_Transmit(hbmp->I2C_Handle, hbmp->DeviceAddress, &reg, sizeof(reg), I2C_TIMEOUT)))
    return result;
  return HAL_I2C_Master_Receive(hbmp->I2C_Handle, hbmp->DeviceAddress + 1, buffer, size, I2C_TIMEOUT);
}

/**
 * @brief Writes BME280 data register(s) from buffer
 * @param hbmp: BME280 handle pointer
 * @param reg: Register starting address
 * @param buffer: Pointer to buffer
 * @param size: Number of bytes to write sequentially
 * @return HAL status
 */
HAL_StatusTypeDef __BME280_WriteRegister(BME280_HandleTypeDef *hbmp, uint8_t reg, uint8_t *buffer, const size_t size) {
  uint8_t tx_buffer[size + 1];
  tx_buffer[0] = reg;
  memcpy(tx_buffer + 1, buffer, size);
  return HAL_I2C_Master_Transmit(hbmp->I2C_Handle, hbmp->DeviceAddress, tx_buffer, size + 1, I2C_TIMEOUT);
}

/**
 * @brief Read chip id from BME280 sensor
 * @param hbmp: BME280 handle pointer
 * @param chipId: Pointer to variable that stores chip id
 * @return HAL status
 */
HAL_StatusTypeDef __BME280_ReadChipId(BME280_HandleTypeDef *hbmp, uint8_t *chipId) {
  return __BME280_ReadRegister(hbmp, 0xD0, chipId, sizeof(*chipId));
}

/**
 * @brief Read calibration data from BME280 sensor
 * This function reads calibration coefficients.
 * @param hbmp: BME280 handle pointer
 * @param sCalibration: Pointer to calibration data structure
 * @return HAL status
 */
HAL_StatusTypeDef __BME280_ReadCalibrationData(BME280_HandleTypeDef *hbmp, BME280_CalibrationDataTypeDef *sCalibration) {
  HAL_StatusTypeDef result;
  // T1 through P9
  if ((result = __BME280_ReadRegister(hbmp, 0x88, (uint8_t *)sCalibration, 24)))
    return result;
  // H1
  if ((result = __BME280_ReadRegister(hbmp, 0xA1, (uint8_t *)sCalibration + 24 , 1)))
    return result;
  // H2, H3
  if ((result = __BME280_ReadRegister(hbmp, 0xE1, (uint8_t *)sCalibration + 25 , 3)))
    return result;
  // H4, H5
  uint8_t buffer[3];
  if ((result = __BME280_ReadRegister(hbmp, 0xE4, buffer , 3)))
    return result;
  sCalibration->dig_H4 = (buffer[0] << 4) | (buffer[1] & 0x0F);
  sCalibration->dig_H5 = (buffer[2] << 4) | (buffer[1] >> 4);
  // H6
  if ((result = __BME280_ReadRegister(hbmp, 0xE7, (uint8_t *)sCalibration + 33 , 1)))
    return result;
  return HAL_OK;
}

/**
 * @brief Read uncompensated values from BME280 sensor
 * This function reads uncompensated values from temperature, atmospheric pressure and relative humidity ADCs
 * @param hbmp: BME280 handle pointer
 * @param precision: required precision
 * @param adc_T: Pointer to variable that stores uncompensated temperature value
 * @param adc_P: Pointer to variable that stores uncompensated atmospheric pressure value
 * @param adc_H: Pointer to variable that stores uncompensated relative humidity value
 * @return HAL status
 */
HAL_StatusTypeDef __BME280_ReadSensor(BME280_HandleTypeDef *hbmp, BME280_PrecisionTypeDef precision, int32_t *adc_T, int32_t *adc_P, int16_t *adc_H) {
#pragma pack(push, 1)
  struct {
    BME280_HumidityControlRegisterTypeDef ctrl_hum;
    BME280_StatusRegisterTypeDef status;
    BME280_ControlRegisterTypeDef ctrl_meas;
    BME280_ConfigRegisterTypeDef config;
  } regs = { 0 };
#pragma pack(pop)

  HAL_StatusTypeDef result;
  do {
    if ((result = __BME280_ReadRegister(hbmp, 0xF3, (uint8_t *)&regs.status, sizeof(regs.status))))
      return result;
    else if (regs.status.im_update)
      HAL_Delay(2);
  } while (regs.status.im_update);

  regs.ctrl_hum.osrs_h = precision;
  regs.status.im_update = 0;
  regs.status.measuring = 0;
  regs.ctrl_meas.osrs_t = precision;
  regs.ctrl_meas.osrs_p = precision;
  regs.ctrl_meas.mode = BME280_MODE_SLEEP;
  regs.config.t_sb = BME280_STANDBY_1ms;
  regs.config.spi3w_en = BME280_SPI3WIRE_OFF;
  regs.config.filter = BME280_FILTER_OFF;

  if ((result = __BME280_WriteRegister(hbmp, 0xF2, (uint8_t *)&regs, sizeof(regs))))
    return result;

  regs.ctrl_meas.mode = BME280_MODE_FORCED;

  if ((result = __BME280_WriteRegister(hbmp, 0xF4, (uint8_t *)&regs.ctrl_meas, sizeof(regs.ctrl_meas))))
    return result;

  do {
    if ((result = __BME280_ReadRegister(hbmp, 0xF3, (uint8_t *)&regs.status, sizeof(regs.status))))
      return result;
    else if (regs.status.measuring)
      HAL_Delay(2);
  } while (regs.status.measuring);

  uint8_t buffer[8];
  if ((result = __BME280_ReadRegister(hbmp, 0xF7, buffer, sizeof(buffer))))
    return result;

  *adc_P = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] >> 4);
  *adc_T = (buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] >> 4);
  *adc_H = (buffer[6] << 8) | buffer[7];

  return HAL_OK;
}

/**
 * @brief Compensate temperature value
 * This function applies calibration coefficients to convert data from ADC to physical units.
 * Output value of "5123" represents 51.23 °C
 * @param hbmp: BME280 handle pointer
 * @param adc_T: Pointer to uncompensated temperature value from ADC
 * @return: Temperature in 0.01 °C
 */
long __BME280_CompensateTemp(BME280_HandleTypeDef *hbmp, int32_t adc_T) {
  int32_t var1, var2;
  var1 = ((((adc_T >> 3) - ((int32_t)hbmp->CalibrationData.dig_T1 << 1))) * ((int32_t)hbmp->CalibrationData.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1) * ((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1)) >> 12) * (int32_t)hbmp->CalibrationData.dig_T3) >> 14;
  return ((var1 + var2) * 5 + 128) >> 8;
}

/**
 * @brief Compensate atmospheric pressure value
 * This function applies calibration coefficients to convert data from ADC to physical units.
 * Output value of "96386" represents 96386 Pa = 963.86 hPa
 * @param hbmp: BME280 handle pointer
 * @param adc_T: Pointer to uncompensated temperature value from ADC
 * @param adc_P: Pointer to uncompensated atmospheric pressure value from ADC
 * @return: Atmospheric pressure value in Pa
 */
unsigned long __BME280_CompensatePressure(BME280_HandleTypeDef *hbmp, int32_t adc_T, int32_t adc_P) {
  int32_t var1, var2, t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)hbmp->CalibrationData.dig_T1 << 1))) * ((int32_t)hbmp->CalibrationData.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1) * ((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1)) >> 12) * (int32_t)hbmp->CalibrationData.dig_T3) >> 14;
  t_fine = var1 + var2;

  uint32_t p;
  var1 = ((int32_t)t_fine >> 1) - 64000L;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11 ) * (int32_t)hbmp->CalibrationData.dig_P6;
  var2 = var2 + ((var1 * (int32_t)hbmp->CalibrationData.dig_P5) << 1);
  var2 = (var2 >> 2) + ((int32_t)hbmp->CalibrationData.dig_P4 << 16);
  var1 = (((hbmp->CalibrationData.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13 )) >> 3) + (((int32_t)hbmp->CalibrationData.dig_P2 * var1) >> 1)) >> 18;
  var1 = ((32768L + var1) * (int32_t)hbmp->CalibrationData.dig_P1) >> 15;
  if (var1 == 0)
    return 0; // avoid exception caused by division by zero
  p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
    p = (p << 1) / (uint32_t)var1;
  else
    p = p / (uint32_t)var1 * 2;
  var1 = ((int32_t)hbmp->CalibrationData.dig_P9 * (int32_t)(((p>>3) * (p>>3)) >> 13)) >> 12;
  var2 = ((int32_t)(p >> 2) * (int32_t)hbmp->CalibrationData.dig_P8) >> 13;
  p = (uint32_t)((int32_t)p + ((var1 + var2 + hbmp->CalibrationData.dig_P7) >> 4));
  return p;
}

/**
 * @brief Compensate relative humidity value
 * This function applies calibration coefficients to convert data from ADC to physical units.
 * Output value of "45678" represents 45.678 %RH
 * @param hbmp: BME280 handle pointer
 * @param adc_T: Pointer to uncompensated temperature value from ADC
 * @param adc_H: Pointer to uncompensated relative humidity value from ADC
 * @return: Relative humidity value in 0.001%
 */
unsigned long __BME280_CompensateHumidity(BME280_HandleTypeDef *hbmp, int32_t adc_T, int16_t adc_H) {
  int32_t var1, var2, t_fine;
  var1 = ((((adc_T >> 3) - ((int32_t)hbmp->CalibrationData.dig_T1 << 1))) * ((int32_t)hbmp->CalibrationData.dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1) * ((adc_T >> 4) - (int32_t)hbmp->CalibrationData.dig_T1)) >> 12) * (int32_t)hbmp->CalibrationData.dig_T3) >> 14;
  t_fine = var1 + var2;

  int32_t x1;
  x1 = t_fine - 76800L;
  x1 = (
    ((((adc_H << 14) - ((int32_t)hbmp->CalibrationData.dig_H4 << 20) - ((int32_t)hbmp->CalibrationData.dig_H5 * x1)) + 16384L) >> 15)
    * (((((((x1 * (int32_t)hbmp->CalibrationData.dig_H6) >> 10) * (((x1 * (int32_t)hbmp->CalibrationData.dig_H3) >> 11) + 32768L)) >> 10)
    + 2097152L) * (int32_t)hbmp->CalibrationData.dig_H2 + 8192) >> 14)
  );

  x1 = x1 - (((((x1 >> 15) * (x1 >> 15)) >> 7) * (int32_t)hbmp->CalibrationData.dig_H1) >> 4);
  x1 = x1 < 0 ? 0 : x1;
  x1 = x1 > 419430400 ? 419430400 : x1;
  return (uint32_t)((x1 >> 12) * 1000 / 1024);
}

/**
* @brief BME280 Initialization
* This function configures the hardware resources used in this example
* @param hbmp: BME280 handle pointer
* @return HAL status
*/
HAL_StatusTypeDef BME280_Init(BME280_HandleTypeDef *hbmp) {
  switch (hbmp->Sensor) {
  case BMP280:
    hbmp->DeviceAddress = 0xEE;
    hbmp->ChipId = 0x58;
  case BME280:
    hbmp->DeviceAddress = 0xEC;
    hbmp->ChipId = 0x60;
    break;
  default:
    return HAL_ERROR;
  }
  uint8_t chipId;
  HAL_StatusTypeDef result;
  if ((result = __BME280_ReadChipId(hbmp, &chipId)))
    return result;
  if (hbmp->ChipId != chipId)
    return HAL_ERROR;
  return __BME280_ReadCalibrationData(hbmp, &hbmp->CalibrationData);
}

/**
* @brief Reset BME280
* This function resets BME280 to power on settings
* @param hbmp: BME280 handle pointer
* @return HAL status
*/
HAL_StatusTypeDef BME280_Reset(BME280_HandleTypeDef *hbmp) {
  uint8_t resetCommand = 0xB6;
  return __BME280_WriteRegister(hbmp, 0xE0, &resetCommand, sizeof(resetCommand));
}

/**
* @brief Read BME280 sensor
* This function resets BME280 to power on settings
* @param hbmp: BME280 handle pointer
* @param temp: Pointer to variable that stores temperature value in 0.01 °C
* @param pressure: Pointer to variable that stores atmospheric pressure in Pa = 0.01 hPa
* @param humidity: Pointer to variable that stores relative humidity in 0.001%
* @return HAL status
*/
HAL_StatusTypeDef BME280_ReadSensor(BME280_HandleTypeDef *hbmp, BME280_PrecisionTypeDef precision, long *temp, unsigned long *pressure, unsigned long *humidity) {
  int32_t adc_T;
  int32_t adc_P;
  int16_t adc_H;
  HAL_StatusTypeDef result;
  if ((result = __BME280_ReadSensor(hbmp, precision, &adc_T, &adc_P, &adc_H)))
    return result;
  *temp = __BME280_CompensateTemp(hbmp, adc_T);
  *pressure = __BME280_CompensatePressure(hbmp, adc_T, adc_P);
  *humidity = __BME280_CompensateHumidity(hbmp, adc_T, adc_H);
  return result;
}
