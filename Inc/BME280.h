/*
 * BME280.h
 *
 *  Created on: 2023-01-20
 *      Author: Alexander Frolov
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "stm32f1xx_hal.h"

#define I2C_TIMEOUT 1000U

typedef enum {
  BMP280 = 279,
  BME280 = 280
} BME280_SensorTypeDef;

typedef enum {
  BME280_PRECISION_NONE = 0,
  BME280_PRECISION_ULTRALOW,
  BME280_PRECISION_LOW,
  BME280_PRECISION_STANDARD,
  BME280_PRECISION_HIGH,
  BME280_PRECISION_ULTRAHIGH
} BME280_PrecisionTypeDef;

typedef enum {
  BME280_SPI3WIRE_OFF = 0,
  BME280_SPI3WIRE_ON
} BME280_SPI3WireTypeDef;

typedef enum {
  BME280_STANDBY_1ms = 0,
  BME280_STANDBY_10ms = 6,
  BME280_STANDBY_20ms = 7,
  BME280_STANDBY_63ms = 1,
  BME280_STANDBY_125ms = 2,
  BME280_STANDBY_250ms = 3,
  BME280_STANDBY_500ms = 4,
  BME280_STANDBY_1000ms = 5
} BME280_StandbyTypeDef;

typedef enum {
  BME280_FILTER_OFF = 0,
  BME280_FILTER_2,
  BME280_FILTER_4,
  BME280_FILTER_8,
  BME280_FILTER_16
} BME280_FilterTypeDef;

typedef enum {
  BME280_MODE_SLEEP = 0,
  BME280_MODE_FORCED = 1,
  BME280_MODE_NORMAL = 3
} BME280_ModeTypeDef;

#pragma pack(push, 1)

typedef struct {
  unsigned short dig_T1;
  short dig_T2;
  short dig_T3;
  unsigned short dig_P1;
  short dig_P2;
  short dig_P3;
  short dig_P4;
  short dig_P5;
  short dig_P6;
  short dig_P7;
  short dig_P8;
  short dig_P9;
  unsigned char dig_H1;
  short dig_H2;
  unsigned char dig_H3;
  short dig_H4;
  short dig_H5;
  char dig_H6;
} BME280_CalibrationDataTypeDef;

typedef struct {
  BME280_SPI3WireTypeDef spi3w_en:1;
  uint8_t unused0:1;
  BME280_FilterTypeDef filter:3;
  BME280_StandbyTypeDef t_sb:3;
} BME280_ConfigRegisterTypeDef;

typedef struct {
  BME280_ModeTypeDef mode:2;
  BME280_PrecisionTypeDef osrs_p:3;
  BME280_PrecisionTypeDef osrs_t:3;
} BME280_ControlRegisterTypeDef;

typedef struct {
  BME280_PrecisionTypeDef osrs_h:3;
} BME280_HumidityControlRegisterTypeDef;

typedef struct {
  unsigned char im_update:1;
  uint8_t unused0:2;
  unsigned char measuring:1;
} BME280_StatusRegisterTypeDef;

#pragma pack(pop)

typedef struct {
  I2C_HandleTypeDef *I2C_Handle;
  BME280_SensorTypeDef Sensor;
  uint8_t DeviceAddress;
  uint8_t ChipId;
  BME280_CalibrationDataTypeDef CalibrationData;
} BME280_HandleTypeDef;

HAL_StatusTypeDef __BME280_ReadRegister(BME280_HandleTypeDef *hbmp, uint8_t reg, uint8_t *buffer, const size_t size);
HAL_StatusTypeDef __BME280_WriteRegister(BME280_HandleTypeDef *hbmp, uint8_t reg, uint8_t *buffer, const size_t size);
HAL_StatusTypeDef __BME280_ReadChipId(BME280_HandleTypeDef *hbmp, uint8_t *chipId);
HAL_StatusTypeDef __BME280_ReadCalibrationData(BME280_HandleTypeDef *hbmp, BME280_CalibrationDataTypeDef *sCalibration);
HAL_StatusTypeDef __BME280_ReadSensor(BME280_HandleTypeDef *hbmp, BME280_PrecisionTypeDef precision, int32_t *adc_T, int32_t *adc_P, int16_t *adc_H);
long __BME280_CompensateTemp(BME280_HandleTypeDef *hbmp, int32_t adc_T);
unsigned long __BME280_CompensatePressure(BME280_HandleTypeDef *hbmp, int32_t adc_T, int32_t adc_P);
unsigned long __BME280_CompensateHumidity(BME280_HandleTypeDef *hbmp, int32_t adc_T, int16_t adc_H);

HAL_StatusTypeDef BME280_Init(BME280_HandleTypeDef *hbmp);
HAL_StatusTypeDef BME280_Reset(BME280_HandleTypeDef *hbmp);
HAL_StatusTypeDef BME280_ConversionComplete(BME280_HandleTypeDef *hbmp);
HAL_StatusTypeDef BME280_ReadSensor(BME280_HandleTypeDef *hbmp, BME280_PrecisionTypeDef precision, long *temp, unsigned long *pressure, unsigned long *humidity);


#endif /* INC_BME280_H_ */
