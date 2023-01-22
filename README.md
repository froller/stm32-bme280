# stm32-bme280

STM32 HAL-based library for Bosch BME280 combined temperature,
atmospheric pressure and relative humidity sensor.

## Synopsys

```C
#include "stm32f1xx_hal.h"
#include "../../BME280/Inc/BME280.h"

I2C_HandleTypeDef hi2c1;
BME280_HandleTypeDef hbme280;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_I2C1_Init();

  hbme280.I2C_Handle = &hi2c1;
  hbme280.Sensor = BME280;
  BME280_Init(&hbme280);

  long temp;
  unsigned long pressure, humidity;
  BME280_ReadSensor(&hbme280, BME280_PRECISION_STANDARD, &temp, &pressure, &humidity);
}
```

## See Also

- https://www.bosch-sensortec.com/products/environmental-sensors/humidity-sensors-bme280/
