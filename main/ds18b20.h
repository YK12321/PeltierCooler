#ifndef DS18B20_H
#define DS18B20_H

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DS18B20_ERROR -127.0f

/**
 * @brief Initialize DS18B20 sensor
 * @param pin GPIO pin number for one-wire bus
 */
void ds18b20_init(gpio_num_t pin);

/**
 * @brief Get the number of devices on the bus
 * @param pin GPIO pin number for one-wire bus
 * @return Number of devices found
 */
int ds18b20_get_device_count(gpio_num_t pin);

/**
 * @brief Request temperature conversion from all sensors
 * @param pin GPIO pin number for one-wire bus
 */
void ds18b20_request_temperatures(gpio_num_t pin);

/**
 * @brief Read temperature from sensor
 * @param pin GPIO pin number for one-wire bus
 * @return Temperature in Celsius or DS18B20_ERROR on error
 */
float ds18b20_get_temp(gpio_num_t pin);

#ifdef __cplusplus
}
#endif

#endif // DS18B20_H
