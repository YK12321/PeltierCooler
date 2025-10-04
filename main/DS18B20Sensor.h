/**
 * @file DS18B20Sensor.h
 * @brief Temperature sensor interface using DS18B20 one-wire device
 */

#ifndef DS18B20_SENSOR_H
#define DS18B20_SENSOR_H

#include "driver/gpio.h"

/**
 * @class DS18B20Sensor
 * @brief Encapsulates DS18B20 temperature sensor operations
 */
class DS18B20Sensor {
public:
    /**
     * @brief Construct a new DS18B20 sensor
     * @param pin GPIO pin for one-wire bus
     */
    explicit DS18B20Sensor(gpio_num_t pin);

    /**
     * @brief Initialize the sensor hardware
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Get the number of devices detected on the bus
     * @return Device count
     */
    int getDeviceCount() const;

    /**
     * @brief Request temperature conversion (non-blocking)
     */
    void requestTemperature();

    /**
     * @brief Read the most recent temperature measurement
     * @param temperature Output parameter for temperature in Celsius
     * @return true if reading is valid
     */
    bool readTemperature(float& temperature);

    /**
     * @brief Check if sensor is connected and functioning
     * @return true if sensor is operational
     */
    bool isConnected() const;

private:
    gpio_num_t m_pin;
    int m_deviceCount;
};

#endif // DS18B20_SENSOR_H
