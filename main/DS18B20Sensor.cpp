/**
 * @file DS18B20Sensor.cpp
 * @brief Temperature sensor implementation
 */

#include "DS18B20Sensor.h"
#include "ds18b20.h"
#include "esp_log.h"

static const char* TAG = "DS18B20Sensor";

DS18B20Sensor::DS18B20Sensor(gpio_num_t pin)
    : m_pin(pin)
    , m_deviceCount(0)
{
}

bool DS18B20Sensor::begin()
{
    ds18b20_init(m_pin);
    m_deviceCount = ds18b20_get_device_count(m_pin);
    
    ESP_LOGI(TAG, "Initialized on GPIO%d, found %d device(s)", m_pin, m_deviceCount);
    
    return m_deviceCount > 0;
}

int DS18B20Sensor::getDeviceCount() const
{
    return m_deviceCount;
}

void DS18B20Sensor::requestTemperature()
{
    ds18b20_request_temperatures(m_pin);
}

bool DS18B20Sensor::readTemperature(float& temperature)
{
    temperature = ds18b20_get_temp(m_pin);
    
    if (temperature == DS18B20_ERROR) {
        ESP_LOGW(TAG, "Failed to read temperature (sensor error)");
        return false;
    }
    
    return true;
}

bool DS18B20Sensor::isConnected() const
{
    return m_deviceCount > 0;
}
