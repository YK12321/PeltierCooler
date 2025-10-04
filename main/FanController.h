/**
 * @file FanController.h
 * @brief PWM-based fan speed controller
 */

#ifndef FAN_CONTROLLER_H
#define FAN_CONTROLLER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

/**
 * @class FanController
 * @brief Controls fan speed using PWM (LEDC peripheral)
 */
class FanController {
public:
    /**
     * @brief Construct a new fan controller
     * @param pin GPIO pin for PWM output
     * @param channel LEDC channel to use
     * @param timerNum LEDC timer to use
     * @param name Human-readable name for logging
     */
    FanController(gpio_num_t pin, 
                  ledc_channel_t channel, 
                  ledc_timer_t timerNum,
                  const char* name);

    /**
     * @brief Initialize PWM hardware for this fan
     * @param frequency PWM frequency in Hz
     * @param resolution PWM resolution in bits
     * @return ESP_OK on success
     */
    esp_err_t begin(uint32_t frequency, ledc_timer_bit_t resolution);

    /**
     * @brief Set fan speed
     * @param dutyCycle Duty cycle from 0.0 (off) to 1.0 (full speed)
     */
    void setSpeed(float dutyCycle);

    /**
     * @brief Get current fan speed
     * @return Current duty cycle (0.0 - 1.0)
     */
    float getSpeed() const;

    /**
     * @brief Get fan name
     * @return Name string
     */
    const char* getName() const;

private:
    gpio_num_t m_pin;
    ledc_channel_t m_channel;
    ledc_timer_t m_timer;
    const char* m_name;
    float m_currentSpeed;
    uint32_t m_maxDuty;
};

#endif // FAN_CONTROLLER_H
