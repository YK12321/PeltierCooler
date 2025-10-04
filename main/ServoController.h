/**
 * @file ServoController.h
 * @brief RC servo motor controller for valve positioning
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

/**
 * @class ServoController
 * @brief Controls servo position using PWM signals
 */
class ServoController {
public:
    /**
     * @brief Construct a new servo controller
     * @param pin GPIO pin for PWM output
     * @param channel LEDC channel to use
     * @param timerNum LEDC timer to use
     * @param minAngle Minimum servo angle (degrees)
     * @param maxAngle Maximum servo angle (degrees)
     * @param minPulseUs Minimum pulse width (microseconds)
     * @param maxPulseUs Maximum pulse width (microseconds)
     */
    ServoController(gpio_num_t pin,
                   ledc_channel_t channel,
                   ledc_timer_t timerNum,
                   float minAngle,
                   float maxAngle,
                   float minPulseUs,
                   float maxPulseUs);

    /**
     * @brief Initialize PWM hardware for servo
     * @param frequency PWM frequency (typically 50 Hz)
     * @param resolution PWM resolution in bits
     * @return ESP_OK on success
     */
    esp_err_t begin(uint32_t frequency, ledc_timer_bit_t resolution);

    /**
     * @brief Set servo angle
     * @param angle Target angle in degrees
     */
    void setAngle(float angle);

    /**
     * @brief Get current servo angle
     * @return Current angle in degrees
     */
    float getAngle() const;

private:
    gpio_num_t m_pin;
    ledc_channel_t m_channel;
    ledc_timer_t m_timer;
    float m_minAngle;
    float m_maxAngle;
    float m_minPulseUs;
    float m_maxPulseUs;
    float m_currentAngle;
    uint32_t m_maxDuty;
    uint32_t m_frequency;
};

#endif // SERVO_CONTROLLER_H
