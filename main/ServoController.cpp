/**
 * @file ServoController.cpp
 * @brief RC servo motor controller implementation
 */

#include "ServoController.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "ServoController";

ServoController::ServoController(gpio_num_t pin,
                                 ledc_channel_t channel,
                                 ledc_timer_t timerNum,
                                 float minAngle,
                                 float maxAngle,
                                 float minPulseUs,
                                 float maxPulseUs)
    : m_pin(pin)
    , m_channel(channel)
    , m_timer(timerNum)
    , m_minAngle(minAngle)
    , m_maxAngle(maxAngle)
    , m_minPulseUs(minPulseUs)
    , m_maxPulseUs(maxPulseUs)
    , m_currentAngle(minAngle)
    , m_maxDuty(0)
    , m_frequency(0)
{
}

esp_err_t ServoController::begin(uint32_t frequency, ledc_timer_bit_t resolution)
{
    m_maxDuty = (1u << resolution) - 1u;
    m_frequency = frequency;
    
    ledc_channel_config_t channel_config = {
        .gpio_num = m_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = m_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = m_timer,
        .duty = 0,
        .hpoint = 0,
        .flags = {0},
    };
    
    esp_err_t ret = ledc_channel_config(&channel_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Servo initialized on GPIO%d", m_pin);
    } else {
        ESP_LOGE(TAG, "Servo failed to initialize: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

void ServoController::setAngle(float angle)
{
    // Clamp angle to valid range
    if (angle < m_minAngle) angle = m_minAngle;
    if (angle > m_maxAngle) angle = m_maxAngle;
    
    m_currentAngle = angle;
    
    // Map angle to pulse width
    const float angleSpan = m_maxAngle - m_minAngle;
    const float normalized = (angle - m_minAngle) / angleSpan;
    const float pulseUs = m_minPulseUs + normalized * (m_maxPulseUs - m_minPulseUs);
    
    // Convert pulse width to duty cycle
    const float dutyCycleFraction = (pulseUs / 1000000.0f) * m_frequency;
    uint32_t duty = static_cast<uint32_t>(roundf(dutyCycleFraction * m_maxDuty));
    
    if (duty > m_maxDuty) {
        duty = m_maxDuty;
    }
    
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Servo failed to set duty: %s", esp_err_to_name(err));
        return;
    }
    
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Servo failed to update duty: %s", esp_err_to_name(err));
    }
}

float ServoController::getAngle() const
{
    return m_currentAngle;
}
