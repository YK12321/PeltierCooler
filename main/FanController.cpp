/**
 * @file FanController.cpp
 * @brief PWM-based fan speed controller implementation
 */

#include "FanController.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "FanController";

FanController::FanController(gpio_num_t pin, 
                             ledc_channel_t channel, 
                             ledc_timer_t timerNum,
                             const char* name)
    : m_pin(pin)
    , m_channel(channel)
    , m_timer(timerNum)
    , m_name(name)
    , m_currentSpeed(0.0f)
    , m_maxDuty(0)
{
}

esp_err_t FanController::begin(uint32_t frequency, ledc_timer_bit_t resolution)
{
    m_maxDuty = (1u << resolution) - 1u;
    
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
        ESP_LOGI(TAG, "%s initialized on GPIO%d", m_name, m_pin);
    } else {
        ESP_LOGE(TAG, "%s failed to initialize: %s", m_name, esp_err_to_name(ret));
    }
    
    return ret;
}

void FanController::setSpeed(float dutyCycle)
{
    // Clamp duty cycle
    if (dutyCycle < 0.0f) dutyCycle = 0.0f;
    if (dutyCycle > 1.0f) dutyCycle = 1.0f;
    
    m_currentSpeed = dutyCycle;
    
    uint32_t duty = static_cast<uint32_t>(roundf(dutyCycle * m_maxDuty));
    if (duty > m_maxDuty) {
        duty = m_maxDuty;
    }
    
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, m_channel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to set duty: %s", m_name, esp_err_to_name(err));
        return;
    }
    
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, m_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s failed to update duty: %s", m_name, esp_err_to_name(err));
    }
}

float FanController::getSpeed() const
{
    return m_currentSpeed;
}

const char* FanController::getName() const
{
    return m_name;
}
