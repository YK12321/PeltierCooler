/**
 * @file TemperatureControlSystem.cpp
 * @brief Main control system orchestrator implementation
 */

#include "TemperatureControlSystem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include <math.h>

static const char* TAG = "TempControl";

// Hardware pin assignments
static constexpr gpio_num_t PIN_TEMP_SENSOR   = GPIO_NUM_4;
static constexpr gpio_num_t PIN_FAN_INTAKE    = GPIO_NUM_25;
static constexpr gpio_num_t PIN_FAN_EXHAUST   = GPIO_NUM_26;
static constexpr gpio_num_t PIN_SERVO_VALVE   = GPIO_NUM_18;

// PWM configuration for fans
static constexpr uint32_t FAN_PWM_FREQUENCY = 25000;
static constexpr ledc_timer_bit_t FAN_PWM_RES = LEDC_TIMER_10_BIT;
static constexpr ledc_timer_t FAN_PWM_TIMER = LEDC_TIMER_0;

// PWM configuration for servo
static constexpr uint32_t SERVO_PWM_FREQUENCY = 50;
static constexpr ledc_timer_bit_t SERVO_PWM_RES = LEDC_TIMER_16_BIT;
static constexpr ledc_timer_t SERVO_PWM_TIMER = LEDC_TIMER_1;

// Servo calibration
static constexpr float SERVO_MIN_ANGLE = 15.0f;
static constexpr float SERVO_MAX_ANGLE = 120.0f;
static constexpr float SERVO_MIN_PULSE_US = 600.0f;
static constexpr float SERVO_MAX_PULSE_US = 2400.0f;

// PID gains
static constexpr float PID_KP = 0.9f;
static constexpr float PID_KI = 0.08f;
static constexpr float PID_KD = 0.1f;

TemperatureControlSystem::TemperatureControlSystem(float targetTemp, uint32_t controlPeriodMs, float errorMargin)
    : m_tempSensor(PIN_TEMP_SENSOR)
    , m_intakeFan(PIN_FAN_INTAKE, LEDC_CHANNEL_0, FAN_PWM_TIMER, "Intake Fan")
    , m_exhaustFan(PIN_FAN_EXHAUST, LEDC_CHANNEL_1, FAN_PWM_TIMER, "Exhaust Fan")
    , m_valveServo(PIN_SERVO_VALVE, LEDC_CHANNEL_2, SERVO_PWM_TIMER,
                   SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 
                   SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US)
    , m_pid(PID_KP, PID_KI, PID_KD)
    , m_targetTemp(targetTemp)
    , m_filteredTemp(targetTemp)
    , m_errorMargin(errorMargin)
    , m_controlPeriodMs(controlPeriodMs)
{
}

bool TemperatureControlSystem::begin()
{
    ESP_LOGI(TAG, "Initializing temperature control system");
    ESP_LOGI(TAG, "Target temperature: %.2f °C", m_targetTemp);
    
    // Configure fan PWM timer
    ledc_timer_config_t fan_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = FAN_PWM_RES,
        .timer_num = FAN_PWM_TIMER,
        .freq_hz = FAN_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    esp_err_t ret = ledc_timer_config(&fan_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure fan timer: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configure servo PWM timer
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_PWM_RES,
        .timer_num = SERVO_PWM_TIMER,
        .freq_hz = SERVO_PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    
    ret = ledc_timer_config(&servo_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure servo timer: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Initialize all components
    if (!m_tempSensor.begin()) {
        ESP_LOGW(TAG, "Temperature sensor not detected - will run in fail-safe mode");
    }
    
    if (m_intakeFan.begin(FAN_PWM_FREQUENCY, FAN_PWM_RES) != ESP_OK) {
        return false;
    }
    
    if (m_exhaustFan.begin(FAN_PWM_FREQUENCY, FAN_PWM_RES) != ESP_OK) {
        return false;
    }
    
    if (m_valveServo.begin(SERVO_PWM_FREQUENCY, SERVO_PWM_RES) != ESP_OK) {
        return false;
    }
    
    ESP_LOGI(TAG, "System initialized successfully");
    return true;
}

void TemperatureControlSystem::update()
{
    // Request temperature reading
    m_tempSensor.requestTemperature();
    
    // Wait for DS18B20 conversion (750ms for 12-bit resolution)
    vTaskDelay(pdMS_TO_TICKS(750));
    
    // Read temperature
    float rawTemp;
    if (!m_tempSensor.readTemperature(rawTemp) || isnan(rawTemp)) {
        ESP_LOGW(TAG, "Temperature read failed - activating fail-safe");
        activateFailSafe();
        return;
    }
    
    // Apply exponential moving average filter
    m_filteredTemp = applyExponentialFilter(rawTemp, m_filteredTemp);
    
    // Calculate error (positive when temperature is above target)
    float error = m_filteredTemp - m_targetTemp;
    
    // Three-zone control strategy based on error margin
    if (error > m_errorMargin) {
        // TOO HOT: Emergency cooling mode
        // Max fans + fully open valve
        ESP_LOGW(TAG, "TOO HOT: T=%.2f°C > target+margin (%.2f°C)", m_filteredTemp, m_targetTemp + m_errorMargin);
        m_intakeFan.setSpeed(FAN_MAX_DUTY);
        m_exhaustFan.setSpeed(FAN_MAX_DUTY);
        m_valveServo.setAngle(SERVO_MAX_ANGLE);
        m_pid.reset(); // Reset PID when leaving normal zone
        
    } else if (error < -m_errorMargin) {
        // TOO COLD: Reduce cooling
        // Minimum fans + close valve to reduce airflow
        ESP_LOGW(TAG, "TOO COLD: T=%.2f°C < target-margin (%.2f°C)", m_filteredTemp, m_targetTemp - m_errorMargin);
        m_intakeFan.setSpeed(FAN_BASE_DUTY);
        m_exhaustFan.setSpeed(FAN_BASE_DUTY);
        m_valveServo.setAngle(SERVO_MIN_ANGLE); // Close valve
        m_pid.reset(); // Reset PID when leaving normal zone
        
    } else {
        // WITHIN MARGIN: Normal PID control with fans only
        // Valve stays fully open, PID controls fan speed
        float dt = m_controlPeriodMs / 1000.0f;
        float controlOutput = m_pid.compute(error, dt);
        
        // Clamp and normalize control output to [0, 1]
        if (controlOutput < 0.0f) controlOutput = 0.0f;
        if (controlOutput > 1.5f) controlOutput = 1.5f;
        float normalized = controlOutput;
        if (normalized > 1.0f) normalized = 1.0f;
        
        // Map control output to fan speeds
        float fanSpeed = FAN_BASE_DUTY + normalized * (FAN_MAX_DUTY - FAN_BASE_DUTY);
        m_intakeFan.setSpeed(fanSpeed);
        m_exhaustFan.setSpeed(fanSpeed);
        m_valveServo.setAngle(SERVO_MAX_ANGLE); // Keep valve open
    }
    
    // Log status
    ESP_LOGI(TAG, "T=%.2f°C (raw %.2f°C), err=%.2f, margin=±%.2f, intake=%.0f%%, exhaust=%.0f%%, valve=%.1f°",
             m_filteredTemp,
             rawTemp,
             error,
             m_errorMargin,
             m_intakeFan.getSpeed() * 100.0f,
             m_exhaustFan.getSpeed() * 100.0f,
             m_valveServo.getAngle());
}

void TemperatureControlSystem::setTargetTemperature(float temp)
{
    m_targetTemp = temp;
    m_pid.reset();  // Reset integral when setpoint changes
    ESP_LOGI(TAG, "Target temperature updated to %.2f °C", temp);
}

float TemperatureControlSystem::getTargetTemperature() const
{
    return m_targetTemp;
}

float TemperatureControlSystem::getCurrentTemperature() const
{
    return m_filteredTemp;
}

void TemperatureControlSystem::setErrorMargin(float margin)
{
    m_errorMargin = margin;
    ESP_LOGI(TAG, "Error margin updated to ±%.2f °C", margin);
}

float TemperatureControlSystem::getErrorMargin() const
{
    return m_errorMargin;
}

void TemperatureControlSystem::activateFailSafe()
{
    ESP_LOGW(TAG, "FAIL-SAFE MODE ACTIVATED");
    m_intakeFan.setSpeed(FAN_MAX_DUTY);
    m_exhaustFan.setSpeed(FAN_MAX_DUTY);
    m_valveServo.setAngle(SERVO_MAX_ANGLE);
}

float TemperatureControlSystem::applyExponentialFilter(float newValue, float oldValue)
{
    return TEMP_FILTER_ALPHA * newValue + (1.0f - TEMP_FILTER_ALPHA) * oldValue;
}

void TemperatureControlSystem::applyControlOutput(float controlOutput)
{
    // NOTE: This method is now deprecated in favor of zone-based control in update()
    // Kept for backward compatibility if needed
    
    // Map control output to fan speeds (baseline + proportional increase)
    float intakeSpeed = FAN_BASE_DUTY + controlOutput * (FAN_MAX_DUTY - FAN_BASE_DUTY);
    float exhaustSpeed = FAN_BASE_DUTY + controlOutput * (FAN_MAX_DUTY - FAN_BASE_DUTY);
    
    m_intakeFan.setSpeed(intakeSpeed);
    m_exhaustFan.setSpeed(exhaustSpeed);
    
    // Map control output to servo angle
    float servoAngle = SERVO_MIN_ANGLE + controlOutput * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);
    m_valveServo.setAngle(servoAngle);
}
