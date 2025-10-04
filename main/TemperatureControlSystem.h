/**
 * @file TemperatureControlSystem.h
 * @brief Main control system orchestrator for Peltier cooling
 */

#ifndef TEMPERATURE_CONTROL_SYSTEM_H
#define TEMPERATURE_CONTROL_SYSTEM_H

#include "DS18B20Sensor.h"
#include "FanController.h"
#include "ServoController.h"
#include "PIDController.h"

/**
 * @class TemperatureControlSystem
 * @brief Orchestrates all components for closed-loop temperature control
 */
class TemperatureControlSystem {
public:
    /**
     * @brief Construct the control system
     * @param targetTemp Target temperature in Celsius
     * @param controlPeriodMs Control loop period in milliseconds
     */
    TemperatureControlSystem(float targetTemp, uint32_t controlPeriodMs);

    /**
     * @brief Initialize all hardware components
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Execute one control loop iteration
     */
    void update();

    /**
     * @brief Set target temperature
     * @param temp New target temperature in Celsius
     */
    void setTargetTemperature(float temp);

    /**
     * @brief Get target temperature
     * @return Target temperature in Celsius
     */
    float getTargetTemperature() const;

    /**
     * @brief Get current filtered temperature
     * @return Filtered temperature in Celsius
     */
    float getCurrentTemperature() const;

    /**
     * @brief Enter fail-safe mode (max cooling)
     */
    void activateFailSafe();

private:
    // Hardware components
    DS18B20Sensor m_tempSensor;
    FanController m_intakeFan;
    FanController m_exhaustFan;
    ServoController m_valveServo;
    
    // Control algorithm
    PIDController m_pid;
    
    // Control parameters
    float m_targetTemp;
    float m_filteredTemp;
    uint32_t m_controlPeriodMs;
    
    // Constants
    static constexpr float TEMP_FILTER_ALPHA = 0.25f;
    static constexpr float FAN_BASE_DUTY = 0.2f;
    static constexpr float FAN_MAX_DUTY = 1.0f;
    
    // Helper methods
    float applyExponentialFilter(float newValue, float oldValue);
    void applyControlOutput(float controlOutput);
};

#endif // TEMPERATURE_CONTROL_SYSTEM_H
