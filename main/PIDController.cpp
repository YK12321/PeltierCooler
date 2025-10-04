/**
 * @file PIDController.cpp
 * @brief PID feedback controller implementation
 */

#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, 
                             float integralMin, 
                             float integralMax)
    : m_kp(kp)
    , m_ki(ki)
    , m_kd(kd)
    , m_integral(0.0f)
    , m_prevError(0.0f)
    , m_integralMin(integralMin)
    , m_integralMax(integralMax)
{
}

float PIDController::compute(float error, float dt)
{
    // Integral term with anti-windup
    m_integral += error * dt;
    if (m_integral < m_integralMin) m_integral = m_integralMin;
    if (m_integral > m_integralMax) m_integral = m_integralMax;
    
    // Derivative term
    float derivative = (error - m_prevError) / dt;
    m_prevError = error;
    
    // PID output
    return m_kp * error + m_ki * m_integral + m_kd * derivative;
}

void PIDController::reset()
{
    m_integral = 0.0f;
    m_prevError = 0.0f;
}

void PIDController::setGains(float kp, float ki, float kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

float PIDController::getIntegral() const
{
    return m_integral;
}
