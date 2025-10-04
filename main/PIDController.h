/**
 * @file PIDController.h
 * @brief PID feedback controller implementation
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

/**
 * @class PIDController
 * @brief Proportional-Integral-Derivative controller for closed-loop control
 */
class PIDController {
public:
    /**
     * @brief Construct a new PID controller
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     * @param integralMin Minimum integral windup limit
     * @param integralMax Maximum integral windup limit
     */
    PIDController(float kp, float ki, float kd, 
                  float integralMin = -50.0f, 
                  float integralMax = 50.0f);

    /**
     * @brief Compute control output for given error
     * @param error Current error (setpoint - measured)
     * @param dt Time step in seconds
     * @return Control output value
     */
    float compute(float error, float dt);

    /**
     * @brief Reset integral and derivative terms
     */
    void reset();

    /**
     * @brief Set PID gains
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief Get current integral term value
     * @return Integral accumulator
     */
    float getIntegral() const;

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_integral;
    float m_prevError;
    float m_integralMin;
    float m_integralMax;
};

#endif // PID_CONTROLLER_H
