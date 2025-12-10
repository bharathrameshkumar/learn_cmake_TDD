#ifndef PID_H
#define PID_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID Controller structure
 * 
 * Used for motor current control, speed control, and position control
 */
typedef struct {
    // PID gains
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    
    // Output limits
    float outputMax;    // Maximum output
    float outputMin;    // Minimum output
    
    // Internal state
    float integral;     // Accumulated integral
    float prevError;    // Previous error for derivative
    
    // Anti-windup
    float integralMax;  // Maximum integral value
    float integralMin;  // Minimum integral value
} PID_Controller_t;

/**
 * @brief Initialize PID controller
 * 
 * @param pid Pointer to PID structure
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param outMin Minimum output limit
 * @param outMax Maximum output limit
 */
void PID_Init(PID_Controller_t* pid, 
              float kp, float ki, float kd,
              float outMin, float outMax);

/**
 * @brief Compute PID output
 * 
 * @param pid Pointer to PID structure
 * @param setpoint Desired value
 * @param measurement Current measured value
 * @param dt Time step in seconds
 * @return Computed PID output
 */
float PID_Compute(PID_Controller_t* pid, 
                  float setpoint, 
                  float measurement, 
                  float dt);

/**
 * @brief Reset PID controller internal state
 * 
 * @param pid Pointer to PID structure
 */
void PID_Reset(PID_Controller_t* pid);

/**
 * @brief Set integral limits for anti-windup
 * 
 * @param pid Pointer to PID structure
 * @param min Minimum integral value
 * @param max Maximum integral value
 */
void PID_SetIntegralLimits(PID_Controller_t* pid, float min, float max);

#endif // PID_H