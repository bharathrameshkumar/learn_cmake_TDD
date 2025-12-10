/**
 * @file pid.c
 * @brief PID Controller Implementation
 * 
 * Motor control PID with anti-windup and proper derivative handling
 */

#include "pid.h"
#include <stddef.h>

// Helper function to clamp values
static float clamp(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

void PID_Init(PID_Controller_t* pid, 
              float kp, float ki, float kd,
              float outMin, float outMax) {
    if (pid == NULL) return;
    
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->outputMin = outMin;
    pid->outputMax = outMax;
    
    // Initialize internal state
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->firstCall = true;  // Prevent derivative kick on first call
    
    // Default integral limits (can be changed with PID_SetIntegralLimits)
    pid->integralMax = outMax;
    pid->integralMin = outMin;
}

float PID_Compute(PID_Controller_t* pid, 
                  float setpoint, 
                  float measurement, 
                  float dt) {
    if (pid == NULL || dt <= 0.0f) return 0.0f;
    
    // Calculate error
    float error = setpoint - measurement;
    
    // Proportional term (always active)
    float proportional = pid->Kp * error;
    
    // Integral term - always accumulates (even on first call)
    // Integration represents area under the curve, starts accumulating immediately
    pid->integral += error * dt;
    
    // Apply anti-windup limits to the integral OUTPUT, not the raw integral
    float integralOutput = pid->Ki * pid->integral;
    integralOutput = clamp(integralOutput, pid->integralMin, pid->integralMax);
    
    // Back-calculate the clamped integral to prevent further windup
    if (pid->Ki != 0.0f) {
        pid->integral = integralOutput / pid->Ki;
    }
    
    // Derivative term - skip on first call to prevent derivative kick
    // Derivative needs two samples to compute rate of change
    float derivative = 0.0f;
    if (!pid->firstCall) {
        derivative = pid->Kd * (error - pid->prevError) / dt;
    }
    
    // Update state for next call
    pid->prevError = error;
    pid->firstCall = false;
    
    // Calculate total output
    float output = proportional + integralOutput + derivative;
    
    // Clamp output to limits
    output = clamp(output, pid->outputMin, pid->outputMax);
    
    return output;
}

void PID_Reset(PID_Controller_t* pid) {
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->firstCall = true;  // Reset the first call flag
}

void PID_SetIntegralLimits(PID_Controller_t* pid, float min, float max) {
    if (pid == NULL) return;
    
    pid->integralMin = min;
    pid->integralMax = max;
    
    // Clamp current integral output to new limits
    if (pid->Ki != 0.0f) {
        float integralOutput = pid->Ki * pid->integral;
        integralOutput = clamp(integralOutput, min, max);
        pid->integral = integralOutput / pid->Ki;
    }
}