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
    
    // Proportional term
    float proportional = pid->Kp * error;
    
    // Integral term with anti-windup
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, pid->integralMin, pid->integralMax);
    float integral = pid->Ki * pid->integral;
    
    // Derivative term
    float derivative = pid->Kd * (error - pid->prevError) / dt;
    pid->prevError = error;
    
    // Calculate total output
    float output = proportional + integral + derivative;
    
    // Clamp output to limits
    output = clamp(output, pid->outputMin, pid->outputMax);
    
    return output;
}

void PID_Reset(PID_Controller_t* pid) {
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
}

void PID_SetIntegralLimits(PID_Controller_t* pid, float min, float max) {
    if (pid == NULL) return;
    
    pid->integralMin = min;
    pid->integralMax = max;
    
    // Clamp current integral to new limits
    pid->integral = clamp(pid->integral, min, max);
}