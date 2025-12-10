/**
 * @file test_pid.c
 * @brief Unit tests for PID controller
 */

#include "unity.h"
#include "pid.h"
#include <math.h>

// Test tolerance for floating point comparisons
#define FLOAT_TOLERANCE 0.001f

PID_Controller_t testPid;

void setUp(void) {
    // This runs before each test
}

void tearDown(void) {
    // This runs after each test
}

// ============================================================================
// INITIALIZATION TESTS
// ============================================================================

void test_PID_Init_SetsGainsCorrectly(void) {
    PID_Init(&testPid, 1.5f, 0.2f, 0.05f, -100.0f, 100.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(1.5f, testPid.Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.2f, testPid.Ki);
    TEST_ASSERT_EQUAL_FLOAT(0.05f, testPid.Kd);
    TEST_ASSERT_EQUAL_FLOAT(-100.0f, testPid.outputMin);
    TEST_ASSERT_EQUAL_FLOAT(100.0f, testPid.outputMax);
}

void test_PID_Init_ResetsInternalState(void) {
    // Pollute the structure first
    testPid.integral = 999.0f;
    testPid.prevError = 888.0f;
    
    PID_Init(&testPid, 1.0f, 0.1f, 0.01f, -50.0f, 50.0f);
    
    TEST_ASSERT_EQUAL_FLOAT(0.0f, testPid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, testPid.prevError);
}

// ============================================================================
// PROPORTIONAL TESTS
// ============================================================================

void test_PID_Compute_ProportionalOnly_PositiveError(void) {
    // Pure P controller (Ki=0, Kd=0)
    PID_Init(&testPid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    
    float output = PID_Compute(&testPid, 50.0f, 30.0f, 0.01f);
    
    // Error = 50-30 = 20, P = 2.0*20 = 40.0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 40.0f, output);
}

void test_PID_Compute_ProportionalOnly_NegativeError(void) {
    PID_Init(&testPid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    
    float output = PID_Compute(&testPid, 30.0f, 50.0f, 0.01f);
    
    // Error = 30-50 = -20, P = 2.0*(-20) = -40.0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, -40.0f, output);
}

void test_PID_Compute_ProportionalOnly_ZeroError(void) {
    PID_Init(&testPid, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    
    float output = PID_Compute(&testPid, 50.0f, 50.0f, 0.01f);
    
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, output);
}

// ============================================================================
// INTEGRAL TESTS
// ============================================================================

void test_PID_Compute_IntegralAccumulation(void) {
    // PI controller (Kd=0)
    PID_Init(&testPid, 1.0f, 0.5f, 0.0f, -100.0f, 100.0f);
    
    // First call: error = 10, dt = 0.1
    float output1 = PID_Compute(&testPid, 20.0f, 10.0f, 0.1f);
    // P = 1.0*10 = 10, I = 0.5*10*0.1 = 0.5, Total = 10.5
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 10.5f, output1);
    
    // Second call: error = 10 again, dt = 0.1
    float output2 = PID_Compute(&testPid, 20.0f, 10.0f, 0.1f);
    // P = 1.0*10 = 10, I = 0.5*(1.0 + 1.0) = 1.0, Total = 11.0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 11.0f, output2);
}

void test_PID_Compute_IntegralWithOppositeErrors(void) {
    PID_Init(&testPid, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    
    // Accumulate positive error
    PID_Compute(&testPid, 10.0f, 0.0f, 0.1f);  // +1.0 to integral
    PID_Compute(&testPid, 10.0f, 0.0f, 0.1f);  // +1.0 to integral
    
    // Now apply negative error - should reduce integral
    float output = PID_Compute(&testPid, 0.0f, 10.0f, 0.1f);  // -1.0 to integral
    
    // Integral should be 1.0 now (2.0 - 1.0), output = 1.0 * 1.0 = 1.0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.0f, output);
}

// ============================================================================
// DERIVATIVE TESTS
// ============================================================================

void test_PID_Compute_DerivativeWithChangingError(void) {
    // PD controller (Ki=0)
    PID_Init(&testPid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
    
    // First call: error = 10 (derivative is 0 due to firstCall flag)
    float output1 = PID_Compute(&testPid, 10.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, output1);
    
    // Second call: error = 20 (rate of change = (20-10)/0.1 = 100)
    float output2 = PID_Compute(&testPid, 20.0f, 0.0f, 0.1f);
    // D = 1.0 * (20-10)/0.1 = 100
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 100.0f, output2);
}

void test_PID_Compute_DerivativeWithConstantError(void) {
    PID_Init(&testPid, 0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
    
    // First call: error = 10
    PID_Compute(&testPid, 10.0f, 0.0f, 0.1f);
    
    // Second call: error = 10 (no change)
    float output = PID_Compute(&testPid, 10.0f, 0.0f, 0.1f);
    
    // D = 1.0 * (10-10)/0.1 = 0
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, output);
}

// ============================================================================
// OUTPUT LIMITING TESTS (Critical for motor control safety!)
// ============================================================================

void test_PID_Compute_OutputLimitPositive(void) {
    PID_Init(&testPid, 10.0f, 0.0f, 0.0f, -50.0f, 50.0f);
    
    // Large error would give 10*100 = 1000, but limit is 50
    float output = PID_Compute(&testPid, 100.0f, 0.0f, 0.01f);
    
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 50.0f, output);
}

void test_PID_Compute_OutputLimitNegative(void) {
    PID_Init(&testPid, 10.0f, 0.0f, 0.0f, -50.0f, 50.0f);
    
    // Large negative error would give 10*(-100) = -1000, but limit is -50
    float output = PID_Compute(&testPid, 0.0f, 100.0f, 0.01f);
    
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, -50.0f, output);
}

void test_PID_Compute_AsymmetricLimits(void) {
    // Motor control often has asymmetric limits (e.g., braking vs acceleration)
    PID_Init(&testPid, 10.0f, 0.0f, 0.0f, -30.0f, 50.0f);
    
    float outputPos = PID_Compute(&testPid, 100.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 50.0f, outputPos);
    
    PID_Reset(&testPid);
    
    float outputNeg = PID_Compute(&testPid, 0.0f, 100.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, -30.0f, outputNeg);
}

// ============================================================================
// ANTI-WINDUP TESTS (Critical for motor control!)
// ============================================================================

void test_PID_SetIntegralLimits_PreventsWindup(void) {
    PID_Init(&testPid, 0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    PID_SetIntegralLimits(&testPid, -10.0f, 10.0f);
    
    // Try to accumulate large integral
    for (int i = 0; i < 20; i++) {
        PID_Compute(&testPid, 100.0f, 0.0f, 0.1f);
    }
    
    // Integral should be clamped to 10.0, so output = 1.0 * 10.0 = 10.0
    float output = PID_Compute(&testPid, 100.0f, 0.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 10.0f, output);
}

// ============================================================================
// RESET TESTS
// ============================================================================

void test_PID_Reset_ClearsIntegralAndDerivative(void) {
    PID_Init(&testPid, 1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
    
    // Accumulate some state
    PID_Compute(&testPid, 50.0f, 30.0f, 0.1f);
    PID_Compute(&testPid, 50.0f, 30.0f, 0.1f);
    
    // Reset
    PID_Reset(&testPid);
    
    // Check state is cleared
    TEST_ASSERT_EQUAL_FLOAT(0.0f, testPid.integral);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, testPid.prevError);
    TEST_ASSERT_TRUE(testPid.firstCall);
    
    // Next computation: error = 20, dt = 0.1
    // P = 1.0*20 = 20
    // I = 1.0*20*0.1 = 2.0 (integral accumulates from first call)
    // D = 0 (first call after reset)
    // Total = 22.0
    float output = PID_Compute(&testPid, 50.0f, 30.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 22.0f, output);
}

// ============================================================================
// REAL-WORLD MOTOR CONTROL SCENARIOS
// ============================================================================

void test_PID_MotorCurrentControl_Scenario(void) {
    // Test realistic motor control scenarios with proper PID behavior
    
    // Scenario 1: Step response from zero
    PID_Init(&testPid, 0.5f, 20.0f, 0.001f, -24.0f, 24.0f);
    PID_SetIntegralLimits(&testPid, -10.0f, 10.0f);
    
    float target = 5.0f;
    float voltage;
    
    // Large initial error should produce strong positive output
    voltage = PID_Compute(&testPid, target, 0.0f, 0.0001f);
    printf("\n[Test 1] Initial: V=%.2f (error=5A)\n", voltage);
    TEST_ASSERT_TRUE(voltage > 2.0f);
    
    // Scenario 2: Overshoot correction (fresh PID)
    PID_Reset(&testPid);
    voltage = PID_Compute(&testPid, target, 8.0f, 0.0001f);
    printf("[Test 2] Overshoot: V=%.2f (error=-3A)\n", voltage);
    TEST_ASSERT_TRUE(voltage < -1.0f);  // Should be negative
    
    // Scenario 3: Undershoot correction (fresh PID)
    PID_Reset(&testPid);
    voltage = PID_Compute(&testPid, target, 2.0f, 0.0001f);
    printf("[Test 3] Undershoot: V=%.2f (error=+3A)\n", voltage);
    TEST_ASSERT_TRUE(voltage > 1.0f);  // Should be positive
    
    // Scenario 4: Gradual approach (realistic simulation)
    PID_Reset(&testPid);
    float current = 0.0f;
    
    printf("[Test 4] Gradual approach:\n");
    for (int i = 0; i < 10; i++) {
        voltage = PID_Compute(&testPid, target, current, 0.0001f);
        
        if (i < 5) {  // Only print first few for readability
            printf("  Step %d: I=%.2fA, V=%.2fV\n", i, current, voltage);
        }
        
        // Voltage should be within limits
        TEST_ASSERT_TRUE(voltage >= -24.0f && voltage <= 24.0f);
        
        // Simulate gradual current increase (realistic motor response)
        current += voltage * 0.1f;  // Increased from 0.05 to 0.1 for faster response
    }
    
    printf("  Final: I=%.2fA\n", current);
    
    // After 10 steps, current should have increased significantly
    TEST_ASSERT_TRUE(current > 1.0f);
    
    // Scenario 5: At setpoint, output should be small
    PID_Reset(&testPid);
    voltage = PID_Compute(&testPid, target, 5.0f, 0.0001f);
    printf("[Test 5] At setpoint: V=%.2f (error=0A)\n", voltage);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, voltage);  // Should be near zero
}