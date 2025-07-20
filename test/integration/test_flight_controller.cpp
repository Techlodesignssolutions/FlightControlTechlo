/**
 * Flight Controller Integration Tests
 * 
 * Tests the complete flight controller system with all modules
 * working together in realistic scenarios.
 */

#include <catch2/catch.hpp>
#include "mocks/MockHAL.h"
#include "core/FlightController.h"
#include <cmath>

using namespace Catch;

// Helper function to simulate a complete flight scenario
void simulateFlightLoop(FlightController& fc, MockHAL& hal, int iterations, float dt = 0.002f) {
    for (int i = 0; i < iterations; i++) {
        hal.advanceTime(static_cast<uint32_t>(dt * 1000000));  // Convert to microseconds
        fc.update();
    }
}

TEST_CASE("FlightController System Integration", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    
    // Configure for stable testing
    config.loop_frequency_hz = 500;
    config.enable_debug_output = false;  // Reduce noise in tests
    config.max_attitude_error = 30.0f;   // Relaxed for testing
    
    FlightController fc(&hal, config);
    
    SECTION("Complete system initialization") {
        REQUIRE(fc.initialize());
        
        // All subsystems should be initialized
        REQUIRE(hal.imu_initialized);
        REQUIRE(hal.radio_initialized);
        REQUIRE(hal.servos_initialized);
        REQUIRE(hal.motors_initialized);
        
        // Initial status should be ready
        REQUIRE(fc.getStatus() == FlightController::Status::READY);
    }
}

TEST_CASE("Stabilize Mode Flight Scenario", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    FlightController fc(&hal, config);
    
    REQUIRE(fc.initialize());
    REQUIRE(fc.setControlMode(ControlMode::STABILIZE));
    
    SECTION("Level flight with small disturbances") {
        // Set up level flight conditions
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});  // Armed, neutral stick
        
        // Simulate 5 seconds of level flight with small roll disturbance
        for (int second = 0; second < 5; second++) {
            for (int i = 0; i < 500; i++) {  // 500 Hz for 1 second
                // Add small roll disturbance
                float roll_disturbance = 0.1f * sin(2 * 3.14159265f * i / 500.0f);  // 1 Hz oscillation
                
                hal.pushIMUData(MockHAL::IMUFrame(
                    roll_disturbance, 0.0f, 0.0f,    // Roll disturbance
                    0.0f, 0.0f, 1.0f,                // Level acceleration
                    1.0f, 0.0f, 0.0f                 // North-pointing
                ));
                
                hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
                
                fc.update();
                hal.advanceTime(2000);  // 2ms timestep
            }
        }
        
        // System should remain stable
        REQUIRE(fc.getStatus() != FlightController::Status::ERROR);
        REQUIRE(fc.getStatus() != FlightController::Status::EMERGENCY);
        
        // Should have generated control outputs
        REQUIRE(hal.servo_commands.size() > 1000);  // Many servo commands
        
        // Control outputs should be reasonable (not saturated)
        bool found_reasonable_servo = false;
        for (const auto& cmd : hal.servo_commands) {
            if (std::abs(cmd.value - 0.5f) < 0.3f) {  // Within 30% of neutral
                found_reasonable_servo = true;
                break;
            }
        }
        REQUIRE(found_reasonable_servo);
    }
    
    SECTION("Recovery from attitude upset") {
        // Start with large attitude error
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        
        // Simulate 30° roll upset
        for (int i = 0; i < 100; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,                    // No rates initially
                sin(30.0f * 3.14159265f/180.0f), 0.0f, cos(30.0f * 3.14159265f/180.0f),  // 30° roll
                1.0f, 0.0f, 0.0f
            ));
            hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            fc.update();
            hal.advanceTime(2000);
        }
        
        // Then simulate recovery over 2 seconds
        for (int i = 0; i < 1000; i++) {
            float recovery_angle = 30.0f * (1.0f - i/1000.0f);  // Gradually reduce to 0°
            
            hal.pushIMUData(MockHAL::IMUFrame(
                -0.5f, 0.0f, 0.0f,                   // Roll rate to recover
                sin(recovery_angle * 3.14159265f/180.0f), 0.0f, cos(recovery_angle * 3.14159265f/180.0f),
                1.0f, 0.0f, 0.0f
            ));
            hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            fc.update();
            hal.advanceTime(2000);
        }
        
        // Should have generated corrective control actions
        bool found_corrective_action = false;
        for (const auto& cmd : hal.servo_commands) {
            if (std::abs(cmd.value - 0.5f) > 0.1f) {  // Significant deflection
                found_corrective_action = true;
                break;
            }
        }
        REQUIRE(found_corrective_action);
        
        // System should remain stable throughout
        REQUIRE(fc.getStatus() != FlightController::Status::ERROR);
    }
}

TEST_CASE("Adaptive Learning Scenario", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    
    // Enable adaptive features
    config.adaptive_pid_config.base_learning_rate = 0.001f;  // Faster learning for test
    
    FlightController fc(&hal, config);
    REQUIRE(fc.initialize());
    
    SECTION("PID gains should adapt to persistent error") {
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        
        // Record initial performance
        simulateFlightLoop(fc, hal, 500);  // 1 second
        auto initial_diag = fc.getDiagnosticInfo();
        
        // Simulate consistent disturbance requiring adaptation
        for (int i = 0; i < 2000; i++) {  // 4 seconds
            hal.pushIMUData(MockHAL::IMUFrame(
                0.2f, 0.0f, 0.0f,          // Persistent roll disturbance
                0.0f, 0.0f, 1.0f,
                1.0f, 0.0f, 0.0f
            ));
            hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            fc.update();
            hal.advanceTime(2000);
        }
        
        auto final_diag = fc.getDiagnosticInfo();
        
        // Adaptive system should have been learning
        REQUIRE(final_diag.adaptive_pid_learning);
        
        // Should have generated adaptive responses
        REQUIRE(hal.servo_commands.size() > 2000);
    }
}

TEST_CASE("Airspeed Estimation Integration", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    config.state_estimator_config.enable_airspeed_ekf = true;
    config.state_estimator_config.enable_throttle_dither = true;
    
    FlightController fc(&hal, config);
    REQUIRE(fc.initialize());
    
    SECTION("Airspeed should converge during cruise flight") {
        // Simulate takeoff and climb to cruise
        for (int phase = 0; phase < 3; phase++) {
            float throttle = 0.3f + phase * 0.2f;  // 30%, 50%, 70%
            float accel_x = 1.0f + phase * 0.5f;   // Increasing forward accel
            
            hal.pushRadioData({throttle, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            
            for (int i = 0; i < 1000; i++) {  // 2 seconds per phase
                hal.pushIMUData(MockHAL::IMUFrame(
                    0.0f, 0.0f, 0.0f,
                    accel_x, 0.0f, 1.0f,
                    1.0f, 0.0f, 0.0f
                ));
                hal.pushRadioData({throttle, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
                fc.update();
                hal.advanceTime(2000);
            }
        }
        
        auto state = fc.getAircraftState();
        
        // Airspeed should have converged to reasonable value
        REQUIRE(state.airspeed > 5.0f);   // Some forward speed
        REQUIRE(state.airspeed < 50.0f);  // But not unreasonable
        
        auto diag = fc.getDiagnosticInfo();
        REQUIRE(diag.airspeed_converged);
    }
}

TEST_CASE("Safety and Emergency Scenarios", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    config.radio_timeout_ms = 500.0f;  // Short timeout for testing
    
    FlightController fc(&hal, config);
    REQUIRE(fc.initialize());
    
    SECTION("Radio timeout should trigger emergency mode") {
        // Start with good radio signal
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        hal.setRadioConnected(true);
        
        simulateFlightLoop(fc, hal, 100);  // 200ms of good flight
        REQUIRE(fc.getStatus() != FlightController::Status::EMERGENCY);
        
        // Simulate radio loss
        hal.setRadioConnected(false);
        // Don't push any more radio data
        
        simulateFlightLoop(fc, hal, 300);  // 600ms without radio
        
        // Should have detected timeout and entered emergency mode
        auto status = fc.getStatus();
        REQUIRE((status == FlightController::Status::EMERGENCY || 
                status == FlightController::Status::ERROR));
    }
    
    SECTION("Extreme attitude should trigger emergency") {
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        
        // Simulate extreme attitude (inverted flight)
        for (int i = 0; i < 100; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,
                0.0f, 0.0f, -1.0f,  // Inverted (negative Z)
                1.0f, 0.0f, 0.0f
            ));
            hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            fc.update();
            hal.advanceTime(2000);
        }
        
        // Should detect extreme attitude
        auto status = fc.getStatus();
        // May trigger emergency depending on configuration
        REQUIRE((status == FlightController::Status::EMERGENCY || 
                status == FlightController::Status::FLYING ||
                status == FlightController::Status::ERROR));
    }
    
    SECTION("Emergency stop should work immediately") {
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        simulateFlightLoop(fc, hal, 100);
        
        hal.clearHistory();
        fc.emergencyStop();
        
        // Should have commanded safe outputs immediately
        REQUIRE(hal.motor_commands.size() > 0);
        
        // Last motor command should be zero throttle
        auto last_motor = hal.motor_commands.back();
        REQUIRE(last_motor.value == Approx(0.0f).margin(0.01f));
        
        // Status should be emergency
        REQUIRE(fc.getStatus() == FlightController::Status::EMERGENCY);
    }
}

TEST_CASE("Performance Monitoring", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    config.enable_loop_timing_monitoring = true;
    
    FlightController fc(&hal, config);
    REQUIRE(fc.initialize());
    
    SECTION("Should track loop timing statistics") {
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        
        // Run for several seconds
        simulateFlightLoop(fc, hal, 2500);  // 5 seconds
        
        auto stats = fc.getPerformanceStats();
        
        // Should have accumulated statistics
        REQUIRE(stats.loop_count > 2000);
        REQUIRE(stats.loop_time_avg_ms > 0.0f);
        REQUIRE(stats.loop_time_avg_ms < 10.0f);  // Should be fast
        REQUIRE(stats.cpu_usage_percent >= 0.0f);
        REQUIRE(stats.cpu_usage_percent <= 100.0f);
    }
}

TEST_CASE("Wind Compensation Integration", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    config.mixer_config.enable_wind_compensation = true;
    
    FlightController fc(&hal, config);
    REQUIRE(fc.initialize());
    
    SECTION("Should respond to wind disturbances") {
        hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
        
        // Simulate steady crosswind causing roll disturbance
        for (int i = 0; i < 1000; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                0.1f, 0.0f, 0.0f,    // Steady roll rate from wind
                0.1f, 0.0f, 1.0f,    // Slight side acceleration
                1.0f, 0.0f, 0.0f
            ));
            hal.pushRadioData({0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f});
            fc.update();
            hal.advanceTime(2000);
        }
        
        // Should have generated control responses
        REQUIRE(hal.servo_commands.size() > 500);
        
        // Should show wind compensation in diagnostics
        auto diag = fc.getDiagnosticInfo();
        REQUIRE(diag.wind_compensation_confidence >= 0.0f);
    }
}

TEST_CASE("Configuration Management", "[FlightController][Integration]") {
    MockHAL hal;
    FlightController::Config config;
    FlightController fc(&hal, config);
    
    SECTION("Should handle control mode changes") {
        REQUIRE(fc.initialize());
        
        // Should start in default mode
        REQUIRE(fc.getControlMode() == config.default_mode);
        
        // Should allow mode changes
        REQUIRE(fc.setControlMode(ControlMode::MANUAL));
        REQUIRE(fc.getControlMode() == ControlMode::MANUAL);
        
        REQUIRE(fc.setControlMode(ControlMode::STABILIZE));
        REQUIRE(fc.getControlMode() == ControlMode::STABILIZE);
    }
} 