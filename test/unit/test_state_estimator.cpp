/**
 * State Estimator Tests
 * 
 * Tests attitude estimation (Madgwick filter) and airspeed estimation (EKF)
 * with various scenarios and edge cases.
 */

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// Disambiguate: pull in only the FP-Approx class, not the vector matcher
using Catch::Detail::Approx;

#include "mocks/MockHAL.h"
#include "estimation/StateEstimator.h"
#include <cmath>

// Helper function to check if two angles are approximately equal (handles wrap-around)
bool angleApprox(float a, float b, float tolerance = 1.0f) {
    float diff = std::abs(a - b);
    if (diff > 180.0f) diff = 360.0f - diff;
    return diff < tolerance;
}

TEST_CASE("StateEstimator Initialization", "[StateEstimator]") {
    MockHAL hal;
    StateEstimator::Config config;
    StateEstimator estimator(&hal, config);
    
    SECTION("Should initialize successfully") {
        REQUIRE(estimator.initialize());
        REQUIRE(hal.imu_initialized);
    }
    
    SECTION("Initial state should be reasonable") {
        estimator.initialize();
        auto state = estimator.getState();
        
        // Initial attitude should be close to level
        REQUIRE(std::abs(state.attitude.roll) < 45.0f);
        REQUIRE(std::abs(state.attitude.pitch) < 45.0f);
        REQUIRE(std::isfinite(state.attitude.yaw));
        
        // Initial airspeed should be zero or small
        REQUIRE(state.airspeed >= 0.0f);
        REQUIRE(state.airspeed < 5.0f);
    }
}

TEST_CASE("Madgwick Attitude Estimation", "[StateEstimator]") {
    MockHAL hal;
    StateEstimator::Config config;
    config.madgwick_beta = 0.1f;
    StateEstimator estimator(&hal, config);
    estimator.initialize();
    
    SECTION("Level flight should converge to zero roll/pitch") {
        // Feed 2 seconds of level flight data
        for (int i = 0; i < 1000; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,    // No rotation
                0.0f, 0.0f, 1.0f,    // 1G downward (level)
                1.0f, 0.0f, 0.0f     // North pointing
            ));
            hal.advanceTime(2000);  // 2ms timestep
            estimator.update(0.002f);
        }
        
        auto attitude = estimator.getAttitude();
        REQUIRE(std::abs(attitude.roll) < 2.0f);   // Within 2 degrees
        REQUIRE(std::abs(attitude.pitch) < 2.0f);  // Within 2 degrees
    }
    
    SECTION("Constant roll rate should integrate correctly") {
        const float roll_rate = 10.0f;  // 10 deg/s
        const float dt = 0.002f;
        const int steps = 500;  // 1 second
        
        for (int i = 0; i < steps; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                roll_rate * 3.14159265f/180.0f, 0.0f, 0.0f,  // Constant roll rate
                0.0f, 0.0f, 1.0f,                     // Level accel
                1.0f, 0.0f, 0.0f                      // North mag
            ));
            hal.advanceTime(2000);
            estimator.update(dt);
        }
        
        auto attitude = estimator.getAttitude();
        float expected_roll = roll_rate * steps * dt;  // Should be ~10°
        REQUIRE(std::abs(attitude.roll - expected_roll) < 3.0f);
    }
    
    SECTION("No magnetometer should still work (6DOF mode)") {
        StateEstimator::Config config_no_mag;
        config_no_mag.use_magnetometer = false;
        StateEstimator estimator_6dof(&hal, config_no_mag);
        estimator_6dof.initialize();
        
        for (int i = 0; i < 500; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,    // No rotation
                0.0f, 0.0f, 1.0f,    // Level
                0.0f, 0.0f, 0.0f     // No mag data
            ));
            hal.advanceTime(2000);
            estimator_6dof.update(0.002f);
        }
        
        auto attitude = estimator_6dof.getAttitude();
        REQUIRE(std::abs(attitude.roll) < 5.0f);   // Less precise without mag
        REQUIRE(std::abs(attitude.pitch) < 5.0f);
        // Yaw will drift without magnetometer - that's expected
    }
}

TEST_CASE("Airspeed EKF Estimation", "[StateEstimator]") {
    MockHAL hal;
    StateEstimator::Config config;
    config.enable_airspeed_ekf = true;
    config.initial_airspeed = 0.0f;
    config.initial_drag_coeff = 0.03f;
    StateEstimator estimator(&hal, config);
    estimator.initialize();
    
    SECTION("Throttle step should increase airspeed estimate") {
        // Start with zero throttle for 1 second
        for (int i = 0; i < 500; i++) {
            estimator.setThrottleCommand(0.0f);
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,    // No rotation
                0.0f, 0.0f, 1.0f,    // No acceleration
                1.0f, 0.0f, 0.0f
            ));
            hal.advanceTime(2000);
            estimator.update(0.002f);
        }
        
        float initial_airspeed = estimator.getAirspeedState().airspeed;
        
        // Apply 50% throttle for 2 seconds
        for (int i = 0; i < 1000; i++) {
            estimator.setThrottleCommand(0.5f);
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,    // No rotation  
                2.0f, 0.0f, 1.0f,    // Forward acceleration (thrust > drag)
                1.0f, 0.0f, 0.0f
            ));
            hal.advanceTime(2000);
            estimator.update(0.002f);
        }
        
        float final_airspeed = estimator.getAirspeedState().airspeed;
        
        // Airspeed should have increased
        REQUIRE(final_airspeed > initial_airspeed + 2.0f);  // At least 2 m/s increase
        REQUIRE(final_airspeed < 30.0f);  // But not unreasonably high
        
        // EKF should converge
        auto airspeed_state = estimator.getAirspeedState();
        REQUIRE(airspeed_state.confidence > 0.3f);  // Should have some confidence
    }
    
    SECTION("Consistent cruise should converge drag coefficient") {
        const float cruise_throttle = 0.4f;
        const float cruise_accel = 0.1f;  // Small forward acceleration
        
        for (int i = 0; i < 2000; i++) {  // 4 seconds
            estimator.setThrottleCommand(cruise_throttle);
            hal.pushIMUData(MockHAL::IMUFrame(
                0.0f, 0.0f, 0.0f,
                cruise_accel, 0.0f, 1.0f,
                1.0f, 0.0f, 0.0f
            ));
            hal.advanceTime(2000);
            estimator.update(0.002f);
        }
        
        auto airspeed_state = estimator.getAirspeedState();
        
        // Drag coefficient should be reasonable
        REQUIRE(airspeed_state.drag_coefficient > 0.01f);
        REQUIRE(airspeed_state.drag_coefficient < 0.1f);
        
        // Should have converged
        REQUIRE(airspeed_state.converged);
        REQUIRE(airspeed_state.confidence > 0.5f);
    }
}

TEST_CASE("Throttle Dithering", "[StateEstimator]") {
    MockHAL hal;
    StateEstimator::Config config;
    config.enable_throttle_dither = true;
    config.dither_amplitude = 0.05f;  // ±5%
    config.dither_frequency = 2.0f;   // 2 Hz
    StateEstimator estimator(&hal, config);
    estimator.initialize();
    
    SECTION("Should apply sinusoidal dither to throttle") {
        float base_throttle = 0.5f;
        estimator.setThrottleCommand(base_throttle);
        
        std::vector<float> effective_throttles;
        
        // Sample over one complete dither cycle
        for (int i = 0; i < 250; i++) {  // 0.5 seconds at 2 Hz = 1 cycle
            hal.advanceTime(2000);
            estimator.update(0.002f);
            effective_throttles.push_back(estimator.getEffectiveThrottle());
        }
        
        // Find min and max effective throttle
        float min_throttle = *std::min_element(effective_throttles.begin(), effective_throttles.end());
        float max_throttle = *std::max_element(effective_throttles.begin(), effective_throttles.end());
        
        // Should vary around base throttle
        REQUIRE(min_throttle >= base_throttle - config.dither_amplitude);
        REQUIRE(max_throttle <= base_throttle + config.dither_amplitude);
        REQUIRE(max_throttle - min_throttle > config.dither_amplitude);  // Should actually vary
    }
}

TEST_CASE("StateEstimator Edge Cases", "[StateEstimator]") {
    MockHAL hal;
    StateEstimator::Config config;
    StateEstimator estimator(&hal, config);
    estimator.initialize();
    
    SECTION("Should handle missing IMU data gracefully") {
        // Don't push any IMU data - queue is empty
        bool result = estimator.update(0.002f);
        
        // Should return false but not crash
        REQUIRE_FALSE(result);
        
        // State should remain valid
        auto state = estimator.getState();
        REQUIRE(std::isfinite(state.attitude.roll));
        REQUIRE(std::isfinite(state.attitude.pitch));
        REQUIRE(std::isfinite(state.attitude.yaw));
    }
    
    SECTION("Should handle extreme acceleration values") {
        // Push extreme acceleration (like during crash)
        hal.pushIMUData(MockHAL::IMUFrame(
            0.0f, 0.0f, 0.0f,
            50.0f, 0.0f, 50.0f,   // 50G acceleration!
            1.0f, 0.0f, 0.0f
        ));
        hal.advanceTime(2000);
        
        bool result = estimator.update(0.002f);
        
        // Should handle gracefully
        REQUIRE(result);
        auto state = estimator.getState();
        REQUIRE(std::isfinite(state.attitude.roll));
        REQUIRE(std::isfinite(state.attitude.pitch));
    }
    
    SECTION("Should calibrate IMU bias correctly") {
        // Add consistent bias to gyro readings
        const float bias_x = 0.1f;  // 0.1 rad/s bias
        
        for (int i = 0; i < 1000; i++) {
            hal.pushIMUData(MockHAL::IMUFrame(
                bias_x, 0.0f, 0.0f,   // Constant bias
                0.0f, 0.0f, 1.0f,
                1.0f, 0.0f, 0.0f
            ));
        }
        
        bool calibrated = estimator.calibrateIMU(1000);
        REQUIRE(calibrated);
        
        // After calibration, bias should be removed
        // (This would require access to internal bias values to test properly)
    }
}

TEST_CASE("Static Utility Functions", "[StateEstimator]") {
    SECTION("Madgwick update should be stable") {
        float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // Identity quaternion
        
        // Apply 100 updates with level data
        for (int i = 0; i < 100; i++) {
            StateEstimator::madgwickUpdate(
                0.0f, 0.0f, 0.0f,    // No gyro
                0.0f, 0.0f, 1.0f,    // Level accel
                1.0f, 0.0f, 0.0f,    // North mag
                0.002f, 0.1f,        // dt, beta
                q0, q1, q2, q3
            );
        }
        
        // Quaternion should remain normalized
        float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        REQUIRE(norm == Approx(1.0f).epsilon(0.01f));
        
        // Should converge to level attitude
        auto attitude = StateEstimator::quaternionToEuler(q0, q1, q2, q3);
        REQUIRE(std::abs(attitude.roll) < 5.0f);
        REQUIRE(std::abs(attitude.pitch) < 5.0f);
    }
    
    SECTION("Angle of attack calculation") {
        Attitude attitude(10.0f, 5.0f, 0.0f);  // 10° roll, 5° pitch
        Vector3 airflow(15.0f, 0.0f, 1.0f);    // Mostly forward, slight up
        
        float aoa = StateEstimator::calculateAngleOfAttack(attitude, airflow);
        
        REQUIRE(std::isfinite(aoa));
        REQUIRE(aoa > -30.0f);
        REQUIRE(aoa < 30.0f);
    }
} 