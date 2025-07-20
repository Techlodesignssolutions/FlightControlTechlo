/**
 * Unit Tests for Adaptive PID Module
 * 
 * This demonstrates how the modular architecture enables desktop testing.
 * These tests can run on your development machine without any hardware.
 * 
 * To build and run:
 * mkdir build && cd build
 * cmake .. -DBUILD_TESTING=ON
 * cmake --build .
 * ctest --output-on-failure
 */

#include <catch2/catch.hpp>
#include "mocks/MockHAL.h"
#include "control/AdaptivePID.h"
#include <cmath>

using namespace Catch;

TEST_CASE("AdaptivePID 4D Gain Interpolation", "[adaptive_pid]") {
    AdaptivePID::Config config;
    
    // Create a simple test gain schedule
    ControlGains test_table[AdaptivePID::Config::THROTTLE_ZONES]
                           [AdaptivePID::Config::AOA_ZONES]
                           [AdaptivePID::Config::PHASE_ZONES]
                           [AdaptivePID::Config::AIRSPEED_ZONES];
    
    // Fill with known test values - all gains = 0.1 except one corner
    for (int t = 0; t < AdaptivePID::Config::THROTTLE_ZONES; t++) {
        for (int a = 0; a < AdaptivePID::Config::AOA_ZONES; a++) {
            for (int p = 0; p < AdaptivePID::Config::PHASE_ZONES; p++) {
                for (int s = 0; s < AdaptivePID::Config::AIRSPEED_ZONES; s++) {
                    test_table[t][a][p][s] = ControlGains(
                        PIDGains(0.1f, 0.02f, 0.005f),  // Roll
                        PIDGains(0.1f, 0.02f, 0.005f),  // Pitch  
                        PIDGains(0.08f, 0.01f, 0.003f)  // Yaw
                    );
                }
            }
        }
    }
    
    // Set one corner to different values for testing interpolation
    test_table[0][0][0][0] = ControlGains(
        PIDGains(0.2f, 0.04f, 0.01f),   // Roll - doubled
        PIDGains(0.2f, 0.04f, 0.01f),   // Pitch - doubled
        PIDGains(0.16f, 0.02f, 0.006f)  // Yaw - doubled
    );
    
    SECTION("Center of gain schedule should return average") {
        FlightRegime regime;
        regime.throttle_fraction = 0.5f;   // Middle of throttle range
        regime.angle_of_attack = 0.0f;     // Middle of AOA range  
        regime.airspeed = 15.0f;           // Middle of airspeed range
        regime.phase = FlightPhase::CRUISE; // Middle phase
        
        auto gains = AdaptivePID::interpolateGains(regime, test_table, config);
        
        // Should be close to 0.1 since most cells are 0.1
            REQUIRE(gains.roll.kp == Approx(0.1f).epsilon(0.05f));
    REQUIRE(gains.pitch.kp == Approx(0.1f).epsilon(0.05f));
    REQUIRE(gains.yaw.kp == Approx(0.08f).epsilon(0.05f));
    }
    
    SECTION("Corner of gain schedule should return exact values") {
        FlightRegime regime;
        regime.throttle_fraction = 0.0f;   // First throttle zone
        regime.angle_of_attack = -15.0f;   // First AOA zone
        regime.airspeed = 0.0f;            // First airspeed zone
        regime.phase = FlightPhase::UNKNOWN; // First phase
        
        auto gains = AdaptivePID::interpolateGains(regime, test_table, config);
        
        // Should match our special corner exactly
        REQUIRE(gains.roll.kp == Approx(0.2f).epsilon(0.001f));
        REQUIRE(gains.pitch.kp == Approx(0.2f).epsilon(0.001f));
        REQUIRE(gains.yaw.kp == Approx(0.16f).epsilon(0.001f));
    }
    
    SECTION("Interpolation should be smooth and bounded") {
        // Test multiple points to ensure smooth interpolation
        for (float throttle = 0.0f; throttle <= 1.0f; throttle += 0.1f) {
            for (float aoa = -10.0f; aoa <= 10.0f; aoa += 5.0f) {
                FlightRegime regime;
                regime.throttle_fraction = throttle;
                regime.angle_of_attack = aoa;
                regime.airspeed = 15.0f;
                regime.phase = FlightPhase::CRUISE;
                
                auto gains = AdaptivePID::interpolateGains(regime, test_table, config);
                
                // Gains should be reasonable and bounded
                REQUIRE(gains.roll.kp >= 0.05f);
                REQUIRE(gains.roll.kp <= 0.25f);
                REQUIRE(gains.pitch.kp >= 0.05f);
                REQUIRE(gains.pitch.kp <= 0.25f);
                REQUIRE(gains.yaw.kp >= 0.04f);
                REQUIRE(gains.yaw.kp <= 0.2f);
            }
        }
    }
}

TEST_CASE("AdaptivePID Grid Index Calculation", "[adaptive_pid]") {
    AdaptivePID::Config config;
    
    SECTION("Should correctly map flight regime to grid indices") {
        FlightRegime regime;
        regime.throttle_fraction = 0.5f;   // Should map to middle zones
        regime.angle_of_attack = 0.0f;
        regime.airspeed = 15.0f;
        regime.phase = FlightPhase::CRUISE;
        
        int t_idx, a_idx, p_idx, s_idx;
        float t_factor, a_factor, p_factor, s_factor;
        
        AdaptivePID::getGridIndices(regime, config, 
                                   t_idx, a_idx, p_idx, s_idx,
                                   t_factor, a_factor, p_factor, s_factor);
        
        // Check indices are in valid ranges
        REQUIRE(t_idx >= 0);
        REQUIRE(t_idx < AdaptivePID::Config::THROTTLE_ZONES - 1);
        REQUIRE(a_idx >= 0);
        REQUIRE(a_idx < AdaptivePID::Config::AOA_ZONES - 1);
        
        // Check interpolation factors are in [0, 1]
        REQUIRE(t_factor >= 0.0f);
        REQUIRE(t_factor <= 1.0f);
        REQUIRE(a_factor >= 0.0f);
        REQUIRE(a_factor <= 1.0f);
        REQUIRE(p_factor >= 0.0f);
        REQUIRE(p_factor <= 1.0f);
        REQUIRE(s_factor >= 0.0f);
        REQUIRE(s_factor <= 1.0f);
    }
    
    SECTION("Boundary conditions should be handled correctly") {
        FlightRegime regime;
        
        // Test lower boundaries
        regime.throttle_fraction = 0.0f;
        regime.angle_of_attack = -20.0f;  // Below minimum
        regime.airspeed = 0.0f;
        regime.phase = FlightPhase::UNKNOWN;
        
        int t_idx, a_idx, p_idx, s_idx;
        float t_factor, a_factor, p_factor, s_factor;
        
        AdaptivePID::getGridIndices(regime, config,
                                   t_idx, a_idx, p_idx, s_idx,
                                   t_factor, a_factor, p_factor, s_factor);
        
        // Should clamp to valid ranges
        REQUIRE(t_idx >= 0);
        REQUIRE(a_idx >= 0);
        REQUIRE(p_idx >= 0);
        REQUIRE(s_idx >= 0);
        
        // Test upper boundaries
        regime.throttle_fraction = 1.5f;  // Above maximum
        regime.angle_of_attack = 50.0f;   // Above maximum
        regime.airspeed = 100.0f;         // Above maximum
        
        AdaptivePID::getGridIndices(regime, config,
                                   t_idx, a_idx, p_idx, s_idx,
                                   t_factor, a_factor, p_factor, s_factor);
        
        // Should clamp to valid ranges
        REQUIRE(t_idx < AdaptivePID::Config::THROTTLE_ZONES);
        REQUIRE(a_idx < AdaptivePID::Config::AOA_ZONES);
        REQUIRE(p_idx < AdaptivePID::Config::PHASE_ZONES);
        REQUIRE(s_idx < AdaptivePID::Config::AIRSPEED_ZONES);
    }
}

TEST_CASE("AdaptivePID Performance Metrics", "[adaptive_pid]") {
    AdaptivePID adaptive_pid;
    adaptive_pid.initialize();
    
    SECTION("Should track performance over time") {
        // Simulate some control errors and outputs
        Attitude errors(5.0f, 3.0f, 1.0f);  // 5° roll, 3° pitch, 1° yaw error
        AngularRates outputs(0.1f, 0.08f, 0.02f);  // PID outputs
        float dt = 0.002f;  // 500 Hz
        
        // Update performance several times
        for (int i = 0; i < 100; i++) {
            adaptive_pid.updatePerformance(errors, outputs, dt);
            
            // Gradually reduce errors (simulating good control)
            errors.roll *= 0.98f;
            errors.pitch *= 0.98f;
            errors.yaw *= 0.98f;
        }
        
        auto roll_metrics = adaptive_pid.getRollMetrics();
        auto pitch_metrics = adaptive_pid.getPitchMetrics();
        auto yaw_metrics = adaptive_pid.getYawMetrics();
        
        // Health scores should be reasonable
        REQUIRE(roll_metrics.health_score >= 0.0f);
        REQUIRE(roll_metrics.health_score <= 100.0f);
        REQUIRE(pitch_metrics.health_score >= 0.0f);
        REQUIRE(pitch_metrics.health_score <= 100.0f);
        REQUIRE(yaw_metrics.health_score >= 0.0f);
        REQUIRE(yaw_metrics.health_score <= 100.0f);
        
        // Control effort should be positive
        REQUIRE(roll_metrics.control_effort > 0.0f);
        REQUIRE(pitch_metrics.control_effort > 0.0f);
        REQUIRE(yaw_metrics.control_effort > 0.0f);
    }
}

/**
 * This is just a sample of what's possible with modular testing!
 * 
 * Additional tests you could write:
 * - Test EKF airspeed estimation with synthetic IMU data
 * - Test wind compensation algorithms with known wind inputs
 * - Test control surface mixing with various configurations
 * - Test safety systems and emergency responses
 * - Performance benchmarks and timing tests
 * 
 * All of this can run on your desktop without any hardware,
 * making development much faster and more reliable!
 */ 