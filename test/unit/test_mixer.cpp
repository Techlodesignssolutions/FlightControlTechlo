/**
 * Fixed Wing Mixer Tests
 * 
 * Tests elevon control surface mixing, wind compensation,
 * airspeed scaling, and safety systems.
 */

#include <catch2/catch.hpp>
#include "mocks/MockHAL.h"
#include "mixing/FixedWingMixer.h"
#include <cmath>

using namespace Catch;

TEST_CASE("FixedWingMixer Initialization", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    FixedWingMixer mixer(&hal, config);
    
    SECTION("Should initialize successfully") {
        REQUIRE(mixer.initialize());
        REQUIRE(hal.servos_initialized);
        REQUIRE(hal.motors_initialized);
    }
    
    SECTION("Initial control outputs should be neutral") {
        mixer.initialize();
        auto controls = mixer.getCurrentControls();
        
            REQUIRE(controls.left_elevon == Approx(0.0f).margin(0.01f));
    REQUIRE(controls.right_elevon == Approx(0.0f).margin(0.01f));
    REQUIRE(controls.rudder == Approx(0.0f).margin(0.01f));
    REQUIRE(controls.throttle == Approx(0.0f).margin(0.01f));
    }
}

TEST_CASE("Basic Elevon Mixing", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    config.use_elevons = true;
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Pure elevator command should affect both elevons equally") {
        AngularRates pid_output(0.0f, 0.3f, 0.0f);  // 30% pitch up
        RadioInputs radio;  // All neutral
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // Both elevons should deflect equally for elevator
        REQUIRE(controls.left_elevon == Approx(controls.right_elevon).epsilon(0.01f));
        REQUIRE(controls.left_elevon > 0.1f);  // Should be positive (trailing edge down)
    }
    
    SECTION("Pure aileron command should create differential elevon deflection") {
        AngularRates pid_output(0.2f, 0.0f, 0.0f);  // 20% roll right
        RadioInputs radio;
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // Elevons should be differential for roll
        REQUIRE(controls.left_elevon != controls.right_elevon);
        REQUIRE(std::abs(controls.left_elevon - controls.right_elevon) > 0.1f);
        
        // For roll right: left elevon down, right elevon up
        REQUIRE(controls.left_elevon > controls.right_elevon);
    }
    
    SECTION("Combined elevator and aileron should mix correctly") {
        AngularRates pid_output(0.1f, 0.2f, 0.0f);  // Roll right + pitch up
        RadioInputs radio;
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // Both elevons should be positive (pitch up component)
        REQUIRE(controls.left_elevon > 0.0f);
        REQUIRE(controls.right_elevon > 0.0f);
        
        // But left should be more positive (roll right component)
        REQUIRE(controls.left_elevon > controls.right_elevon);
    }
    
    SECTION("Yaw command should affect rudder") {
        AngularRates pid_output(0.0f, 0.0f, 0.15f);  // 15% yaw right
        RadioInputs radio;
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // Rudder should deflect
        REQUIRE(controls.rudder > 0.1f);
        
        // Elevons should be unaffected
        REQUIRE(std::abs(controls.left_elevon) < 0.01f);
        REQUIRE(std::abs(controls.right_elevon) < 0.01f);
    }
}

TEST_CASE("Static Mixing Functions", "[FixedWingMixer]") {
    SECTION("Static elevon mixing should work correctly") {
        float elevator = 0.2f;   // 20% up
        float aileron = 0.1f;    // 10% right
        float rudder = 0.05f;    // 5% right
        float throttle = 0.6f;   // 60% throttle
        
        auto controls = FixedWingMixer::mixElevons(elevator, aileron, rudder, throttle);
        
        // Check elevon mixing: left = elevator + aileron, right = elevator - aileron
        REQUIRE(controls.left_elevon == Approx(elevator + aileron).epsilon(0.01f));
        REQUIRE(controls.right_elevon == Approx(elevator - aileron).epsilon(0.01f));
        REQUIRE(controls.rudder == Approx(rudder).epsilon(0.01f));
        REQUIRE(controls.throttle == Approx(throttle).epsilon(0.01f));
    }
}

TEST_CASE("Airspeed Scaling", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    config.enable_airspeed_scaling = true;
    config.nominal_airspeed = 15.0f;
    config.min_airspeed_scaling = 0.5f;
    config.max_airspeed_scaling = 1.5f;
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Low airspeed should increase control authority") {
        AngularRates pid_output(0.0f, 0.2f, 0.0f);  // 20% pitch
        RadioInputs radio;
        float low_airspeed = 7.5f;  // Half nominal airspeed
        
        auto controls = mixer.mix(pid_output, radio, low_airspeed);
        
        // At half airspeed, should get more control deflection
        REQUIRE(std::abs(controls.left_elevon) > 0.15f);  // More than 15%
    }
    
    SECTION("High airspeed should reduce control authority") {
        AngularRates pid_output(0.0f, 0.2f, 0.0f);  // 20% pitch
        RadioInputs radio;
        float high_airspeed = 30.0f;  // Double nominal airspeed
        
        auto controls = mixer.mix(pid_output, radio, high_airspeed);
        
        // At double airspeed, should get less control deflection
        REQUIRE(std::abs(controls.left_elevon) < 0.15f);  // Less than 15%
    }
    
    SECTION("Static airspeed scaling function") {
        float command = 0.2f;
        float nominal = 15.0f;
        float min_scale = 0.5f;
        float max_scale = 1.5f;
        
        // Low airspeed should scale up
        float scaled_low = FixedWingMixer::scaleControlAuthority(command, 7.5f, nominal, min_scale, max_scale);
        REQUIRE(scaled_low > command);
        
        // High airspeed should scale down  
        float scaled_high = FixedWingMixer::scaleControlAuthority(command, 30.0f, nominal, min_scale, max_scale);
        REQUIRE(scaled_high < command);
        
        // Nominal airspeed should be unchanged
        float scaled_nominal = FixedWingMixer::scaleControlAuthority(command, nominal, nominal, min_scale, max_scale);
        REQUIRE(scaled_nominal == Approx(command).epsilon(0.01f));
    }
}

TEST_CASE("Wind Compensation", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    config.enable_wind_compensation = true;
    config.wind_compensation_gain = 1.0f;
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Crosswind compensation should create roll bias") {
        AngularRates pid_output;  // No PID correction
        RadioInputs radio;        // Neutral radio
        float airspeed = 15.0f;
        
        FixedWingMixer::WindCompensation wind_comp;
        wind_comp.roll_compensation = 0.1f;   // 10% left aileron for crosswind
        wind_comp.confidence = 0.8f;          // High confidence
        
        auto controls = mixer.mix(pid_output, radio, airspeed, wind_comp);
        
        // Should have differential elevon deflection to counteract crosswind
        REQUIRE(controls.left_elevon != controls.right_elevon);
        REQUIRE(std::abs(controls.left_elevon - controls.right_elevon) > 0.05f);
    }
    
    SECTION("Headwind compensation should affect throttle") {
        AngularRates pid_output;
        RadioInputs radio;
        radio.throttle = 0.5f;    // 50% base throttle
        float airspeed = 15.0f;
        
        FixedWingMixer::WindCompensation wind_comp;
        wind_comp.throttle_compensation = 0.1f;  // +10% for headwind
        wind_comp.confidence = 0.9f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed, wind_comp);
        
        // Throttle should be increased
        REQUIRE(controls.throttle > radio.throttle);
        REQUIRE(controls.throttle == Approx(0.6f).epsilon(0.05f));
    }
    
    SECTION("Low confidence wind data should have minimal effect") {
        AngularRates pid_output;
        RadioInputs radio;
        float airspeed = 15.0f;
        
        FixedWingMixer::WindCompensation wind_comp;
        wind_comp.roll_compensation = 0.2f;   // Large compensation
        wind_comp.confidence = 0.1f;          // But low confidence
        
        auto controls = mixer.mix(pid_output, radio, airspeed, wind_comp);
        
        // Should have minimal effect due to low confidence
        REQUIRE(std::abs(controls.left_elevon - controls.right_elevon) < 0.05f);
    }
}

TEST_CASE("Rudder Coordination", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    config.enable_rudder_coordination = true;
    config.coordination_gain = 0.1f;
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Roll rate should induce coordinating yaw") {
        AngularRates pid_output(0.0f, 0.0f, 0.0f);  // No explicit yaw command
        AngularRates rates(2.0f, 0.0f, 0.0f);       // But aircraft is rolling right
        RadioInputs radio;
        float airspeed = 15.0f;
        
        // This test would require access to internal coordination calculation
        // For now, test the static function
        float coordination = FixedWingMixer::calculateCoordination(2.0f, 0.0f, 15.0f);
        REQUIRE(coordination > 0.0f);  // Should command right rudder for right roll
    }
    
    SECTION("Static coordination calculation") {
        // Right roll should need right rudder
        float coord_right = FixedWingMixer::calculateCoordination(2.0f, 0.0f, 15.0f);
        REQUIRE(coord_right > 0.0f);
        
        // Left roll should need left rudder
        float coord_left = FixedWingMixer::calculateCoordination(-2.0f, 0.0f, 15.0f);
        REQUIRE(coord_left < 0.0f);
        
        // Higher airspeed should need less rudder
        float coord_fast = FixedWingMixer::calculateCoordination(2.0f, 0.0f, 30.0f);
        REQUIRE(std::abs(coord_fast) < std::abs(coord_right));
    }
}

TEST_CASE("Safety Limits", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    config.max_elevon_deflection = 0.6f;   // ±60%
    config.max_rudder_deflection = 0.4f;   // ±40%
    config.max_throttle = 0.9f;            // 90% max
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Should limit excessive control commands") {
        AngularRates pid_output(1.5f, 1.5f, 1.5f);  // 150% commands (excessive!)
        RadioInputs radio;
        radio.throttle = 1.2f;  // 120% throttle (excessive!)
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // All outputs should be within configured limits
        REQUIRE(std::abs(controls.left_elevon) <= config.max_elevon_deflection);
        REQUIRE(std::abs(controls.right_elevon) <= config.max_elevon_deflection);
        REQUIRE(std::abs(controls.rudder) <= config.max_rudder_deflection);
        REQUIRE(controls.throttle <= config.max_throttle);
        REQUIRE(controls.throttle >= 0.0f);
    }
    
    SECTION("Normal commands should pass through unchanged") {
        AngularRates pid_output(0.1f, 0.2f, 0.05f);  // Reasonable commands
        RadioInputs radio;
        radio.throttle = 0.5f;  // 50% throttle
        float airspeed = 15.0f;
        
        auto controls = mixer.mix(pid_output, radio, airspeed);
        
        // Should be within limits but not clipped
        REQUIRE(std::abs(controls.left_elevon) < config.max_elevon_deflection);
        REQUIRE(std::abs(controls.right_elevon) < config.max_elevon_deflection);
        REQUIRE(std::abs(controls.rudder) < config.max_rudder_deflection);
        REQUIRE(controls.throttle > 0.1f);  // Should have some throttle
        REQUIRE(controls.throttle < config.max_throttle);
    }
}

TEST_CASE("Hardware Output", "[FixedWingMixer]") {
    MockHAL hal;
    FixedWingMixer::Config config;
    FixedWingMixer mixer(&hal, config);
    mixer.initialize();
    
    SECTION("Should write servo commands to hardware") {
        hal.clearHistory();
        
        ControlSurfaces controls;
        controls.left_elevon = 0.3f;   // 30%
        controls.right_elevon = -0.2f; // -20%
        controls.rudder = 0.1f;        // 10%
        controls.throttle = 0.6f;      // 60%
        
        mixer.applyControls(controls);
        
        // Should have written to servo channels
        REQUIRE(hal.servo_commands.size() >= 3);  // At least 3 servos
        REQUIRE(hal.motor_commands.size() >= 1);  // At least 1 motor
        
        // Check some specific commands were recorded
        bool found_servo = false;
        for (const auto& cmd : hal.servo_commands) {
            if (cmd.channel == 0 && std::abs(cmd.value - 0.65f) < 0.1f) {  // 30% -> ~65% servo position
                found_servo = true;
            }
        }
        REQUIRE(found_servo);
    }
    
    SECTION("Emergency stop should set safe positions") {
        hal.clearHistory();
        
        mixer.emergencyStop();
        
        // Should have written safe commands
        REQUIRE(hal.servo_commands.size() > 0);
        REQUIRE(hal.motor_commands.size() > 0);
        
        // Motors should be at minimum throttle
        auto last_motor = hal.motor_commands.back();
        REQUIRE(last_motor.value == Approx(0.0f).margin(0.01f));
    }
}

TEST_CASE("Configuration Variants", "[FixedWingMixer]") {
    MockHAL hal;
    
    SECTION("Traditional elevator/aileron configuration") {
        FixedWingMixer::Config config;
        config.use_elevons = false;  // Traditional setup
        FixedWingMixer mixer(&hal, config);
        mixer.initialize();
        
        // This would test traditional elevator + aileron mixing
        // Implementation would differ from elevon mixing
        REQUIRE(mixer.initialize());  // Should still initialize
    }
    
    SECTION("Reversed control surfaces") {
        FixedWingMixer::Config config;
        config.reverse_left_elevon = true;
        config.reverse_rudder = true;
        FixedWingMixer mixer(&hal, config);
        mixer.initialize();
        
        REQUIRE(mixer.initialize());  // Should handle reversed surfaces
    }
} 