/**
 * Adaptive Flight Controller - Arduino Wrapper
 * 
 * This is a minimal Arduino sketch that demonstrates the clean, modular architecture.
 * All the complex algorithms are in separate, testable modules.
 * 
 * The main .ino file is now just:
 * 1. Include the modules
 * 2. Create hardware abstraction
 * 3. Initialize the flight controller
 * 4. Run the control loop
 * 
 * That's it! Clean, simple, maintainable.
 */

#include "src/core/FlightController.h"
#include "src/core/TeensyHAL.h"

// Hardware configuration for Teensy 4.0 with fixed-wing elevon aircraft
HardwareConfig hw_config = {
    .use_magnetometer = true,
    .num_radio_channels = 6,
    .servo_pins = {2, 3, 4, 5, 6, 7, 8, 9},     // PWM pins for servos
    .servo_min_us = 900,
    .servo_max_us = 2100,
    .motor_pins = {20, 21, 22, 23, 24, 25},     // OneShot125 pins for motors
    .loop_frequency_hz = 500,                   // 500 Hz control loop
    .status_led_pin = 13
};

// Flight controller configuration
FlightController::Config fc_config = {
    .loop_frequency_hz = 500,
    .default_mode = ControlMode::STABILIZE,
    
    // Configure state estimator
    .state_estimator_config = {
        .madgwick_beta = 0.1f,
        .use_magnetometer = true,
        .enable_airspeed_ekf = true,
        .enable_throttle_dither = true
    },
    
    // Configure adaptive PID
    .adaptive_pid_config = {
        .base_learning_rate = 0.0003f,
        .momentum_factor = 0.9f,
        .confidence_threshold = 0.7f
    },
    
    // Configure fixed-wing mixer  
    .mixer_config = {
        .use_elevons = true,
        .enable_airspeed_scaling = true,
        .nominal_airspeed = 15.0f,
        .enable_rudder_coordination = true,
        .enable_wind_compensation = true
    },
    
    // Safety parameters
    .max_attitude_error = 45.0f,
    .radio_timeout_ms = 1000.0f,
    .enable_debug_output = true,
    .debug_output_rate_hz = 10
};

// Create hardware abstraction and flight controller
TeensyHAL hal(hw_config);
FlightController flight_controller(&hal, fc_config);

// Simple status tracking
bool system_initialized = false;
uint32_t last_heartbeat = 0;

void setup() {
    // Initialize serial for debug output
    Serial.begin(500000);
    delay(1000);  // Give time for serial to initialize
    
    Serial.println("========================================");
    Serial.println("Adaptive Flight Controller v2.0");
    Serial.println("Modular Architecture - Clean & Testable");
    Serial.println("========================================");
    
    // Initialize the flight controller
    Serial.println("Initializing flight controller...");
    
    if (flight_controller.initialize()) {
        system_initialized = true;
        Serial.println("✓ Flight controller initialized successfully!");
        Serial.println("✓ Ready for flight operations");
        
        // Print system configuration
        Serial.println("\nSystem Configuration:");
        Serial.print("- Control Loop: "); Serial.print(fc_config.loop_frequency_hz); Serial.println(" Hz");
        Serial.print("- Control Mode: "); Serial.println("STABILIZE");
        Serial.print("- Adaptive PID: "); Serial.println("ENABLED");
        Serial.print("- Airspeed EKF: "); Serial.println("ENABLED"); 
        Serial.print("- Wind Rejection: "); Serial.println("ENABLED");
        Serial.print("- Debug Output: "); Serial.println("ENABLED");
        Serial.println();
        
    } else {
        Serial.println("✗ Flight controller initialization FAILED!");
        Serial.println("✗ Check hardware connections and try again");
        
        // Blink error pattern
        while (true) {
            digitalWrite(13, HIGH);
            delay(100);
            digitalWrite(13, LOW);
            delay(100);
        }
    }
    
    Serial.println("Entering main control loop...");
    Serial.println("========================================");
}

void loop() {
    if (!system_initialized) {
        // Safety: do nothing if not initialized
        delay(100);
        return;
    }
    
    // This is the entire control loop!
    // All the complexity is handled inside the flight controller modules
    flight_controller.update();
    
    // Simple heartbeat indicator
    uint32_t now = millis();
    if (now - last_heartbeat > 1000) {
        last_heartbeat = now;
        
        // Print basic status every second
        auto status = flight_controller.getStatus();
        auto stats = flight_controller.getPerformanceStats();
        auto diag = flight_controller.getDiagnosticInfo();
        
        Serial.print("Status: ");
        switch (status) {
            case FlightController::Status::READY: Serial.print("READY"); break;
            case FlightController::Status::ARMED: Serial.print("ARMED"); break;
            case FlightController::Status::FLYING: Serial.print("FLYING"); break;
            case FlightController::Status::EMERGENCY: Serial.print("EMERGENCY"); break;
            default: Serial.print("UNKNOWN"); break;
        }
        
        Serial.print(" | Loop: ");
        Serial.print(stats.loop_time_avg_ms, 2);
        Serial.print("ms | CPU: ");
        Serial.print(stats.cpu_usage_percent, 1);
        Serial.print("% | IMU: ");
        Serial.print(diag.imu_healthy ? "OK" : "FAIL");
        Serial.print(" | Radio: ");
        Serial.print(diag.radio_healthy ? "OK" : "FAIL");
        Serial.print(" | Learning: ");
        Serial.println(diag.adaptive_pid_learning ? "ACTIVE" : "IDLE");
        
        // Blink status LED
        digitalWrite(13, !digitalRead(13));
    }
    
    // Safety: Emergency stop on any critical error
    if (flight_controller.getStatus() == FlightController::Status::ERROR) {
        Serial.println("CRITICAL ERROR - EMERGENCY STOP ACTIVATED");
        flight_controller.emergencyStop();
        
        // Halt execution with error blink pattern
        while (true) {
            for (int i = 0; i < 3; i++) {
                digitalWrite(13, HIGH); delay(100);
                digitalWrite(13, LOW); delay(100);
            }
            delay(500);
        }
    }
}

/**
 * That's it! The entire main sketch is ~150 lines including comments.
 * 
 * Compare this to the previous 5000+ line monolith:
 * 
 * BEFORE (Monolithic):
 * - 5000+ lines in one file
 * - Global variables everywhere  
 * - Impossible to test individual components
 * - Slow compilation
 * - Hard to debug
 * 
 * AFTER (Modular):
 * - ~150 lines in main sketch
 * - Clean module interfaces
 * - Each algorithm easily unit tested
 * - Fast compilation of individual modules
 * - Easy to debug and maintain
 * 
 * The complex adaptive PID, wind rejection, and airspeed estimation
 * algorithms are all still there - they're just properly organized
 * into testable, maintainable modules!
 */ 