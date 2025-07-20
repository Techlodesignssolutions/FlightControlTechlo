#pragma once
#include "HAL.h"
#include "Types.h"
#include "../estimation/StateEstimator.h"
#include "../control/AdaptivePID.h"
#include "../mixing/FixedWingMixer.h"
#include <stdint.h>

/**
 * Main Flight Controller Class
 * 
 * This is the top-level coordinator that brings together all subsystems:
 * - State estimation (attitude, airspeed)
 * - Adaptive PID control with gain scheduling
 * - Wind disturbance rejection
 * - Fixed-wing control mixing
 * - Safety monitoring and failsafes
 * 
 * Design principles:
 * - Clean module interfaces
 * - Testable components
 * - Configurable behavior
 * - Robust error handling
 */
class FlightController {
public:
    /**
     * Overall flight controller configuration
     */
    struct Config {
        // System configuration
        uint32_t loop_frequency_hz = 500;      // Control loop frequency
        ControlMode default_mode = ControlMode::STABILIZE;
        
        // Module configurations
        StateEstimator::Config state_estimator_config;
        AdaptivePID::Config adaptive_pid_config;
        FixedWingMixer::Config mixer_config;
        
        // Safety parameters
        float max_attitude_error = 45.0f;      // Maximum attitude error before emergency (degrees)
        float radio_timeout_ms = 1000.0f;      // Radio signal timeout (ms)
        float low_battery_voltage = 10.5f;     // Low battery warning threshold (V)
        
        // Control loop timing
        float max_loop_time_ms = 3.0f;         // Maximum acceptable loop time (ms)
        bool enable_loop_timing_monitoring = true;
        
        // Debug and logging
        bool enable_debug_output = true;
        uint32_t debug_output_rate_hz = 10;    // Debug output frequency
    };
    
    /**
     * Flight controller status
     */
    enum class Status {
        INITIALIZING,    // System starting up
        READY,          // Ready for flight
        ARMED,          // Armed and ready for takeoff
        FLYING,         // Actively flying
        EMERGENCY,      // Emergency mode - safe landing
        ERROR           // System error - land immediately
    };
    
    /**
     * Constructor
     */
    FlightController(HAL* hal, const Config& config = Config());
    
    /**
     * Initialize the flight controller
     * Returns true if initialization successful
     */
    bool initialize();
    
    /**
     * Main control loop update
     * Call this at the configured loop frequency
     */
    void update();
    
    /**
     * Get current system status
     */
    Status getStatus() const { return status_; }
    
    /**
     * Get current aircraft state
     */
    AircraftState getAircraftState() const;
    
    /**
     * Get current control mode
     */
    ControlMode getControlMode() const { return control_mode_; }
    
    /**
     * Set control mode (manual, stabilize, auto, etc.)
     */
    bool setControlMode(ControlMode mode);
    
    /**
     * Emergency stop - immediately safe the aircraft
     */
    void emergencyStop();
    
    /**
     * Get performance statistics
     */
    struct PerformanceStats {
        float loop_time_avg_ms;     // Average loop time
        float loop_time_max_ms;     // Maximum loop time
        uint32_t loop_count;        // Total loop iterations
        uint32_t overrun_count;     // Number of loop overruns
        float cpu_usage_percent;    // Estimated CPU usage
    };
    PerformanceStats getPerformanceStats() const { return performance_stats_; }
    
    /**
     * Configuration and tuning
     */
    bool loadConfiguration(const void* data, size_t size);
    bool saveConfiguration(void* data, size_t* size) const;
    
    /**
     * Diagnostic information
     */
    struct DiagnosticInfo {
        bool imu_healthy;
        bool radio_healthy;
        bool airspeed_converged;
        bool adaptive_pid_learning;
        float wind_compensation_confidence;
        uint32_t uptime_ms;
    };
    DiagnosticInfo getDiagnosticInfo() const;
    
private:
    HAL* hal_;
    Config config_;
    Status status_;
    ControlMode control_mode_;
    
    // Core modules
    StateEstimator state_estimator_;
    AdaptivePID adaptive_pid_;
    FixedWingMixer mixer_;
    
    // Current system state
    AircraftState aircraft_state_;
    RadioInputs radio_inputs_;
    ControlSurfaces control_outputs_;
    
    // Control loop state
    uint32_t loop_start_time_;
    uint32_t last_loop_time_;
    float loop_dt_;
    bool armed_;
    bool emergency_mode_;
    
    // Performance monitoring
    PerformanceStats performance_stats_;
    static const int TIMING_SAMPLES = 100;
    float loop_time_history_[TIMING_SAMPLES];
    int timing_index_;
    
    // Safety monitoring
    uint32_t last_radio_update_;
    uint32_t radio_timeout_start_;
    bool radio_timeout_active_;
    float battery_voltage_;
    
    // Debug output
    uint32_t last_debug_output_;
    uint32_t debug_counter_;
    
    // Internal methods - Control Loop Steps
    bool readRadioInputs();
    bool updateStateEstimation();
    bool runControlLoops();
    bool mixAndOutputControls();
    void updateSafetyMonitoring();
    void updatePerformanceStats();
    void outputDebugInfo();
    
    // Safety and error handling
    bool checkSystemHealth();
    void handleRadioTimeout();
    void handleBatteryLow();
    void handleSystemError(const char* error_message);
    bool isInEmergencyCondition();
    
    // Control mode implementations
    void runManualMode();
    void runStabilizeMode();
    void runAltitudeMode();
    void runPositionMode();
    void runAutoMode();
    
    // Utility functions
    float calculateLoopTime();
    void updateLoopTimingStats(float loop_time_ms);
    bool isRadioSignalValid() const;
    float readBatteryVoltage();
    
    // Configuration helpers
    void setDefaultConfiguration();
    bool validateConfiguration(const Config& config) const;
}; 