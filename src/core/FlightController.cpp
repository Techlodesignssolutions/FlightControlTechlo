#include "FlightController.h"
#include <cmath>
#include <algorithm>
#include <cstdio>  // for snprintf

#ifdef _WIN32
#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#endif

// Constructor
FlightController::FlightController(HAL* hal, const Config& config) 
    : hal_(hal), config_(config), status_(Status::INITIALIZING), control_mode_(config.default_mode),
      state_estimator_(hal_, config_.state_estimator_config),
      adaptive_pid_(config_.adaptive_pid_config),
      mixer_(hal_, config_.mixer_config),
      loop_start_time_(0), last_loop_time_(0), loop_dt_(0.0f),
      armed_(false), emergency_mode_(false), timing_index_(0),
      last_radio_update_(0), radio_timeout_start_(0), radio_timeout_active_(false),
      battery_voltage_(11.1f), last_debug_output_(0), debug_counter_(0) {
    
    // Initialize performance stats
    performance_stats_ = PerformanceStats{0.0f, 0.0f, 0, 0, 0.0f};
    
    // Initialize loop timing history
    for (int i = 0; i < TIMING_SAMPLES; i++) {
        loop_time_history_[i] = 0.0f;
    }
}

// Initialize all subsystems
bool FlightController::initialize() {
    if (!hal_) {
        return false;
    }
    
    // Initialize HAL
    if (!hal_->initIMU() || !hal_->initRadio() || !hal_->initServos() || !hal_->initMotors()) {
        status_ = Status::ERROR;
        return false;
    }
    
    // Initialize modules
    if (!state_estimator_.initialize()) {
        status_ = Status::ERROR;
        return false;
    }
    
    if (!adaptive_pid_.initialize()) {
        status_ = Status::ERROR;
        return false;
    }
    
    if (!mixer_.initialize()) {
        status_ = Status::ERROR;
        return false;
    }
    
    // Mark as ready
    status_ = Status::READY;
    last_loop_time_ = hal_->micros();
    
    return true;
}

// Main control loop update
void FlightController::update() {
    loop_start_time_ = hal_->micros();
    uint32_t dt_us = loop_start_time_ - last_loop_time_;
    loop_dt_ = dt_us / 1000000.0f;
    
    // Skip if called too frequently
    if (loop_dt_ < 0.001f) return;
    
    last_loop_time_ = loop_start_time_;
    
    // Read radio inputs
    if (!readRadioInputs()) {
        return;
    }
    
    // Update state estimation
    if (!updateStateEstimation()) {
        return;
    }
    
    // Run control loops
    if (!runControlLoops()) {
        return;
    }
    
    // Mix and output controls
    if (!mixAndOutputControls()) {
        return;
    }
    
    // Update safety monitoring
    updateSafetyMonitoring();
    
    // Update performance statistics
    updatePerformanceStats();
    
    // Output debug info
    outputDebugInfo();
    
    performance_stats_.loop_count++;
}

// Get current aircraft state
AircraftState FlightController::getAircraftState() const {
    return aircraft_state_;
}

// Set control mode
bool FlightController::setControlMode(ControlMode mode) {
    if (status_ != Status::READY && status_ != Status::ARMED && status_ != Status::FLYING) {
        return false;
    }
    
    control_mode_ = mode;
    return true;
}

// Emergency stop
void FlightController::emergencyStop() {
    status_ = Status::EMERGENCY;
    emergency_mode_ = true;
    mixer_.emergencyStop();
    
    // Log emergency
    if (config_.enable_debug_output && hal_) {
        hal_->serialPrintln("EMERGENCY STOP ACTIVATED");
    }
}

// Get diagnostic info
FlightController::DiagnosticInfo FlightController::getDiagnosticInfo() const {
    DiagnosticInfo info;
    info.imu_healthy = true;  // Simplified for now
    info.radio_healthy = isRadioSignalValid();
    info.airspeed_converged = state_estimator_.getAirspeedState().converged;
    info.adaptive_pid_learning = adaptive_pid_.getLearningState().is_learning;
    info.wind_compensation_confidence = 0.0f;  // Simplified
    info.uptime_ms = hal_->millis();
    return info;
}

// ========== PRIVATE METHOD IMPLEMENTATIONS ==========

// Read radio inputs
bool FlightController::readRadioInputs() {
    // Read raw radio data
    float channels[6];
    if (!hal_->readRadio(channels, 6)) {
        return false;
    }
    
    // Map channels to radio inputs
    radio_inputs_.throttle = std::max(0.0f, std::min(1.0f, channels[0]));
    radio_inputs_.roll = std::max(-1.0f, std::min(1.0f, channels[1]));
    radio_inputs_.pitch = std::max(-1.0f, std::min(1.0f, channels[2]));
    radio_inputs_.yaw = std::max(-1.0f, std::min(1.0f, channels[3]));
    radio_inputs_.aux1 = std::max(-1.0f, std::min(1.0f, channels[4]));
    radio_inputs_.aux2 = std::max(-1.0f, std::min(1.0f, channels[5]));
    radio_inputs_.last_update_time = hal_->millis();
    radio_inputs_.armed = (radio_inputs_.aux1 > 0.5f);  // Simple arming logic
    
    last_radio_update_ = hal_->millis();
    return true;
}

// Update state estimation
bool FlightController::updateStateEstimation() {
    state_estimator_.update(loop_dt_);
    aircraft_state_ = state_estimator_.getState();
    return true;
}

// Run control loops
bool FlightController::runControlLoops() {
    switch (control_mode_) {
        case ControlMode::MANUAL:
            runManualMode();
            break;
        case ControlMode::STABILIZE:
            runStabilizeMode();
            break;
        case ControlMode::ALTITUDE:
            runAltitudeMode();
            break;
        case ControlMode::POSITION:
            runPositionMode();
            break;
        case ControlMode::AUTO:
            runAutoMode();
            break;
    }
    return true;
}

// Mix and output controls
bool FlightController::mixAndOutputControls() {
    // Get control commands from the appropriate mode
    AngularRates control_commands;
    
    // This would be set by the mode-specific control functions
    // For now, using simplified approach
    control_commands = AngularRates(0, 0, 0);
    
    // Mix controls
    control_outputs_ = mixer_.mix(control_commands, radio_inputs_, aircraft_state_.airspeed);
    
    // Apply to hardware
    mixer_.applyControls(control_outputs_);
    
    return true;
}

// Update safety monitoring
void FlightController::updateSafetyMonitoring() {
    battery_voltage_ = readBatteryVoltage();
    
    if (!isRadioSignalValid()) {
        handleRadioTimeout();
    }
    
    if (battery_voltage_ < config_.low_battery_voltage) {
        handleBatteryLow();
    }
}

// Update performance statistics
void FlightController::updatePerformanceStats() {
    uint32_t loop_time_us = hal_->micros() - loop_start_time_;
    float loop_time_ms = loop_time_us / 1000.0f;
    
    updateLoopTimingStats(loop_time_ms);
    
    // Count overruns
    if (loop_time_ms > config_.max_loop_time_ms) {
        performance_stats_.overrun_count++;
    }
    
    // Estimate CPU usage
    float target_loop_time = 1000.0f / config_.loop_frequency_hz;
    performance_stats_.cpu_usage_percent = (loop_time_ms / target_loop_time) * 100.0f;
}

// Output debug info
void FlightController::outputDebugInfo() {
    if (!config_.enable_debug_output) return;
    
    uint32_t now = hal_->millis();
    if (now - last_debug_output_ > (1000 / config_.debug_output_rate_hz)) {
        char status_buf[16];
        snprintf(status_buf, sizeof(status_buf), "%d", static_cast<int>(status_));
        
        char loop_buf[16];
        snprintf(loop_buf, sizeof(loop_buf), "%.2f", performance_stats_.loop_time_avg_ms);
        
        hal_->serialPrint("Status: ");
        hal_->serialPrint(status_buf);
        hal_->serialPrint(", Loop: ");
        hal_->serialPrint(loop_buf);
        hal_->serialPrintln("ms");
        
        last_debug_output_ = now;
    }
}

// Safety and error handling
bool FlightController::checkSystemHealth() {
    return true; // Simplified
}

void FlightController::handleRadioTimeout() {
    if (!radio_timeout_active_) {
        radio_timeout_start_ = hal_->millis();
        radio_timeout_active_ = true;
    }
    
    if (hal_->millis() - radio_timeout_start_ > config_.radio_timeout_ms) {
        emergencyStop();
    }
}

void FlightController::handleBatteryLow() {
    // Could trigger safe landing mode
}

void FlightController::handleSystemError(const char* error_message) {
    status_ = Status::ERROR;
    if (hal_) {
        hal_->serialPrint("System Error: ");
        hal_->serialPrintln(error_message);
    }
}

bool FlightController::isInEmergencyCondition() {
    return status_ == Status::EMERGENCY || status_ == Status::ERROR;
}

// Control mode implementations
void FlightController::runManualMode() {
    // Direct passthrough implementation
}

void FlightController::runStabilizeMode() {
    // Attitude stabilization implementation
}

void FlightController::runAltitudeMode() {
    // Altitude hold implementation
}

void FlightController::runPositionMode() {
    // Position hold implementation
}

void FlightController::runAutoMode() {
    // Autonomous flight implementation
}

// Utility functions
float FlightController::calculateLoopTime() {
    return loop_dt_ * 1000.0f; // Return in milliseconds
}

void FlightController::updateLoopTimingStats(float loop_time_ms) {
    // Update running averages
    if (performance_stats_.loop_count == 0) {
        performance_stats_.loop_time_avg_ms = loop_time_ms;
        performance_stats_.loop_time_max_ms = loop_time_ms;
    } else {
        // Exponential moving average
        float alpha = 0.1f;
        performance_stats_.loop_time_avg_ms = 
            alpha * loop_time_ms + (1.0f - alpha) * performance_stats_.loop_time_avg_ms;
        
        if (loop_time_ms > performance_stats_.loop_time_max_ms) {
            performance_stats_.loop_time_max_ms = loop_time_ms;
        }
    }
}

bool FlightController::isRadioSignalValid() const {
    uint32_t now = hal_->millis();
    return (now - last_radio_update_) < config_.radio_timeout_ms;
}

float FlightController::readBatteryVoltage() {
    // Simplified - would read from ADC in real implementation
    return 11.1f; // Mock voltage
}

// Configuration helpers
void FlightController::setDefaultConfiguration() {
    // Set reasonable defaults
}

bool FlightController::validateConfiguration(const Config& config) const {
    return config.loop_frequency_hz > 0 && config.loop_frequency_hz <= 2000;
} 