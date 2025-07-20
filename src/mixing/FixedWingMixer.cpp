#include "FixedWingMixer.h"
#include <cmath>
#include <algorithm>

#ifdef _WIN32
#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#endif

// Constructor
FixedWingMixer::FixedWingMixer(HAL* hal, const Config& config) 
    : hal_(hal), config_(config), initialized_(false), throttle_passthrough_(false), emergency_mode_(false),
      current_controls_{0.0f, 0.0f, 0.0f, 0.0f},  // Initialize in initializer list
      wind_compensation_{0.0f, 0.0f, 0.0f, 0.0f, 0.0f} {  // Initialize in initializer list
    
    // Initialize servo channels
    left_elevon_channel_ = 0;
    right_elevon_channel_ = 1;
    rudder_channel_ = 2;
    throttle_channel_ = 0;
    
    // Initialize trim positions
    left_elevon_trim_ = 0.0f;
    right_elevon_trim_ = 0.0f;
    rudder_trim_ = 0.0f;
    
    // Initialize rate limiting history
    history_index_ = 0;
    for (int i = 0; i < RATE_LIMIT_SAMPLES; i++) {
        left_elevon_history_[i] = 0.0f;
        right_elevon_history_[i] = 0.0f;
        rudder_history_[i] = 0.0f;
    }
    
    last_update_time_ = 0;
}

// Initialize the mixer
bool FixedWingMixer::initialize() {
    if (!hal_) {
        return false;
    }
    
    // Initialize servo outputs
    if (!hal_->initServos()) {
        return false;
    }
    
    // Initialize motor outputs  
    if (!hal_->initMotors()) {
        return false;
    }
    
    // Set all controls to neutral/safe positions
    emergencyStop();
    
    initialized_ = true;
    last_update_time_ = hal_->millis();
    
    return true;
}

// Main mixing function
ControlSurfaces FixedWingMixer::mix(const AngularRates& pid_outputs, 
                                   const RadioInputs& radio_inputs,
                                   float airspeed,
                                   const WindCompensation& wind_comp) {
    
    if (!initialized_) {
        return ControlSurfaces();
    }
    
    uint32_t current_time = hal_->millis();
    float dt = (current_time - last_update_time_) / 1000.0f;
    last_update_time_ = current_time;
    
    // Store wind compensation for later use
    wind_compensation_ = wind_comp;
    
    ControlSurfaces controls = mixControls(pid_outputs, radio_inputs, airspeed);
    
    // Apply wind compensation
    applyWindCompensation(controls, airspeed);
    
    // Apply airspeed scaling
    applyAirspeedScaling(controls, airspeed);
    
    // Apply rudder coordination
    applyRudderCoordination(controls, pid_outputs, airspeed);
    
    // Apply safety limits
    applySafetyLimits(controls);
    
    // Apply rate limiting
    applyRateLimiting(controls, dt);
    
    // Store current controls
    current_controls_ = controls;
    
    return controls;
}

// Apply controls to hardware
void FixedWingMixer::applyControls(const ControlSurfaces& controls) {
    if (!hal_ || !initialized_ || emergency_mode_) {
        return;
    }
    
    // Convert control surface commands to servo positions
    // Assuming servo range is 0.0 to 1.0, centered at 0.5
    
    // Left elevon
    float left_elevon_pos = 0.5f + (controls.left_elevon * 0.5f);
    writeServoPosition(left_elevon_channel_, left_elevon_pos);
    
    // Right elevon
    float right_elevon_pos = 0.5f + (controls.right_elevon * 0.5f);
    writeServoPosition(right_elevon_channel_, right_elevon_pos);
    
    // Rudder
    float rudder_pos = 0.5f + (controls.rudder * 0.5f);
    writeServoPosition(rudder_channel_, rudder_pos);
    
    // Throttle
    writeThrottlePosition(throttle_channel_, controls.throttle);
}

// Emergency stop - set all controls to safe positions
void FixedWingMixer::emergencyStop() {
    if (!hal_) {
        return;
    }
    
    emergency_mode_ = true;
    
    // Set all servos to neutral (0.5 normalized position)
    for (int i = 0; i < 8; i++) { // Assuming max 8 servo channels
        writeServoPosition(i, 0.5f);
    }
    
    // Set all motors to minimum (0.0 throttle)
    for (int i = 0; i < 6; i++) { // Assuming max 6 motor channels
        writeThrottlePosition(i, 0.0f);
    }
    
    // Reset current controls
    current_controls_ = ControlSurfaces();
}

// Static elevon mixing function
ControlSurfaces FixedWingMixer::mixElevons(float elevator_cmd, float aileron_cmd, float rudder_cmd, float throttle_cmd) {
    ControlSurfaces controls;
    
    // Standard elevon mixing
    controls.left_elevon = elevator_cmd + aileron_cmd;   // Left elevon: elevator + aileron
    controls.right_elevon = elevator_cmd - aileron_cmd;  // Right elevon: elevator - aileron
    controls.rudder = rudder_cmd;
    controls.throttle = throttle_cmd;
    
    return controls;
}

// Scale control authority based on airspeed
float FixedWingMixer::scaleControlAuthority(float command, float airspeed, float nominal_airspeed, 
                                           float min_scaling, float max_scaling) {
    if (airspeed <= 0.001f || nominal_airspeed <= 0.001f) {
        return command; // Avoid division by zero
    }
    
    // Scale control authority inversely with airspeed
    // Higher airspeed = less control deflection needed
    float scaling_factor = nominal_airspeed / airspeed;
    
    // Apply limits
    scaling_factor = std::max(min_scaling, std::min(max_scaling, scaling_factor));
    
    return command * scaling_factor;
}

// Calculate coordination rudder command
float FixedWingMixer::calculateCoordination(float roll_rate, float yaw_rate, float airspeed) {
    // Simplified coordinated turn calculation
    // In a coordinated turn, yaw rate should be proportional to roll rate
    float desired_yaw_rate = roll_rate * 0.1f; // Simplified relationship
    return (desired_yaw_rate - yaw_rate) * 0.5f;
}

// ========== PRIVATE METHOD IMPLEMENTATIONS ==========

// Mix controls
ControlSurfaces FixedWingMixer::mixControls(const AngularRates& pid_outputs, const RadioInputs& radio_inputs, float airspeed) {
    ControlSurfaces controls;
    
    // Convert PID angular rate outputs to control surface commands
    float elevator_command = pid_outputs.pitch_rate * config_.elevator_mix_ratio;
    float aileron_command = pid_outputs.roll_rate * config_.aileron_mix_ratio;
    float rudder_command = pid_outputs.yaw_rate * config_.rudder_mix_ratio;
    
    // Mix to elevons or separate surfaces
    if (config_.use_elevons) {
        controls.left_elevon = elevator_command + aileron_command;
        controls.right_elevon = elevator_command - aileron_command;
        
        // Apply elevon direction reversals
        if (config_.reverse_left_elevon) {
            controls.left_elevon = -controls.left_elevon;
        }
        if (config_.reverse_right_elevon) {
            controls.right_elevon = -controls.right_elevon;
        }
    } else {
        // Separate elevator and aileron (simplified)
        controls.left_elevon = elevator_command;
        controls.right_elevon = aileron_command;
    }
    
    // Handle rudder
    controls.rudder = rudder_command;
    if (config_.reverse_rudder) {
        controls.rudder = -controls.rudder;
    }
    
    // Handle throttle
    if (throttle_passthrough_) {
        controls.throttle = radio_inputs.throttle;
    } else {
        controls.throttle = radio_inputs.throttle; // Could be modified by auto-throttle
    }
    
    return controls;
}

// Apply wind compensation
void FixedWingMixer::applyWindCompensation(ControlSurfaces& controls, float airspeed) {
    if (!config_.enable_wind_compensation) {
        return;
    }
    
    // Apply feedforward compensation for wind disturbances
    controls.left_elevon += wind_compensation_.pitch_compensation * wind_compensation_.confidence;
    controls.right_elevon += wind_compensation_.pitch_compensation * wind_compensation_.confidence;
    controls.left_elevon += wind_compensation_.roll_compensation * wind_compensation_.confidence;
    controls.right_elevon -= wind_compensation_.roll_compensation * wind_compensation_.confidence;
    controls.rudder += wind_compensation_.yaw_compensation * wind_compensation_.confidence;
    controls.throttle += wind_compensation_.throttle_compensation * wind_compensation_.confidence;
}

// Apply airspeed scaling
void FixedWingMixer::applyAirspeedScaling(ControlSurfaces& controls, float airspeed) {
    if (!config_.enable_airspeed_scaling) {
        return;
    }
    
    float scaling_factor = config_.nominal_airspeed / std::max(airspeed, 1.0f);
    scaling_factor = std::max(config_.min_airspeed_scaling, 
                             std::min(config_.max_airspeed_scaling, scaling_factor));
    
    controls.left_elevon *= scaling_factor;
    controls.right_elevon *= scaling_factor;
    controls.rudder *= scaling_factor;
}

// Apply rudder coordination
void FixedWingMixer::applyRudderCoordination(ControlSurfaces& controls, const AngularRates& rates, float airspeed) {
    if (!config_.enable_rudder_coordination) {
        return;
    }
    
    float coordination_rudder = calculateCoordination(rates.roll_rate, rates.yaw_rate, airspeed);
    controls.rudder += coordination_rudder * config_.coordination_gain;
}

// Apply safety limits
void FixedWingMixer::applySafetyLimits(ControlSurfaces& controls) {
    // Limit elevon deflections
    controls.left_elevon = std::max(-config_.max_elevon_deflection, 
                                   std::min(config_.max_elevon_deflection, controls.left_elevon));
    controls.right_elevon = std::max(-config_.max_elevon_deflection, 
                                    std::min(config_.max_elevon_deflection, controls.right_elevon));
    
    // Limit rudder deflection
    controls.rudder = std::max(-config_.max_rudder_deflection, 
                              std::min(config_.max_rudder_deflection, controls.rudder));
    
    // Limit throttle
    controls.throttle = std::max(config_.min_throttle, 
                                std::min(config_.max_throttle, controls.throttle));
}

// Apply rate limiting
void FixedWingMixer::applyRateLimiting(ControlSurfaces& controls, float dt) {
    if (dt <= 0.0f) return;
    
    float max_rate = 1.0f; // Maximum rate of change per second
    
    controls.left_elevon = rateLimitControl(controls.left_elevon, left_elevon_history_, RATE_LIMIT_SAMPLES, max_rate, dt);
    controls.right_elevon = rateLimitControl(controls.right_elevon, right_elevon_history_, RATE_LIMIT_SAMPLES, max_rate, dt);
    controls.rudder = rateLimitControl(controls.rudder, rudder_history_, RATE_LIMIT_SAMPLES, max_rate, dt);
    
    // Update history
    updateControlHistory(controls.left_elevon, left_elevon_history_);
    updateControlHistory(controls.right_elevon, right_elevon_history_);
    updateControlHistory(controls.rudder, rudder_history_);
    
    history_index_ = (history_index_ + 1) % RATE_LIMIT_SAMPLES;
}

// Rate limit a single control
float FixedWingMixer::rateLimitControl(float new_value, const float* history, int samples, float max_rate, float dt) {
    if (samples <= 0) return new_value;
    
    float previous_value = history[(history_index_ - 1 + samples) % samples];
    float max_change = max_rate * dt;
    float change = new_value - previous_value;
    
    if (change > max_change) {
        return previous_value + max_change;
    } else if (change < -max_change) {
        return previous_value - max_change;
    }
    
    return new_value;
}

// Update control history
void FixedWingMixer::updateControlHistory(float value, float* history) {
    history[history_index_] = value;
}

// Write servo position
void FixedWingMixer::writeServoPosition(int channel, float position_normalized) {
    if (!hal_) return;
    
    // Clamp to valid range
    position_normalized = std::max(0.0f, std::min(1.0f, position_normalized));
    
    hal_->writeServo(channel, position_normalized);
}

// Write throttle position
void FixedWingMixer::writeThrottlePosition(int channel, float throttle_normalized) {
    if (!hal_) return;
    
    // Clamp to valid range
    throttle_normalized = std::max(0.0f, std::min(1.0f, throttle_normalized));
    
    hal_->writeMotor(channel, throttle_normalized);
} 