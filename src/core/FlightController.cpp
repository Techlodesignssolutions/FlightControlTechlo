// ---------- FlightController.cpp ----------
#include "FlightController.h"
#include <cstdio>
#include <cmath>
#include <algorithm>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif

// Constructor
FlightController::FlightController(HAL* hal, const Config& config)
    : hal_(hal)
    , config_(config)
    , status_(Status::INITIALIZING)
    , control_mode_(config.default_mode)
    , state_estimator_(hal_, config.state_estimator_config)
    , adaptive_pid_(config.adaptive_pid_config)
    , mixer_(hal_, config.mixer_config)
    , loop_start_time_(0)
    , last_loop_time_(0)
    , loop_dt_(0.0f)
    , pending_phase_(FlightPhase::UNKNOWN)
    , phase_change_start_time_(0)
{
    performance_stats_ = PerformanceStats{0.0f, 0.0f, 0, 0, 0.0f};
    for (int i = 0; i < TIMING_SAMPLES; ++i) loop_time_history_[i] = 0.0f;
}

bool FlightController::initialize() {
    if (!hal_) return false;
    if (!hal_->initIMU() || !hal_->initRadio() || !hal_->initServos() || !hal_->initMotors()) {
        status_ = Status::ERROR;
        return false;
    }
    if (!state_estimator_.initialize() || !adaptive_pid_.initialize() || !mixer_.initialize()) {
        status_ = Status::ERROR;
        return false;
    }
    status_ = Status::READY;
    last_loop_time_ = hal_->micros();
    return true;
}

void FlightController::update() {
    loop_start_time_ = hal_->micros();
    std::uint32_t dt_us = loop_start_time_ - last_loop_time_;
    loop_dt_ = dt_us / 1e6f;
    if (loop_dt_ < 0.001f) return;
    last_loop_time_ = loop_start_time_;

    if (!readRadioInputs() || !updateStateEstimation() || !runControlLoops() || !mixAndOutputControls())
        return;

    updateSafetyMonitoring();
    updatePerformanceStats();
    outputDebugInfo();
    ++performance_stats_.loop_count;
}

AircraftState FlightController::getAircraftState() const {
    return aircraft_state_;
}

bool FlightController::setControlMode(ControlMode mode) {
    if (status_ != Status::READY && status_ != Status::ARMED && status_ != Status::FLYING)
        return false;
    control_mode_ = mode;
    return true;
}

void FlightController::emergencyStop() {
    mixer_.emergencyStop();
    status_ = Status::EMERGENCY;
    armed_ = false; // Disarm immediately
    adaptive_pid_.freezeLearning(true); // Stop learning during emergency
}

bool FlightController::arm() {
    // Safety checks before arming
    if (status_ != Status::READY) {
        return false; // Can only arm from READY state
    }
    
    if (!checkSystemHealth()) {
        return false; // System health check failed
    }
    
    if (battery_voltage_ < config_.low_battery_voltage + 0.5f) {
        return false; // Battery too low to arm
    }
    
    if (!isRadioSignalValid()) {
        return false; // No valid radio signal
    }
    
    // Check that throttle is at minimum
    if (radio_inputs_.throttle > 0.1f) {
        return false; // Throttle must be low to arm
    }
    
    // Arm the system
    armed_ = true;
    status_ = Status::ARMED;
    mixer_.clearEmergency(); // Allow normal control
    
    if (hal_) {
        hal_->serialPrintln("ARMED - Flight controller ready");
    }
    
    return true;
}

bool FlightController::disarm() {
    if (status_ == Status::FLYING) {
        return false; // Cannot disarm while flying
    }
    
    armed_ = false;
    status_ = Status::READY;
    mixer_.emergencyStop(); // Cut motors immediately
    
    if (hal_) {
        hal_->serialPrintln("DISARMED - Motors stopped");
    }
    
    return true;
}

DiagnosticInfo FlightController::getDiagnosticInfo() const {
    DiagnosticInfo info;
    info.imu_healthy                  = true;
    info.radio_healthy                = hal_->isRadioConnected();
    info.airspeed_converged           = state_estimator_.getAirspeedState().converged;
    info.adaptive_pid_learning        = adaptive_pid_.getLearningState().is_learning;
    info.wind_compensation_confidence = state_estimator_.getWindState().confidence.x;
    info.uptime_ms                    = hal_->millis();
    return info;
}

bool FlightController::readRadioInputs() {
    float channels[6];
    if (!hal_->readRadio(channels, 6)) return false;
    radio_inputs_.throttle = std::clamp(channels[0], 0.0f, 1.0f);
    radio_inputs_.roll     = std::clamp(channels[1], -1.0f, 1.0f);
    radio_inputs_.pitch    = std::clamp(channels[2], -1.0f, 1.0f);
    radio_inputs_.yaw      = std::clamp(channels[3], -1.0f, 1.0f);
    radio_inputs_.aux1     = std::clamp(channels[4], -1.0f, 1.0f);
    radio_inputs_.aux2     = std::clamp(channels[5], -1.0f, 1.0f);
    radio_inputs_.last_update_time = hal_->millis();
    last_radio_update_ = hal_->millis();
    return true;
}

bool FlightController::updateStateEstimation() {
    state_estimator_.setThrottleCommand(radio_inputs_.throttle);
    state_estimator_.update(loop_dt_);
    aircraft_state_ = state_estimator_.getState();
    return true;
}

bool FlightController::runControlLoops() {
    switch (control_mode_) {
        case ControlMode::MANUAL:    runManualMode();    break;
        case ControlMode::STABILIZE: runStabilizeMode(); break;
        case ControlMode::ALTITUDE:  runAltitudeMode();  break;
        case ControlMode::POSITION:  runPositionMode();  break;
        case ControlMode::AUTO:      runAutoMode();      break;
    }
    return true;
}

bool FlightController::mixAndOutputControls() {
    FixedWingMixer::WindCompensation wind_comp;
    auto& w = aircraft_state_.wind_disturbance;
    
    // Map our wind disturbance to mixer's wind compensation format
    wind_comp.pitch_compensation    = w.x;  // headwind affects pitch
    wind_comp.roll_compensation     = w.y;  // crosswind affects roll
    wind_comp.yaw_compensation      = w.z * 0.1f;  // vertical wind can affect yaw slightly
    wind_comp.throttle_compensation = w.x * 0.2f;  // headwind affects throttle requirement
    wind_comp.confidence            = 1.0f;
    
    // Safety check: only allow motor output if armed
    RadioInputs safe_radio_inputs = radio_inputs_;
    if (!armed_ || status_ == Status::EMERGENCY) {
        safe_radio_inputs.throttle = 0.0f; // Force throttle to zero if not armed
    }
    
    control_outputs_ = mixer_.mix(control_commands_, safe_radio_inputs, aircraft_state_.airspeed, wind_comp);
    mixer_.applyControls(control_outputs_);
    
    // Update flight status based on throttle
    if (armed_ && safe_radio_inputs.throttle > 0.1f) {
        if (status_ == Status::ARMED) {
            status_ = Status::FLYING;
        }
    } else if (status_ == Status::FLYING && safe_radio_inputs.throttle < 0.05f) {
        status_ = Status::ARMED; // Back to armed but not flying
    }
    
    return true;
}

void FlightController::updateSafetyMonitoring() {
    battery_voltage_ = readBatteryVoltage();
    if (!isRadioSignalValid())      handleRadioTimeout();
    if (battery_voltage_ < config_.low_battery_voltage) handleBatteryLow();
}

void FlightController::updatePerformanceStats() {
    float ms = (hal_->micros() - loop_start_time_) / 1e3f;
    updateLoopTimingStats(ms);
    if (ms > config_.max_loop_time_ms)
        ++performance_stats_.overrun_count;
    // avoid std::max macro collision
    performance_stats_.loop_time_max_ms =
        (performance_stats_.loop_time_max_ms > ms ? performance_stats_.loop_time_max_ms : ms);
    performance_stats_.cpu_usage_percent = (ms / (1000.0f / config_.loop_frequency_hz)) * 100.0f;
}

void FlightController::outputDebugInfo() {
    if (!config_.enable_debug_output) return;
    std::uint32_t now = hal_->millis();
    if (now - last_debug_output_ < (1000 / config_.debug_output_rate_hz)) return;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Status: %d, Loop: %.2fms",
                  static_cast<int>(status_), performance_stats_.loop_time_avg_ms);
    hal_->serialPrintln(buf);
    last_debug_output_ = now;
}

bool FlightController::checkSystemHealth() {
    // IMU health check
    if (!hal_->isIMUHealthy()) {
        if (hal_) hal_->serialPrintln("HEALTH: IMU failure");
        return false;
    }
    
    // Radio connection check
    if (!hal_->isRadioConnected()) {
        if (hal_) hal_->serialPrintln("HEALTH: Radio disconnected");
        return false;
    }
    
    // State estimator convergence check
    if (!state_estimator_.isConverged()) {
        if (hal_) hal_->serialPrintln("HEALTH: State estimator not converged");
        return false;
    }
    
    // Performance check - don't arm if adaptive PID is performing poorly
    auto learning_state = adaptive_pid_.getLearningState();
    if (learning_state.confidence < 0.3f) {
        if (hal_) hal_->serialPrintln("HEALTH: Low control confidence");
        return false;
    }
    
    // Check for recent emergency conditions
    if (isInEmergencyCondition()) {
        if (hal_) hal_->serialPrintln("HEALTH: Emergency condition active");
        return false;
    }
    
    return true;
}

void FlightController::handleRadioTimeout() {
    if (!radio_timeout_active_) {
        radio_timeout_start_   = hal_->millis();
        radio_timeout_active_  = true;
    }
    if (hal_->millis() - radio_timeout_start_ > config_.radio_timeout_ms)
        emergencyStop();
}

void FlightController::handleBatteryLow() {
    // Set emergency mode immediately
    status_ = Status::EMERGENCY;
    
    // Throttle to idle to prevent further battery drain
    ControlSurfaces emergency_controls;
    emergency_controls.throttle = config_.mixer_config.idle_throttle;
    emergency_controls.left_elevon = 0.0f;   // Level wings
    emergency_controls.right_elevon = 0.0f;  // Level wings
    emergency_controls.rudder = 0.0f;        // Center rudder
    
    // Force mixer to emergency mode and apply safe controls
    mixer_.emergencyStop();
    mixer_.applyControls(emergency_controls);
    
    // Log the emergency
    if (hal_) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "BATTERY LOW: %.2fV - EMERGENCY LANDING", battery_voltage_);
        hal_->serialPrintln(msg);
    }
    
    // Freeze adaptive learning during emergency
    adaptive_pid_.freezeLearning(true);
}

void FlightController::handleSystemError(const char* error_message) {
    status_ = Status::ERROR;
    if (hal_) {
        hal_->serialPrint("System Error: ");
        hal_->serialPrintln(error_message);
    }
}

bool FlightController::isInEmergencyCondition() const {
    return status_ == Status::EMERGENCY || status_ == Status::ERROR;
}

void FlightController::runManualMode() {
    // Manual mode: direct passthrough from radio to control commands
    // No PID control, just scale radio inputs to rate commands
    control_commands_.roll_rate  = radio_inputs_.roll * 180.0f;   // Max 180 deg/s
    control_commands_.pitch_rate = radio_inputs_.pitch * 90.0f;   // Max 90 deg/s  
    control_commands_.yaw_rate   = radio_inputs_.yaw * 90.0f;     // Max 90 deg/s
}

void FlightController::runStabilizeMode() {
    calculateFlightRegime();
    adaptive_pid_.updateFlightRegime(current_regime_);

    // Compute attitude error
    Attitude tgt{
        radio_inputs_.roll  * 45.0f,
        radio_inputs_.pitch * 30.0f,
        radio_inputs_.yaw   * 180.0f
    };
    Attitude err{
        tgt.roll  - aircraft_state_.attitude.roll,
        tgt.pitch - aircraft_state_.attitude.pitch,
        tgt.yaw   - aircraft_state_.attitude.yaw
    };

    ControlGains gains = adaptive_pid_.getGains(current_regime_);
    // Assign explicitly to avoid initializer issues
    control_commands_.roll_rate  = gains.roll.kp  * err.roll;
    control_commands_.pitch_rate = gains.pitch.kp * err.pitch;
    control_commands_.yaw_rate   = gains.yaw.kp   * err.yaw;

    adaptive_pid_.updatePerformance(err, control_commands_, loop_dt_);
}

void FlightController::runAltitudeMode() {
    // TODO: Implement altitude hold mode
    // For now, fall back to stabilize mode with altitude feedback
    runStabilizeMode();
}

void FlightController::runPositionMode() {
    // TODO: Implement position hold mode (requires GPS)
    // For now, fall back to altitude mode
    runAltitudeMode();
}

void FlightController::runAutoMode() {
    // TODO: Implement autonomous waypoint navigation
    // For now, fall back to position hold mode
    runPositionMode();
}

float FlightController::calculateLoopTime() const { return loop_dt_ * 1000.0f; }

void FlightController::updateLoopTimingStats(float ms) {
    if (performance_stats_.loop_count == 0) {
        performance_stats_.loop_time_avg_ms = ms;
        performance_stats_.loop_time_max_ms = ms;
    } else {
        const float alpha = 0.1f;
        performance_stats_.loop_time_avg_ms = alpha * ms + (1 - alpha) * performance_stats_.loop_time_avg_ms;
        performance_stats_.loop_time_max_ms =
            (performance_stats_.loop_time_max_ms > ms ? performance_stats_.loop_time_max_ms : ms);
    }
    ++performance_stats_.loop_count;
    loop_time_history_[timing_index_] = ms;
    timing_index_ = (timing_index_ + 1) % TIMING_SAMPLES;
}

void FlightController::calculateFlightRegime() {
    current_regime_.throttle_fraction = radio_inputs_.throttle;
    current_regime_.airspeed         = aircraft_state_.airspeed;
    current_regime_.angle_of_attack  = aircraft_state_.attitude.pitch;
    
    // Flight phase state machine with hysteresis
    FlightPhase new_phase = determineFlightPhase();
    
    // Apply hysteresis to prevent rapid phase changes
    if (new_phase != current_regime_.phase) {
        if (new_phase != pending_phase_) {
            // Starting transition to new phase
            pending_phase_ = new_phase;
            phase_change_start_time_ = hal_->millis();
        } else {
            // Check if we've held new phase long enough
            if (hal_->millis() - phase_change_start_time_ > PHASE_HYSTERESIS_MS) {
                current_regime_.phase = new_phase;
            }
        }
    } else {
        // Reset pending phase if we're staying in current phase
        pending_phase_ = current_regime_.phase;
    }
}

FlightPhase FlightController::determineFlightPhase() {
    float throttle = radio_inputs_.throttle;
    float airspeed = aircraft_state_.airspeed;
    float vertical_rate = aircraft_state_.rates.pitch_rate; // Simplified - would use proper vertical speed
    
    // Configuration thresholds
    constexpr float PREFLIGHT_THROTTLE_MAX = 0.05f;
    constexpr float TAKEOFF_AIRSPEED_MIN = 3.0f;
    constexpr float CRUISE_AIRSPEED_MIN = 12.0f;
    constexpr float CLIMB_RATE_MIN = 2.0f;        // deg/s pitch rate indicating climb
    constexpr float DESCENT_RATE_MAX = -1.0f;    // deg/s pitch rate indicating descent
    constexpr float LANDING_THROTTLE_MAX = 0.15f;
    constexpr float LANDING_AIRSPEED_MAX = 8.0f;
    
    // State machine logic
    switch (current_regime_.phase) {
        case FlightPhase::UNKNOWN:
        case FlightPhase::PREFLIGHT:
            if (throttle > PREFLIGHT_THROTTLE_MAX && airspeed > TAKEOFF_AIRSPEED_MIN) {
                return FlightPhase::TAKEOFF;
            }
            return FlightPhase::PREFLIGHT;
            
        case FlightPhase::TAKEOFF:
            if (airspeed > CRUISE_AIRSPEED_MIN && vertical_rate < CLIMB_RATE_MIN) {
                return FlightPhase::CRUISE;
            }
            if (vertical_rate > CLIMB_RATE_MIN) {
                return FlightPhase::CLIMB;
            }
            return FlightPhase::TAKEOFF;
            
        case FlightPhase::CLIMB:
            if (vertical_rate < CLIMB_RATE_MIN && airspeed > CRUISE_AIRSPEED_MIN) {
                return FlightPhase::CRUISE;
            }
            if (throttle < LANDING_THROTTLE_MAX && vertical_rate < DESCENT_RATE_MAX) {
                return FlightPhase::DESCENT;
            }
            return FlightPhase::CLIMB;
            
        case FlightPhase::CRUISE:
            if (vertical_rate > CLIMB_RATE_MIN) {
                return FlightPhase::CLIMB;
            }
            if (throttle < LANDING_THROTTLE_MAX && vertical_rate < DESCENT_RATE_MAX) {
                return FlightPhase::DESCENT;
            }
            return FlightPhase::CRUISE;
            
        case FlightPhase::DESCENT:
            if (airspeed < LANDING_AIRSPEED_MAX && throttle < LANDING_THROTTLE_MAX) {
                return FlightPhase::APPROACH;
            }
            if (vertical_rate > CLIMB_RATE_MIN) {
                return FlightPhase::CLIMB;
            }
            return FlightPhase::DESCENT;
            
        case FlightPhase::APPROACH:
            if (airspeed < TAKEOFF_AIRSPEED_MIN && throttle < PREFLIGHT_THROTTLE_MAX) {
                return FlightPhase::LANDING;
            }
            if (throttle > LANDING_THROTTLE_MAX || vertical_rate > 0) {
                return FlightPhase::CLIMB;
            }
            return FlightPhase::APPROACH;
            
        case FlightPhase::LANDING:
            if (airspeed < TAKEOFF_AIRSPEED_MIN && throttle < PREFLIGHT_THROTTLE_MAX) {
                return FlightPhase::PREFLIGHT;
            }
            if (throttle > PREFLIGHT_THROTTLE_MAX) {
                return FlightPhase::TAKEOFF;
            }
            return FlightPhase::LANDING;
            
        case FlightPhase::EMERGENCY:
        default:
            return FlightPhase::EMERGENCY;
    }
}

bool FlightController::isRadioSignalValid() const {
    return (hal_->millis() - last_radio_update_) < config_.radio_timeout_ms;
}

float FlightController::readBatteryVoltage() const { return 11.1f; }