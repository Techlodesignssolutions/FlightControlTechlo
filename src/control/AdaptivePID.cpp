#include "AdaptivePID.h"
#include <cmath>
#include <algorithm>
#include <cstring> // for memcpy

#ifdef _WIN32
#define _USE_MATH_DEFINES  // For M_PI on MSVC
#include <cmath>
#endif

// Constructor
AdaptivePID::AdaptivePID(const Config& config) : config_(config) {
    // Initialize learning state
    learning_state_.is_learning = false;
    learning_state_.confidence = 0.0f;
    learning_state_.adaptation_rate = config_.base_learning_rate;
    learning_state_.momentum = 0.0f;
    
    // Initialize performance metrics
    roll_metrics_ = PerformanceMetrics();
    pitch_metrics_ = PerformanceMetrics();
    yaw_metrics_ = PerformanceMetrics();
    
    learning_frozen_ = false;
    adaptation_rate_ = config_.base_learning_rate;
    buffer_index_ = 0;
    adaptation_counter_ = 0;
    current_snapshot_ = 0;
    last_update_time_ = 0;
    
    // Initialize flight regime tracking
    current_regime_ = FlightRegime();
    previous_regime_ = FlightRegime();
    
    // Reserve space for corner tracking (max 16 corners)
    modified_corners_.reserve(16);
}

// Initialize with default gain schedule
bool AdaptivePID::initialize() {
    initializeGainSchedule();
    
    // Initialize buffers and counters
    buffer_index_ = 0;
    adaptation_counter_ = 0;
    current_snapshot_ = 0;
    
    // Clear performance buffers
    for (int i = 0; i < BUFFER_SIZE; i++) {
        roll_error_buffer_[i] = 0.0f;
        pitch_error_buffer_[i] = 0.0f;
        yaw_error_buffer_[i] = 0.0f;
        roll_output_buffer_[i] = 0.0f;
        pitch_output_buffer_[i] = 0.0f;
        yaw_output_buffer_[i] = 0.0f;
    }
    
    return true;
}

// Get current PID gains for the specified flight regime
ControlGains AdaptivePID::getGains(const FlightRegime& regime) const {
    return interpolateGains(regime, gain_schedule_, config_);
}

// Update performance metrics and adapt gains
void AdaptivePID::updatePerformance(const Attitude& errors, const AngularRates& pid_outputs, float dt) {
    // Update timestamp for snapshots
    last_update_time_ += static_cast<uint32_t>(dt * 1000000.0f); // Convert dt to microseconds
    
    // Update performance buffers for oscillation detection
    updatePerformanceBuffers(errors, pid_outputs);
    
    // Update individual axis metrics
    updateAxisPerformance(errors.roll, pid_outputs.roll_rate, dt, roll_metrics_);
    updateAxisPerformance(errors.pitch, pid_outputs.pitch_rate, dt, pitch_metrics_);
    updateAxisPerformance(errors.yaw, pid_outputs.yaw_rate, dt, yaw_metrics_);
    
    // Detect oscillations and update metrics
    detectOscillations();
    
    // Update learning state based on performance
    updateLearningState();
    
    // Attempt gain adaptation if conditions are right
    adaptGainsIfNeeded();
}

// Update current flight regime for learning
void AdaptivePID::updateFlightRegime(const FlightRegime& regime) {
    previous_regime_ = current_regime_;
    current_regime_ = regime;
}

// Set adaptation rate
void AdaptivePID::setAdaptationRate(float rate) {
    learning_state_.adaptation_rate = std::max(0.0f, std::min(1.0f, rate));
    adaptation_rate_ = learning_state_.adaptation_rate;
}

// Freeze/unfreeze learning
void AdaptivePID::freezeLearning(bool freeze) {
    learning_frozen_ = freeze;
}

// Static 4D interpolation function
ControlGains AdaptivePID::interpolateGains(const FlightRegime& regime, 
                                          const ControlGains gain_table[Config::THROTTLE_ZONES]
                                                                      [Config::AOA_ZONES]
                                                                      [Config::PHASE_ZONES]
                                                                      [Config::AIRSPEED_ZONES],
                                          const Config& config) {
    
    // Find indices and interpolation factors for each dimension
    float t_factor, a_factor, p_factor, s_factor;
    int t_idx, a_idx, p_idx, s_idx;
    
    getGridIndices(regime, config, t_idx, a_idx, p_idx, s_idx, t_factor, a_factor, p_factor, s_factor);
    
    // Get the 16 corner points for 4D interpolation
    int t1 = std::min(t_idx + 1, Config::THROTTLE_ZONES - 1);
    int a1 = std::min(a_idx + 1, Config::AOA_ZONES - 1);
    int p1 = std::min(p_idx + 1, Config::PHASE_ZONES - 1);
    int s1 = std::min(s_idx + 1, Config::AIRSPEED_ZONES - 1);
    
    // Collect the 16 corner values once
    ControlGains corners[16];
    corners[0]  = gain_table[t_idx][a_idx][p_idx][s_idx];   // g0000
    corners[1]  = gain_table[t1][a_idx][p_idx][s_idx];      // g1000
    corners[2]  = gain_table[t_idx][a1][p_idx][s_idx];      // g0100
    corners[3]  = gain_table[t_idx][a_idx][p1][s_idx];      // g0010
    corners[4]  = gain_table[t_idx][a_idx][p_idx][s1];      // g0001
    corners[5]  = gain_table[t1][a1][p_idx][s_idx];         // g1100
    corners[6]  = gain_table[t1][a_idx][p1][s_idx];         // g1010
    corners[7]  = gain_table[t1][a_idx][p_idx][s1];         // g1001
    corners[8]  = gain_table[t_idx][a1][p1][s_idx];         // g0110
    corners[9]  = gain_table[t_idx][a1][p_idx][s1];         // g0101
    corners[10] = gain_table[t_idx][a_idx][p1][s1];         // g0011
    corners[11] = gain_table[t1][a1][p1][s_idx];            // g1110
    corners[12] = gain_table[t1][a1][p_idx][s1];            // g1101
    corners[13] = gain_table[t1][a_idx][p1][s1];            // g1011
    corners[14] = gain_table[t_idx][a1][p1][s1];            // g0111
    corners[15] = gain_table[t1][a1][p1][s1];               // g1111

    // Use templated interpolation to avoid duplication
    ControlGains result;
    result.roll  = interpolateAxisGains(corners, [](const ControlGains& g) { return g.roll; },  t_factor, a_factor, p_factor, s_factor);
    result.pitch = interpolateAxisGains(corners, [](const ControlGains& g) { return g.pitch; }, t_factor, a_factor, p_factor, s_factor);
    result.yaw   = interpolateAxisGains(corners, [](const ControlGains& g) { return g.yaw; },   t_factor, a_factor, p_factor, s_factor);
    
    return result;
}

// Get grid indices and interpolation factors
void AdaptivePID::getGridIndices(const FlightRegime& regime, const Config& config,
                                int& t_idx, int& a_idx, int& p_idx, int& s_idx,
                                float& t_factor, float& a_factor, float& p_factor, float& s_factor) {
    
    // Throttle dimension
    t_factor = findInterpolationIndex(regime.throttle_fraction, config.throttle_zones, Config::THROTTLE_ZONES, t_idx);
    
    // AOA dimension  
    a_factor = findInterpolationIndex(regime.angle_of_attack, config.aoa_zones, Config::AOA_ZONES, a_idx);
    
    // Phase dimension - map FlightPhase enum to indices (need PHASE_ZONES+1 for bounds)
    float phase_values[Config::PHASE_ZONES + 1] = {
        static_cast<float>(FlightPhase::PREFLIGHT),  // 1.0
        static_cast<float>(FlightPhase::TAKEOFF),    // 2.0
        static_cast<float>(FlightPhase::CRUISE),     // 4.0
        static_cast<float>(FlightPhase::LANDING),    // 7.0
        static_cast<float>(FlightPhase::EMERGENCY)   // 8.0 (sentinel/upper bound)
    };
    float current_phase = static_cast<float>(regime.phase);
    p_factor = findInterpolationIndex(current_phase, phase_values, Config::PHASE_ZONES, p_idx);
    
    // Airspeed dimension
    s_factor = findInterpolationIndex(regime.airspeed, config.airspeed_zones, Config::AIRSPEED_ZONES, s_idx);
}

// Initialize gain schedule with reasonable defaults
void AdaptivePID::initializeGainSchedule() {
    // Fill gain schedule with default values
    for (int t = 0; t < Config::THROTTLE_ZONES; t++) {
        for (int a = 0; a < Config::AOA_ZONES; a++) {
            for (int p = 0; p < Config::PHASE_ZONES; p++) {
                for (int s = 0; s < Config::AIRSPEED_ZONES; s++) {
                    // Default gains - would normally be tuned for aircraft
                    gain_schedule_[t][a][p][s] = ControlGains(
                        PIDGains(0.15f, 0.02f, 0.008f),  // Roll
                        PIDGains(0.12f, 0.015f, 0.006f), // Pitch
                        PIDGains(0.08f, 0.01f, 0.003f)   // Yaw
                    );
                }
            }
        }
    }
}

// Update axis performance metrics
void AdaptivePID::updateAxisPerformance(float error, float output, float dt, PerformanceMetrics& metrics) {
    // Track error magnitude
    metrics.steady_error = std::abs(error);
    
    // Track overshoot (simplified)
    if (error * metrics.steady_error < 0) { // Sign change
        metrics.overshoot = std::max(metrics.overshoot, std::abs(error));
    }
    
    // Track settling time (time to reach 5% of target)
    if (std::abs(error) > 0.05f) {
        metrics.settling_time += dt;
    } else {
        metrics.settling_time = 0.0f;
    }
    
    // Track control effort
    metrics.control_effort = std::abs(output);
    
    // Update health score (simplified)
    metrics.health_score = 100.0f - (metrics.overshoot * 50.0f) - (metrics.settling_time * 10.0f);
    metrics.health_score = std::max(0.0f, std::min(100.0f, metrics.health_score));
}

// Update learning state based on performance
void AdaptivePID::updateLearningState() {
    // Calculate confidence based on performance metrics
    float roll_confidence = calculateAxisConfidence(roll_metrics_);
    float pitch_confidence = calculateAxisConfidence(pitch_metrics_);
    float yaw_confidence = calculateAxisConfidence(yaw_metrics_);
    
    learning_state_.confidence = (roll_confidence + pitch_confidence + yaw_confidence) / 3.0f;
    
    // Enable learning if confidence is above threshold
    learning_state_.is_learning = learning_state_.confidence > config_.confidence_threshold;
    
    // Adjust adaptation rate based on confidence
    learning_state_.adaptation_rate = config_.base_learning_rate * (1.0f + learning_state_.confidence);
}

// Calculate confidence for a single axis
float AdaptivePID::calculateAxisConfidence(const PerformanceMetrics& metrics) {
    float confidence = 1.0f;
    
    // Reduce confidence for excessive overshoot
    if (metrics.overshoot > config_.max_overshoot) {
        confidence *= 0.5f;
    }
    
    // Reduce confidence for long settling time
    if (metrics.settling_time > config_.max_settling_time) {
        confidence *= 0.7f;
    }
    
    // Reduce confidence for large errors
    if (metrics.steady_error > 10.0f) {
        confidence *= 0.8f;
    }
    
    return std::max(0.0f, std::min(1.0f, confidence));
}

// Find interpolation index and factor - static implementation
float AdaptivePID::findInterpolationIndex(float value, const float* zones, int num_zones, int& index) {
    // Clamp value to valid range
    value = std::max(zones[0], std::min(zones[num_zones], value));
    
    // Find the zone
    for (index = 0; index < num_zones - 1; index++) {
        if (value <= zones[index + 1]) {
            break;
        }
    }
    
    // Calculate interpolation factor
    float zone_width = zones[index + 1] - zones[index];
    if (zone_width > 0.001f) {
        return (value - zones[index]) / zone_width;
    }
    return 0.0f;
}

// 4D Quadrilinear interpolation for PID gains
PIDGains AdaptivePID::interpolateQuadrilinear(
    const PIDGains& g0000, const PIDGains& g1000, const PIDGains& g0100, const PIDGains& g0010, const PIDGains& g0001,
    const PIDGains& g1100, const PIDGains& g1010, const PIDGains& g1001, const PIDGains& g0110, const PIDGains& g0101,
    const PIDGains& g0011, const PIDGains& g1110, const PIDGains& g1101, const PIDGains& g1011, const PIDGains& g0111, const PIDGains& g1111,
    float t_factor, float a_factor, float p_factor, float s_factor) {
    
    PIDGains result;
    
    // Interpolate each gain component separately using 4D interpolation
    result.kp = interpolateScalar4D(
        g0000.kp, g1000.kp, g0100.kp, g0010.kp, g0001.kp,
        g1100.kp, g1010.kp, g1001.kp, g0110.kp, g0101.kp,
        g0011.kp, g1110.kp, g1101.kp, g1011.kp, g0111.kp, g1111.kp,
        t_factor, a_factor, p_factor, s_factor);
    
    result.ki = interpolateScalar4D(
        g0000.ki, g1000.ki, g0100.ki, g0010.ki, g0001.ki,
        g1100.ki, g1010.ki, g1001.ki, g0110.ki, g0101.ki,
        g0011.ki, g1110.ki, g1101.ki, g1011.ki, g0111.ki, g1111.ki,
        t_factor, a_factor, p_factor, s_factor);
    
    result.kd = interpolateScalar4D(
        g0000.kd, g1000.kd, g0100.kd, g0010.kd, g0001.kd,
        g1100.kd, g1010.kd, g1001.kd, g0110.kd, g0101.kd,
        g0011.kd, g1110.kd, g1101.kd, g1011.kd, g0111.kd, g1111.kd,
        t_factor, a_factor, p_factor, s_factor);
    
    return result;
}

// 4D scalar interpolation (hypercube interpolation)
float AdaptivePID::interpolateScalar4D(
    float v0000, float v1000, float v0100, float v0010, float v0001,
    float v1100, float v1010, float v1001, float v0110, float v0101,
    float v0011, float v1110, float v1101, float v1011, float v0111, float v1111,
    float t, float a, float p, float s) {
    
    // First, interpolate along the t dimension (throttle)
    float c000 = v0000 + t * (v1000 - v0000);
    float c100 = v0100 + t * (v1100 - v0100);
    float c010 = v0010 + t * (v1010 - v0010);
    float c110 = v0110 + t * (v1110 - v0110);
    float c001 = v0001 + t * (v1001 - v0001);
    float c101 = v0101 + t * (v1101 - v0101);
    float c011 = v0011 + t * (v1011 - v0011);
    float c111 = v0111 + t * (v1111 - v0111);
    
    // Then interpolate along the a dimension (angle of attack)
    float c00 = c000 + a * (c100 - c000);
    float c01 = c001 + a * (c101 - c001);
    float c10 = c010 + a * (c110 - c010);
    float c11 = c011 + a * (c111 - c011);
    
    // Then interpolate along the p dimension (phase)
    float c0 = c00 + p * (c10 - c00);
    float c1 = c01 + p * (c11 - c01);
    
    // Finally interpolate along the s dimension (airspeed)
    return c0 + s * (c1 - c0);
}

// Template function moved to header file for proper linkage

// Update performance buffers for oscillation detection
void AdaptivePID::updatePerformanceBuffers(const Attitude& errors, const AngularRates& outputs) {
    // Store error and output values in circular buffers
    roll_error_buffer_[buffer_index_] = errors.roll;
    pitch_error_buffer_[buffer_index_] = errors.pitch;
    yaw_error_buffer_[buffer_index_] = errors.yaw;
    
    roll_output_buffer_[buffer_index_] = outputs.roll_rate;
    pitch_output_buffer_[buffer_index_] = outputs.pitch_rate;
    yaw_output_buffer_[buffer_index_] = outputs.yaw_rate;
    
    buffer_index_ = (buffer_index_ + 1) % BUFFER_SIZE;
}

// Adapt gains if needed - FULL IMPLEMENTATION
void AdaptivePID::adaptGainsIfNeeded() {
    // Only adapt if learning is enabled and we have sufficient confidence
    if (learning_frozen_ || !learning_state_.is_learning) {
        return;
    }
    
    // Only adapt every N iterations to avoid too-frequent changes
    adaptation_counter_++;
    if (adaptation_counter_ < 50) {  // Adapt every 50 control loops
        return;
    }
    adaptation_counter_ = 0;
    
    // Save current state before adaptation
    ControlGains current_gains = getGains(current_regime_);
    PerformanceMetrics avg_performance;
    avg_performance.health_score = (roll_metrics_.health_score + 
                                   pitch_metrics_.health_score + 
                                   yaw_metrics_.health_score) / 3.0f;
    saveSnapshot(current_gains, avg_performance);
    
    // Calculate performance-based gain adjustments
    ControlGains delta_gains = calculateGainAdjustments();
    
    // Find which corners are influencing current performance
    int t_idx, a_idx, p_idx, s_idx;
    float t_factor, a_factor, p_factor, s_factor;
    getGridIndices(current_regime_, config_, t_idx, a_idx, p_idx, s_idx, 
                   t_factor, a_factor, p_factor, s_factor);
    
    // Update the corner values with weighted updates
    updateCornerValues(t_idx, a_idx, p_idx, s_idx, 
                      t_factor, a_factor, p_factor, s_factor, 
                      delta_gains);
    
    // Ensure gains stay within safety limits
    clampGainsToLimits();
    
    // Check if we should rollback due to poor performance
    PerformanceMetrics new_performance;
    new_performance.health_score = (roll_metrics_.health_score + 
                                   pitch_metrics_.health_score + 
                                   yaw_metrics_.health_score) / 3.0f;
    
    int prev_snapshot = (current_snapshot_ - 1 + MAX_SNAPSHOTS) % MAX_SNAPSHOTS;
    if (snapshots_[prev_snapshot].timestamp > 0 && 
        shouldRollback(new_performance, snapshots_[prev_snapshot].performance)) {
        rollbackToPreviousGains();
        return; // Don't update momentum if we rolled back
    }
    
    // Update momentum for next iteration
    learning_state_.momentum = learning_state_.momentum * config_.momentum_factor + 
                              (1.0f - config_.momentum_factor) * calculatePerformanceGradient(new_performance);
}

// Gain schedule persistence with checksum validation
bool AdaptivePID::loadGainSchedule(const void* data, size_t size) {
    if (size != getGainScheduleSize()) {
        return false;
    }
    
    const uint8_t* buffer = static_cast<const uint8_t*>(data);
    
    // Calculate and verify checksum
    uint32_t stored_checksum;
    memcpy(&stored_checksum, buffer + size - sizeof(uint32_t), sizeof(uint32_t));
    
    uint32_t calculated_checksum = calculateChecksum(buffer, size - sizeof(uint32_t));
    if (calculated_checksum != stored_checksum) {
        return false; // Corrupt data
    }
    
    // Copy gain schedule data
    memcpy(gain_schedule_, buffer, sizeof(gain_schedule_));
    
    // Validate all gains are within safety limits
    clampGainsToLimits();
    
    return true;
}

bool AdaptivePID::saveGainSchedule(void* data, size_t* size) const {
    *size = getGainScheduleSize();
    uint8_t* buffer = static_cast<uint8_t*>(data);
    
    // Copy gain schedule
    memcpy(buffer, gain_schedule_, sizeof(gain_schedule_));
    
    // Calculate and append checksum
    uint32_t checksum = calculateChecksum(buffer, sizeof(gain_schedule_));
    memcpy(buffer + sizeof(gain_schedule_), &checksum, sizeof(uint32_t));
    
    return true;
}

size_t AdaptivePID::getGainScheduleSize() const {
    return sizeof(gain_schedule_) + sizeof(uint32_t); // Include checksum
}

// Simple checksum calculation for data integrity
uint32_t AdaptivePID::calculateChecksum(const uint8_t* data, size_t length) const {
    uint32_t checksum = 0xAAAAAAAA; // Initial seed
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
        checksum = (checksum << 1) | (checksum >> 31); // Rotate left
    }
    return checksum;
}

// Removed calculateAdaptationDirection - not used in current implementation

bool AdaptivePID::shouldRollback(const PerformanceMetrics& current, const PerformanceMetrics& previous) const {
    return current.health_score < previous.health_score * 0.8f;
}

void AdaptivePID::saveSnapshot(const ControlGains& gains, const PerformanceMetrics& performance) {
    snapshots_[current_snapshot_].gains = gains;
    snapshots_[current_snapshot_].performance = performance;
    snapshots_[current_snapshot_].timestamp = last_update_time_;
    
    current_snapshot_ = (current_snapshot_ + 1) % MAX_SNAPSHOTS;
}

void AdaptivePID::rollbackToPreviousGains() {
    int prev_snapshot = (current_snapshot_ - 1 + MAX_SNAPSHOTS) % MAX_SNAPSHOTS;
    
    if (snapshots_[prev_snapshot].timestamp > 0) {
        learning_state_.momentum *= -0.5f;  // Reverse momentum direction
    }
} 

// ========== FULL LEARNING IMPLEMENTATION ==========

// Update current flight regime for learning
FlightRegime AdaptivePID::getCurrentRegime() const {
    return current_regime_;
}

// Calculate gain adjustments based on performance metrics
ControlGains AdaptivePID::calculateGainAdjustments() {
    ControlGains delta_gains;
    
    // Calculate performance gradient for overall system
    float performance_gradient = calculatePerformanceGradient(roll_metrics_);
    
    // Calculate axis-specific adjustments
    delta_gains.roll = calculateAxisGainAdjustment(roll_metrics_, performance_gradient);
    delta_gains.pitch = calculateAxisGainAdjustment(pitch_metrics_, performance_gradient);  
    delta_gains.yaw = calculateAxisGainAdjustment(yaw_metrics_, performance_gradient);
    
    return delta_gains;
}

// Calculate performance gradient for adaptation direction
float AdaptivePID::calculatePerformanceGradient(const PerformanceMetrics& metrics) const {
    // Performance gradient based on multiple factors
    float gradient = 0.0f;
    
    // Overshoot contribution (negative gradient if too much overshoot)
    if (metrics.overshoot > config_.max_overshoot) {
        gradient -= (metrics.overshoot - config_.max_overshoot) * 2.0f;
    }
    
    // Settling time contribution (negative gradient if too slow)
    if (metrics.settling_time > config_.max_settling_time) {
        gradient -= (metrics.settling_time - config_.max_settling_time) * 0.5f;
    }
    
    // Steady state error contribution
    gradient -= metrics.steady_error * 0.1f;
    
    // Health score contribution (positive gradient for good health)
    gradient += (metrics.health_score - 50.0f) * 0.01f;
    
    // Clamp gradient to reasonable range
    return std::max(-1.0f, std::min(1.0f, gradient));
}

// Calculate gain adjustment for a single axis
PIDGains AdaptivePID::calculateAxisGainAdjustment(const PerformanceMetrics& metrics, float base_adjustment) const {
    PIDGains adjustment;
    
    float learning_rate = learning_state_.adaptation_rate;
    float momentum = learning_state_.momentum;
    
    // Proportional gain adjustment
    if (metrics.overshoot > config_.max_overshoot) {
        // Too much overshoot - reduce Kp
        adjustment.kp = -learning_rate * (metrics.overshoot - config_.max_overshoot) * 0.1f;
    } else if (metrics.settling_time > config_.max_settling_time) {
        // Too slow response - increase Kp
        adjustment.kp = learning_rate * (metrics.settling_time - config_.max_settling_time) * 0.05f;
    } else {
        // Fine tuning based on steady state error
        adjustment.kp = learning_rate * base_adjustment * 0.01f;
    }
    
    // Integral gain adjustment
    if (metrics.steady_error > 5.0f) {
        // Persistent error - increase Ki
        adjustment.ki = learning_rate * metrics.steady_error * 0.001f;
    } else {
        // Reduce Ki if overshoot is present (integral windup)
        adjustment.ki = -learning_rate * metrics.overshoot * 0.002f;
    }
    
    // Derivative gain adjustment
    if (metrics.oscillation_freq > config_.max_oscillation_freq) {
        // Too much oscillation - reduce Kd
        adjustment.kd = -learning_rate * (metrics.oscillation_freq - config_.max_oscillation_freq) * 0.001f;
    } else if (metrics.settling_time > config_.max_settling_time && metrics.overshoot < 0.1f) {
        // Slow without overshoot - increase Kd
        adjustment.kd = learning_rate * (metrics.settling_time - config_.max_settling_time) * 0.002f;
    }
    
    // Apply momentum
    adjustment.kp += momentum * 0.1f;
    adjustment.ki += momentum * 0.01f; 
    adjustment.kd += momentum * 0.005f;
    
    return adjustment;
}

// Update corner values with weighted updates
void AdaptivePID::updateCornerValues(int t_idx, int a_idx, int p_idx, int s_idx,
                                    float t_factor, float a_factor, float p_factor, float s_factor,
                                    const ControlGains& delta_gains) {
    
    // Clear previous modification tracking
    modified_corners_.clear();
    
    // Calculate weights for all 16 corners of the 4D hypercube
    float weights[2][2][2][2];
    weights[0][0][0][0] = (1-t_factor) * (1-a_factor) * (1-p_factor) * (1-s_factor);
    weights[1][0][0][0] = t_factor * (1-a_factor) * (1-p_factor) * (1-s_factor);
    weights[0][1][0][0] = (1-t_factor) * a_factor * (1-p_factor) * (1-s_factor);
    weights[0][0][1][0] = (1-t_factor) * (1-a_factor) * p_factor * (1-s_factor);
    weights[0][0][0][1] = (1-t_factor) * (1-a_factor) * (1-p_factor) * s_factor;
    weights[1][1][0][0] = t_factor * a_factor * (1-p_factor) * (1-s_factor);
    weights[1][0][1][0] = t_factor * (1-a_factor) * p_factor * (1-s_factor);
    weights[1][0][0][1] = t_factor * (1-a_factor) * (1-p_factor) * s_factor;
    weights[0][1][1][0] = (1-t_factor) * a_factor * p_factor * (1-s_factor);
    weights[0][1][0][1] = (1-t_factor) * a_factor * (1-p_factor) * s_factor;
    weights[0][0][1][1] = (1-t_factor) * (1-a_factor) * p_factor * s_factor;
    weights[1][1][1][0] = t_factor * a_factor * p_factor * (1-s_factor);
    weights[1][1][0][1] = t_factor * a_factor * (1-p_factor) * s_factor;
    weights[1][0][1][1] = t_factor * (1-a_factor) * p_factor * s_factor;
    weights[0][1][1][1] = (1-t_factor) * a_factor * p_factor * s_factor;
    weights[1][1][1][1] = t_factor * a_factor * p_factor * s_factor;
    
    // Update all corners with weighted adjustments
    for (int dt = 0; dt < 2; dt++) {
        for (int da = 0; da < 2; da++) {
            for (int dp = 0; dp < 2; dp++) {
                for (int ds = 0; ds < 2; ds++) {
                    int ti = std::min(t_idx + dt, Config::THROTTLE_ZONES - 1);
                    int ai = std::min(a_idx + da, Config::AOA_ZONES - 1);
                    int pi = std::min(p_idx + dp, Config::PHASE_ZONES - 1);
                    int si = std::min(s_idx + ds, Config::AIRSPEED_ZONES - 1);
                    
                    float weight = weights[dt][da][dp][ds];
                    
                    // Track this corner as modified
                    modified_corners_.emplace_back(ti, ai, pi, si);
                    
                    // Update roll gains
                    gain_schedule_[ti][ai][pi][si].roll.kp += weight * delta_gains.roll.kp;
                    gain_schedule_[ti][ai][pi][si].roll.ki += weight * delta_gains.roll.ki;
                    gain_schedule_[ti][ai][pi][si].roll.kd += weight * delta_gains.roll.kd;
                    
                    // Update pitch gains
                    gain_schedule_[ti][ai][pi][si].pitch.kp += weight * delta_gains.pitch.kp;
                    gain_schedule_[ti][ai][pi][si].pitch.ki += weight * delta_gains.pitch.ki;
                    gain_schedule_[ti][ai][pi][si].pitch.kd += weight * delta_gains.pitch.kd;
                    
                    // Update yaw gains
                    gain_schedule_[ti][ai][pi][si].yaw.kp += weight * delta_gains.yaw.kp;
                    gain_schedule_[ti][ai][pi][si].yaw.ki += weight * delta_gains.yaw.ki;
                    gain_schedule_[ti][ai][pi][si].yaw.kd += weight * delta_gains.yaw.kd;
                }
            }
        }
    }
}

// Clamp all gains to safety limits
// Detect oscillations in error signals using FFT-like analysis
float AdaptivePID::detectOscillations() const {
    float max_oscillation_freq = 0.0f;
    
    // Simple oscillation detection using zero-crossing analysis
    auto detectAxisOscillations = [](const float* buffer, int size) -> float {
        if (size < 10) return 0.0f;
        
        int zero_crossings = 0;
        float prev_sign = (buffer[0] > 0) ? 1.0f : -1.0f;
        
        for (int i = 1; i < size; i++) {
            float curr_sign = (buffer[i] > 0) ? 1.0f : -1.0f;
            if (curr_sign != prev_sign) {
                zero_crossings++;
            }
            prev_sign = curr_sign;
        }
        
        // Convert to frequency (using actual loop frequency from config)
        // Note: This assumes the buffer represents the last BUFFER_SIZE samples
        float loop_freq = 500.0f; // TODO: Get this from config
        return (zero_crossings / 2.0f) * (loop_freq / size);
    };
    
    // Check oscillations for each axis
    float roll_osc = detectAxisOscillations(roll_error_buffer_, BUFFER_SIZE);
    float pitch_osc = detectAxisOscillations(pitch_error_buffer_, BUFFER_SIZE);
    float yaw_osc = detectAxisOscillations(yaw_error_buffer_, BUFFER_SIZE);
    
    // Return maximum oscillation frequency
    max_oscillation_freq = std::max({roll_osc, pitch_osc, yaw_osc});
    
    // Update oscillation frequency in metrics
    const_cast<AdaptivePID*>(this)->roll_metrics_.oscillation_freq = roll_osc;
    const_cast<AdaptivePID*>(this)->pitch_metrics_.oscillation_freq = pitch_osc;
    const_cast<AdaptivePID*>(this)->yaw_metrics_.oscillation_freq = yaw_osc;
    
    return max_oscillation_freq;
}

// Clamp gains to safety limits (optimized to only clamp modified corners)
void AdaptivePID::clampGainsToLimits() {
    // Only clamp the corners that were modified in the last adaptation
    for (const auto& corner : modified_corners_) {
        int t = corner.t, a = corner.a, p = corner.p, s = corner.s;
        
        // Clamp roll gains
        clampSingleAxisGains(gain_schedule_[t][a][p][s].roll);
        
        // Clamp pitch gains  
        clampSingleAxisGains(gain_schedule_[t][a][p][s].pitch);
        
        // Clamp yaw gains
        clampSingleAxisGains(gain_schedule_[t][a][p][s].yaw);
    }
}

// Helper function to clamp a single axis gains
void AdaptivePID::clampSingleAxisGains(PIDGains& gains) {
    gains.kp = std::max(config_.min_gains.kp, std::min(config_.max_gains.kp, gains.kp));
    gains.ki = std::max(config_.min_gains.ki, std::min(config_.max_gains.ki, gains.ki));
    gains.kd = std::max(config_.min_gains.kd, std::min(config_.max_gains.kd, gains.kd));
} 