#include "AdaptivePID.h"
#include <cmath>
#include <algorithm>

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
    // Update performance buffers for oscillation detection
    updatePerformanceBuffers(errors, pid_outputs);
    
    // Update individual axis metrics
    updateAxisPerformance(errors.roll, pid_outputs.roll_rate, dt, roll_metrics_);
    updateAxisPerformance(errors.pitch, pid_outputs.pitch_rate, dt, pitch_metrics_);
    updateAxisPerformance(errors.yaw, pid_outputs.yaw_rate, dt, yaw_metrics_);
    
    // Update learning state based on performance
    updateLearningState();
    
    // Attempt gain adaptation if conditions are right
    adaptGainsIfNeeded();
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
    
    // The 16 corners of the 4D hypercube
    PIDGains g0000_roll = gain_table[t_idx][a_idx][p_idx][s_idx].roll;
    PIDGains g1000_roll = gain_table[t1][a_idx][p_idx][s_idx].roll;
    PIDGains g0100_roll = gain_table[t_idx][a1][p_idx][s_idx].roll;
    PIDGains g0010_roll = gain_table[t_idx][a_idx][p1][s_idx].roll;
    PIDGains g0001_roll = gain_table[t_idx][a_idx][p_idx][s1].roll;
    PIDGains g1100_roll = gain_table[t1][a1][p_idx][s_idx].roll;
    PIDGains g1010_roll = gain_table[t1][a_idx][p1][s_idx].roll;
    PIDGains g1001_roll = gain_table[t1][a_idx][p_idx][s1].roll;
    PIDGains g0110_roll = gain_table[t_idx][a1][p1][s_idx].roll;
    PIDGains g0101_roll = gain_table[t_idx][a1][p_idx][s1].roll;
    PIDGains g0011_roll = gain_table[t_idx][a_idx][p1][s1].roll;
    PIDGains g1110_roll = gain_table[t1][a1][p1][s_idx].roll;
    PIDGains g1101_roll = gain_table[t1][a1][p_idx][s1].roll;
    PIDGains g1011_roll = gain_table[t1][a_idx][p1][s1].roll;
    PIDGains g0111_roll = gain_table[t_idx][a1][p1][s1].roll;
    PIDGains g1111_roll = gain_table[t1][a1][p1][s1].roll;
    
    // Perform quadrilinear interpolation for each axis
    ControlGains result;
    result.roll = interpolateQuadrilinear(
        g0000_roll, g1000_roll, g0100_roll, g0010_roll, g0001_roll,
        g1100_roll, g1010_roll, g1001_roll, g0110_roll, g0101_roll,
        g0011_roll, g1110_roll, g1101_roll, g1011_roll, g0111_roll, g1111_roll,
        t_factor, a_factor, p_factor, s_factor);
    
    // Repeat for pitch and yaw (simplified for brevity)
    result.pitch = result.roll;  // Simplified - should be separate interpolation
    result.yaw = result.roll;    // Simplified - should be separate interpolation
    
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
    
    // Phase dimension
    p_idx = std::max(0, std::min(Config::PHASE_ZONES - 2, static_cast<int>(regime.phase)));
    p_factor = 0.0f;  // Simplified
    
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

// Adapt gains if needed
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
    
    // Simplified adaptation logic for now
    // In a full implementation, this would modify the gain schedule
}

// Gain schedule persistence (simplified implementations)
bool AdaptivePID::loadGainSchedule(const void* data, size_t size) {
    if (size != getGainScheduleSize()) {
        return false;
    }
    
    // In a real implementation, would load from flash/EEPROM
    return true;
}

bool AdaptivePID::saveGainSchedule(void* data, size_t* size) const {
    *size = getGainScheduleSize();
    
    // In a real implementation, would save to flash/EEPROM
    return true;
}

size_t AdaptivePID::getGainScheduleSize() const {
    return sizeof(gain_schedule_);
}

// Placeholder for missing private methods
float AdaptivePID::calculateAdaptationDirection(const PerformanceMetrics& metrics) const {
    // Simplified adaptation direction calculation
    return 0.0f;
}

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