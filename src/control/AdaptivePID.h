#pragma once
#include "../core/Types.h"
#include <stdint.h>
#include <vector>

/**
 * 4D Adaptive PID Controller with Gain Scheduling
 * 
 * This module implements:
 * - 4D gain scheduling (throttle x AOA x phase x airspeed)
 * - Quadrilinear interpolation between gain schedule points
 * - Performance-based adaptive learning
 * - Wind-aware adaptation rate control
 * 
 * The module is designed to be:
 * - Hardware agnostic (no direct hardware dependencies)
 * - Testable (algorithms can be unit tested on desktop)
 * - Configurable (gain schedule can be loaded/saved)
 */
class AdaptivePID {
public:
    /**
     * Configuration for adaptive PID behavior
     */
    struct Config {
        // Gain schedule dimensions
        static const int THROTTLE_ZONES = 5;
        static const int AOA_ZONES = 4; 
        static const int PHASE_ZONES = 4;
        static const int AIRSPEED_ZONES = 3;
        
        // Learning parameters
        float base_learning_rate = 0.0003f;
        float momentum_factor = 0.9f;
        float confidence_threshold = 0.7f;
        
        // Performance thresholds
        float max_overshoot = 0.3f;
        float max_settling_time = 2.0f;
        float max_oscillation_freq = 5.0f;
        
        // Safety limits
        PIDGains min_gains = PIDGains(0.01f, 0.0f, 0.0f);
        PIDGains max_gains = PIDGains(0.5f, 0.1f, 0.05f);
        
        // Zone boundaries
        float throttle_zones[THROTTLE_ZONES + 1] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
        float aoa_zones[AOA_ZONES + 1] = {-15.0f, -5.0f, 5.0f, 15.0f, 30.0f};
        float airspeed_zones[AIRSPEED_ZONES + 1] = {0.0f, 10.0f, 20.0f, 40.0f};
    };
    
    /**
     * Learning state information
     */
    struct LearningState {
        bool is_learning = false;         // Whether adaptation is active
        float confidence = 0.0f;          // Confidence in current performance (0-1)
        float adaptation_rate = 0.0003f;  // Current learning rate
        float momentum = 0.0f;            // Momentum for gradient-like adaptation
    };
    
    /**
     * Constructor
     */
    AdaptivePID(const Config& config = Config());
    
    /**
     * Initialize with default or loaded gain schedule
     */
    bool initialize();
    
    /**
     * Get current PID gains for the specified flight regime
     * This is the main interface - given current flight conditions,
     * returns the appropriate PID gains via 4D interpolation
     */
    ControlGains getGains(const FlightRegime& regime) const;
    
    /**
     * Update performance metrics and adapt gains
     * Call this every control loop with current errors and outputs
     */
    void updatePerformance(const Attitude& errors, const AngularRates& pid_outputs, float dt);
    
    /**
     * Update current flight regime for learning
     * Call this with current flight conditions
     */
    void updateFlightRegime(const FlightRegime& regime);
    
    /**
     * Set adaptation rate (typically controlled by wind analysis)
     */
    void setAdaptationRate(float rate);
    
    /**
     * Freeze/unfreeze learning (for wind gust rejection)
     */
    void freezeLearning(bool freeze);
    
    /**
     * Get current learning state
     */
    LearningState getLearningState() const { return learning_state_; }
    
    /**
     * Get performance metrics for each axis
     */
    PerformanceMetrics getRollMetrics() const { return roll_metrics_; }
    PerformanceMetrics getPitchMetrics() const { return pitch_metrics_; }
    PerformanceMetrics getYawMetrics() const { return yaw_metrics_; }
    
    /**
     * Load/save gain schedule (for persistence)
     */
    bool loadGainSchedule(const void* data, size_t size);
    bool saveGainSchedule(void* data, size_t* size) const;
    size_t getGainScheduleSize() const;
    
    /**
     * Static utility functions (easily unit testable)
     */
    static ControlGains interpolateGains(const FlightRegime& regime, 
                                        const ControlGains gain_table[Config::THROTTLE_ZONES]
                                                                    [Config::AOA_ZONES]
                                                                    [Config::PHASE_ZONES] 
                                                                    [Config::AIRSPEED_ZONES],
                                        const Config& config);
    
    static void getGridIndices(const FlightRegime& regime, const Config& config,
                              int& t_idx, int& a_idx, int& p_idx, int& s_idx,
                              float& t_factor, float& a_factor, float& p_factor, float& s_factor);
    
private:
    Config config_;
    LearningState learning_state_;
    float adaptation_rate_;
    bool learning_frozen_;
    
    // Current flight regime tracking for learning
    FlightRegime current_regime_;
    FlightRegime previous_regime_;
    
    // 4D gain schedule lookup table
    ControlGains gain_schedule_[Config::THROTTLE_ZONES]
                              [Config::AOA_ZONES] 
                              [Config::PHASE_ZONES]
                              [Config::AIRSPEED_ZONES];
    
    // PID state for each axis
    struct PIDState {
        float integral = 0.0f;
        float last_error = 0.0f;
        float filtered_derivative = 0.0f;  // Low-pass filtered derivative
        static constexpr float DERIVATIVE_FILTER_ALPHA = 0.8f;  // Filter constant
    };
    
    PIDState roll_state_;
    PIDState pitch_state_;
    PIDState yaw_state_;
    
    // Performance tracking
    PerformanceMetrics roll_metrics_;
    PerformanceMetrics pitch_metrics_;
    PerformanceMetrics yaw_metrics_;
    
    // Learning history for rollback
    struct LearningSnapshot {
        ControlGains gains;
        PerformanceMetrics performance;
        uint32_t timestamp;
    };
    
    static const int MAX_SNAPSHOTS = 10;
    LearningSnapshot snapshots_[MAX_SNAPSHOTS];
    int current_snapshot_;
    
    // Performance analysis buffers
    static const int BUFFER_SIZE = 100;
    float roll_error_buffer_[BUFFER_SIZE];
    float pitch_error_buffer_[BUFFER_SIZE];
    float yaw_error_buffer_[BUFFER_SIZE];
    float roll_output_buffer_[BUFFER_SIZE];
    float pitch_output_buffer_[BUFFER_SIZE];
    float yaw_output_buffer_[BUFFER_SIZE];
    int buffer_index_;
    
    // Timing
    uint32_t last_update_time_;
    uint32_t adaptation_counter_;
    
    // Track which corners were modified for efficient clamping
    struct CornerIndex {
        int t, a, p, s;
        CornerIndex(int t_, int a_, int p_, int s_) : t(t_), a(a_), p(p_), s(s_) {}
    };
    std::vector<CornerIndex> modified_corners_;
    
    // Internal methods
    void initializeGainSchedule();
    void updateAxisPerformance(float error, float output, float dt, PerformanceMetrics& metrics);
    void adaptGainsIfNeeded();
    bool shouldRollback(const PerformanceMetrics& current, const PerformanceMetrics& previous) const;
    void saveSnapshot(const ControlGains& gains, const PerformanceMetrics& performance);
    void rollbackToPreviousGains();
    void updateLearningState();
    float calculateAxisConfidence(const PerformanceMetrics& metrics);
    static float findInterpolationIndex(float value, const float* zones, int num_zones, int& index);
    
    // Full learning implementation methods
    ControlGains calculateGainAdjustments();
    void updateCornerValues(int t_idx, int a_idx, int p_idx, int s_idx,
                           float t_factor, float a_factor, float p_factor, float s_factor,
                           const ControlGains& delta_gains);
    FlightRegime getCurrentRegime() const;
    void clampGainsToLimits();
    void clampSingleAxisGains(PIDGains& gains);
    float calculatePerformanceGradient(const PerformanceMetrics& metrics) const;
    PIDGains calculateAxisGainAdjustment(const PerformanceMetrics& metrics, float base_adjustment) const;
    
    // Enhanced performance analysis
    void updatePerformanceBuffers(const Attitude& errors, const AngularRates& outputs);
    float detectOscillations() const;
    
    // True 4D interpolation methods
    static PIDGains interpolateQuadrilinear(
        const PIDGains& g0000, const PIDGains& g1000, const PIDGains& g0100, const PIDGains& g0010, const PIDGains& g0001,
        const PIDGains& g1100, const PIDGains& g1010, const PIDGains& g1001, const PIDGains& g0110, const PIDGains& g0101,
        const PIDGains& g0011, const PIDGains& g1110, const PIDGains& g1101, const PIDGains& g1011, const PIDGains& g0111, const PIDGains& g1111,
        float t_factor, float a_factor, float p_factor, float s_factor);
    
    static float interpolateScalar4D(
        float v0000, float v1000, float v0100, float v0010, float v0001,
        float v1100, float v1010, float v1001, float v0110, float v0101,
        float v0011, float v1110, float v1101, float v1011, float v0111, float v1111,
        float t, float a, float p, float s);
    
    // Template function to interpolate axis gains from corner values (defined inline for linkage)
    template<typename AxisExtractor>
    static PIDGains interpolateAxisGains(const ControlGains corners[16], 
                                        AxisExtractor extractor,
                                        float t_factor, float a_factor, float p_factor, float s_factor) {
        PIDGains result;
        
        // Extract the 16 corner gains for this axis
        PIDGains axis_corners[16];
        for (int i = 0; i < 16; i++) {
            axis_corners[i] = extractor(corners[i]);
        }
        
        // Interpolate each PID component separately
        result.kp = interpolateScalar4D(
            axis_corners[0].kp,  axis_corners[1].kp,  axis_corners[2].kp,  axis_corners[3].kp,  axis_corners[4].kp,
            axis_corners[5].kp,  axis_corners[6].kp,  axis_corners[7].kp,  axis_corners[8].kp,  axis_corners[9].kp,
            axis_corners[10].kp, axis_corners[11].kp, axis_corners[12].kp, axis_corners[13].kp, axis_corners[14].kp, axis_corners[15].kp,
            t_factor, a_factor, p_factor, s_factor);
        
        result.ki = interpolateScalar4D(
            axis_corners[0].ki,  axis_corners[1].ki,  axis_corners[2].ki,  axis_corners[3].ki,  axis_corners[4].ki,
            axis_corners[5].ki,  axis_corners[6].ki,  axis_corners[7].ki,  axis_corners[8].ki,  axis_corners[9].ki,
            axis_corners[10].ki, axis_corners[11].ki, axis_corners[12].ki, axis_corners[13].ki, axis_corners[14].ki, axis_corners[15].ki,
            t_factor, a_factor, p_factor, s_factor);
        
        result.kd = interpolateScalar4D(
            axis_corners[0].kd,  axis_corners[1].kd,  axis_corners[2].kd,  axis_corners[3].kd,  axis_corners[4].kd,
            axis_corners[5].kd,  axis_corners[6].kd,  axis_corners[7].kd,  axis_corners[8].kd,  axis_corners[9].kd,
            axis_corners[10].kd, axis_corners[11].kd, axis_corners[12].kd, axis_corners[13].kd, axis_corners[14].kd, axis_corners[15].kd,
            t_factor, a_factor, p_factor, s_factor);
        
        return result;
    }
    
    // Data integrity
    uint32_t calculateChecksum(const uint8_t* data, size_t length) const;
}; 