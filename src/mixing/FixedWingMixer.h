#pragma once
#include "../core/Types.h"
#include "../core/HAL.h"

/**
 * Fixed-Wing Control Mixer
 * 
 * Converts PID controller outputs to control surface commands for fixed-wing aircraft.
 * Specifically designed for elevon configuration (combined elevator + aileron).
 * 
 * Features:
 * - Elevon mixing (elevator + aileron combination)  
 * - Wind compensation integration
 * - Airspeed-dependent control authority scaling
 * - Coordinated flight control
 * - Safety limits and failsafes
 */
class FixedWingMixer {
public:
    /**
     * Configuration for fixed-wing mixing
     */
    struct Config {
        // Control surface configuration
        bool use_elevons = true;           // True for elevon config, false for separate elevator/aileron
        bool reverse_left_elevon = false;  // Reverse left elevon direction
        bool reverse_right_elevon = false; // Reverse right elevon direction
        bool reverse_rudder = false;       // Reverse rudder direction
        
        // Control authority limits
        float max_elevon_deflection = 0.8f;    // Maximum elevon deflection (±80%)
        float max_rudder_deflection = 0.6f;    // Maximum rudder deflection (±60%)
        float max_throttle = 1.0f;             // Maximum throttle (100%)
        float min_throttle = 0.0f;             // Minimum throttle (0%)
        
        // Airspeed scaling
        bool enable_airspeed_scaling = true;   // Scale control authority with airspeed
        float nominal_airspeed = 15.0f;        // Nominal airspeed for 100% authority (m/s)
        float min_airspeed_scaling = 0.3f;     // Minimum scaling factor at low airspeed
        float max_airspeed_scaling = 1.5f;     // Maximum scaling factor at high airspeed
        
        // Mixing ratios
        float elevator_mix_ratio = 1.0f;       // How much elevator input affects elevons
        float aileron_mix_ratio = 1.0f;        // How much aileron input affects elevons  
        float rudder_mix_ratio = 1.0f;         // How much yaw input affects rudder
        
        // Coordination assistance
        bool enable_rudder_coordination = true; // Auto-coordinate turns with rudder
        float coordination_gain = 0.1f;         // Gain for automatic rudder coordination
        
        // Wind compensation
        bool enable_wind_compensation = true;   // Enable wind feedforward compensation
        float wind_compensation_gain = 1.0f;    // Overall wind compensation scaling
        
        // Throttle management
        bool enable_throttle_dithering = true; // Allow throttle dithering for airspeed estimation
        float idle_throttle = 0.1f;            // Idle throttle setting
    };
    
    /**
     * Wind compensation inputs (from wind rejection module)
     */
    struct WindCompensation {
        float pitch_compensation;   // Elevator compensation for head/tail wind
        float roll_compensation;    // Aileron compensation for crosswind
        float yaw_compensation;     // Rudder compensation for wind-induced yaw
        float throttle_compensation; // Throttle compensation for head/tail wind
        float confidence;           // Confidence in wind estimates (0-1)
        
        WindCompensation(float p, float r, float y, float t, float c)
            : pitch_compensation(p), roll_compensation(r), yaw_compensation(y), 
              throttle_compensation(t), confidence(c) {}
        
        // zero-default
        WindCompensation() : WindCompensation(0.0f, 0.0f, 0.0f, 0.0f, 0.0f) {}
    };
    
    /**
     * Constructor
     */
    FixedWingMixer(HAL* hal, const Config& config = Config());
    
    /**
     * Initialize the mixer
     */
    bool initialize();
    
    /**
     * Mix PID outputs to control surface commands
     * This is the main mixing function
     */
    ControlSurfaces mix(const AngularRates& pid_outputs, 
                       const RadioInputs& radio_inputs,
                       float airspeed,
                       const WindCompensation& wind_comp = WindCompensation());
    
    /**
     * Apply the control surface commands to hardware
     */
    void applyControls(const ControlSurfaces& controls);
    
    /**
     * Emergency stop - set all controls to safe positions
     */
    void emergencyStop();
    
    /**
     * Get current control surface positions (for feedback/logging)
     */
    ControlSurfaces getCurrentControls() const { return current_controls_; }
    
    /**
     * Enable/disable throttle pass-through for manual control
     */
    void setThrottlePassthrough(bool enable) { throttle_passthrough_ = enable; }
    
    /**
     * Set wind compensation data
     */
    void setWindCompensation(const WindCompensation& wind_comp) { wind_compensation_ = wind_comp; }
    
    /**
     * Static utility functions (testable)
     */
    static ControlSurfaces mixElevons(float elevator_cmd, float aileron_cmd, float rudder_cmd, float throttle_cmd);
    
    static float scaleControlAuthority(float command, float airspeed, float nominal_airspeed, 
                                     float min_scaling, float max_scaling);
    
    static float calculateCoordination(float roll_rate, float yaw_rate, float airspeed);
    
private:
    HAL* hal_;
    Config config_;
    ControlSurfaces current_controls_;
    WindCompensation wind_compensation_;
    
    bool initialized_;
    bool throttle_passthrough_;
    bool emergency_mode_;
    
    // Servo mapping
    int left_elevon_channel_;
    int right_elevon_channel_;
    int rudder_channel_;
    int throttle_channel_;
    
    // Control surface trim positions
    float left_elevon_trim_;
    float right_elevon_trim_;
    float rudder_trim_;
    
    // Rate limiting for smooth control
    static const int RATE_LIMIT_SAMPLES = 5;
    float left_elevon_history_[RATE_LIMIT_SAMPLES];
    float right_elevon_history_[RATE_LIMIT_SAMPLES];
    float rudder_history_[RATE_LIMIT_SAMPLES];
    int history_index_;
    
    // Timing
    uint32_t last_update_time_;
    
    // Internal methods
    ControlSurfaces mixControls(const AngularRates& pid_outputs, const RadioInputs& radio_inputs, float airspeed);
    void applyWindCompensation(ControlSurfaces& controls, float airspeed);
    void applyAirspeedScaling(ControlSurfaces& controls, float airspeed);
    void applyRudderCoordination(ControlSurfaces& controls, const AngularRates& rates, float airspeed);
    void applySafetyLimits(ControlSurfaces& controls);
    void applyRateLimiting(ControlSurfaces& controls, float dt);
    
    float rateLimitControl(float new_value, const float* history, int samples, float max_rate, float dt);
    void updateControlHistory(float value, float* history);
    
    // Servo output helpers
    void writeServoPosition(int channel, float position_normalized);
    void writeThrottlePosition(int channel, float throttle_normalized);
}; 