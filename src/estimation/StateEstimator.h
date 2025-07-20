#pragma once
#include "../core/Types.h"
#include "../core/HAL.h"
#include <stdint.h>

/**
 * State Estimator Module
 * 
 * Responsible for estimating aircraft state from sensor inputs:
 * - Attitude estimation using Madgwick filter
 * - Airspeed estimation using EKF and thrust-drag dynamics
 * - Sensor fusion and filtering
 * 
 * Design principles:
 * - Hardware agnostic (uses HAL interface)
 * - Testable (algorithms can be verified with synthetic data) 
 * - Modular (attitude and airspeed can be used independently)
 */
class StateEstimator {
public:
    /**
     * Configuration for state estimation
     */
    struct Config {
        // Madgwick filter parameters
        float madgwick_beta = 0.1f;      // Filter gain (higher = more responsive, more noise)
        bool use_magnetometer = true;     // Use mag for yaw correction
        
        // Airspeed EKF parameters
        bool enable_airspeed_ekf = true;
        float process_noise_airspeed = 0.1f;
        float process_noise_drag = 0.01f;
        float measurement_noise = 0.5f;
        float initial_airspeed = 0.0f;
        float initial_drag_coeff = 0.03f;
        
        // Throttle dithering for observability
        bool enable_throttle_dither = true;
        float dither_amplitude = 0.02f;   // ±2% throttle variation
        float dither_frequency = 2.0f;    // 2 Hz sinusoidal dither
        
        // Zone-based drag learning
        int num_aoa_zones = 5;
        float aoa_zone_boundaries[6] = {-15.0f, -5.0f, 0.0f, 5.0f, 15.0f, 30.0f};
        
        // Sensor calibration
        Vector3 gyro_bias = Vector3(0, 0, 0);
        Vector3 accel_bias = Vector3(0, 0, 0);
        Vector3 mag_bias = Vector3(0, 0, 0);
        Vector3 mag_scale = Vector3(1, 1, 1);
    };
    
    /**
     * Airspeed estimation state
     */
    struct AirspeedState {
        float airspeed;          // Estimated airspeed (m/s)
        float drag_coefficient;  // Current drag coefficient
        float confidence;        // Estimation confidence (0-1)
        bool converged;          // Has EKF converged?
        float innovation;        // Latest measurement innovation
        uint32_t last_update;    // Last update timestamp
    };
    
    /**
     * Constructor
     */
    StateEstimator(HAL* hal, const Config& config = Config());
    
    /**
     * Initialize the state estimator
     */
    bool initialize();
    
    /**
     * Update state estimation with latest sensor data
     * Call this every control loop
     */
    bool update(float dt);
    
    /**
     * Get current aircraft state estimate
     */
    AircraftState getState() const { return current_state_; }
    
    /**
     * Get attitude estimate
     */
    Attitude getAttitude() const { return current_state_.attitude; }
    
    /**
     * Get angular rates estimate
     */
    AngularRates getRates() const { return current_state_.rates; }
    
    /**
     * Get airspeed state
     */
    AirspeedState getAirspeedState() const { return airspeed_state_; }
    
    /**
     * Set throttle command for airspeed estimation
     * The estimator may apply dithering for observability
     */
    void setThrottleCommand(float throttle);
    
    /**
     * Get effective throttle (with dithering applied)
     */
    float getEffectiveThrottle() const { return effective_throttle_; }
    
    /**
     * Calibrate sensors (call when stationary)
     */
    bool calibrateIMU(int samples = 1000);
    
    /**
     * Static utility functions (testable)
     */
    static void madgwickUpdate(float gx, float gy, float gz, 
                              float ax, float ay, float az,
                              float mx, float my, float mz,
                              float dt, float beta,
                              float& q0, float& q1, float& q2, float& q3);
    
    static Attitude quaternionToEuler(float q0, float q1, float q2, float q3);
    
    static float calculateAngleOfAttack(const Attitude& attitude, const Vector3& airflow);
    
private:
    HAL* hal_;
    Config config_;
    AircraftState current_state_;
    AirspeedState airspeed_state_;
    
    // Attitude estimation (Madgwick filter)
    float q0_, q1_, q2_, q3_;  // Quaternion state
    bool attitude_initialized_;
    
    // Airspeed EKF state
    float ekf_state_[2];      // [airspeed, drag_coefficient]
    float ekf_covariance_[4]; // 2x2 covariance matrix (stored as [P00, P01, P10, P11])
    bool ekf_initialized_;
    
    // Throttle dithering
    float commanded_throttle_;
    float effective_throttle_;
    float dither_phase_;
    uint32_t last_dither_update_;
    
    // Drag coefficient learning per AOA zone
    struct DragZone {
        float drag_coeff;
        float confidence;
        int sample_count;
        float rls_p;  // RLS parameter estimate covariance
    };
    DragZone drag_zones_[6];  // One per AOA zone
    
    // Sensor data buffers
    Vector3 raw_gyro_;
    Vector3 raw_accel_;
    Vector3 raw_mag_;
    Vector3 filtered_gyro_;
    Vector3 filtered_accel_;
    Vector3 filtered_mag_;
    
    // Timing
    uint32_t last_update_time_;
    
    // Internal methods
    bool readSensors();
    void filterSensors(float dt);
    void updateAttitude(float dt);
    void updateAirspeed(float dt);
    void updateThrottleDither(float dt);
    void updateDragZone(float aoa, float drag_estimate);
    float getDragCoefficientForAOA(float aoa) const;
    int getAOAZoneIndex(float aoa) const;
    
    // EKF implementation
    void ekfPredict(float dt, float thrust_force);
    void ekfUpdate(float measured_accel_x);
    float thrustToForce(float throttle_fraction) const;
    
    // Low-pass filter helper
    static Vector3 lowPassFilter(const Vector3& input, const Vector3& previous, float alpha);
}; 