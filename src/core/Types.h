#pragma once
#include <cmath>
#include <cstdint>  // for uint32_t

/**
 * Core data types used throughout the flight controller
 */

/**
 * 3D vector for representing positions, velocities, attitudes, etc.
 */
struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    
    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    
    Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    
    float magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
    
    void normalize() {
        float mag = magnitude();
        if (mag > 0.001f) {
            x /= mag; y /= mag; z /= mag;
        }
    }
};

/**
 * Aircraft attitude (orientation)
 */
struct Attitude {
    float roll;     // degrees, positive = right wing down
    float pitch;    // degrees, positive = nose up  
    float yaw;      // degrees, positive = nose right
    
    Attitude() : roll(0), pitch(0), yaw(0) {}
    Attitude(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

/**
 * Aircraft angular rates
 */
struct AngularRates {
    float roll_rate;   // deg/s, positive = rolling right
    float pitch_rate;  // deg/s, positive = pitching up
    float yaw_rate;    // deg/s, positive = yawing right
    
    AngularRates() : roll_rate(0), pitch_rate(0), yaw_rate(0) {}
    AngularRates(float r, float p, float y) : roll_rate(r), pitch_rate(p), yaw_rate(y) {}
};

/**
 * PID controller gains
 */
struct PIDGains {
    float kp;  // Proportional gain
    float ki;  // Integral gain  
    float kd;  // Derivative gain
    
    PIDGains() : kp(0), ki(0), kd(0) {}
    PIDGains(float p, float i, float d) : kp(p), ki(i), kd(d) {}
};

/**
 * Complete set of PID gains for all axes
 */
struct ControlGains {
    PIDGains roll;
    PIDGains pitch;
    PIDGains yaw;
    
    ControlGains() {}
    ControlGains(const PIDGains& r, const PIDGains& p, const PIDGains& y) 
        : roll(r), pitch(p), yaw(y) {}
};

/**
 * Radio control inputs (normalized -1 to +1)
 */
struct RadioInputs {
    float throttle;    // 0 to 1
    float roll;        // -1 to +1, positive = roll right
    float pitch;       // -1 to +1, positive = pitch up
    float yaw;         // -1 to +1, positive = yaw right
    float aux1;        // -1 to +1, auxiliary channel 1
    float aux2;        // -1 to +1, auxiliary channel 2
    bool armed;        // true if aircraft should be armed
    uint32_t last_update_time;  // timestamp of last radio update (milliseconds)
    
    RadioInputs() : throttle(0.0f), roll(0.0f), pitch(0.0f), yaw(0.0f), 
                   aux1(0.0f), aux2(0.0f), armed(false), last_update_time(0) {}
};

/**
 * Control surface commands (normalized)
 */
struct ControlSurfaces {
    float left_elevon;   // -1 to +1, positive = trailing edge down
    float right_elevon;  // -1 to +1, positive = trailing edge down  
    float rudder;        // -1 to +1, positive = trailing edge left
    float throttle;      // 0 to 1, motor throttle
    
    ControlSurfaces() : left_elevon(0), right_elevon(0), rudder(0), throttle(0) {}
    ControlSurfaces(float l, float r, float rud, float thr) 
        : left_elevon(l), right_elevon(r), rudder(rud), throttle(thr) {}
};

/**
 * Aircraft state estimate
 */
struct AircraftState {
    Attitude attitude;
    AngularRates rates;
    Vector3 acceleration;      // Body frame, m/s² (bias and gravity corrected)
    Vector3 linear_acceleration; // Body frame linear accel (gravity removed)
    Vector3 wind_disturbance;  // Body frame wind acceleration (m/s²)
    float airspeed;            // m/s
    float altitude;            // m (if available)
    uint32_t timestamp;        // microseconds
    
    AircraftState() : airspeed(0), altitude(0), timestamp(0) {}
};

/**
 * Flight phase enumeration
 */
enum class FlightPhase {
    UNKNOWN = 0,
    PREFLIGHT = 1,
    TAKEOFF = 2, 
    CLIMB = 3,
    CRUISE = 4,
    DESCENT = 5,
    APPROACH = 6,
    LANDING = 7,
    EMERGENCY = 8
};

/**
 * Control mode enumeration
 */
enum class ControlMode {
    MANUAL = 0,      // Direct servo passthrough
    STABILIZE = 1,   // Attitude stabilization
    ALTITUDE = 2,    // Altitude hold
    POSITION = 3,    // Position hold
    AUTO = 4         // Autonomous flight
};

/**
 * Flight regime for adaptive control
 */
struct FlightRegime {
    float throttle_fraction;  // 0 to 1
    float angle_of_attack;    // degrees
    float airspeed;          // m/s
    FlightPhase phase;
    
    FlightRegime() : throttle_fraction(0), angle_of_attack(0), airspeed(0), phase(FlightPhase::UNKNOWN) {}
};

/**
 * Performance metrics for adaptive control
 */
struct PerformanceMetrics {
    float overshoot;        // Maximum overshoot (0-1)
    float settling_time;    // Time to settle (seconds)
    float steady_error;     // Steady-state error
    float oscillation_freq; // Oscillation frequency (Hz)
    float control_effort;   // RMS control effort
    float health_score;     // Overall health (0-100)
    
    PerformanceMetrics() : overshoot(0), settling_time(0), steady_error(0), 
                          oscillation_freq(0), control_effort(0), health_score(50) {}
}; 

/**
 * Performance statistics for loop timing and CPU usage
 */
struct PerformanceStats {
    float loop_time_avg_ms;     // Average loop time in milliseconds
    float loop_time_max_ms;     // Maximum loop time in milliseconds
    uint32_t loop_count;        // Total number of loops executed
    uint32_t overrun_count;     // Number of loop overruns
    float cpu_usage_percent;    // CPU usage percentage
    
    PerformanceStats() : loop_time_avg_ms(0.0f), loop_time_max_ms(0.0f), 
                        loop_count(0), overrun_count(0), cpu_usage_percent(0.0f) {}
    
    PerformanceStats(float avg, float max, uint32_t count, uint32_t overruns, float cpu)
        : loop_time_avg_ms(avg), loop_time_max_ms(max), loop_count(count), 
          overrun_count(overruns), cpu_usage_percent(cpu) {}
};

/**
 * Wind compensation parameters for flight control
 */
struct WindCompensation {
    float headwind_compensation;   // Compensation for headwind (m/s²)
    float crosswind_compensation;  // Compensation for crosswind (m/s²) 
    float vertical_compensation;   // Compensation for vertical wind (m/s²)
    float confidence;              // Confidence level (0-1)
    bool active;                   // Whether wind compensation is active
    
    WindCompensation() : headwind_compensation(0.0f), crosswind_compensation(0.0f),
                        vertical_compensation(0.0f), confidence(0.0f), active(false) {}
};

/**
 * Diagnostic information for system health monitoring
 */
struct DiagnosticInfo {
    bool imu_healthy;                     // IMU sensor health
    bool radio_healthy;                   // Radio receiver health
    bool airspeed_converged;              // Airspeed estimation convergence
    bool adaptive_pid_learning;           // PID learning state
    float wind_compensation_confidence;   // Wind compensation confidence
    uint32_t uptime_ms;                   // System uptime in milliseconds
    
    DiagnosticInfo() : imu_healthy(false), radio_healthy(false), 
                      airspeed_converged(false), adaptive_pid_learning(false),
                      wind_compensation_confidence(0.0f), uptime_ms(0) {}
}; 