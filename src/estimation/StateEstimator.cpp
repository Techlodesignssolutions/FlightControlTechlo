#ifdef _WIN32
#define _USE_MATH_DEFINES  // For M_PI on MSVC
#endif

#include "StateEstimator.h"
#include <cmath>
#include <algorithm>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Constructor
StateEstimator::StateEstimator(HAL* hal, const Config& config) 
    : hal_(hal), config_(config) {
    
    // Initialize quaternion to identity
    q0_ = 1.0f; q1_ = 0.0f; q2_ = 0.0f; q3_ = 0.0f;
    attitude_initialized_ = false;
    
    // Initialize state
    current_state_ = AircraftState();
    airspeed_state_ = AirspeedState();
    wind_state_ = WindState();
    
    // Initialize EKF state - Extended: [airspeed, drag_coefficient, wind_disturbance_x]
    airspeed_state_.airspeed = config_.initial_airspeed;
    airspeed_state_.drag_coefficient = config_.initial_drag_coeff;
    airspeed_state_.confidence = 0.0f;
    airspeed_state_.converged = false;
    airspeed_state_.innovation = 0.0f;
    airspeed_state_.last_update = 0;
    
    // Initialize wind state
    wind_state_.disturbance_accel = Vector3(0, 0, 0);
    wind_state_.confidence = Vector3(0, 0, 0);
    wind_state_.update_rate = 0.0f;
    wind_state_.converged = false;
    wind_state_.last_update = 0;
    
    // Initialize Extended EKF - 3 states: [airspeed, drag_coeff, wind_disturbance_x]
    ekf_state_[0] = config_.initial_airspeed;
    ekf_state_[1] = config_.initial_drag_coeff;
    ekf_state_[2] = 0.0f; // Initial wind disturbance
    
    // Initialize 3x3 covariance matrix
    for (int i = 0; i < 9; i++) ekf_covariance_[i] = 0.0f;
    ekf_covariance_[0] = 1.0f;  // P(0,0) - airspeed uncertainty
    ekf_covariance_[4] = 1.0f;  // P(1,1) - drag coeff uncertainty  
    ekf_covariance_[8] = 0.1f;  // P(2,2) - wind disturbance uncertainty
    ekf_initialized_ = false;
    
    // Initialize throttle dithering
    commanded_throttle_ = 0.0f;
    effective_throttle_ = 0.0f;
    dither_phase_ = 0.0f;
    last_dither_update_ = 0;
    
    // Initialize drag zones
    for (int i = 0; i < 6; i++) {
        drag_zones_[i].drag_coeff = config_.initial_drag_coeff;
        drag_zones_[i].confidence = 0.0f;
        drag_zones_[i].sample_count = 0;
        drag_zones_[i].rls_p = 1.0f;
    }
    
    // Initialize sensor data
    raw_gyro_ = Vector3(0, 0, 0);
    raw_accel_ = Vector3(0, 0, 0);
    raw_mag_ = Vector3(0, 0, 0);
    filtered_gyro_ = Vector3(0, 0, 0);
    filtered_accel_ = Vector3(0, 0, 0);
    filtered_mag_ = Vector3(0, 0, 0);
    
    // Initialize timing
    last_update_time_ = 0;
}

// Initialize the state estimator
bool StateEstimator::initialize() {
    if (!hal_) {
        return false;
    }
    
    // Initialize IMU
    if (!hal_->initIMU()) {
        return false;
    }
    
    // Calibrate gyro bias (simplified)
    if (!calibrateIMU()) {
        return false;
    }
    
    last_update_time_ = hal_->micros();
    attitude_initialized_ = true;
    ekf_initialized_ = true;
    
    return true;
}

// Main update function
bool StateEstimator::update(float dt) {
    if (!attitude_initialized_ || !hal_) {
        return false;
    }
    
    uint32_t current_time = hal_->micros();
    if (dt <= 0.0f) {
        dt = (current_time - last_update_time_) / 1000000.0f;
    }
    
    // Skip if called too frequently
    if (dt < 0.0005f) return true; // 2kHz max
    
    last_update_time_ = current_time;
    
    // Read and filter sensors
    if (!readSensors()) {
        return false;
    }
    filterSensors(dt);
    
    // Update attitude estimation
    updateAttitude(dt);
    
    // Update airspeed estimation with wind separation
    if (config_.enable_airspeed_ekf) {
        updateAirspeed(dt);
        updateWindEstimation(dt);
    }
    
    // Update throttle dithering
    if (config_.enable_throttle_dither) {
        updateThrottleDither(dt);
    }
    
    // Update current state structure
    current_state_.timestamp = current_time;
    current_state_.airspeed = airspeed_state_.airspeed;
    current_state_.wind_disturbance = wind_state_.disturbance_accel;
    
    return true;
}

// Set throttle command for airspeed estimation
void StateEstimator::setThrottleCommand(float throttle) {
    commanded_throttle_ = std::max(0.0f, std::min(1.0f, throttle));
    effective_throttle_ = commanded_throttle_;
}

// Calibrate IMU (simplified)
bool StateEstimator::calibrateIMU(int samples) {
    if (!hal_) return false;
    
    Vector3 gyro_sum(0, 0, 0);
    Vector3 accel_sum(0, 0, 0);
    
    // Collect samples
    for (int i = 0; i < samples; i++) {
        if (!readSensors()) {
            return false;
        }
        
        gyro_sum = gyro_sum + raw_gyro_;
        accel_sum = accel_sum + raw_accel_;
        hal_->delay(1); // 1ms delay between samples
    }
    
    // Calculate averages (bias)
    config_.gyro_bias.x = gyro_sum.x / samples;
    config_.gyro_bias.y = gyro_sum.y / samples;
    config_.gyro_bias.z = gyro_sum.z / samples;
    
    // Check if aircraft is level for accel calibration
    accel_sum.x /= samples;
    accel_sum.y /= samples;
    accel_sum.z /= samples;
    
    float accel_magnitude = accel_sum.magnitude();
    if (std::abs(accel_magnitude - 9.81f) < 2.0f) { // Within reasonable range
        config_.accel_bias.x = accel_sum.x;
        config_.accel_bias.y = accel_sum.y;
        config_.accel_bias.z = accel_sum.z - 9.81f; // Remove gravity
    }
    
    return true;
}

// Static Madgwick filter implementation
void StateEstimator::madgwickUpdate(float gx, float gy, float gz, 
                                   float ax, float ay, float az,
                                   float mx, float my, float mz,
                                   float dt, float beta,
                                   float& q0, float& q1, float& q2, float& q3) {
    
    float norm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        norm = std::sqrt(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;
        
        // Auxiliary variables
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;
        
        // Gradient descent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        // Normalise step magnitude
        norm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        s0 /= norm;
        s1 /= norm;
        s2 /= norm;
        s3 /= norm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    
    // Normalise quaternion
    norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

// Convert quaternion to Euler angles
Attitude StateEstimator::quaternionToEuler(float q0, float q1, float q2, float q3) {
    Attitude attitude;
    
    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    attitude.roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0f / 3.14159265f;
    
    // Pitch (y-axis rotation)
    float sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        attitude.pitch = std::copysign(90.0f, sinp); // Use 90 degrees if out of range
    else
        attitude.pitch = std::asin(sinp) * 180.0f / 3.14159265f;
    
    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    attitude.yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0f / 3.14159265f;
    
    return attitude;
}

// Calculate angle of attack
float StateEstimator::calculateAngleOfAttack(const Attitude& attitude, const Vector3& airflow) {
    // Simplified calculation
    return attitude.pitch;
}

// ========== PRIVATE METHOD IMPLEMENTATIONS ==========

// Read sensors
bool StateEstimator::readSensors() {
    float gyro_data[3], accel_data[3], mag_data[3];
    
    if (!hal_->readIMU(gyro_data, accel_data, mag_data)) {
        return false;
    }
    
    raw_gyro_ = Vector3(gyro_data[0], gyro_data[1], gyro_data[2]);
    raw_accel_ = Vector3(accel_data[0], accel_data[1], accel_data[2]);
    raw_mag_ = Vector3(mag_data[0], mag_data[1], mag_data[2]);
    
    return true;
}

// Filter sensors
void StateEstimator::filterSensors(float dt) {
    // Apply bias correction
    Vector3 corrected_gyro = raw_gyro_ - config_.gyro_bias;
    Vector3 corrected_accel = raw_accel_ - config_.accel_bias;
    Vector3 corrected_mag = Vector3(
        (raw_mag_.x - config_.mag_bias.x) * config_.mag_scale.x,
        (raw_mag_.y - config_.mag_bias.y) * config_.mag_scale.y,
        (raw_mag_.z - config_.mag_bias.z) * config_.mag_scale.z
    );
    
    // Apply low-pass filtering
    float alpha = 0.8f; // Filter coefficient
    filtered_gyro_ = lowPassFilter(corrected_gyro, filtered_gyro_, alpha);
    filtered_accel_ = lowPassFilter(corrected_accel, filtered_accel_, alpha);
    filtered_mag_ = lowPassFilter(corrected_mag, filtered_mag_, alpha);
}

// Update attitude estimation
void StateEstimator::updateAttitude(float dt) {
    // Use Madgwick filter
    madgwickUpdate(filtered_gyro_.x, filtered_gyro_.y, filtered_gyro_.z,
                   filtered_accel_.x, filtered_accel_.y, filtered_accel_.z,
                   filtered_mag_.x, filtered_mag_.y, filtered_mag_.z,
                   dt, config_.madgwick_beta,
                   q0_, q1_, q2_, q3_);
    
    // Convert quaternion to Euler angles
    current_state_.attitude = quaternionToEuler(q0_, q1_, q2_, q3_);
    
    // Store angular rates (with bias correction)
    current_state_.rates.roll_rate = filtered_gyro_.x;
    current_state_.rates.pitch_rate = filtered_gyro_.y;
    current_state_.rates.yaw_rate = filtered_gyro_.z;
    
    // Store acceleration
    current_state_.acceleration = filtered_accel_;
}

// Update airspeed estimation using Extended EKF
void StateEstimator::updateAirspeed(float dt) {
    // Use Extended EKF for thrust-drag-wind estimation
    float thrust_force = thrustToForce(effective_throttle_);
    
    // EKF predict step
    ekfPredictExtended(dt, thrust_force);
    
    // The EKF update will be called from updateWindEstimation()
    // to ensure proper sensor frame conversion and gravity removal
    
    // Update confidence based on innovation
    float abs_innovation = std::abs(airspeed_state_.innovation);
    if (abs_innovation < 0.5f) { // Good innovation
    airspeed_state_.confidence = std::min(1.0f, airspeed_state_.confidence + dt * 0.1f);
    } else { // Poor innovation
        airspeed_state_.confidence = std::max(0.1f, airspeed_state_.confidence - dt * 0.05f);
    }
    
    airspeed_state_.converged = airspeed_state_.confidence > 0.8f;
    airspeed_state_.last_update = hal_->millis();
    
    // Update current state
    current_state_.airspeed = airspeed_state_.airspeed;
}

// Update throttle dithering (simple and robust)
void StateEstimator::updateThrottleDither(float dt) {
    // Generate simple sinusoidal dither
    dither_phase_ += 2.0f * M_PI * config_.dither_frequency * dt;
    
    // Keep wrapping in reasonable range
    if (dither_phase_ > 2.0f * M_PI) {
        dither_phase_ -= 2.0f * M_PI;
    }
    
    // Apply dither to commanded throttle
    float dither = config_.dither_amplitude * std::sin(dither_phase_);
    effective_throttle_ = commanded_throttle_ + dither;
    
    // Clamp to valid throttle range
    effective_throttle_ = std::max(0.0f, std::min(1.0f, effective_throttle_));
    
    // Let the EKF learn whatever correlations actually exist
    // No complex motor lag modeling - keep it simple and robust!
    last_dither_update_ = hal_->millis();
}

// Update drag zone
void StateEstimator::updateDragZone(float aoa, float drag_estimate) {
    int zone_idx = getAOAZoneIndex(aoa);
    if (zone_idx >= 0 && zone_idx < 6) {
        drag_zones_[zone_idx].drag_coeff = drag_estimate;
        drag_zones_[zone_idx].sample_count++;
        drag_zones_[zone_idx].confidence = std::min(1.0f, drag_zones_[zone_idx].sample_count / 100.0f);
    }
}

// Get drag coefficient for AOA
float StateEstimator::getDragCoefficientForAOA(float aoa) const {
    int zone_idx = getAOAZoneIndex(aoa);
    if (zone_idx >= 0 && zone_idx < 6) {
        return drag_zones_[zone_idx].drag_coeff;
    }
    return config_.initial_drag_coeff;
}

// Get AOA zone index
int StateEstimator::getAOAZoneIndex(float aoa) const {
    for (int i = 0; i < config_.num_aoa_zones; i++) {
        if (aoa >= config_.aoa_zone_boundaries[i] && aoa < config_.aoa_zone_boundaries[i + 1]) {
            return i;
        }
    }
    return -1; // Invalid zone
}

// Thrust to force conversion
float StateEstimator::thrustToForce(float throttle_fraction) const {
    return throttle_fraction * 10.0f; // Simplified thrust model
}

// Low-pass filter helper
Vector3 StateEstimator::lowPassFilter(const Vector3& input, const Vector3& previous, float alpha) {
    return Vector3(
        alpha * previous.x + (1.0f - alpha) * input.x,
        alpha * previous.y + (1.0f - alpha) * input.y,
        alpha * previous.z + (1.0f - alpha) * input.z
    );
}

// ========== ENHANCED WIND ESTIMATION METHODS ==========

// Convert sensor frame acceleration to body frame
Vector3 StateEstimator::sensorToBodyFrame(const Vector3& sensor_accel) const {
    // Build rotation matrix from quaternion
    float R[9];
    buildRotationMatrix(R);
    
    // Apply rotation: body_accel = R * sensor_accel
    return Vector3(
        R[0] * sensor_accel.x + R[1] * sensor_accel.y + R[2] * sensor_accel.z,
        R[3] * sensor_accel.x + R[4] * sensor_accel.y + R[5] * sensor_accel.z,
        R[6] * sensor_accel.x + R[7] * sensor_accel.y + R[8] * sensor_accel.z
    );
}

// Remove gravity from body-frame acceleration
Vector3 StateEstimator::removeGravityFromAccel(const Vector3& body_accel) const {
    // Gravity in body frame (rotated from [0, 0, -9.81])
    Vector3 gravity_body;
    float R[9];
    buildRotationMatrix(R);
    
    // Earth gravity is [0, 0, -9.81] in earth frame
    // Rotate to body frame: gravity_body = R * [0, 0, -9.81]
    gravity_body.x = R[2] * (-9.81f);
    gravity_body.y = R[5] * (-9.81f);
    gravity_body.z = R[8] * (-9.81f);
    
    // Remove gravity to get linear acceleration
    return body_accel - gravity_body;
}

// Build 3x3 rotation matrix from quaternion
void StateEstimator::buildRotationMatrix(float R[9]) const {
    // Convert quaternion to rotation matrix
    float q0 = q0_, q1 = q1_, q2 = q2_, q3 = q3_;
    
    R[0] = 1 - 2*(q2*q2 + q3*q3);  R[1] = 2*(q1*q2 - q0*q3);      R[2] = 2*(q1*q3 + q0*q2);
    R[3] = 2*(q1*q2 + q0*q3);      R[4] = 1 - 2*(q1*q1 + q3*q3);  R[5] = 2*(q2*q3 - q0*q1);
    R[6] = 2*(q1*q3 - q0*q2);      R[7] = 2*(q2*q3 + q0*q1);      R[8] = 1 - 2*(q1*q1 + q2*q2);
}

// Update wind estimation using thrust-drag residual method
void StateEstimator::updateWindEstimation(float dt) {
    // Convert sensor acceleration to body frame and remove gravity
    Vector3 body_accel = sensorToBodyFrame(filtered_accel_);
    Vector3 linear_accel = removeGravityFromAccel(body_accel);
    
    // Store in state for debugging/logging
    current_state_.acceleration = body_accel;
    current_state_.linear_acceleration = linear_accel;
    
    // === 1D THRUST-DRAG BALANCE ON X-AXIS ===
    // Predict longitudinal acceleration from thrust and drag
    float thrust_force = thrustToForce(effective_throttle_);
    float drag_force = ekf_state_[1] * ekf_state_[0] * ekf_state_[0]; // cd * v^2
    float predicted_accel_x = (thrust_force - drag_force) / 1.0f; // assume 1kg mass
    
    // Residual = measured - predicted (this is wind + model error)
    float residual_x = linear_accel.x - predicted_accel_x;
    
    // === FREQUENCY SEPARATION ===
    // Fast component (throttle dither frequency) -> drag coefficient error
    // Slow component (below dither frequency) -> wind disturbance
    
    // Update Extended EKF with X-axis measurement
    ekfUpdateExtended(linear_accel.x);
    
    // Wind disturbance estimate from EKF state[2]
    wind_state_.disturbance_accel.x = ekf_state_[2];
    
    // For Y and Z axes, no thrust/drag model -> pure wind estimation
    // Apply simple low-pass filter to get slowly varying wind
    float wind_alpha = 0.95f; // Heavy filtering for wind (slow dynamics)
    wind_state_.disturbance_accel.y = wind_alpha * wind_state_.disturbance_accel.y + 
                                     (1.0f - wind_alpha) * linear_accel.y;
    wind_state_.disturbance_accel.z = wind_alpha * wind_state_.disturbance_accel.z + 
                                     (1.0f - wind_alpha) * linear_accel.z;
    
    // Update confidence based on filter convergence
    wind_state_.confidence.x = std::min(1.0f, wind_state_.confidence.x + dt * 0.1f);
    wind_state_.confidence.y = std::min(1.0f, wind_state_.confidence.y + dt * 0.05f);
    wind_state_.confidence.z = std::min(1.0f, wind_state_.confidence.z + dt * 0.05f);
    wind_state_.converged = (wind_state_.confidence.x > 0.8f);
    wind_state_.last_update = hal_->millis();
}

// Extended EKF predict step - now includes wind disturbance state
void StateEstimator::ekfPredictExtended(float dt, float thrust_force) {
    // State: [airspeed, drag_coefficient, wind_disturbance_x]
    // Dynamics: 
    //   airspeed: v' = (thrust - drag + wind) / mass
    //   drag_coeff: constant (with process noise)
    //   wind_disturbance: constant (with process noise)
    
    float predicted_accel = (thrust_force - ekf_state_[1] * ekf_state_[0] * ekf_state_[0] + ekf_state_[2]) / 1.0f;
    ekf_state_[0] += predicted_accel * dt;
    // ekf_state_[1] unchanged (drag coefficient)
    // ekf_state_[2] unchanged (wind disturbance)
    
    // Process noise covariance update (simplified)
    ekf_covariance_[0] += config_.process_noise_airspeed * dt;        // Airspeed uncertainty
    ekf_covariance_[4] += config_.process_noise_drag * dt;            // Drag coeff uncertainty  
    ekf_covariance_[8] += 0.01f * dt;                                 // Wind disturbance uncertainty
}

// Extended EKF update step
void StateEstimator::ekfUpdateExtended(float measured_accel_x) {
    // Measurement model: z = (thrust - drag + wind) / mass
    float predicted_measurement = (thrustToForce(effective_throttle_) - 
                                  ekf_state_[1] * ekf_state_[0] * ekf_state_[0] + 
                                  ekf_state_[2]) / 1.0f;
    
    float innovation = measured_accel_x - predicted_measurement;
    
    // Measurement Jacobian H = [∂h/∂v, ∂h/∂cd, ∂h/∂wind] = [-2*cd*v/m, -v²/m, 1/m]
    float H[3];
    H[0] = -2.0f * ekf_state_[1] * ekf_state_[0] / 1.0f;  // ∂h/∂airspeed
    H[1] = -ekf_state_[0] * ekf_state_[0] / 1.0f;         // ∂h/∂drag_coeff
    H[2] = 1.0f / 1.0f;                                   // ∂h/∂wind_disturbance
    
    // Innovation covariance S = H*P*H' + R
    float S = 0.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S += H[i] * ekf_covariance_[i*3 + j] * H[j];
        }
    }
    S += config_.measurement_noise;
    
    // Kalman gain K = P*H'/S
    float K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            K[i] += ekf_covariance_[i*3 + j] * H[j];
        }
        K[i] /= S;
    }
    
    // State update: x = x + K*innovation
    for (int i = 0; i < 3; i++) {
        ekf_state_[i] += K[i] * innovation;
    }
    
    // Covariance update: P = (I - K*H)*P (simplified)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ekf_covariance_[i*3 + j] -= K[i] * H[j] * ekf_covariance_[i*3 + j];
        }
    }
    
    // Update airspeed state
    airspeed_state_.airspeed = ekf_state_[0];
    airspeed_state_.drag_coefficient = ekf_state_[1];
    airspeed_state_.innovation = innovation;
} 