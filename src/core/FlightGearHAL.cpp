#include "FlightGearHAL.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <random>

FlightGearHAL::FlightGearHAL(const HardwareConfig& config) 
    : config_(config)
    , fg_data_valid_(false)
    , radio_connected_(false)
    , imu_initialized_(false)
    , radio_initialized_(false)
    , servos_initialized_(false)
    , motors_initialized_(false)
    , networking_initialized_(false)
{
    // Initialize Windows networking if needed
    #ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("[FlightGearHAL] ERROR: WSAStartup failed\n");
    } else {
        networking_initialized_ = true;
    }
    #else
    networking_initialized_ = true;
    #endif
    
    // Initialize timing
    updateSystemTime();
    sim_start_time_us_ = micros();
    sim_start_time_ms_ = millis();
    
    // Initialize data structures
    memset(&fg_data_, 0, sizeof(fg_data_));
    memset(&control_outputs_, 0, sizeof(control_outputs_));
    memset(radio_channels_, 0, sizeof(radio_channels_));
    
    // Initialize biases with small random values for realism
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<float> bias_dist(0.0f, 0.01f);
    
    for (int i = 0; i < 3; i++) {
        gyro_bias_[i] = bias_dist(gen);
        accel_bias_[i] = bias_dist(gen);
    }
    
    last_fg_update_ = 0;
    last_radio_update_ = 0;
}

FlightGearHAL::~FlightGearHAL() {
    // Cleanup Windows networking
    #ifdef _WIN32
    if (networking_initialized_) {
        WSACleanup();
    }
    #endif
}

void FlightGearHAL::updateSystemTime() {
    // Get system time for simulation
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto millis_count = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    auto micros_count = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    
    sim_start_time_ms_ = static_cast<uint64_t>(millis_count);
    sim_start_time_us_ = static_cast<uint64_t>(micros_count);
}

uint32_t FlightGearHAL::micros() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    return static_cast<uint32_t>(us - sim_start_time_us_);
}

uint32_t FlightGearHAL::millis() {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    return static_cast<uint32_t>(ms - sim_start_time_ms_);
}

void FlightGearHAL::delay(uint32_t ms) {
    #ifdef _WIN32
    Sleep(ms);
    #else
    usleep(ms * 1000);
    #endif
}

//=== Serial Communication ===
void FlightGearHAL::serialBegin(uint32_t baud) {
    printf("[FlightGearHAL] Serial initialized at %u baud\n", baud);
}

void FlightGearHAL::serialPrint(const char* str) {
    printf("%s", str);
    fflush(stdout);  // Ensure immediate output on Windows
}

void FlightGearHAL::serialPrintln(const char* str) {
    printf("%s\n", str);
    fflush(stdout);  // Ensure immediate output on Windows
}

void FlightGearHAL::serialPrintFloat(float value, int decimals) {
    printf("%.*f", decimals, value);
    fflush(stdout);  // Ensure immediate output on Windows
}

//=== FlightGear Data Interface ===
void FlightGearHAL::setFlightGearData(const FlightGearData& data) {
    fg_data_ = data;
    fg_data_valid_ = true;
    last_fg_update_ = millis();
}

void FlightGearHAL::setRadioInputs(float aileron, float elevator, float rudder, float throttle) {
    radio_channels_[0] = throttle;   // Channel 0: Throttle (0 to 1)
    radio_channels_[1] = aileron;    // Channel 1: Roll (-1 to 1)
    radio_channels_[2] = elevator;   // Channel 2: Pitch (-1 to 1)
    radio_channels_[3] = rudder;     // Channel 3: Yaw (-1 to 1)
    radio_channels_[4] = 0.0f;       // Channel 4: Aux1
    radio_channels_[5] = 0.0f;       // Channel 5: Aux2
    
    radio_connected_ = true;
    last_radio_update_ = millis();
}

//=== IMU Interface ===
bool FlightGearHAL::initIMU() {
    printf("[FlightGearHAL] IMU initialized (simulated - gyro + accel only, no magnetometer)\n");
    imu_initialized_ = true;
    return true;
}

float FlightGearHAL::addNoise(float value, float noise_std) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::normal_distribution<float> noise_dist(0.0f, noise_std);
    return value + noise_dist(gen);
}

bool FlightGearHAL::readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) {
    if (!imu_initialized_) {
        return false;
    }
    
    // If FlightGear data not available yet, provide default stationary aircraft data
    if (!fg_data_valid_) {
        // Stationary aircraft on ground - zero rates
        gyro_xyz[0] = addNoise(0.0f + gyro_bias_[0], 0.01f);   // Roll rate
        gyro_xyz[1] = addNoise(0.0f + gyro_bias_[1], 0.01f);   // Pitch rate  
        gyro_xyz[2] = addNoise(0.0f + gyro_bias_[2], 0.01f);   // Yaw rate
        
        // Gravity vector (aircraft level)
        accel_xyz[0] = addNoise(0.0f + accel_bias_[0], 0.02f);     // X: 0 (level)
        accel_xyz[1] = addNoise(0.0f + accel_bias_[1], 0.02f);     // Y: 0 (level)
        accel_xyz[2] = addNoise(-9.81f + accel_bias_[2], 0.02f);   // Z: -9.81 (gravity down)
        
        // No magnetometer
        if (mag_xyz) {
            mag_xyz[0] = 0.0f;
            mag_xyz[1] = 0.0f;
            mag_xyz[2] = 0.0f;
        }
        
        return true;
    }
    
    // Convert FlightGear data to IMU format with realistic noise
    
    // Gyroscope (deg/s) - add bias and noise
    gyro_xyz[0] = addNoise(fg_data_.roll_rate_degps + gyro_bias_[0], 0.1f);   // Roll rate
    gyro_xyz[1] = addNoise(fg_data_.pitch_rate_degps + gyro_bias_[1], 0.1f);  // Pitch rate
    gyro_xyz[2] = addNoise(fg_data_.yaw_rate_degps + gyro_bias_[2], 0.1f);    // Yaw rate
    
    // Accelerometer (m/s²) - convert from ft/s² and add bias/noise
    accel_xyz[0] = addNoise(feetPerSecToMPS(fg_data_.x_accel_fps2) + accel_bias_[0], 0.05f);
    accel_xyz[1] = addNoise(feetPerSecToMPS(fg_data_.y_accel_fps2) + accel_bias_[1], 0.05f);
    accel_xyz[2] = addNoise(feetPerSecToMPS(fg_data_.z_accel_fps2) + accel_bias_[2], 0.05f);
    
    // No magnetometer available - set to null if pointer provided
    // This simulates a flight controller without magnetometer hardware
    if (mag_xyz) {
        mag_xyz[0] = 0.0f;
        mag_xyz[1] = 0.0f;
        mag_xyz[2] = 0.0f;
    }
    
    return true;
}

bool FlightGearHAL::isIMUHealthy() {
    return imu_initialized_ && fg_data_valid_ && (millis() - last_fg_update_ < 100);
}

//=== Radio Interface ===
bool FlightGearHAL::initRadio() {
    printf("[FlightGearHAL] Radio initialized (simulated)\n");
    radio_initialized_ = true;
    return true;
}

bool FlightGearHAL::readRadio(float* channels, int num_channels) {
    if (!radio_initialized_ || !radio_connected_) {
        return false;
    }
    
    int copy_count = (num_channels < 8) ? num_channels : 8;
    for (int i = 0; i < copy_count; i++) {
        channels[i] = radio_channels_[i];
    }
    
    return true;
}

bool FlightGearHAL::isRadioConnected() {
    return radio_connected_ && (millis() - last_radio_update_ < 1000);
}

//=== Servo Interface ===
bool FlightGearHAL::initServos() {
    printf("[FlightGearHAL] Servos initialized (simulated)\n");
    servos_initialized_ = true;
    return true;
}

void FlightGearHAL::writeServo(int channel, float position_0_to_1) {
    if (!servos_initialized_) return;
    
    // Map servo positions to control surfaces
    // Convert 0-1 range to -1 to +1 range for control surfaces
    float control_value = (position_0_to_1 - 0.5f) * 2.0f;
    
    switch (channel) {
        case 0: // Left elevon
        case 1: // Right elevon
            // Average elevon positions for elevator
            control_outputs_.elevator = control_value;
            break;
        case 2: // Rudder
            control_outputs_.rudder = control_value;
            break;
        default:
            // Additional channels if needed
            break;
    }
}

//=== Motor Interface ===
bool FlightGearHAL::initMotors() {
    printf("[FlightGearHAL] Motors initialized (simulated)\n");
    motors_initialized_ = true;
    return true;
}

void FlightGearHAL::writeMotor(int channel, float throttle_0_to_1) {
    if (!motors_initialized_) return;
    
    if (channel == 0) {  // Main throttle
        control_outputs_.throttle = throttle_0_to_1;
    }
}

//=== Digital I/O ===
void FlightGearHAL::digitalWrite(int pin, bool high) {
    // Simulated - could log for debugging
}

bool FlightGearHAL::digitalRead(int pin) {
    // Simulated - return false for now
    return false;
}

void FlightGearHAL::pinMode(int pin, int mode) {
    // Simulated - no action needed
}

//=== LED/Status ===
void FlightGearHAL::setStatusLED(bool on) {
    printf("[LED] %s\n", on ? "ON" : "OFF");
}

void FlightGearHAL::blinkStatusLED(int count, int on_ms, int off_ms) {
    printf("[LED] Blink %d times (%dms on, %dms off)\n", count, on_ms, off_ms);
} 