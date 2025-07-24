#pragma once
#include "HAL.h"
#include <cstring>

// Platform-specific networking includes
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    #define sleep(x) Sleep((x)*1000)
    #define usleep(x) Sleep((x)/1000)
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
#endif

/**
 * FlightGear simulation implementation of Hardware Abstraction Layer
 * Interfaces with FlightGear via UDP for HITL simulation
 */
class FlightGearHAL : public HAL {
public:
    struct FlightGearData {
        float roll_deg;           // Aircraft roll angle
        float pitch_deg;          // Aircraft pitch angle  
        float heading_deg;        // Aircraft heading
        float roll_rate_degps;    // Roll rate deg/s
        float pitch_rate_degps;   // Pitch rate deg/s
        float yaw_rate_degps;     // Yaw rate deg/s
        float x_accel_fps2;       // X acceleration ft/s²
        float y_accel_fps2;       // Y acceleration ft/s²
        float z_accel_fps2;       // Z acceleration ft/s²
    };
    
    struct ControlData {
        float aileron;   // -1 to +1
        float elevator;  // -1 to +1
        float rudder;    // -1 to +1
        float throttle;  // 0 to 1
    };
    
    FlightGearHAL(const HardwareConfig& config);
    ~FlightGearHAL();
    
    // Core initialization
    bool initNetworking();
    void setFlightGearData(const FlightGearData& data);
    void setRadioInputs(float aileron, float elevator, float rudder, float throttle);
    
    // Timing
    uint32_t micros() override;
    uint32_t millis() override;
    void delay(uint32_t ms) override;
    
    // Serial Communication
    void serialBegin(uint32_t baud) override;
    void serialPrint(const char* str) override;
    void serialPrintln(const char* str) override;
    void serialPrintFloat(float value, int decimals = 2) override;
    
    // IMU Interface (simulated from FlightGear data - no magnetometer)
    bool initIMU() override;
    bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) override;
    bool isIMUHealthy() override;
    
    // Radio Interface (simulated/external input)
    bool initRadio() override;
    bool readRadio(float* channels, int num_channels) override;
    bool isRadioConnected() override;
    
    // Servo Interface (outputs to FlightGear)
    bool initServos() override;
    void writeServo(int channel, float position_0_to_1) override;
    
    // Motor Interface (outputs to FlightGear)
    bool initMotors() override;
    void writeMotor(int channel, float throttle_0_to_1) override;
    
    // Digital I/O (simulated)
    void digitalWrite(int pin, bool high) override;
    bool digitalRead(int pin) override;
    void pinMode(int pin, int mode) override;
    
    // LED/Status (console output)
    void setStatusLED(bool on) override;
    void blinkStatusLED(int count, int on_ms, int off_ms) override;
    
    // Get current control outputs for UDP transmission
    ControlData getControlOutputs() const { return control_outputs_; }

private:
    HardwareConfig config_;
    
    // Timing simulation
    uint64_t sim_start_time_us_;
    uint64_t sim_start_time_ms_;
    
    // FlightGear data
    FlightGearData fg_data_;
    bool fg_data_valid_;
    uint32_t last_fg_update_;
    
    // Radio simulation
    float radio_channels_[8];
    bool radio_connected_;
    uint32_t last_radio_update_;
    
    // Control outputs
    ControlData control_outputs_;
    
    // IMU bias simulation (for realism)
    float gyro_bias_[3];
    float accel_bias_[3];
    
    // Status
    bool imu_initialized_;
    bool radio_initialized_;
    bool servos_initialized_;
    bool motors_initialized_;
    bool networking_initialized_;
    
    // Helper methods
    void updateSystemTime();
    float degreesToRadians(float degrees) { return degrees * 0.017453292f; }
    float radiansToDegrees(float radians) { return radians * 57.295779f; }
    float feetPerSecToMPS(float fps) { return fps * 0.3048f; }
    float addNoise(float value, float noise_std);
}; 