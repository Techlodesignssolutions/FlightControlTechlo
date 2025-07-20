#pragma once
#include "core/HAL.h"
#include <vector>
#include <queue>
#include <map>
#include <string>
#include <cstring>
#include <cstdio>

/**
 * Mock Hardware Abstraction Layer for Testing
 * 
 * This provides controllable "fake" hardware that allows testing
 * flight controller algorithms on desktop without real sensors/actuators.
 */
class MockHAL : public HAL {
public:
    //=== Time Control ===
    uint32_t current_micros = 0;
    uint32_t current_millis = 0;
    
    void advanceTime(uint32_t microseconds) { 
        current_micros += microseconds; 
        current_millis = current_micros / 1000; 
    }
    
    uint32_t micros() override { return current_micros; }
    uint32_t millis() override { return current_millis; }
    void delay(uint32_t ms) override { advanceTime(ms * 1000); }
    
    //=== Serial Communication (Captured for Verification) ===
    std::vector<std::string> serial_output;
    
    void serialBegin(uint32_t baud) override { 
        serial_output.push_back("Serial.begin(" + std::to_string(baud) + ")");
    }
    
    void serialPrint(const char* str) override { 
        serial_output.push_back(str); 
    }
    
    void serialPrintln(const char* str) override { 
        serial_output.push_back(std::string(str) + "\n"); 
    }
    
    void serialPrintFloat(float value, int decimals = 2) override {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.*f", decimals, value);
        serial_output.push_back(buf);
    }
    
    //=== IMU Data Queue ===
    struct IMUFrame {
        float gyro[3];    // rad/s
        float accel[3];   // m/s²
        float mag[3];     // normalized
        
        IMUFrame(float gx=0, float gy=0, float gz=0,
                float ax=0, float ay=0, float az=1,
                float mx=1, float my=0, float mz=0) {
            gyro[0]=gx; gyro[1]=gy; gyro[2]=gz;
            accel[0]=ax; accel[1]=ay; accel[2]=az;
            mag[0]=mx; mag[1]=my; mag[2]=mz;
        }
    };
    
    std::queue<IMUFrame> imu_data_queue;
    bool imu_initialized = false;
    
    bool initIMU() override { 
        imu_initialized = true;
        return true; 
    }
    
    bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) override {
        if (imu_data_queue.empty()) return false;
        
        auto frame = imu_data_queue.front();
        imu_data_queue.pop();
        
        memcpy(gyro_xyz, frame.gyro, 3 * sizeof(float));
        memcpy(accel_xyz, frame.accel, 3 * sizeof(float));
        memcpy(mag_xyz, frame.mag, 3 * sizeof(float));
        
        return true;
    }
    
    // Helper methods for tests
    void pushIMUData(const IMUFrame& frame) {
        imu_data_queue.push(frame);
    }
    
    void pushLevelIMUData(int count = 1) {
        for (int i = 0; i < count; i++) {
            pushIMUData(IMUFrame(0,0,0, 0,0,1, 1,0,0));  // Level, no rotation
        }
    }
    
    //=== Radio Data Queue ===
    std::queue<std::vector<float>> radio_data_queue;
    bool radio_initialized = false;
    bool radio_connected = true;
    
    bool initRadio() override { 
        radio_initialized = true;
        return true; 
    }
    
    bool readRadio(float* channels, int num_channels) override {
        if (radio_data_queue.empty()) return false;
        
        auto data = radio_data_queue.front();
        radio_data_queue.pop();
        
        for (int i = 0; i < num_channels && i < (int)data.size(); i++) {
            channels[i] = data[i];
        }
        
        return true;
    }
    
    bool isRadioConnected() override { return radio_connected; }
    
    // Helper methods for tests
    void pushRadioData(const std::vector<float>& channels) {
        radio_data_queue.push(channels);
    }
    
    void setRadioConnected(bool connected) { radio_connected = connected; }
    
    //=== Servo/Motor Output Recording ===
    struct ActuatorCommand {
        int channel;
        float value;
        uint32_t timestamp;
        
        ActuatorCommand(int ch, float val, uint32_t ts) 
            : channel(ch), value(val), timestamp(ts) {}
    };
    
    std::vector<ActuatorCommand> servo_commands;
    std::vector<ActuatorCommand> motor_commands;
    bool servos_initialized = false;
    bool motors_initialized = false;
    
    bool initServos() override { 
        servos_initialized = true;
        return true; 
    }
    
    void writeServo(int channel, float position_0_to_1) override {
        servo_commands.emplace_back(channel, position_0_to_1, current_micros);
    }
    
    bool initMotors() override { 
        motors_initialized = true;
        return true; 
    }
    
    void writeMotor(int channel, float throttle_0_to_1) override {
        motor_commands.emplace_back(channel, throttle_0_to_1, current_micros);
    }
    
    //=== Digital I/O ===
    std::map<int, bool> pin_states;
    std::map<int, int> pin_modes;
    
    void digitalWrite(int pin, bool high) override {
        pin_states[pin] = high;
    }
    
    bool digitalRead(int pin) override {
        return pin_states[pin];  // defaults to false
    }
    
    void pinMode(int pin, int mode) override {
        pin_modes[pin] = mode;
    }
    
    //=== Status LED ===
    bool status_led_on = false;
    std::vector<std::pair<uint32_t, bool>> led_history;
    
    void setStatusLED(bool on) override {
        status_led_on = on;
        led_history.emplace_back(current_micros, on);
    }
    
    void blinkStatusLED(int count, int on_ms, int off_ms) override {
        for (int i = 0; i < count; i++) {
            setStatusLED(true);
            advanceTime(on_ms * 1000);
            setStatusLED(false);
            advanceTime(off_ms * 1000);
        }
    }
    
    //=== Test Helper Methods ===
    void clearHistory() {
        serial_output.clear();
        servo_commands.clear();
        motor_commands.clear();
        led_history.clear();
    }
    
    ActuatorCommand getLastServoCommand(int channel) const {
        for (auto it = servo_commands.rbegin(); it != servo_commands.rend(); ++it) {
            if (it->channel == channel) return *it;
        }
        return ActuatorCommand(-1, 0, 0);  // Not found
    }
    
    ActuatorCommand getLastMotorCommand(int channel) const {
        for (auto it = motor_commands.rbegin(); it != motor_commands.rend(); ++it) {
            if (it->channel == channel) return *it;
        }
        return ActuatorCommand(-1, 0, 0);  // Not found
    }
    
    std::string getSerialOutput() const {
        std::string result;
        for (const auto& line : serial_output) {
            result += line;
        }
        return result;
    }
    
    bool wasSerialOutputContaining(const std::string& text) const {
        std::string full_output = getSerialOutput();
        return full_output.find(text) != std::string::npos;
    }
}; 