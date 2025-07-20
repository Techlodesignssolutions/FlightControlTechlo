#pragma once
#include "HAL.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>

/**
 * Teensy implementation of Hardware Abstraction Layer
 */
class TeensyHAL : public HAL {
public:
    TeensyHAL(const HardwareConfig& config);
    
    // Timing
    uint32_t micros() override { return ::micros(); }
    uint32_t millis() override { return ::millis(); }
    void delay(uint32_t ms) override { ::delay(ms); }
    
    // Serial Communication
    void serialBegin(uint32_t baud) override;
    void serialPrint(const char* str) override;
    void serialPrintln(const char* str) override;
    void serialPrintFloat(float value, int decimals = 2) override;
    
    // IMU Interface
    bool initIMU() override;
    bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) override;
    
    // Radio Interface
    bool initRadio() override;
    bool readRadio(float* channels, int num_channels) override;
    bool isRadioConnected() override;
    
    // Servo Interface
    bool initServos() override;
    void writeServo(int channel, float position_0_to_1) override;
    
    // Motor Interface
    bool initMotors() override;
    void writeMotor(int channel, float throttle_0_to_1) override;
    
    // Digital I/O
    void digitalWrite(int pin, bool high) override;
    bool digitalRead(int pin) override;
    void pinMode(int pin, int mode) override;
    
    // LED/Status
    void setStatusLED(bool on) override;
    void blinkStatusLED(int count, int on_ms, int off_ms) override;
    
private:
    HardwareConfig config_;
    Servo servos_[8];
    bool imu_initialized_ = false;
    bool radio_initialized_ = false;
    bool servos_initialized_ = false;
    bool motors_initialized_ = false;
    
    // IMU-specific variables
    float gyro_bias_[3] = {0, 0, 0};
    float accel_bias_[3] = {0, 0, 0};
    
    // Radio-specific variables
    uint32_t last_radio_update_ = 0;
    bool radio_connected_ = false;
    
    // Internal helper methods
    bool calibrateIMU();
    void updateRadioStatus();
    float mapServoPosition(float position_0_to_1);
    uint16_t mapMotorThrottle(float throttle_0_to_1);
};

#endif // ARDUINO 