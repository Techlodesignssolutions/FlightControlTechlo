#pragma once
#include <stdint.h>

/**
 * Hardware Abstraction Layer
 * 
 * This interface allows the flight controller to be hardware-agnostic
 * and enables desktop testing with mocked hardware
 */
class HAL {
public:
    virtual ~HAL() = default;
    
    // Timing
    virtual uint32_t micros() = 0;
    virtual uint32_t millis() = 0;
    virtual void delay(uint32_t ms) = 0;
    
    // Serial Communication
    virtual void serialBegin(uint32_t baud) = 0;
    virtual void serialPrint(const char* str) = 0;
    virtual void serialPrintln(const char* str) = 0;
    virtual void serialPrintFloat(float value, int decimals = 2) = 0;
    
    // IMU Interface
    virtual bool initIMU() = 0;
    virtual bool readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) = 0;
    
    // Radio Interface  
    virtual bool initRadio() = 0;
    virtual bool readRadio(float* channels, int num_channels) = 0;
    virtual bool isRadioConnected() = 0;
    
    // Servo Interface (for control surfaces)
    virtual bool initServos() = 0;
    virtual void writeServo(int channel, float position_0_to_1) = 0;  // 0.0 = 0°, 1.0 = 180°
    
    // Motor Interface (for propellers)
    virtual bool initMotors() = 0;
    virtual void writeMotor(int channel, float throttle_0_to_1) = 0;  // 0.0 = off, 1.0 = full
    
    // Digital I/O
    virtual void digitalWrite(int pin, bool high) = 0;
    virtual bool digitalRead(int pin) = 0;
    virtual void pinMode(int pin, int mode) = 0;  // INPUT=0, OUTPUT=1
    
    // LED/Status
    virtual void setStatusLED(bool on) = 0;
    virtual void blinkStatusLED(int count, int on_ms, int off_ms) = 0;
};

/**
 * Configuration structure for hardware setup
 */
struct HardwareConfig {
    // IMU Configuration
    bool use_magnetometer = true;
    
    // Radio Configuration  
    int num_radio_channels = 6;
    
    // Servo Configuration
    int servo_pins[8] = {2, 3, 4, 5, 6, 7, 8, 9};  // PWM pins for servos
    int servo_min_us = 900;   // Minimum servo pulse width
    int servo_max_us = 2100;  // Maximum servo pulse width
    
    // Motor Configuration
    int motor_pins[6] = {20, 21, 22, 23, 24, 25};  // OneShot125 pins
    
    // Timing
    uint32_t loop_frequency_hz = 500;  // Target loop frequency
    
    // Status LED
    int status_led_pin = 13;
}; 