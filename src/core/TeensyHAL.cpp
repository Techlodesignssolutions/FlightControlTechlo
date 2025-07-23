#include "TeensyHAL.h"
#include <cmath>  // for sqrt, abs

#if defined(ARDUINO)

// Include dRehm's proven hardware libraries
#include "src/MPU6050/MPU6050.h"

// Hardware instances
MPU6050 mpu6050_;
PWMServo servos_[8];

// Scale factors from dRehm's implementation
#if defined(GYRO_250DPS)
  #define GYRO_SCALE_FACTOR 131.0
  #define GYRO_SCALE 0
#elif defined(GYRO_500DPS)
  #define GYRO_SCALE_FACTOR 65.5
  #define GYRO_SCALE 1
#elif defined(GYRO_1000DPS)
  #define GYRO_SCALE_FACTOR 32.8
  #define GYRO_SCALE 2
#elif defined(GYRO_2000DPS)
  #define GYRO_SCALE_FACTOR 16.4
  #define GYRO_SCALE 3
#endif

#if defined(ACCEL_2G)
  #define ACCEL_SCALE_FACTOR 16384.0
  #define ACCEL_SCALE 0
#elif defined(ACCEL_4G)
  #define ACCEL_SCALE_FACTOR 8192.0
  #define ACCEL_SCALE 1
#elif defined(ACCEL_8G)
  #define ACCEL_SCALE_FACTOR 4096.0
  #define ACCEL_SCALE 2
#elif defined(ACCEL_16G)
  #define ACCEL_SCALE_FACTOR 2048.0
  #define ACCEL_SCALE 3
#endif

// Constructor
TeensyHAL::TeensyHAL(const HardwareConfig& config) : config_(config) {
    imu_initialized_ = false;
    radio_initialized_ = false;
    servos_initialized_ = false;
    motors_initialized_ = false;
    
    // Initialize bias arrays
    for (int i = 0; i < 3; i++) {
        gyro_bias_[i] = 0.0f;
        accel_bias_[i] = 0.0f;
    }
    
    last_radio_update_ = 0;
    radio_connected_ = false;
}

//=== Serial Communication (from dRehm's proven approach) ===
void TeensyHAL::serialBegin(uint32_t baud) {
    Serial.begin(baud);
    delay(100); // Give time for serial to initialize
}

void TeensyHAL::serialPrint(const char* str) {
    Serial.print(str);
}

void TeensyHAL::serialPrintln(const char* str) {
    Serial.println(str);
}

void TeensyHAL::serialPrintFloat(float value, int decimals) {
    Serial.print(value, decimals);
}

//=== IMU Interface (extracted from dRehm's working code) ===
bool TeensyHAL::initIMU() {
    // dRehm's proven MPU6050 initialization
    Wire.begin();
    Wire.setClock(1000000); // dRehm's fast I2C speed
    
    mpu6050_.initialize();
    
    if (mpu6050_.testConnection() == false) {
        serialPrintln("MPU6050 initialization unsuccessful");
        serialPrintln("Check MPU6050 wiring or try cycling power");
        return false;
    }
    
    // Set scale ranges (from dRehm's working config)
    mpu6050_.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050_.setFullScaleAccelRange(ACCEL_SCALE);
    
    // Calibrate IMU biases
    if (!calibrateIMU()) {
        serialPrintln("IMU calibration failed");
        return false;
    }
    
    imu_initialized_ = true;
    serialPrintln("✓ IMU initialized successfully");
    return true;
}

bool TeensyHAL::readIMU(float* gyro_xyz, float* accel_xyz, float* mag_xyz) {
    if (!imu_initialized_) return false;
    
    // Read raw data (dRehm's approach)
    int16_t ax, ay, az, gx, gy, gz;
    mpu6050_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert to physical units and apply bias correction
    accel_xyz[0] = (ax / ACCEL_SCALE_FACTOR) - accel_bias_[0]; // m/s² 
    accel_xyz[1] = (ay / ACCEL_SCALE_FACTOR) - accel_bias_[1];
    accel_xyz[2] = (az / ACCEL_SCALE_FACTOR) - accel_bias_[2];
    
    gyro_xyz[0] = ((gx / GYRO_SCALE_FACTOR) - gyro_bias_[0]) * M_PI/180.0f; // rad/s
    gyro_xyz[1] = ((gy / GYRO_SCALE_FACTOR) - gyro_bias_[1]) * M_PI/180.0f;
    gyro_xyz[2] = ((gz / GYRO_SCALE_FACTOR) - gyro_bias_[2]) * M_PI/180.0f;
    
    // MPU6050 doesn't have magnetometer, set to reasonable defaults
    mag_xyz[0] = 1.0f; // Normalized magnetic field
    mag_xyz[1] = 0.0f;
    mag_xyz[2] = 0.0f;
    
    return true;
}

bool TeensyHAL::isIMUHealthy() {
    if (!imu_initialized_) return false;
    
    // Try to read IMU data to check if it's responding
    float gyro[3], accel[3], mag[3];
    bool read_success = readIMU(gyro, accel, mag);
    
    if (!read_success) return false;
    
    // Basic sanity checks on the data
    // Check for reasonable acceleration magnitude (should be close to 1g when stationary)
    float accel_mag = sqrt(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (accel_mag < 5.0f || accel_mag > 15.0f) {  // 0.5g to 1.5g seems reasonable
        return false; // Accelerometer reading is suspicious
    }
    
    // Check for reasonable gyro rates (shouldn't be saturated)
    for (int i = 0; i < 3; i++) {
        if (abs(gyro[i]) > 34.9f) { // 2000 deg/s is typical max range
            return false; // Gyro might be saturated or faulty
        }
    }
    
    return true;
}

//=== Radio Interface (dRehm's interrupt-based PWM) ===
bool TeensyHAL::initRadio() {
    if (radio_initialized_) return true;
    
    // Store global instance pointer for ISRs
    teensy_instance_ptr = this;
    
    // Set input pins
    for (int i=0;i<6;i++) {
        pinMode(config_.servo_pins[i], INPUT_PULLUP);
    }
    delay(20);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[0]), ISR_Ch1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[1]), ISR_Ch2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[2]), ISR_Ch3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[3]), ISR_Ch4, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[4]), ISR_Ch5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(config_.servo_pins[5]), ISR_Ch6, CHANGE);
    
    radio_initialized_ = true;
    serialPrintln("✓ Radio initialized (PWM)");
    return true;
}

bool TeensyHAL::readRadio(float* channels, int num_channels) {
    if (!radio_initialized_) return false;
    
    uint32_t now = micros();
    bool new_pulse = (now - last_radio_update_ < 50000); // 50ms freshness check
    
    for (int i=0;i<num_channels && i<6;i++) {
        uint32_t pw = ch_width_us_[i];
        pw = constrain(pw, 1000, 2000);
        if (i==0) {
            channels[0] = (pw - 1000) / 1000.0f;           // throttle 0-1
        } else {
            channels[i] = ((int)pw - 1500) / 500.0f;       // -1 to +1
        }
    }
    radio_connected_ = new_pulse;
    if (new_pulse) last_radio_update_ = now;
    return true;
}

bool TeensyHAL::isRadioConnected() {
    return radio_connected_;
}

//=== Servo Interface (dRehm's PWMServo library) ===
bool TeensyHAL::initServos() {
    // Initialize servo objects (dRehm's approach)
    for (int i = 0; i < 8; i++) {
        servos_[i].attach(config_.servo_pins[i], config_.servo_min_us, config_.servo_max_us);
        servos_[i].write(90); // Center position
    }
    
    delay(100);
    servos_initialized_ = true;
    serialPrintln("✓ Servos initialized");
    return true;
}

void TeensyHAL::writeServo(int channel, float position_0_to_1) {
    if (!servos_initialized_ || channel >= 8) return;
    
    // Map 0-1 to servo angle 0-180 degrees
    int angle = constrain(position_0_to_1 * 180.0f, 0, 180);
    servos_[channel].write(angle);
}

//=== Motor Interface (dRehm's OneShot125 protocol) ===
bool TeensyHAL::initMotors() {
    // Setup motor pins as outputs
    for (int i = 0; i < 6; i++) {
        pinMode(config_.motor_pins[i], OUTPUT);
        digitalWrite(config_.motor_pins[i], LOW);
    }
    
    delay(100);
    motors_initialized_ = true;
    serialPrintln("✓ Motors initialized (OneShot125)");
    return true;
}

void TeensyHAL::writeMotor(int channel, float throttle_0_to_1) {
    if (!motors_initialized_ || channel >= 6) return;
    
    // Convert 0-1 throttle to OneShot125 pulse width (125-250μs)
    int pulse_us = constrain(125 + (throttle_0_to_1 * 125), 125, 250);
    
    // Generate OneShot125 pulse (simplified - dRehm's was more complex)
    digitalWrite(config_.motor_pins[channel], HIGH);
    delayMicroseconds(pulse_us);
    digitalWrite(config_.motor_pins[channel], LOW);
}

//=== Digital I/O ===
void TeensyHAL::digitalWrite(int pin, bool high) {
    ::digitalWrite(pin, high ? HIGH : LOW);
}

bool TeensyHAL::digitalRead(int pin) {
    return ::digitalRead(pin) == HIGH;
}

void TeensyHAL::pinMode(int pin, int mode) {
    ::pinMode(pin, mode == 0 ? INPUT : OUTPUT);
}

//=== LED/Status ===
void TeensyHAL::setStatusLED(bool on) {
    ::digitalWrite(config_.status_led_pin, on ? HIGH : LOW);
}

void TeensyHAL::blinkStatusLED(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        setStatusLED(true);
        delay(on_ms);
        setStatusLED(false);
        delay(off_ms);
    }
}

//=== Private Helper Methods ===
bool TeensyHAL::calibrateIMU() {
    // dRehm's IMU calibration approach
    serialPrintln("Calibrating IMU - keep vehicle level and stationary...");
    
    const int samples = 2000;
    float gyro_sum[3] = {0, 0, 0};
    float accel_sum[3] = {0, 0, 0};
    
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050_.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        gyro_sum[0] += gx / GYRO_SCALE_FACTOR;
        gyro_sum[1] += gy / GYRO_SCALE_FACTOR;  
        gyro_sum[2] += gz / GYRO_SCALE_FACTOR;
        
        accel_sum[0] += ax / ACCEL_SCALE_FACTOR;
        accel_sum[1] += ay / ACCEL_SCALE_FACTOR;
        accel_sum[2] += az / ACCEL_SCALE_FACTOR;
        
        delay(1);
    }
    
    // Calculate bias values
    gyro_bias_[0] = gyro_sum[0] / samples;
    gyro_bias_[1] = gyro_sum[1] / samples;
    gyro_bias_[2] = gyro_sum[2] / samples;
    
    accel_bias_[0] = accel_sum[0] / samples;
    accel_bias_[1] = accel_sum[1] / samples;
    accel_bias_[2] = (accel_sum[2] / samples) - 1.0f; // Remove gravity
    
    serialPrintln("✓ IMU calibration complete");
    return true;
}

#if defined(ARDUINO)

volatile uint32_t TeensyHAL::ch_start_[6] = {0};
volatile uint32_t TeensyHAL::ch_width_us_[6] = {1500,1500,1500,1500,1500,1500};

// Helper macro to create ISR bodies
#define DEFINE_ISR(N) \
void TeensyHAL::ISR_Ch##N() { \
    bool level = digitalRead(TeensyHAL::instance_->config_.servo_pins[N-1]); \
    uint32_t now = micros(); \
    if (level) { ch_start_[N-1] = now; } \
    else { ch_width_us_[N-1] = now - ch_start_[N-1]; } }

// Need singleton pointer for pin mapping
static TeensyHAL* teensy_instance_ptr = nullptr;
#define INSTANCE teensy_instance_ptr

DEFINE_ISR(1)
DEFINE_ISR(2)
DEFINE_ISR(3)
DEFINE_ISR(4)
DEFINE_ISR(5)
DEFINE_ISR(6)

#endif // ARDUINO 