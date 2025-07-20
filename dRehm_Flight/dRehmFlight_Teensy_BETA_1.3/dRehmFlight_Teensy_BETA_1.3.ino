//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3
 
//========================================================================================================================//

//CREDITS + SPECIAL THANKS
/*
Some elements inspired by:
http://www.brokking.net/ymfc-32_main.html

Madgwick filter function adapted from:
https://github.com/arduino-libraries/MadgwickAHRS

MPU9250 implementation based on MPU9250 library by:
brian.taylor@bolderflight.com
http://www.bolderflight.com

Thank you to:
RcGroups 'jihlein' - IMU implementation overhaul + SBUS implementation.
Everyone that sends me pictures and videos of your flying creations! -Nick

*/



//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
static const uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//



//REQUIRED LIBRARIES (included with download in main sketch folder)

#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> //Commanding any extra actuators, installed with teensyduino installer
#include <EEPROM.h>   //For saving adaptive PID gains

#if defined USE_SBUS_RX
  #include "src/SBUS/SBUS.h"   //sBus interface
#endif

#if defined USE_DSM_RX
  #include "src/DSMRX/DSMRX.h"  
#endif

//========================================================================================================================//
//                                           DEBUG AND COMPILATION GUARDS                                                //                           
//========================================================================================================================//

//Enable/disable advanced features for compilation
#define ENABLE_ADAPTIVE_PID 0           //Enable 4D adaptive gain scheduling system (SET TO 1 WHEN READY)
#define ENABLE_WIND_REJECTION 0         //Enable intelligent wind disturbance rejection (SET TO 1 WHEN READY) 
#define ENABLE_AIRSPEED_ESTIMATION 0    //Enable physics-based airspeed estimation (SET TO 1 WHEN READY)
#define ENABLE_ADVANCED_DEBUG 1         //Enable comprehensive debug output
#define ENABLE_EEPROM_STORAGE 0         //Enable EEPROM gain storage (set to 1 when ready)

//========================================================================================================================//
//                                           MISSING VARIABLE DECLARATIONS                                                //                           
//========================================================================================================================//

//Basic flight variables that debug functions expect
float RollAng = 0.0, PitchAng = 0.0, YawAng = 0.0;     //Current attitude angles (degrees)
float angle_of_attack = 0.0;                            //Angle of attack (degrees)
float error_roll = 0.0, error_pitch = 0.0, error_yaw = 0.0;  //PID errors (degrees)
bool armed = false;                                      //Aircraft armed status
float dt = 0.002;                                        //Loop time (seconds)
unsigned long current_time = 0;                         //Current time (microseconds)

//PID output variables
float roll_PID = 0.0, pitch_PID = 0.0, yaw_PID = 0.0;

//PID gain variables that get updated by gain scheduling
float Kp_roll_cmd = 0.1, Ki_roll_cmd = 0.02, Kd_roll_cmd = 0.01;
float Kp_pitch_cmd = 0.1, Ki_pitch_cmd = 0.02, Kd_pitch_cmd = 0.01;
float Kp_yaw_cmd = 0.08, Ki_yaw_cmd = 0.01, Kd_yaw_cmd = 0.005;

//Command variables for motors and servos
float m1_command_scaled = 0, m2_command_scaled = 0, m3_command_scaled = 0;
float m4_command_scaled = 0, m5_command_scaled = 0, m6_command_scaled = 0;
float s1_command_scaled = 0.5, s2_command_scaled = 0.5, s3_command_scaled = 0.5;
float s4_command_scaled = 0.5, s5_command_scaled = 0.5, s6_command_scaled = 0.5, s7_command_scaled = 0.5;

int m1_command_PWM = 125, m2_command_PWM = 125, m3_command_PWM = 125;
int m4_command_PWM = 125, m5_command_PWM = 125, m6_command_PWM = 125;
int s1_command_PWM = 90, s2_command_PWM = 90, s3_command_PWM = 90;
int s4_command_PWM = 90, s5_command_PWM = 90, s6_command_PWM = 90, s7_command_PWM = 90;

//Radio input variables
float thro_des = 0.0;  //Desired throttle (0.0 to 1.0)
bool thro_cut = false; //Throttle cut switch

//Timer variables for debug functions
unsigned long print_counter = 0;

#if defined USE_MPU6050_I2C
  #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#elif defined USE_MPU9250_SPI
  #include "src/MPU9250/MPU9250.h"
  MPU9250 mpu9250(SPI2,36);
#else
  #error No MPU defined... 
#endif



//========================================================================================================================//



//Setup gyro and accel full scale value selection and scale factor

#if defined USE_MPU6050_I2C
  #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
  #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
  #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
  #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
  #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
  #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
  #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
  #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#elif defined USE_MPU9250_SPI
  #define GYRO_FS_SEL_250    mpu9250.GYRO_RANGE_250DPS
  #define GYRO_FS_SEL_500    mpu9250.GYRO_RANGE_500DPS
  #define GYRO_FS_SEL_1000   mpu9250.GYRO_RANGE_1000DPS                                                        
  #define GYRO_FS_SEL_2000   mpu9250.GYRO_RANGE_2000DPS
  #define ACCEL_FS_SEL_2     mpu9250.ACCEL_RANGE_2G
  #define ACCEL_FS_SEL_4     mpu9250.ACCEL_RANGE_4G
  #define ACCEL_FS_SEL_8     mpu9250.ACCEL_RANGE_8G
  #define ACCEL_FS_SEL_16    mpu9250.ACCEL_RANGE_16G
#endif
  
#if defined GYRO_250DPS
  #define GYRO_SCALE GYRO_FS_SEL_250
  #define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
  #define GYRO_SCALE GYRO_FS_SEL_500
  #define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
  #define GYRO_SCALE GYRO_FS_SEL_1000
  #define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
  #define GYRO_SCALE GYRO_FS_SEL_2000
  #define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
  #define ACCEL_SCALE ACCEL_FS_SEL_2
  #define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
  #define ACCEL_SCALE ACCEL_FS_SEL_4
  #define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
  #define ACCEL_SCALE ACCEL_FS_SEL_8
  #define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
  #define ACCEL_SCALE ACCEL_FS_SEL_16
  #define ACCEL_SCALE_FACTOR 2048.0
#endif



//========================================================================================================================//
//                                               USER-SPECIFIED VARIABLES                                                 //                           
//========================================================================================================================//

//Radio failsafe values for every channel in the event that bad reciever data is detected. Recommended defaults:
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut
unsigned long channel_6_fs = 2000; //aux1

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
float B_mag = 1.0;        //Magnetometer LP filter parameter

//Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
float MagErrorX = 0.0;
float MagErrorY = 0.0; 
float MagErrorZ = 0.0;
float MagScaleX = 1.0;
float MagScaleY = 1.0;
float MagScaleZ = 1.0;

//IMU calibration parameters - calibrate IMU using calculate_IMU_error() in the void setup() to get these values, then comment out calculate_IMU_error()
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters (take note of defaults before modifying!): 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.15;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0002;  //Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
float Kp_pitch_rate = 0.15;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0002; //Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw = 0.05;          //Yaw I-gain
float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

//========================================================================================================================//
//                                       PRODUCTION-GRADE ADAPTIVE PID CONTROLLER                                        //                           
//========================================================================================================================//

//Enable/disable adaptive PID system
bool adaptivePID_enabled = true;
bool gainScheduling_enabled = true;
bool extremumSeeking_enabled = false;  //Advanced optimization mode

//Multi-dimensional flight state grid
#define NUM_THROTTLE_ZONES 7
#define NUM_AOA_ZONES 5  
#define NUM_PHASE_ZONES 6
#define NUM_AIRSPEED_ZONES 4

//Flight phase enumeration
enum FlightPhase {
  PHASE_HOVER = 0,
  PHASE_CLIMB = 1,
  PHASE_CRUISE = 2,
  PHASE_DESCENT = 3,
  PHASE_TURN = 4,
  PHASE_TRANSITION = 5
};

//Comprehensive PID gains structure
struct PIDGains {
  float Kp_roll, Ki_roll, Kd_roll;
  float Kp_pitch, Ki_pitch, Kd_pitch;
  float Kp_yaw, Ki_yaw, Kd_yaw;
  
  //Metadata for gain validation
  float confidence;      //How well-tuned this set is (0-1)
  unsigned long usage_count; //How often this set is used
  float last_performance; //Last measured performance
};

//4D gain scheduling grid [throttle][aoa][phase][airspeed]
PIDGains gainSchedule[NUM_THROTTLE_ZONES][NUM_AOA_ZONES][NUM_PHASE_ZONES][NUM_AIRSPEED_ZONES];

//Current flight state indices and interpolation factors
struct FlightState {
  int throttle_idx, aoa_idx, phase_idx, airspeed_idx;
  float throttle_factor, aoa_factor, phase_factor, airspeed_factor;
  FlightPhase current_phase;
  float airspeed_estimate;
  float vertical_velocity;
  float turn_rate;
  bool transitioning;
};
FlightState flightState;

//Advanced performance metrics per axis
struct PerformanceMetrics {
  //Time domain metrics
  float mean_abs_error;     //Average tracking error
  float rms_error;          //RMS error for power content
  float peak_error;         //Maximum error in window
  float settling_time;      //Time to settle within 5% of setpoint
  float overshoot;          //Maximum overshoot percentage
  float rise_time;          //10% to 90% rise time
  
  //Frequency domain metrics  
  float oscillation_freq;   //Dominant oscillation frequency
  float damping_ratio;      //System damping estimate
  float control_effort;     //Average control output magnitude
  
  //Stability metrics
  float phase_margin;       //Estimated phase margin
  float gain_margin;        //Estimated gain margin
  int oscillation_count;    //Zero crossings per window
  
  //Bias and drift detection
  float steady_state_error; //Persistent bias
  float error_trend;        //Error trending up/down
  
  //Overall health score (0-100)
  float health_score;
};

PerformanceMetrics rollMetrics, pitchMetrics, yawMetrics;

//Sliding window buffers for rich performance analysis
#define METRIC_WINDOW_SIZE 200
#define SETTLE_THRESHOLD 0.05  //5% settling criterion
#define STEP_DETECT_THRESHOLD 5.0  //Step input detection

struct MetricBuffers {
  float error[METRIC_WINDOW_SIZE];
  float setpoint[METRIC_WINDOW_SIZE];
  float output[METRIC_WINDOW_SIZE];
  float derivative[METRIC_WINDOW_SIZE];
  unsigned long timestamps[METRIC_WINDOW_SIZE];
  int index;
  bool buffer_full;
};

MetricBuffers rollBuffer, pitchBuffer, yawBuffer;

//Learning state machine
enum LearningState {
  LEARNING_DISABLED = 0,
  LEARNING_OBSERVING = 1,   //Collecting data, not adapting
  LEARNING_ADAPTING = 2,    //Actively learning
  LEARNING_FROZEN = 3,      //Frozen due to instability
  LEARNING_ROLLBACK = 4     //Rolling back bad gains
};

LearningState learningState = LEARNING_OBSERVING;

//Adaptation control variables
float adaptation_gain = 0.001;        //Base learning rate
float adaptation_momentum = 0.9;      //Momentum term for stability
unsigned long learning_freeze_timer = 0;
unsigned long last_adaptation_time = 0;
bool rollback_triggered = false;

//Safety and stability monitoring
struct SafetyMonitor {
  float max_error_threshold;      //Emergency stop threshold
  float oscillation_threshold;    //Too much oscillation
  float control_saturation_limit; //Control output saturation
  int consecutive_bad_samples;    //Counter for degrading performance
  bool emergency_freeze;          //Emergency learning freeze
  PIDGains backup_gains;          //Safe fallback gains
  float performance_trend[10];    //Recent performance trend
  int trend_index;
};

SafetyMonitor safetyMonitor;

//Model Reference Adaptive Control (MRAC) components
struct MRACController {
  bool enabled;
  float reference_model_wn;       //Reference natural frequency
  float reference_model_zeta;     //Reference damping ratio
  float adaptation_gamma;         //MRAC adaptation gain
  float sigma_modification;       //Sigma modification parameter
  
  //Reference model states
  float ref_state1, ref_state2;
  float ref_input;
  
  //Adaptive parameters
  float theta_hat[3];            //Parameter estimates
  float adaptation_error;        //Model-following error
};

MRACController mracRoll, mracPitch, mracYaw;

//Extremum seeking controller for automatic optimization
struct ExtremumSeeker {
  bool enabled;
  float perturbation_freq;        //Dither frequency (Hz)
  float perturbation_amplitude;   //Dither amplitude
  float highpass_cutoff;          //High-pass filter cutoff
  float integrator_gain;          //Seeking integrator gain
  
  //Internal states
  float dither_signal;
  float cost_function;
  float filtered_cost;
  float parameter_estimate;
  float phase;
};

ExtremumSeeker seekerRoll, seekerPitch, seekerYaw;

//Flight condition detection and classification
struct FlightDetector {
  //Temporal windows for classification
  float throttle_history[20];
  float aoa_history[20];
  float turn_rate_history[20];
  float vertical_vel_history[20];
  int history_index;
  
  //Gust and turbulence detection
  float accel_variance_x, accel_variance_y, accel_variance_z;
  float gyro_variance_x, gyro_variance_y, gyro_variance_z;
  float turbulence_index;
  bool in_turbulence;
  
  //Maneuver detection
  bool rapid_pitch_input;
  bool rapid_roll_input;
  bool rapid_yaw_input;
  bool coordinated_turn;
  unsigned long maneuver_start_time;
  
  //Steady-state detection
  float stick_input_variance;
  bool steady_flight;
  unsigned long steady_start_time;
  float steady_duration_required;  //Minimum time in steady flight
};

FlightDetector flightDetector;

//========================================================================================================================//
//                                         PHYSICS-BASED AIRSPEED ESTIMATION                                             //                           
//========================================================================================================================//

//Thrust mapping and calibration
#define THRUST_MAP_SIZE 50
#define NUM_AOA_DRAG_ZONES 6
#define DITHER_FREQUENCY 0.5  //Hz for throttle dithering
#define DITHER_AMPLITUDE 0.015  //1.5% throttle dither

//Thrust calibration lookup table (PWM vs Thrust in Newtons)
struct ThrustCalibration {
  float pwm_points[THRUST_MAP_SIZE];
  float thrust_values[THRUST_MAP_SIZE];  //Thrust in Newtons
  float voltage_compensation;            //Voltage compensation factor
  bool calibrated;
};

ThrustCalibration thrustMap;

//Extended Kalman Filter for airspeed and drag estimation
struct AirspeedEKF {
  //State vector: [airspeed, drag_coefficient]
  float state[2];                  //[V, k_current_zone]
  float state_prediction[2];       //Predicted state
  
  //Covariance matrices
  float P[2][2];                   //State covariance
  float Q[2][2];                   //Process noise covariance
  float R;                         //Measurement noise variance
  
  //System matrices
  float F[2][2];                   //State transition matrix
  float H[2];                      //Measurement matrix
  float K[2];                      //Kalman gain
  
  //Measurement and innovation
  float innovation;                //Innovation (residual)
  float innovation_covariance;     //Innovation covariance
  
  //Filter tuning parameters
  float process_noise_V;           //Process noise for airspeed
  float process_noise_k;           //Process noise for drag coefficient
  float measurement_noise;         //IMU acceleration noise
  
  //State validity checks
  bool initialized;
  bool converged;
  float convergence_threshold;
  unsigned long convergence_time;
};

AirspeedEKF airspeedFilter;

//Zone-based drag coefficient learning
struct DragZone {
  float aoa_min, aoa_max;          //AOA range for this zone
  float k_coefficient;             //Learned drag coefficient
  float k_variance;                //Uncertainty in k
  unsigned long samples;           //Number of samples in this zone
  bool converged;                  //Has this zone converged?
  
  //Recursive Least Squares for zone-specific learning
  float rls_P;                     //RLS covariance
  float rls_gain;                  //RLS gain
  float forgetting_factor;         //RLS forgetting factor
};

DragZone dragZones[NUM_AOA_DRAG_ZONES];

//Airspeed estimation state and control
struct AirspeedEstimator {
  float estimated_airspeed;        //Current airspeed estimate (m/s)
  float true_airspeed;             //True airspeed (corrected for density)
  float indicated_airspeed;        //Indicated airspeed
  
  //Thrust and drag components
  float current_thrust;            //Current thrust (N)
  float estimated_drag;            //Estimated drag (N)
  float thrust_to_weight_ratio;    //T/W ratio
  
  //Measurement inputs
  float forward_accel;             //Forward acceleration from IMU
  float forward_accel_filtered;    //Low-pass filtered acceleration
  float gravity_compensation;      //Gravity component correction
  
  //Throttle dithering for observability
  float base_throttle;             //Base throttle command
  float dither_signal;             //Sinusoidal dither
  float dithered_throttle;         //Final throttle with dither
  unsigned long dither_timer;     //Dither phase timer
  bool dither_enabled;             //Enable/disable dithering
  
  //Zone management
  int current_aoa_zone;            //Current AOA zone index
  int previous_aoa_zone;           //Previous zone for transition detection
  unsigned long zone_entry_time;   //Time when entered current zone
  float zone_stability_time;       //Required time in zone for learning
  
  //Estimation quality metrics
  float estimation_confidence;     //Confidence in current estimate (0-1)
  float innovation_variance;       //Recent innovation variance
  float bias_estimate;             //Estimated bias in measurements
  bool estimation_valid;           //Is current estimate valid?
  
  //Aircraft parameters
  float aircraft_mass;             //Aircraft mass (kg)
  float air_density;               //Air density (kg/m³)
  float reference_area;            //Reference area for drag (m²)
};

AirspeedEstimator airspeedEst;

//Environmental and atmospheric models
struct AtmosphericModel {
  float sea_level_pressure;       //Sea level pressure (Pa)
  float temperature;               //Temperature (K)
  float altitude_estimate;         //Estimated altitude (m)
  float density_ratio;             //ρ/ρ₀ ratio
  float pressure_ratio;            //P/P₀ ratio
};

AtmosphericModel atmosphere;

//========================================================================================================================//



//========================================================================================================================//
//                                                     DECLARE PINS                                                       //                           
//========================================================================================================================//                                          

//NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
//Radio:
//Note: If using SBUS, connect to pin 21 (RX5), if using DSM, connect to pin 15 (RX3)
const int ch1Pin = 15; //throttle
const int ch2Pin = 16; //ail
const int ch3Pin = 17; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
const int ch6Pin = 22; //aux1 (free aux channel)
const int PPM_Pin = 23;
//OneShot125 ESC pin outputs:
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;
//PWM servo or ESC outputs:
const int servo1Pin = 6;
const int servo2Pin = 7;
const int servo3Pin = 8;
const int servo4Pin = 9;
const int servo5Pin = 10;
const int servo6Pin = 11;
const int servo7Pin = 12;
PWMServo servo1;  //Create servo objects to control a servo or ESC with PWM
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;
PWMServo servo6;
PWMServo servo7;



//========================================================================================================================//



//DECLARE GLOBAL VARIABLES

//General stuff
float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;

#if defined USE_SBUS_RX
  SBUS sbus(Serial5);
  uint16_t sbusChannels[16];
  bool sbusFailSafe;
  bool sbusLostFrame;
#endif
#if defined USE_DSM_RX
  DSM1024 DSM;
#endif

//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

//Normalized desired state:
float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM, m2_command_PWM, m3_command_PWM, m4_command_PWM, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;

//Flight status
bool armedFly = false;

//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000); //USB serial
  delay(500);
  
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  servo1.attach(servo1Pin, 900, 2100); //Pin, min PWM value, max PWM value
  servo2.attach(servo2Pin, 900, 2100);
  servo3.attach(servo3Pin, 900, 2100);
  servo4.attach(servo4Pin, 900, 2100);
  servo5.attach(servo5Pin, 900, 2100);
  servo6.attach(servo6Pin, 900, 2100);
  servo7.attach(servo7Pin, 900, 2100);

  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;
  channel_5_pwm = channel_5_fs;
  channel_6_pwm = channel_6_fs;

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  //calculate_IMU_error(); //Calibration parameters printed to serial monitor. Paste these in the user specified variables section, then comment this out forever.

  //Arm servo channels
  servo1.write(0); //Command servo angle from 0-180 degrees (1000 to 2000 PWM)
  servo2.write(0); //Set these to 90 for servos if you do not want them to briefly max out on startup
  servo3.write(0); //Keep these at 0 if you are using servo outputs for motors
  servo4.write(0);
  servo5.write(0);
  servo6.write(0);
  servo7.write(0);
  
  delay(5);

  //calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
  //Code will not proceed past here if this function is uncommented!

  //Arm OneShot125 motors
  m1_command_PWM = 125; //Command OneShot125 ESC from 125 to 250us pulse length
  m2_command_PWM = 125;
  m3_command_PWM = 125;
  m4_command_PWM = 125;
  m5_command_PWM = 125;
  m6_command_PWM = 125;
  armMotors(); //Loop over commandMotors() until ESCs happily arm
  
  //Initialize production-grade adaptive PID system
  #if ENABLE_ADAPTIVE_PID
    initializeAdaptiveController(); //Initialize all adaptive controller components
  #endif
  
  #if ENABLE_AIRSPEED_ESTIMATION
    initializeAirspeedEstimator(); //Initialize physics-based airspeed estimation
  #endif
  
  #if ENABLE_WIND_REJECTION
    initializeWindDisturbanceRejection(); //Initialize wind disturbance rejection
  #endif
  
  loadGainScheduleFromEEPROM(); //Load previously learned gains (always safe)
  
  //Print startup message and debug help
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("dRehmFlight Teensy Adaptive Flight Controller"));
  Serial.println(F("With Intelligent Wind Disturbance Rejection"));
  Serial.println(F("Fixed-Wing Elevon Configuration"));
  Serial.println(F("==================================================="));
  Serial.println(F("System Initialized Successfully!"));
  Serial.print(F("Serial Rate: 500000 baud"));
  Serial.println(F(""));
  Serial.println(F("COMPILATION STATUS:"));
  Serial.print(F("Adaptive PID: ")); Serial.println(ENABLE_ADAPTIVE_PID ? F("ENABLED") : F("DISABLED"));
  Serial.print(F("Wind Rejection: ")); Serial.println(ENABLE_WIND_REJECTION ? F("ENABLED") : F("DISABLED"));
  Serial.print(F("Airspeed Estimation: ")); Serial.println(ENABLE_AIRSPEED_ESTIMATION ? F("ENABLED") : F("DISABLED"));
  Serial.print(F("Advanced Debug: ")); Serial.println(ENABLE_ADVANCED_DEBUG ? F("ENABLED") : F("DISABLED"));
  Serial.println(F(""));
  Serial.println(F("To enable advanced features, set the #define flags"));
  Serial.println(F("at the top of the code to 1 and recompile."));
  Serial.println(F(""));
  Serial.println(F("Debug output enabled: printGettingStartedDebug()"));
  Serial.println(F("(See debug options below after initialization)"));
  Serial.println(F("==================================================="));
  Serial.println(F(""));
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)
  
  //Print debug help after initialization complete
  delay(1000);  //Wait for user to see startup message
  printDebugHelp();

  //If using MPU9250 IMU, uncomment for one-time magnetometer calibration (may need to repeat for new locations)
  //calibrateMagnetometer(); //Generates magentometer error and scale factors to be pasted in user-specified variables section

}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  //Keep track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
  //printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
  //printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
  //printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
  //printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
  //printServoCommands(); //Prints the values being written to the servos (expected: 0 to 180)
  //printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)
  
  //Adaptive PID Debug Functions (uncomment one at a time):
  //printAdaptivePIDStatus();   //Prints current flight regime and learning status
  //printCurrentGains();        //Prints current PID gains being used
  //printPerformanceMetrics();  //Prints performance metrics for each axis
  
  //Physics-based Airspeed Debug Functions (uncomment one at a time):
  //printAirspeedEstimation();  //Prints airspeed estimate, thrust, and confidence
  //printDragCoefficients();    //Prints current drag zone and coefficients
  //printThrottleDither();      //Prints throttle dithering status
  //printEKFStates();           //Prints Extended Kalman Filter states
  
  //Wind Disturbance Rejection Debug Functions (uncomment one at a time):
  //printWindAnalysis();        //Prints gust vs steady wind analysis
  //printWindCompensation();    //Prints feedforward wind compensation gains
  //printAdaptationRates();     //Prints current adaptation rates and emergency status
  //printWindModel();           //Prints wind model parameters and convergence
  
  //Fixed-Wing Specific Debug Functions (uncomment one at a time):
  //printFixedWingControlSurfaces();  //Prints elevon, rudder, and throttle commands
  //printFixedWingWindBias();         //Prints wind bias estimates in body frame
  //printFixedWingWindEffects();      //Prints bank angle, pitch, wind magnitude, emergency mode
  
  //4D Gain Schedule & Adaptive System Visualization (uncomment one at a time):
  #if ENABLE_ADVANCED_DEBUG && ENABLE_ADAPTIVE_PID
    //printGainScheduleDebug();         //FULL 4D gain schedule interpolation detail
    //printFlightRegimeVisualization(); //Visual bar charts of throttle, AOA, airspeed, phase
    //printPerformanceMetricsVisualization(); //Performance health scores with visual bars
  #endif
  
  #if ENABLE_ADVANCED_DEBUG && ENABLE_WIND_REJECTION
    //printWindVisualization();         //Wind analysis with visual gust/wind separation
  #endif
  
  //Getting Started Debug (uncomment for initial testing):
  #if ENABLE_ADVANCED_DEBUG
    printGettingStartedDebug();       //Simple, clean overview - PERFECT FOR FIRST FLIGHTS
  #endif
  
  //Comprehensive Debug Output (uncomment for full system visualization):
  #if ENABLE_ADVANCED_DEBUG
    //printComprehensiveDebugOutput();  //ALL debug info - use for full system analysis
  #endif

  // Get arming status
  armedStatus(); //Check if the throttle cut is off and throttle is low.

  //Get vehicle state
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values based on saturated control limits
  
  //Physics-based airspeed estimation
  #if ENABLE_AIRSPEED_ESTIMATION
    updateAirspeedEstimation(); //Estimate airspeed using thrust-drag dynamics
  #endif
  
  //Wind disturbance rejection
  #if ENABLE_WIND_REJECTION
    updateWindDisturbanceRejection(); //Analyze wind conditions and update adaptation rates
  #endif
  
  //Adaptive PID System
  #if ENABLE_ADAPTIVE_PID
    detectAdvancedFlightRegime(); //Determine current flight regime based on throttle and AOA
    updateGainScheduling(); //Update PID gains based on flight regime
    monitorPerformance(); //Track control performance for learning
    adaptivePIDLearning(); //Adapt gains based on performance metrics
  #endif
  
  //PID Controller - SELECT ONE:
  controlANGLE(); //Stabilize on angle setpoint
  //controlANGLE2(); //Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
  //controlRATE(); //Stabilize on rate setpoint

  //Actuator mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here
  scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)

  //Throttle cut check
  throttleCut(); //Directly sets motor commands to low based on state of ch5

  //Command actuators
  commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
  servo1.write(s1_command_PWM); //Writes PWM value to servo object
  servo2.write(s2_command_PWM);
  servo3.write(s3_command_PWM);
  servo4.write(s4_command_PWM);
  servo5.write(s5_command_PWM);
  servo6.write(s6_command_PWM);
  servo7.write(s7_command_PWM);
    
  //Get vehicle commands for next loop iteration
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Save adaptive gains periodically (every 30 seconds)
  static unsigned long eeprom_save_timer = 0;
  if (current_time - eeprom_save_timer > 30000000) { // 30 seconds
    if (adaptivePID_enabled) {
      saveGainScheduleToEEPROM();
    }
    eeprom_save_timer = current_time;
  }

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//



void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
  /*
   * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
   * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
   * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
   * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
   * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
   * in preparation to be sent to the motor ESCs and servos.
   * 
   *Relevant variables:
   *thro_des - direct thottle control
   *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
   *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
   *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
   */
   
  //Use dithered throttle for airspeed estimation observability
  float effective_throttle = airspeedEst.dither_enabled ? airspeedEst.dithered_throttle : thro_des;
  
  //Fixed-wing elevon mixing
  //Elevons combine elevator and aileron function
  float left_elevon_base = pitch_PID + roll_PID;   //Left elevon: pitch + roll
  float right_elevon_base = pitch_PID - roll_PID;  //Right elevon: pitch - roll
  float rudder_base = yaw_PID;                     //Rudder for yaw control
  
  //Apply intelligent wind compensation to control surfaces
  float control_commands[4] = {left_elevon_base, right_elevon_base, rudder_base, effective_throttle};
  applyFixedWingWindCompensation(control_commands);
  
  //Assign to servo outputs with proper scaling and limits
  s1_command_scaled = 0.5 + constrain(control_commands[0], -0.5, 0.5); //Left elevon (servo)
  s2_command_scaled = 0.5 + constrain(control_commands[1], -0.5, 0.5); //Right elevon (servo)
  s3_command_scaled = 0.5 + constrain(control_commands[2], -0.5, 0.5); //Rudder (servo)
  
  //Motor throttle (single propeller)
  m1_command_scaled = constrain(control_commands[3], 0.0, 1.0);  //Throttle
  m2_command_scaled = 0;  //Unused
  m3_command_scaled = 0;  //Unused
  m4_command_scaled = 0;  //Unused
  m5_command_scaled = 0;
  m6_command_scaled = 0;

  //0.5 is centered servo, 0.0 is zero throttle if connecting to ESC for conventional PWM, 1.0 is max throttle
  s1_command_scaled = 0;
  s2_command_scaled = 0;
  s3_command_scaled = 0;
  s4_command_scaled = 0;
  s5_command_scaled = 0;
  s6_command_scaled = 0;
  s7_command_scaled = 0;
 
}

void armedStatus() {
  //DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
  if ((channel_5_pwm < 1500) && (channel_1_pwm < 1050)) {
    armedFly = true;
  }
}

void IMUinit() {
  //DESCRIPTION: Initialize IMU
  /*
   * Don't worry about how this works.
   */
  #if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    
  #elif defined USE_MPU9250_SPI
    int status = mpu9250.begin();    

    if (status < 0) {
      Serial.println("MPU9250 initialization unsuccessful");
      Serial.println("Check MPU9250 wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu9250.setGyroRange(GYRO_SCALE);
    mpu9250.setAccelRange(ACCEL_SCALE);
    mpu9250.setMagCalX(MagErrorX, MagScaleX);
    mpu9250.setMagCalY(MagErrorY, MagScaleY);
    mpu9250.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu9250.setSrd(0); //sets gyro and accel read to 1khz, magnetometer read to 100hz
  #endif
}

void getIMUdata() {
  //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
  /*
   * Reads accelerometer, gyro, and magnetometer data from IMU as AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ. 
   * These values are scaled according to the IMU datasheet to put them into correct units of g's, deg/sec, and uT. A simple first-order
   * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
   * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
   * the readings. The filter parameters B_gyro and B_accel are set to be good for a 2kHz loop rate. Finally,
   * the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;

  #if defined USE_MPU6050_I2C
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
  #elif defined USE_MPU9250_SPI
    mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
  #endif

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;

  //Magnetometer
  MagX = MgX/6.0; //uT
  MagY = MgY/6.0;
  MagZ = MgZ/6.0;
  //Correct the outputs with the calculated error values
  MagX = (MagX - MagErrorX)*MagScaleX;
  MagY = (MagY - MagErrorY)*MagScaleY;
  MagZ = (MagZ - MagErrorZ)*MagScaleZ;
  //LP filter magnetometer data
  MagX = (1.0 - B_mag)*MagX_prev + B_mag*MagX;
  MagY = (1.0 - B_mag)*MagY_prev + B_mag*MagY;
  MagZ = (1.0 - B_mag)*MagZ_prev + B_mag*MagZ;
  MagX_prev = MagX;
  MagY_prev = MagY;
  MagZ_prev = MagZ;
}

void calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
   * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
   * to boot. 
   */
  //Warm up IMU and madgwick filter in simulated main loop
  for (int i = 0; i <= 10000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt);
    loopRate(2000); //do not exceed 2000Hz
  }
}

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
  /*
   * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
   * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
   * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
   * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
   * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

  //use 6DOF algorithm if MPU6050 is being used
  #if defined USE_MPU6050_I2C 
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  #endif
  
  //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq);
    return;
  }

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    //Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    //Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalize quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  
  //compute angles - NWU
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void getDesState() {
  //DESCRIPTION: Normalizes desired control values to appropriate values
  /*
   * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
   * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
   * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
   * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
   * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
   */
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
  //DESCRIPTION: Computes control commands based on state error (angle)
  /*
   * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
   * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
   * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
   * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
   * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
   * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
   * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
   * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
   */
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  //DESCRIPTION: Computes control commands based on state error (angle) in cascaded scheme
  /*
   * Gives better performance than controlANGLE() but requires much more tuning. Not reccommended for first-time setup.
   * See the documentation for tuning this controller.
   */
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (roll_IMU - roll_IMU_prev)/dt; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol;// - Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = (pitch_IMU - pitch_IMU_prev)/dt;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol;// - Kd_pitch_angle*derivative_pitch;

  //Apply loop gain, constrain, and LP filter for artificial damping
  float Kl = 30.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;

}

void controlRATE() {
  //DESCRIPTION: Computes control commands based on state error (rate)
  /*
   * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
   */
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

//========================================================================================================================//
//                                    PHYSICS-BASED AIRSPEED ESTIMATION FUNCTIONS                                        //                           
//========================================================================================================================//

void initializeThrustMapping() {
  //DESCRIPTION: Initialize thrust mapping with default values
  
  //Default thrust curve for typical brushless motor + prop
  //These should be replaced with actual bench test data
  for (int i = 0; i < THRUST_MAP_SIZE; i++) {
    float pwm_normalized = (float)i / (THRUST_MAP_SIZE - 1);  //0 to 1
    thrustMap.pwm_points[i] = 1000 + pwm_normalized * 1000;  //1000-2000 PWM
    
    //Quadratic thrust curve: T = a*PWM² + b*PWM + c
    //Typical values for small quadcopter (adjust for your setup)
    float a = 0.0000015;  //Quadratic coefficient
    float b = -0.002;     //Linear coefficient  
    float c = 1.0;        //Constant offset
    
    float pwm = thrustMap.pwm_points[i];
    thrustMap.thrust_values[i] = a * pwm * pwm + b * pwm + c;
    
    //Ensure non-negative thrust
    if (thrustMap.thrust_values[i] < 0) thrustMap.thrust_values[i] = 0;
  }
  
  thrustMap.voltage_compensation = 1.0;  //Default, no compensation
  thrustMap.calibrated = false;          //Mark as using default values
}

float lookupThrust(float pwm_command) {
  //DESCRIPTION: Interpolate thrust from PWM command using calibration table
  
  //Clamp PWM to valid range
  pwm_command = constrain(pwm_command, thrustMap.pwm_points[0], 
                         thrustMap.pwm_points[THRUST_MAP_SIZE-1]);
  
  //Find interpolation bounds
  int i = 0;
  while (i < THRUST_MAP_SIZE-1 && thrustMap.pwm_points[i+1] < pwm_command) {
    i++;
  }
  
  if (i >= THRUST_MAP_SIZE-1) {
    return thrustMap.thrust_values[THRUST_MAP_SIZE-1] * thrustMap.voltage_compensation;
  }
  
  //Linear interpolation
  float alpha = (pwm_command - thrustMap.pwm_points[i]) / 
                (thrustMap.pwm_points[i+1] - thrustMap.pwm_points[i]);
  
  float thrust = thrustMap.thrust_values[i] + 
                 alpha * (thrustMap.thrust_values[i+1] - thrustMap.thrust_values[i]);
  
  return thrust * thrustMap.voltage_compensation;
}

void initializeAirspeedEKF() {
  //DESCRIPTION: Initialize Extended Kalman Filter for airspeed estimation
  
  //Initial state: [airspeed=0, drag_coefficient=rough_guess]
  airspeedFilter.state[0] = 0.0;      //Initial airspeed
  airspeedFilter.state[1] = 0.02;     //Initial drag coefficient guess
  
  //Initialize covariance matrix P (state uncertainty)
  airspeedFilter.P[0][0] = 25.0;      //High uncertainty in airspeed initially
  airspeedFilter.P[0][1] = 0.0;       //No cross-correlation initially
  airspeedFilter.P[1][0] = 0.0;
  airspeedFilter.P[1][1] = 0.01;      //Moderate uncertainty in drag coefficient
  
  //Process noise covariance Q
  airspeedFilter.process_noise_V = 0.1;   //Airspeed process noise
  airspeedFilter.process_noise_k = 0.001; //Drag coefficient process noise
  airspeedFilter.Q[0][0] = airspeedFilter.process_noise_V;
  airspeedFilter.Q[0][1] = 0.0;
  airspeedFilter.Q[1][0] = 0.0;
  airspeedFilter.Q[1][1] = airspeedFilter.process_noise_k;
  
  //Measurement noise variance R (IMU acceleration noise)
  airspeedFilter.measurement_noise = 0.05;  //~0.05 m/s² noise
  airspeedFilter.R = airspeedFilter.measurement_noise;
  
  //Convergence parameters
  airspeedFilter.convergence_threshold = 0.1;
  airspeedFilter.initialized = true;
  airspeedFilter.converged = false;
}

void initializeDragZones() {
  //DESCRIPTION: Initialize angle-of-attack zones for drag coefficient learning
  
  float aoa_range = 45.0;  //Total AOA range (degrees)
  float zone_width = aoa_range / NUM_AOA_DRAG_ZONES;
  
  for (int i = 0; i < NUM_AOA_DRAG_ZONES; i++) {
    dragZones[i].aoa_min = i * zone_width;
    dragZones[i].aoa_max = (i + 1) * zone_width;
    dragZones[i].k_coefficient = 0.02 * (1.0 + 0.1 * i);  //Increasing drag with AOA
    dragZones[i].k_variance = 0.01;
    dragZones[i].samples = 0;
    dragZones[i].converged = false;
    
    //RLS parameters
    dragZones[i].rls_P = 1.0;           //Initial covariance
    dragZones[i].rls_gain = 0.1;        //Initial gain
    dragZones[i].forgetting_factor = 0.99;  //Slow forgetting
  }
}

void initializeAirspeedEstimator() {
  //DESCRIPTION: Initialize the complete airspeed estimation system
  
  //Initialize aircraft parameters (adjust for your aircraft)
  airspeedEst.aircraft_mass = 1.5;        //kg - adjust for your drone
  airspeedEst.air_density = 1.225;        //kg/m³ at sea level
  airspeedEst.reference_area = 0.1;       //m² - effective drag area
  
  //Initialize estimation state
  airspeedEst.estimated_airspeed = 0.0;
  airspeedEst.true_airspeed = 0.0;
  airspeedEst.indicated_airspeed = 0.0;
  airspeedEst.current_thrust = 0.0;
  airspeedEst.estimated_drag = 0.0;
  
  //Initialize measurement processing
  airspeedEst.forward_accel = 0.0;
  airspeedEst.forward_accel_filtered = 0.0;
  airspeedEst.gravity_compensation = 0.0;
  
  //Initialize throttle dithering
  airspeedEst.base_throttle = 0.0;
  airspeedEst.dither_signal = 0.0;
  airspeedEst.dithered_throttle = 0.0;
  airspeedEst.dither_enabled = true;      //Enable by default
  airspeedEst.dither_timer = 0;
  
  //Initialize zone management
  airspeedEst.current_aoa_zone = 0;
  airspeedEst.previous_aoa_zone = 0;
  airspeedEst.zone_entry_time = 0;
  airspeedEst.zone_stability_time = 3.0;  //3 seconds in zone required
  
  //Initialize quality metrics
  airspeedEst.estimation_confidence = 0.0;
  airspeedEst.innovation_variance = 0.0;
  airspeedEst.bias_estimate = 0.0;
  airspeedEst.estimation_valid = false;
  
  //Initialize atmospheric model
  atmosphere.sea_level_pressure = 101325.0;  //Pa
  atmosphere.temperature = 288.15;           //K (15°C)
  atmosphere.altitude_estimate = 0.0;        //m
  atmosphere.density_ratio = 1.0;
  atmosphere.pressure_ratio = 1.0;
  
  //Initialize all subsystems
  initializeThrustMapping();
  initializeAirspeedEKF();
  initializeDragZones();
}

int determineAOAZone(float aoa_degrees) {
  //DESCRIPTION: Determine which AOA zone the current angle falls into
  
  aoa_degrees = abs(aoa_degrees);  //Use absolute AOA
  
  for (int i = 0; i < NUM_AOA_DRAG_ZONES; i++) {
    if (aoa_degrees >= dragZones[i].aoa_min && aoa_degrees < dragZones[i].aoa_max) {
      return i;
    }
  }
  
  return NUM_AOA_DRAG_ZONES - 1;  //Return last zone if outside range
}

void updateThrottleDither() {
  //DESCRIPTION: Generate sinusoidal throttle dither for observability
  
  if (!airspeedEst.dither_enabled) {
    airspeedEst.dithered_throttle = airspeedEst.base_throttle;
    return;
  }
  
  //Generate sine wave at dither frequency
  float time_seconds = (float)(current_time - airspeedEst.dither_timer) / 1000000.0;
  airspeedEst.dither_signal = DITHER_AMPLITUDE * sin(2.0 * 3.14159 * DITHER_FREQUENCY * time_seconds);
  
  //Apply dither to base throttle
  airspeedEst.dithered_throttle = airspeedEst.base_throttle * (1.0 + airspeedEst.dither_signal);
  
  //Ensure throttle stays within bounds
  airspeedEst.dithered_throttle = constrain(airspeedEst.dithered_throttle, 0.0, 1.0);
}

void computeForwardAcceleration() {
  //DESCRIPTION: Extract forward acceleration from IMU with gravity compensation
  
  //Get body-frame acceleration (already corrected for IMU mounting)
  float accel_body_x = AccX;  //Forward acceleration in body frame
  
  //Gravity compensation for pitch angle
  float gravity_component = sin(pitch_IMU * 0.0174533) * 9.81;  //gravity * sin(pitch)
  
  //Compensate for gravity and convert to forward acceleration
  airspeedEst.forward_accel = accel_body_x * 9.81 - gravity_component;
  
  //Low-pass filter to reduce noise (critical for airspeed estimation)
  static float filter_alpha = 0.1;  //Adjust based on IMU noise
  airspeedEst.forward_accel_filtered = (1.0 - filter_alpha) * airspeedEst.forward_accel_filtered + 
                                       filter_alpha * airspeedEst.forward_accel;
  
  airspeedEst.gravity_compensation = gravity_component;
}

void ekfTimeUpdate() {
  //DESCRIPTION: EKF prediction step - propagate state forward in time
  
  if (!airspeedFilter.initialized) return;
  
  float V = airspeedFilter.state[0];  //Current airspeed estimate
  float k = airspeedFilter.state[1];  //Current drag coefficient
  
  //Get current thrust from throttle command
  float current_pwm = (airspeedEst.dithered_throttle * 1000.0) + 1000.0;  //Convert to PWM
  airspeedEst.current_thrust = lookupThrust(current_pwm);
  
  //State prediction using dynamics: V̇ = T/m - k*V²/m
  float V_dot_predicted = (airspeedEst.current_thrust / airspeedEst.aircraft_mass) - 
                          (k * V * V / airspeedEst.aircraft_mass);
  
  //Predict next state
  airspeedFilter.state_prediction[0] = V + dt * V_dot_predicted;
  airspeedFilter.state_prediction[1] = k;  //Assume constant drag coefficient
  
  //State transition matrix F (Jacobian of f with respect to state)
  airspeedFilter.F[0][0] = 1.0 - dt * (2.0 * k * V / airspeedEst.aircraft_mass);  //∂V̇/∂V
  airspeedFilter.F[0][1] = -dt * (V * V / airspeedEst.aircraft_mass);             //∂V̇/∂k
  airspeedFilter.F[1][0] = 0.0;                                                   //∂k̇/∂V
  airspeedFilter.F[1][1] = 1.0;                                                   //∂k̇/∂k
  
  //Covariance prediction: P = F*P*F' + Q
  float P_temp[2][2];
  
  //P_temp = F * P
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_temp[i][j] = 0.0;
      for (int k = 0; k < 2; k++) {
        P_temp[i][j] += airspeedFilter.F[i][k] * airspeedFilter.P[k][j];
      }
    }
  }
  
  //P = P_temp * F' + Q
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      airspeedFilter.P[i][j] = airspeedFilter.Q[i][j];  //Start with process noise
      for (int k = 0; k < 2; k++) {
        airspeedFilter.P[i][j] += P_temp[i][k] * airspeedFilter.F[j][k];  //F[j][k] is F'[k][j]
      }
    }
  }
}

void ekfMeasurementUpdate() {
  //DESCRIPTION: EKF correction step using acceleration measurement
  
  if (!airspeedFilter.initialized) return;
  
  float V_pred = airspeedFilter.state_prediction[0];
  float k_pred = airspeedFilter.state_prediction[1];
  
  //Predicted measurement (expected V̇ based on current state)
  float V_dot_expected = (airspeedEst.current_thrust / airspeedEst.aircraft_mass) - 
                         (k_pred * V_pred * V_pred / airspeedEst.aircraft_mass);
  
  //Innovation (measurement - prediction)
  airspeedFilter.innovation = airspeedEst.forward_accel_filtered - V_dot_expected;
  
  //Measurement matrix H (Jacobian of h with respect to state)
  airspeedFilter.H[0] = -2.0 * k_pred * V_pred / airspeedEst.aircraft_mass;  //∂h/∂V
  airspeedFilter.H[1] = -V_pred * V_pred / airspeedEst.aircraft_mass;        //∂h/∂k
  
  //Innovation covariance: S = H*P*H' + R
  airspeedFilter.innovation_covariance = airspeedFilter.R;  //Start with measurement noise
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      airspeedFilter.innovation_covariance += airspeedFilter.H[i] * airspeedFilter.P[i][j] * airspeedFilter.H[j];
    }
  }
  
  //Kalman gain: K = P*H' / S
  for (int i = 0; i < 2; i++) {
    airspeedFilter.K[i] = 0.0;
    for (int j = 0; j < 2; j++) {
      airspeedFilter.K[i] += airspeedFilter.P[i][j] * airspeedFilter.H[j];
    }
    airspeedFilter.K[i] /= airspeedFilter.innovation_covariance;
  }
  
  //State update: x = x_pred + K * innovation
  for (int i = 0; i < 2; i++) {
    airspeedFilter.state[i] = airspeedFilter.state_prediction[i] + 
                              airspeedFilter.K[i] * airspeedFilter.innovation;
  }
  
  //Covariance update: P = (I - K*H) * P
  float I_KH[2][2];
  I_KH[0][0] = 1.0 - airspeedFilter.K[0] * airspeedFilter.H[0];
  I_KH[0][1] = -airspeedFilter.K[0] * airspeedFilter.H[1];
  I_KH[1][0] = -airspeedFilter.K[1] * airspeedFilter.H[0];
  I_KH[1][1] = 1.0 - airspeedFilter.K[1] * airspeedFilter.H[1];
  
  float P_new[2][2];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P_new[i][j] = 0.0;
      for (int k = 0; k < 2; k++) {
        P_new[i][j] += I_KH[i][k] * airspeedFilter.P[k][j];
      }
    }
  }
  
  //Copy back to P
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      airspeedFilter.P[i][j] = P_new[i][j];
    }
  }
  
  //Apply safety bounds to states
  airspeedFilter.state[0] = constrain(airspeedFilter.state[0], 0.0, 50.0);  //0-50 m/s airspeed
  airspeedFilter.state[1] = constrain(airspeedFilter.state[1], 0.001, 0.5); //Reasonable drag range
}

void updateZoneDragCoefficient() {
  //DESCRIPTION: Update drag coefficient for current AOA zone using RLS
  
  int zone = airspeedEst.current_aoa_zone;
  if (zone < 0 || zone >= NUM_AOA_DRAG_ZONES) return;
  
  //Check if we've been in this zone long enough for stable learning
  if ((current_time - airspeedEst.zone_entry_time) < (airspeedEst.zone_stability_time * 1000000)) {
    return;  //Not enough time in zone yet
  }
  
  //Only update if we have good observability (throttle is changing)
  if (abs(airspeedEst.dither_signal) < 0.001 && abs(airspeedEst.forward_accel_filtered) < 0.1) {
    return;  //Poor observability conditions
  }
  
  DragZone* zone_ptr = &dragZones[zone];
  
  //RLS update for this zone's drag coefficient
  float V = airspeedFilter.state[0];
  float thrust = airspeedEst.current_thrust;
  float measured_accel = airspeedEst.forward_accel_filtered;
  
  //Expected acceleration without drag: a_expected = T/m
  float a_no_drag = thrust / airspeedEst.aircraft_mass;
  
  //Drag acceleration: a_drag = measured - a_no_drag = -k*V²/m
  float a_drag = measured_accel - a_no_drag;
  
  //Regressor: x = -V²/m (relates drag acceleration to drag coefficient)
  float regressor = -(V * V) / airspeedEst.aircraft_mass;
  
  if (abs(regressor) > 0.001) {  //Avoid division by zero
    //RLS equations
    float prediction_error = a_drag - zone_ptr->k_coefficient * regressor;
    zone_ptr->rls_gain = zone_ptr->rls_P * regressor / 
                         (zone_ptr->forgetting_factor + regressor * zone_ptr->rls_P * regressor);
    
    //Update coefficient and covariance
    zone_ptr->k_coefficient += zone_ptr->rls_gain * prediction_error;
    zone_ptr->rls_P = (zone_ptr->rls_P - zone_ptr->rls_gain * regressor * zone_ptr->rls_P) / 
                      zone_ptr->forgetting_factor;
    
    //Update zone statistics
    zone_ptr->samples++;
    zone_ptr->k_variance = zone_ptr->rls_P;
    
    //Check convergence
    if (zone_ptr->samples > 100 && zone_ptr->k_variance < 0.001) {
      zone_ptr->converged = true;
    }
    
    //Apply bounds to drag coefficient
    zone_ptr->k_coefficient = constrain(zone_ptr->k_coefficient, 0.001, 0.2);
  }
}

void updateAirspeedEstimation() {
  //DESCRIPTION: Main airspeed estimation function - runs in control loop
  
  if (!airspeedFilter.initialized) return;
  
  //Store base throttle command (before dithering)
  airspeedEst.base_throttle = thro_des;
  
  //Update throttle dithering
  updateThrottleDither();
  
  //Compute forward acceleration from IMU
  computeForwardAcceleration();
  
  //Determine current AOA zone
  float current_aoa = abs(pitch_IMU);
  int new_zone = determineAOAZone(current_aoa);
  
  //Check for zone transitions
  if (new_zone != airspeedEst.current_aoa_zone) {
    airspeedEst.previous_aoa_zone = airspeedEst.current_aoa_zone;
    airspeedEst.current_aoa_zone = new_zone;
    airspeedEst.zone_entry_time = current_time;
  }
  
  //Update drag coefficient for current zone
  if (dragZones[airspeedEst.current_aoa_zone].converged) {
    airspeedFilter.state[1] = dragZones[airspeedEst.current_aoa_zone].k_coefficient;
  }
  
  //Run EKF time update (prediction)
  ekfTimeUpdate();
  
  //Run EKF measurement update (correction)
  ekfMeasurementUpdate();
  
  //Update zone-specific drag coefficient using RLS
  updateZoneDragCoefficient();
  
  //Extract final airspeed estimate
  airspeedEst.estimated_airspeed = airspeedFilter.state[0];
  airspeedEst.estimated_drag = dragZones[airspeedEst.current_aoa_zone].k_coefficient * 
                               airspeedEst.estimated_airspeed * airspeedEst.estimated_airspeed;
  
  //Compute true airspeed (corrected for air density)
  airspeedEst.true_airspeed = airspeedEst.estimated_airspeed / sqrt(atmosphere.density_ratio);
  airspeedEst.indicated_airspeed = airspeedEst.estimated_airspeed;  //For now, same as estimated
  
  //Update estimation quality metrics
  airspeedEst.estimation_confidence = constrain(1.0 / (1.0 + abs(airspeedFilter.innovation)), 0.0, 1.0);
  airspeedEst.innovation_variance = 0.9 * airspeedEst.innovation_variance + 
                                    0.1 * airspeedFilter.innovation * airspeedFilter.innovation;
  
  //Validate estimation
  airspeedEst.estimation_valid = (airspeedEst.estimated_airspeed >= 0.0) && 
                                 (airspeedEst.estimated_airspeed < 50.0) &&
                                 (airspeedEst.estimation_confidence > 0.3) &&
                                 airspeedFilter.converged;
  
  //Update thrust-to-weight ratio for performance analysis
  airspeedEst.thrust_to_weight_ratio = airspeedEst.current_thrust / 
                                       (airspeedEst.aircraft_mass * 9.81);
}

float getPhysicsBasedAirspeed() {
  //DESCRIPTION: Get current airspeed estimate for use in gain scheduling
  
  if (airspeedEst.estimation_valid) {
    return airspeedEst.estimated_airspeed;
  } else {
    //Fallback to simple estimation if physics-based method fails
    return estimateAirspeed(thro_des, abs(pitch_IMU), flightState.vertical_velocity);
  }
}



void getCommands() {
  //DESCRIPTION: Get raw PWM values for every channel from the radio
  /*
   * Updates radio PWM commands in loop based on current available commands. channel_x_pwm is the raw command used in the rest of 
   * the loop. If using a PWM or PPM receiver, the radio commands are retrieved from a function in the readPWM file separate from this one which 
   * is running a bunch of interrupts to continuously update the radio readings. If using an SBUS receiver, the alues are pulled from the SBUS library directly.
   * The raw radio commands are filtered with a first order low-pass filter to eliminate any really high frequency noise. 
   */

  #if defined USE_PPM_RX || defined USE_PWM_RX
    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
    
  #elif defined USE_SBUS_RX
    if (sbus.read(&sbusChannels[0], &sbusFailSafe, &sbusLostFrame))
    {
      //sBus scaling below is for Taranis-Plus and X4R-SB
      float scale = 0.615;  
      float bias  = 895.0; 
      channel_1_pwm = sbusChannels[0] * scale + bias;
      channel_2_pwm = sbusChannels[1] * scale + bias;
      channel_3_pwm = sbusChannels[2] * scale + bias;
      channel_4_pwm = sbusChannels[3] * scale + bias;
      channel_5_pwm = sbusChannels[4] * scale + bias;
      channel_6_pwm = sbusChannels[5] * scale + bias; 
    }

  #elif defined USE_DSM_RX
    if (DSM.timedOut(micros())) {
        //Serial.println("*** DSM RX TIMED OUT ***");
    }
    else if (DSM.gotNewFrame()) {
        uint16_t values[num_DSM_channels];
        DSM.getChannelValues(values, num_DSM_channels);

        channel_1_pwm = values[0];
        channel_2_pwm = values[1];
        channel_3_pwm = values[2];
        channel_4_pwm = values[3];
        channel_5_pwm = values[4];
        channel_6_pwm = values[5];
    }
  #endif
  
  //Low-pass the critical commands and update previous values
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  //DESCRIPTION: If radio gives garbage values, set all commands to default values
  /*
   * Radio connection failsafe used to check if the getCommands() function is returning acceptable pwm values. If any of 
   * the commands are lower than 800 or higher than 2200, then we can be certain that there is an issue with the radio
   * connection (most likely hardware related). If any of the channels show this failure, then all of the radio commands 
   * channel_x_pwm are set to default failsafe values specified in the setup. Comment out this function when troubleshooting 
   * your radio connection in case any extreme values are triggering this function to overwrite the printed variables.
   */
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;
  if (channel_6_pwm > maxVal || channel_6_pwm < minVal) check6 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5 + check6) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    channel_5_pwm = channel_5_fs;
    channel_6_pwm = channel_6_fs;
  }
}

void commandMotors() {
  //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
  /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
  int wentLow = 0;
  int pulseStart, timer;
  int flagM1 = 0;
  int flagM2 = 0;
  int flagM3 = 0;
  int flagM4 = 0;
  int flagM5 = 0;
  int flagM6 = 0;
  
  //Write all motor pins high
  digitalWrite(m1Pin, HIGH);
  digitalWrite(m2Pin, HIGH);
  digitalWrite(m3Pin, HIGH);
  digitalWrite(m4Pin, HIGH);
  digitalWrite(m5Pin, HIGH);
  digitalWrite(m6Pin, HIGH);
  pulseStart = micros();

  //Write each motor pin low as correct pulse length is reached
  while (wentLow < 6 ) { //Keep going until final (6th) pulse is finished, then done
    timer = micros();
    if ((m1_command_PWM <= timer - pulseStart) && (flagM1==0)) {
      digitalWrite(m1Pin, LOW);
      wentLow = wentLow + 1;
      flagM1 = 1;
    }
    if ((m2_command_PWM <= timer - pulseStart) && (flagM2==0)) {
      digitalWrite(m2Pin, LOW);
      wentLow = wentLow + 1;
      flagM2 = 1;
    }
    if ((m3_command_PWM <= timer - pulseStart) && (flagM3==0)) {
      digitalWrite(m3Pin, LOW);
      wentLow = wentLow + 1;
      flagM3 = 1;
    }
    if ((m4_command_PWM <= timer - pulseStart) && (flagM4==0)) {
      digitalWrite(m4Pin, LOW);
      wentLow = wentLow + 1;
      flagM4 = 1;
    } 
    if ((m5_command_PWM <= timer - pulseStart) && (flagM5==0)) {
      digitalWrite(m5Pin, LOW);
      wentLow = wentLow + 1;
      flagM5 = 1;
    } 
    if ((m6_command_PWM <= timer - pulseStart) && (flagM6==0)) {
      digitalWrite(m6Pin, LOW);
      wentLow = wentLow + 1;
      flagM6 = 1;
    } 
  }
}

void armMotors() {
  //DESCRIPTION: Sends many command pulses to the motors, to be used to arm motors in the void setup()
  /*  
   *  Loops over the commandMotors() function 50 times with a delay in between, simulating how the commandMotors()
   *  function is used in the main loop. Ensures motors arm within the void setup() where there are some delays
   *  for other processes that sometimes prevent motors from arming.
   */
  for (int i = 0; i <= 50; i++) {
    commandMotors();
    delay(2);
  }
}

void calibrateESCs() {
  //DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
  /*  
   *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
   *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
   *  uncommented when performing an ESC calibration.
   */
   while (true) {
      prev_time = current_time;      
      current_time = micros();      
      dt = (current_time - prev_time)/1000000.0;
    
      digitalWrite(13, HIGH); //LED on to indicate we are not in main loop

      getCommands(); //Pulls current available radio commands
      failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer data from IMU and LP filters to remove noise
      Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, MagY, -MagX, MagZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU (degrees)
      getDesState(); //Convert raw commands to normalized values based on saturated control limits
      
      m1_command_scaled = thro_des;
      m2_command_scaled = thro_des;
      m3_command_scaled = thro_des;
      m4_command_scaled = thro_des;
      m5_command_scaled = thro_des;
      m6_command_scaled = thro_des;
      s1_command_scaled = thro_des;
      s2_command_scaled = thro_des;
      s3_command_scaled = thro_des;
      s4_command_scaled = thro_des;
      s5_command_scaled = thro_des;
      s6_command_scaled = thro_des;
      s7_command_scaled = thro_des;
      scaleCommands(); //Scales motor commands to 125 to 250 range (oneshot125 protocol) and servo PWM commands to 0 to 180 (for servo library)
    
      //throttleCut(); //Directly sets motor commands to low based on state of ch5
      
      servo1.write(s1_command_PWM); 
      servo2.write(s2_command_PWM);
      servo3.write(s3_command_PWM);
      servo4.write(s4_command_PWM);
      servo5.write(s5_command_PWM);
      servo6.write(s6_command_PWM);
      servo7.write(s7_command_PWM);
      commandMotors(); //Sends command pulses to each motor pin using OneShot125 protocol
      
      //printRadioData(); //Radio pwm values (expected: 1000 to 2000)
      
      loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
   }
}

float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //Difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //Maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //Minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //Constrain param within max bounds
  
  return param;
}

float floatFaderLinear2(float param, float param_des, float param_lower, float param_upper, float fadeTime_up, float fadeTime_down, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable from its current value to the desired value, up or down
  /*  
   *  Takes in a float variable to be modified, desired new position, upper value, lower value, fade time, and the loop frequency 
   *  and linearly fades that param variable up or down to the desired value. This function can be called in controlMixer()
   *  to fade up or down between flight modes monitored by an auxillary radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. 
   *  
   */
  if (param > param_des) { //Need to fade down to get to desired
    float diffParam = (param_upper - param_des)/(fadeTime_down*loopFreq);
    param = param - diffParam;
  }
  else if (param < param_des) { //Need to fade up to get to desired
    float diffParam = (param_des - param_lower)/(fadeTime_up*loopFreq);
    param = param + diffParam;
  }

  param = constrain(param, param_lower, param_upper); //Constrain param within max bounds
  
  return param;
}

void switchRollYaw(int reverseRoll, int reverseYaw) {
  //DESCRIPTION: Switches roll_des and yaw_des variables for tailsitter-type configurations
  /*
   * Takes in two integers (either 1 or -1) corresponding to the desired reversing of the roll axis and yaw axis, respectively.
   * Reversing of the roll or yaw axis may be needed when switching between the two for some dynamic configurations. Inputs of 1, 1 does not 
   * reverse either of them, while -1, 1 will reverse the output corresponding to the new roll axis. 
   * This function may be replaced in the future by a function that switches the IMU data instead (so that angle can also be estimated with the 
   * IMU tilted 90 degrees from default level).
   */
  float switch_holder;

  switch_holder = yaw_des;
  yaw_des = reverseYaw*roll_des;
  roll_des = reverseRoll*switch_holder;
}

void throttleCut() {
  //DESCRIPTION: Directly set actuator outputs to minimum value if triggered
  /*
      Monitors the state of radio command channel_5_pwm and directly sets the mx_command_PWM values to minimum (120 is
      minimum for oneshot125 protocol, 0 is minimum for standard PWM servo library used) if channel 5 is high. This is the last function
      called before commandMotors() is called so that the last thing checked is if the user is giving permission to command
      the motors to anything other than minimum value. Safety first.

      channel_5_pwm is LOW then throttle cut is OFF and throttle value can change. (ThrottleCut is DEACTIVATED)
      channel_5_pwm is HIGH then throttle cut is ON and throttle value = 120 only. (ThrottleCut is ACTIVATED), (drone is DISARMED)
  */
  if ((channel_5_pwm > 1500) || (armedFly == false)) {
    armedFly = false;
    m1_command_PWM = 120;
    m2_command_PWM = 120;
    m3_command_PWM = 120;
    m4_command_PWM = 120;
    m5_command_PWM = 120;
    m6_command_PWM = 120;

    //Uncomment if using servo PWM variables to control motor ESCs
    //s1_command_PWM = 0;
    //s2_command_PWM = 0;
    //s3_command_PWM = 0;
    //s4_command_PWM = 0;
    //s5_command_PWM = 0;
    //s6_command_PWM = 0;
    //s7_command_PWM = 0;
  }
}

void calibrateMagnetometer() {
  #if defined USE_MPU9250_SPI 
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if(success) {
      Serial.println("Calibration Successful!");
      Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
      Serial.print("float MagErrorX = ");
      Serial.print(mpu9250.getMagBiasX_uT());
      Serial.println(";");
      Serial.print("float MagErrorY = ");
      Serial.print(mpu9250.getMagBiasY_uT());
      Serial.println(";");
      Serial.print("float MagErrorZ = ");
      Serial.print(mpu9250.getMagBiasZ_uT());
      Serial.println(";");
      Serial.print("float MagScaleX = ");
      Serial.print(mpu9250.getMagScaleFactorX());
      Serial.println(";");
      Serial.print("float MagScaleY = ");
      Serial.print(mpu9250.getMagScaleFactorY());
      Serial.println(";");
      Serial.print("float MagScaleZ = ");
      Serial.print(mpu9250.getMagScaleFactorZ());
      Serial.println(";");
      Serial.println(" ");
      Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else {
      Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }
  
    while(1); //Halt code so it won't enter main loop until this function commented out
  #endif
  Serial.println("Error: MPU9250 not selected. Cannot calibrate non-existent magnetometer.");
  while(1); //Halt code so it won't enter main loop until this function commented out
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1:"));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2:"));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3:"));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4:"));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5:"));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6:"));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des:"));
    Serial.print(thro_des);
    Serial.print(F(" roll_des:"));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des:"));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des:"));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX:"));
    Serial.print(GyroX);
    Serial.print(F(" GyroY:"));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ:"));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX:"));
    Serial.print(AccX);
    Serial.print(F(" AccY:"));
    Serial.print(AccY);
    Serial.print(F(" AccZ:"));
    Serial.println(AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX:"));
    Serial.print(MagX);
    Serial.print(F(" MagY:"));
    Serial.print(MagY);
    Serial.print(F(" MagZ:"));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll:"));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch:"));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw:"));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID:"));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID:"));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID:"));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command:"));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command:"));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command:"));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command:"));
    Serial.print(m4_command_PWM);
    Serial.print(F(" m5_command:"));
    Serial.print(m5_command_PWM);
    Serial.print(F(" m6_command:"));
    Serial.println(m6_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command:"));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command:"));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command:"));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command:"));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command:"));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command:"));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command:"));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt:"));
    Serial.println(dt*1000000.0);
  }
}

void printAdaptivePIDStatus() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Flight Regime - Throttle Zone:"));
    Serial.print(current_throttle_zone);
    Serial.print(F(" AOA Zone:"));
    Serial.print(current_aoa_zone);
    Serial.print(F(" Learning Active:"));
    Serial.println(learning_active);
  }
}

void printCurrentGains() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Current Gains - Kp_roll:"));
    Serial.print(Kp_roll_angle);
    Serial.print(F(" Ki_roll:"));
    Serial.print(Ki_roll_angle);
    Serial.print(F(" Kp_pitch:"));
    Serial.print(Kp_pitch_angle);
    Serial.print(F(" Ki_pitch:"));
    Serial.println(Ki_pitch_angle);
  }
}

void printPerformanceMetrics() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Performance - Roll Error:"));
    Serial.print(performance_metric_roll);
    Serial.print(F(" Pitch Error:"));
    Serial.print(performance_metric_pitch);
    Serial.print(F(" Yaw Error:"));
    Serial.println(performance_metric_yaw);
  }
}

//=========================================================================================//

//HELPER FUNCTIONS

float invSqrt(float x) {
  //Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  /*
  //alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
  float tmp = *(float*)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
  */
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}

void initializeAdaptiveController() {
  //DESCRIPTION: Initialize all adaptive controller components
  
  //Initialize 4D gain scheduling grid with baseline values
  for (int t = 0; t < NUM_THROTTLE_ZONES; t++) {
    for (int a = 0; a < NUM_AOA_ZONES; a++) {
      for (int p = 0; p < NUM_PHASE_ZONES; p++) {
        for (int s = 0; s < NUM_AIRSPEED_ZONES; s++) {
          PIDGains* gains = &gainSchedule[t][a][p][s];
          
          //Base gains with phase and airspeed modulation
          float throttle_factor = (float)t / (NUM_THROTTLE_ZONES - 1);
          float aoa_factor = (float)a / (NUM_AOA_ZONES - 1);
          float phase_factor = phaseGainModifier((FlightPhase)p);
          float airspeed_factor = 1.0 + 0.3 * ((float)s / (NUM_AIRSPEED_ZONES - 1));
          
          gains->Kp_roll = (0.15 + 0.15 * throttle_factor + 0.1 * aoa_factor) * phase_factor * airspeed_factor;
          gains->Ki_roll = (0.2 + 0.2 * throttle_factor) * phase_factor;
          gains->Kd_roll = (0.0002 + 0.0008 * throttle_factor) * airspeed_factor;
          
          gains->Kp_pitch = gains->Kp_roll;
          gains->Ki_pitch = gains->Ki_roll;
          gains->Kd_pitch = gains->Kd_roll;
          
          gains->Kp_yaw = 0.3 + 0.1 * throttle_factor;
          gains->Ki_yaw = 0.05 + 0.03 * throttle_factor;
          gains->Kd_yaw = 0.00015 + 0.0001 * throttle_factor;
          
          gains->confidence = 0.5;
          gains->usage_count = 0;
          gains->last_performance = 50.0;
        }
      }
    }
  }
  
  //Initialize metric buffers
  memset(&rollBuffer, 0, sizeof(MetricBuffers));
  memset(&pitchBuffer, 0, sizeof(MetricBuffers));
  memset(&yawBuffer, 0, sizeof(MetricBuffers));
  
  //Initialize safety monitor
  safetyMonitor.max_error_threshold = 45.0;
  safetyMonitor.oscillation_threshold = 15.0;
  safetyMonitor.control_saturation_limit = 0.95;
  safetyMonitor.consecutive_bad_samples = 0;
  safetyMonitor.emergency_freeze = false;
  
  //Initialize flight detector
  memset(&flightDetector, 0, sizeof(FlightDetector));
  flightDetector.steady_duration_required = 3.0;
}

float phaseGainModifier(FlightPhase phase) {
  //DESCRIPTION: Return gain modifier based on flight phase
  switch (phase) {
    case PHASE_HOVER: return 0.8;
    case PHASE_CLIMB: return 1.2;
    case PHASE_CRUISE: return 1.0;
    case PHASE_DESCENT: return 0.9;
    case PHASE_TURN: return 1.3;
    case PHASE_TRANSITION: return 1.1;
    default: return 1.0;
  }
}

void detectAdvancedFlightRegime() {
  //DESCRIPTION: Sophisticated multi-dimensional flight regime detection
  
  //Update flight detector history
  int idx = flightDetector.history_index;
  flightDetector.throttle_history[idx] = thro_des;
  flightDetector.aoa_history[idx] = abs(pitch_IMU);
  flightDetector.turn_rate_history[idx] = abs(GyroZ);
  
  //Estimate vertical velocity from accelerometer
  static float prev_AccZ = 0;
  flightState.vertical_velocity = (AccZ - prev_AccZ) * 9.81 / dt;
  prev_AccZ = AccZ;
  flightDetector.vertical_vel_history[idx] = flightState.vertical_velocity;
  
  flightDetector.history_index = (idx + 1) % 20;
  
  //Calculate moving averages for stability
  float avg_throttle = 0, avg_aoa = 0, avg_turn_rate = 0, avg_vertical_vel = 0;
  for (int i = 0; i < 20; i++) {
    avg_throttle += flightDetector.throttle_history[i];
    avg_aoa += flightDetector.aoa_history[i];
    avg_turn_rate += flightDetector.turn_rate_history[i];
    avg_vertical_vel += flightDetector.vertical_vel_history[i];
  }
  avg_throttle /= 20.0;
  avg_aoa /= 20.0;
  avg_turn_rate /= 20.0;
  avg_vertical_vel /= 20.0;
  
  //Detect flight phase with hysteresis
  FlightPhase new_phase = flightState.current_phase;
  
  if (avg_turn_rate > 30.0) {
    new_phase = PHASE_TURN;
  } else if (avg_vertical_vel > 2.0 && avg_throttle > 0.6) {
    new_phase = PHASE_CLIMB;
  } else if (avg_vertical_vel < -1.5) {
    new_phase = PHASE_DESCENT;
  } else if (avg_throttle < 0.3 && avg_aoa < 15.0) {
    new_phase = PHASE_HOVER;
  } else if (avg_throttle > 0.4 && avg_aoa < 20.0) {
    new_phase = PHASE_CRUISE;
  } else {
    new_phase = PHASE_TRANSITION;
  }
  
  //Apply hysteresis
  static FlightPhase phase_candidate = PHASE_HOVER;
  static int phase_hold_count = 0;
  
  if (new_phase == phase_candidate) {
    phase_hold_count++;
    if (phase_hold_count > 40) {
      flightState.current_phase = new_phase;
      flightState.transitioning = false;
    }
  } else {
    phase_candidate = new_phase;
    phase_hold_count = 0;
    flightState.transitioning = true;
  }
  
  //Calculate 4D grid indices with smooth interpolation
  float throttle_continuous = avg_throttle * (NUM_THROTTLE_ZONES - 1);
  flightState.throttle_idx = constrain((int)throttle_continuous, 0, NUM_THROTTLE_ZONES - 2);
  flightState.throttle_factor = throttle_continuous - flightState.throttle_idx;
  
  float aoa_normalized = constrain(avg_aoa / 45.0, 0.0, 1.0);
  float aoa_continuous = aoa_normalized * aoa_normalized * (NUM_AOA_ZONES - 1);
  flightState.aoa_idx = constrain((int)aoa_continuous, 0, NUM_AOA_ZONES - 2);
  flightState.aoa_factor = aoa_continuous - flightState.aoa_idx;
  
  flightState.phase_idx = (int)flightState.current_phase;
  flightState.phase_factor = flightState.transitioning ? 0.5 : 0.0;
  
  flightState.airspeed_estimate = getPhysicsBasedAirspeed();
  float airspeed_continuous = constrain(flightState.airspeed_estimate / 20.0, 0.0, 1.0) * (NUM_AIRSPEED_ZONES - 1);
  flightState.airspeed_idx = constrain((int)airspeed_continuous, 0, NUM_AIRSPEED_ZONES - 2);
  flightState.airspeed_factor = airspeed_continuous - flightState.airspeed_idx;
  
  flightState.turn_rate = avg_turn_rate;
  flightDetector.coordinated_turn = (abs(avg_turn_rate) > 15.0) && (abs(roll_IMU) > 10.0);
}

float estimateAirspeed(float throttle, float aoa, float vertical_vel) {
  //DESCRIPTION: Rough airspeed estimation for gain scheduling
  float base_speed = throttle * 15.0;
  float aoa_factor = cos(aoa * 0.0174533);
  float horizontal_speed = sqrt(max(0.0, base_speed * base_speed - vertical_vel * vertical_vel));
  return horizontal_speed * aoa_factor;
}

PIDGains quadrilinearInterpolate() {
  //DESCRIPTION: 4D interpolation across the gain scheduling grid
  
  PIDGains result = {0};
  
  //Get the 16 corner points of the 4D hypercube
  for (int dt = 0; dt <= 1; dt++) {
    for (int da = 0; da <= 1; da++) {
      for (int dp = 0; dp <= 1; dp++) {
        for (int ds = 0; ds <= 1; ds++) {
          int t_idx = min(flightState.throttle_idx + dt, NUM_THROTTLE_ZONES - 1);
          int a_idx = min(flightState.aoa_idx + da, NUM_AOA_ZONES - 1);
          int p_idx = min(flightState.phase_idx + dp, NUM_PHASE_ZONES - 1);
          int s_idx = min(flightState.airspeed_idx + ds, NUM_AIRSPEED_ZONES - 1);
          
          PIDGains* corner = &gainSchedule[t_idx][a_idx][p_idx][s_idx];
          
          //Calculate 4D interpolation weights
          float wt = dt == 0 ? (1.0 - flightState.throttle_factor) : flightState.throttle_factor;
          float wa = da == 0 ? (1.0 - flightState.aoa_factor) : flightState.aoa_factor;
          float wp = dp == 0 ? (1.0 - flightState.phase_factor) : flightState.phase_factor;
          float ws = ds == 0 ? (1.0 - flightState.airspeed_factor) : flightState.airspeed_factor;
          
          float weight = wt * wa * wp * ws;
          
          //Accumulate weighted gains
          result.Kp_roll += weight * corner->Kp_roll;
          result.Ki_roll += weight * corner->Ki_roll;
          result.Kd_roll += weight * corner->Kd_roll;
          result.Kp_pitch += weight * corner->Kp_pitch;
          result.Ki_pitch += weight * corner->Ki_pitch;
          result.Kd_pitch += weight * corner->Kd_pitch;
          result.Kp_yaw += weight * corner->Kp_yaw;
          result.Ki_yaw += weight * corner->Ki_yaw;
          result.Kd_yaw += weight * corner->Kd_yaw;
          
          //Update usage statistics
          corner->usage_count++;
        }
      }
    }
  }
  
  return result;
}

void updateGainScheduling() {
  //DESCRIPTION: Update PID gains using smooth 4D interpolation
  
  if (!gainScheduling_enabled) return;
  
  PIDGains interpolated = quadrilinearInterpolate();
  
  //Apply interpolated gains to control system
  Kp_roll_angle = interpolated.Kp_roll;
  Ki_roll_angle = interpolated.Ki_roll;
  Kd_roll_angle = interpolated.Kd_roll;
  Kp_pitch_angle = interpolated.Kp_pitch;
  Ki_pitch_angle = interpolated.Ki_pitch;
  Kd_pitch_angle = interpolated.Kd_pitch;
  Kp_yaw = interpolated.Kp_yaw;
  Ki_yaw = interpolated.Ki_yaw;
  Kd_yaw = interpolated.Kd_yaw;
}

void updatePerformanceBuffers() {
  //DESCRIPTION: Update sliding window buffers for rich performance analysis
  
  //Roll axis buffer update
  rollBuffer.error[rollBuffer.index] = error_roll;
  rollBuffer.setpoint[rollBuffer.index] = roll_des;
  rollBuffer.output[rollBuffer.index] = roll_PID;
  rollBuffer.derivative[rollBuffer.index] = GyroX;
  rollBuffer.timestamps[rollBuffer.index] = current_time;
  
  //Pitch axis buffer update
  pitchBuffer.error[pitchBuffer.index] = error_pitch;
  pitchBuffer.setpoint[pitchBuffer.index] = pitch_des;
  pitchBuffer.output[pitchBuffer.index] = pitch_PID;
  pitchBuffer.derivative[pitchBuffer.index] = GyroY;
  pitchBuffer.timestamps[pitchBuffer.index] = current_time;
  
  //Yaw axis buffer update
  yawBuffer.error[yawBuffer.index] = error_yaw;
  yawBuffer.setpoint[yawBuffer.index] = yaw_des;
  yawBuffer.output[yawBuffer.index] = yaw_PID;
  yawBuffer.derivative[yawBuffer.index] = GyroZ;
  yawBuffer.timestamps[yawBuffer.index] = current_time;
  
  //Increment indices and check if buffers are full
  rollBuffer.index = (rollBuffer.index + 1) % METRIC_WINDOW_SIZE;
  pitchBuffer.index = (pitchBuffer.index + 1) % METRIC_WINDOW_SIZE;
  yawBuffer.index = (yawBuffer.index + 1) % METRIC_WINDOW_SIZE;
  
  if (rollBuffer.index == 0) rollBuffer.buffer_full = true;
  if (pitchBuffer.index == 0) pitchBuffer.buffer_full = true;
  if (yawBuffer.index == 0) yawBuffer.buffer_full = true;
}

void computeAdvancedMetrics(MetricBuffers* buffer, PerformanceMetrics* metrics) {
  //DESCRIPTION: Compute comprehensive performance metrics from buffer data
  
  if (!buffer->buffer_full && buffer->index < 20) return; //Need minimum data
  
  int samples = buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index;
  
  //Time domain metrics
  float error_sum = 0, error_sq_sum = 0, peak_error = 0;
  float control_sum = 0;
  
  for (int i = 0; i < samples; i++) {
    float abs_error = abs(buffer->error[i]);
    error_sum += abs_error;
    error_sq_sum += buffer->error[i] * buffer->error[i];
    peak_error = max(peak_error, abs_error);
    control_sum += abs(buffer->output[i]);
  }
  
  metrics->mean_abs_error = error_sum / samples;
  metrics->rms_error = sqrt(error_sq_sum / samples);
  metrics->peak_error = peak_error;
  metrics->control_effort = control_sum / samples;
  
  //Settling time and overshoot analysis
  computeStepResponse(buffer, metrics);
  
  //Frequency domain analysis
  computeFrequencyMetrics(buffer, metrics);
  
  //Bias and trend detection
  computeBiasAndTrend(buffer, metrics);
  
  //Overall health score (weighted combination)
  float error_score = constrain(100.0 - metrics->mean_abs_error * 10.0, 0.0, 100.0);
  float stability_score = constrain(100.0 - metrics->oscillation_count * 2.0, 0.0, 100.0);
  float effort_score = constrain(100.0 - metrics->control_effort * 100.0, 0.0, 100.0);
  
  metrics->health_score = 0.5 * error_score + 0.3 * stability_score + 0.2 * effort_score;
}

void computeStepResponse(MetricBuffers* buffer, PerformanceMetrics* metrics) {
  //DESCRIPTION: Analyze step response characteristics
  
  //Detect step inputs (large setpoint changes)
  float max_step = 0;
  int step_index = -1;
  
  for (int i = 1; i < (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index); i++) {
    float step_size = abs(buffer->setpoint[i] - buffer->setpoint[i-1]);
    if (step_size > max_step && step_size > STEP_DETECT_THRESHOLD) {
      max_step = step_size;
      step_index = i;
    }
  }
  
  if (step_index < 0) return; //No significant step found
  
  //Analyze response after step
  float final_value = buffer->setpoint[step_index];
  float overshoot = 0;
  int settle_idx = -1;
  int rise_start = -1, rise_end = -1;
  
  for (int i = step_index; i < (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index); i++) {
    float current_value = buffer->setpoint[i] - buffer->error[i]; //Actual value
    
    //Track overshoot
    if (buffer->setpoint[step_index] > buffer->setpoint[step_index-1]) { //Positive step
      overshoot = max(overshoot, current_value - final_value);
    } else { //Negative step
      overshoot = max(overshoot, final_value - current_value);
    }
    
    //Find settling time (within 5% of final value)
    if (abs(current_value - final_value) < SETTLE_THRESHOLD * abs(max_step) && settle_idx < 0) {
      settle_idx = i;
    }
    
    //Find rise time (10% to 90% of step)
    float rise_level = (current_value - buffer->setpoint[step_index-1]) / max_step;
    if (rise_level >= 0.1 && rise_start < 0) rise_start = i;
    if (rise_level >= 0.9 && rise_end < 0) rise_end = i;
  }
  
  metrics->overshoot = (overshoot / abs(max_step)) * 100.0; //Percentage
  metrics->settling_time = settle_idx > 0 ? (float)(settle_idx - step_index) / 2000.0 : 999.0; //Seconds
  metrics->rise_time = (rise_start > 0 && rise_end > 0) ? (float)(rise_end - rise_start) / 2000.0 : 999.0;
}

void computeFrequencyMetrics(MetricBuffers* buffer, PerformanceMetrics* metrics) {
  //DESCRIPTION: Simplified frequency domain analysis
  
  //Count zero crossings for oscillation frequency
  int zero_crossings = 0;
  float prev_error = buffer->error[0];
  
  for (int i = 1; i < (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index); i++) {
    if ((buffer->error[i] > 0 && prev_error <= 0) || (buffer->error[i] <= 0 && prev_error > 0)) {
      zero_crossings++;
    }
    prev_error = buffer->error[i];
  }
  
  metrics->oscillation_count = zero_crossings;
  
  //Estimate dominant frequency (very rough)
  float window_time = (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index) / 2000.0;
  metrics->oscillation_freq = zero_crossings / (2.0 * window_time); //Hz
  
  //Estimate damping ratio from envelope decay (simplified)
  float early_peak = 0, late_peak = 0;
  int mid_point = (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index) / 2;
  
  for (int i = 0; i < mid_point; i++) {
    early_peak = max(early_peak, abs(buffer->error[i]));
  }
  for (int i = mid_point; i < (buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index); i++) {
    late_peak = max(late_peak, abs(buffer->error[i]));
  }
  
  if (early_peak > 0) {
    float decay_ratio = late_peak / early_peak;
    metrics->damping_ratio = constrain(-log(decay_ratio) / 3.14159, 0.0, 2.0);
  } else {
    metrics->damping_ratio = 1.0; //Assume critically damped
  }
}

void computeBiasAndTrend(MetricBuffers* buffer, PerformanceMetrics* metrics) {
  //DESCRIPTION: Detect steady-state bias and error trends
  
  int samples = buffer->buffer_full ? METRIC_WINDOW_SIZE : buffer->index;
  if (samples < 10) return;
  
  //Compute steady-state error (average of last 25% of samples)
  int steady_samples = samples / 4;
  int start_idx = samples - steady_samples;
  float steady_sum = 0;
  
  for (int i = start_idx; i < samples; i++) {
    int idx = (buffer->index - samples + i + METRIC_WINDOW_SIZE) % METRIC_WINDOW_SIZE;
    steady_sum += buffer->error[idx];
  }
  
  metrics->steady_state_error = steady_sum / steady_samples;
  
  //Compute trend (linear regression on error magnitude)
  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
  
  for (int i = 0; i < samples; i++) {
    int idx = (buffer->index - samples + i + METRIC_WINDOW_SIZE) % METRIC_WINDOW_SIZE;
    float x = (float)i;
    float y = abs(buffer->error[idx]);
    
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_x2 += x * x;
  }
  
  float slope = (samples * sum_xy - sum_x * sum_y) / (samples * sum_x2 - sum_x * sum_x);
  metrics->error_trend = slope; //Positive = error increasing, negative = decreasing
}

void detectFlightConditions() {
  //DESCRIPTION: Advanced flight condition and disturbance detection
  
  //Compute control input variance for maneuver detection
  float roll_input_var = 0, pitch_input_var = 0, yaw_input_var = 0;
  float roll_mean = 0, pitch_mean = 0, yaw_mean = 0;
  
  //Calculate means
  for (int i = 0; i < 20; i++) {
    roll_mean += abs(roll_des);
    pitch_mean += abs(pitch_des);
    yaw_mean += abs(yaw_des);
  }
  roll_mean /= 20.0;
  pitch_mean /= 20.0;
  yaw_mean /= 20.0;
  
  //Calculate variances
  for (int i = 0; i < 20; i++) {
    roll_input_var += (abs(roll_des) - roll_mean) * (abs(roll_des) - roll_mean);
    pitch_input_var += (abs(pitch_des) - pitch_mean) * (abs(pitch_des) - pitch_mean);
    yaw_input_var += (abs(yaw_des) - yaw_mean) * (abs(yaw_des) - yaw_mean);
  }
  flightDetector.stick_input_variance = (roll_input_var + pitch_input_var + yaw_input_var) / 60.0;
  
  //Detect rapid inputs
  flightDetector.rapid_roll_input = abs(roll_des - roll_des_prev) > 15.0;
  flightDetector.rapid_pitch_input = abs(pitch_des - pitch_des_prev) > 15.0;
  flightDetector.rapid_yaw_input = abs(yaw_des - error_yaw_prev) > 30.0;
  
  //Compute turbulence index from high-frequency accelerometer content
  static float accel_history_x[10], accel_history_y[10], accel_history_z[10];
  static int accel_idx = 0;
  
  accel_history_x[accel_idx] = AccX;
  accel_history_y[accel_idx] = AccY;
  accel_history_z[accel_idx] = AccZ;
  accel_idx = (accel_idx + 1) % 10;
  
  float accel_mean_x = 0, accel_mean_y = 0, accel_mean_z = 0;
  for (int i = 0; i < 10; i++) {
    accel_mean_x += accel_history_x[i];
    accel_mean_y += accel_history_y[i];
    accel_mean_z += accel_history_z[i];
  }
  accel_mean_x /= 10.0;
  accel_mean_y /= 10.0;
  accel_mean_z /= 10.0;
  
  flightDetector.accel_variance_x = 0;
  flightDetector.accel_variance_y = 0;
  flightDetector.accel_variance_z = 0;
  
  for (int i = 0; i < 10; i++) {
    flightDetector.accel_variance_x += (accel_history_x[i] - accel_mean_x) * (accel_history_x[i] - accel_mean_x);
    flightDetector.accel_variance_y += (accel_history_y[i] - accel_mean_y) * (accel_history_y[i] - accel_mean_y);
    flightDetector.accel_variance_z += (accel_history_z[i] - accel_mean_z) * (accel_history_z[i] - accel_mean_z);
  }
  
  flightDetector.turbulence_index = sqrt(flightDetector.accel_variance_x + 
                                        flightDetector.accel_variance_y + 
                                        flightDetector.accel_variance_z);
  
  flightDetector.in_turbulence = flightDetector.turbulence_index > 0.5; //Threshold for turbulence
  
  //Determine if in steady flight for learning
  bool stable_inputs = flightDetector.stick_input_variance < 25.0;
  bool no_maneuvers = !flightDetector.rapid_roll_input && !flightDetector.rapid_pitch_input && !flightDetector.rapid_yaw_input;
  bool low_turbulence = !flightDetector.in_turbulence;
  bool reasonable_attitude = abs(roll_IMU) < 30.0 && abs(pitch_IMU) < 30.0;
  
  flightDetector.steady_flight = stable_inputs && no_maneuvers && low_turbulence && reasonable_attitude;
  
  if (flightDetector.steady_flight) {
    if (flightDetector.steady_start_time == 0) {
      flightDetector.steady_start_time = current_time;
    }
  } else {
    flightDetector.steady_start_time = 0;
  }
}

void manageAdaptiveLearning() {
  //DESCRIPTION: State machine for adaptive learning management
  
  static unsigned long state_change_time = 0;
  
  //Emergency freeze conditions
  if (rollMetrics.health_score < 20.0 || pitchMetrics.health_score < 20.0 || 
      rollMetrics.oscillation_count > 30 || pitchMetrics.oscillation_count > 30) {
    if (learningState != LEARNING_FROZEN) {
      learningState = LEARNING_FROZEN;
      learning_freeze_timer = current_time;
      safetyMonitor.emergency_freeze = true;
      safetyMonitor.consecutive_bad_samples++;
    }
  }
  
  //State machine transitions
  switch (learningState) {
    case LEARNING_DISABLED:
      if (adaptivePID_enabled) {
        learningState = LEARNING_OBSERVING;
        state_change_time = current_time;
      }
      break;
      
    case LEARNING_OBSERVING:
      //Collect performance data for 10 seconds before adapting
      if (current_time - state_change_time > 10000000) {
        if (flightDetector.steady_flight && 
            (current_time - flightDetector.steady_start_time) > (flightDetector.steady_duration_required * 1000000)) {
          learningState = LEARNING_ADAPTING;
          state_change_time = current_time;
        }
      }
      break;
      
    case LEARNING_ADAPTING:
      //Active learning - check for degrading performance
      if (!flightDetector.steady_flight) {
        learningState = LEARNING_OBSERVING;
        state_change_time = current_time;
      } else if (rollMetrics.health_score < 40.0 || pitchMetrics.health_score < 40.0) {
        learningState = LEARNING_ROLLBACK;
        rollback_triggered = true;
        state_change_time = current_time;
      }
      break;
      
    case LEARNING_FROZEN:
      //Wait 30 seconds before attempting to resume
      if (current_time - learning_freeze_timer > 30000000) {
        if (rollMetrics.health_score > 60.0 && pitchMetrics.health_score > 60.0) {
          learningState = LEARNING_OBSERVING;
          safetyMonitor.emergency_freeze = false;
          state_change_time = current_time;
        }
      }
      break;
      
    case LEARNING_ROLLBACK:
      //Roll back to previous good gains and resume observing
      restorePreviousGains();
      learningState = LEARNING_OBSERVING;
      rollback_triggered = false;
      state_change_time = current_time;
      break;
  }
}

void performanceBasedAdaptation() {
  //DESCRIPTION: Advanced adaptive learning using multiple performance metrics
  
  if (learningState != LEARNING_ADAPTING) return;
  
  //Respect wind-controlled adaptation freezing
  if (windAnalyzer.learning_frozen || adaptation_gain == 0.0) return;
  
  //Only adapt every 1000 iterations (0.5 seconds) for stability
  static unsigned long adaptation_counter = 0;
  adaptation_counter++;
  if (adaptation_counter % 1000 != 0) return;
  
  //Get current grid position
  int t_idx = flightState.throttle_idx;
  int a_idx = flightState.aoa_idx;
  int p_idx = flightState.phase_idx;
  int s_idx = flightState.airspeed_idx;
  
  PIDGains* current_gains = &gainSchedule[t_idx][a_idx][p_idx][s_idx];
  
  //Store previous gains for potential rollback
  static PIDGains previous_gains = *current_gains;
  
  //Determine adaptation direction and magnitude based on metrics
  float roll_adaptation = calculateAdaptationDirection(&rollMetrics, "roll");
  float pitch_adaptation = calculateAdaptationDirection(&pitchMetrics, "pitch");
  float yaw_adaptation = calculateAdaptationDirection(&yawMetrics, "yaw");
  
  //Adaptive learning rate based on confidence, performance trend, and wind conditions
  float base_learning_rate = adaptation_gain;  //Use wind-controlled adaptation rate
  float confidence_factor = current_gains->confidence;
  float performance_factor = constrain((rollMetrics.health_score + pitchMetrics.health_score) / 200.0, 0.1, 1.0);
  
  float learning_rate = base_learning_rate * confidence_factor * performance_factor;
  
  //Apply momentum for stability
  static float roll_momentum = 0, pitch_momentum = 0, yaw_momentum = 0;
  roll_momentum = adaptation_momentum * roll_momentum + (1.0 - adaptation_momentum) * roll_adaptation;
  pitch_momentum = adaptation_momentum * pitch_momentum + (1.0 - adaptation_momentum) * pitch_adaptation;
  yaw_momentum = adaptation_momentum * yaw_momentum + (1.0 - adaptation_momentum) * yaw_adaptation;
  
  //Update gains with safety bounds (adjusted for fixed-wing)
  current_gains->Kp_roll = constrain(current_gains->Kp_roll + learning_rate * roll_momentum, 0.02, 0.5);
  current_gains->Ki_roll = adaptIntegralGain(current_gains->Ki_roll, rollMetrics.steady_state_error, learning_rate);
  current_gains->Kd_roll = adaptDerivativeGain(current_gains->Kd_roll, rollMetrics.oscillation_freq, rollMetrics.damping_ratio, learning_rate);
  
  current_gains->Kp_pitch = constrain(current_gains->Kp_pitch + learning_rate * pitch_momentum, 0.02, 0.4);
  current_gains->Ki_pitch = adaptIntegralGain(current_gains->Ki_pitch, pitchMetrics.steady_state_error, learning_rate);
  current_gains->Kd_pitch = adaptDerivativeGain(current_gains->Kd_pitch, pitchMetrics.oscillation_freq, pitchMetrics.damping_ratio, learning_rate);
  
  current_gains->Kp_yaw = constrain(current_gains->Kp_yaw + learning_rate * yaw_momentum, 0.05, 0.3);
  current_gains->Ki_yaw = adaptIntegralGain(current_gains->Ki_yaw, yawMetrics.steady_state_error, learning_rate * 0.5);
  current_gains->Kd_yaw = adaptDerivativeGain(current_gains->Kd_yaw, yawMetrics.oscillation_freq, yawMetrics.damping_ratio, learning_rate * 0.5);
  
  //Update confidence based on performance improvement
  float performance_delta = (rollMetrics.health_score + pitchMetrics.health_score) / 2.0 - current_gains->last_performance;
  current_gains->confidence = constrain(current_gains->confidence + 0.01 * performance_delta / 10.0, 0.1, 1.0);
  current_gains->last_performance = (rollMetrics.health_score + pitchMetrics.health_score) / 2.0;
  
  //Check for performance degradation and trigger rollback if needed
  if (performance_delta < -10.0 && current_gains->confidence > 0.3) {
    *current_gains = previous_gains; //Restore previous gains
    learningState = LEARNING_ROLLBACK;
  } else {
    previous_gains = *current_gains; //Update backup
  }
}

float calculateAdaptationDirection(PerformanceMetrics* metrics, const char* axis) {
  //DESCRIPTION: Calculate adaptation direction based on multiple performance indicators
  
  float adaptation_signal = 0.0;
  
  //Error-based adaptation
  if (metrics->mean_abs_error > 3.0) {
    adaptation_signal += 0.3; //Increase P gain for high error
  } else if (metrics->mean_abs_error < 0.5) {
    adaptation_signal -= 0.1; //Reduce P gain for very low error
  }
  
  //Overshoot-based adaptation
  if (metrics->overshoot > 15.0) {
    adaptation_signal -= 0.4; //Reduce P gain for excessive overshoot
  } else if (metrics->overshoot < 2.0 && metrics->rise_time > 0.5) {
    adaptation_signal += 0.2; //Increase P gain for sluggish response
  }
  
  //Oscillation-based adaptation
  if (metrics->oscillation_count > 20) {
    adaptation_signal -= 0.5; //Strong reduction for oscillations
  } else if (metrics->oscillation_count < 5 && metrics->damping_ratio < 0.5) {
    adaptation_signal += 0.1; //Slight increase for underdamped
  }
  
  //Control effort consideration
  if (metrics->control_effort > 0.8) {
    adaptation_signal -= 0.2; //Reduce gain for excessive control effort
  }
  
  //Bound the adaptation signal
  return constrain(adaptation_signal, -0.5, 0.5);
}

float adaptIntegralGain(float current_Ki, float steady_state_error, float learning_rate) {
  //DESCRIPTION: Adaptive integral gain tuning based on steady-state error
  
  float Ki_adaptation = 0.0;
  
  //Increase Ki for persistent bias
  if (abs(steady_state_error) > 1.0) {
    Ki_adaptation = 0.5 * learning_rate;
  }
  //Decrease Ki if no steady-state error (avoid integrator windup)
  else if (abs(steady_state_error) < 0.1) {
    Ki_adaptation = -0.2 * learning_rate;
  }
  
  return constrain(current_Ki + Ki_adaptation, 0.01, 0.8);
}

float adaptDerivativeGain(float current_Kd, float oscillation_freq, float damping_ratio, float learning_rate) {
  //DESCRIPTION: Adaptive derivative gain tuning based on oscillation characteristics
  
  float Kd_adaptation = 0.0;
  
  //Increase Kd for underdamped systems
  if (damping_ratio < 0.4 && oscillation_freq > 2.0) {
    Kd_adaptation = 0.3 * learning_rate;
  }
  //Decrease Kd for overdamped or high-frequency noise
  else if (damping_ratio > 1.2 || oscillation_freq > 15.0) {
    Kd_adaptation = -0.4 * learning_rate;
  }
  
  return constrain(current_Kd + Kd_adaptation, 0.0001, 0.02);
}

void restorePreviousGains() {
  //DESCRIPTION: Restore backup gains after performance degradation
  
  //Load backup gains from safety monitor
  if (safetyMonitor.backup_gains.Kp_roll > 0) {
    Kp_roll_angle = safetyMonitor.backup_gains.Kp_roll;
    Ki_roll_angle = safetyMonitor.backup_gains.Ki_roll;
    Kd_roll_angle = safetyMonitor.backup_gains.Kd_roll;
    Kp_pitch_angle = safetyMonitor.backup_gains.Kp_pitch;
    Ki_pitch_angle = safetyMonitor.backup_gains.Ki_pitch;
    Kd_pitch_angle = safetyMonitor.backup_gains.Kd_pitch;
    Kp_yaw = safetyMonitor.backup_gains.Kp_yaw;
    Ki_yaw = safetyMonitor.backup_gains.Ki_yaw;
    Kd_yaw = safetyMonitor.backup_gains.Kd_yaw;
  }
}

void monitorPerformance() {
  //DESCRIPTION: Main performance monitoring function - replaces old version
  
  //Update performance buffers with current data
  updatePerformanceBuffers();
  
  //Compute comprehensive metrics for each axis
  computeAdvancedMetrics(&rollBuffer, &rollMetrics);
  computeAdvancedMetrics(&pitchBuffer, &pitchMetrics);
  computeAdvancedMetrics(&yawBuffer, &yawMetrics);
  
  //Detect current flight conditions and disturbances
  detectFlightConditions();
  
  //Update safety monitor trend
  safetyMonitor.performance_trend[safetyMonitor.trend_index] = 
    (rollMetrics.health_score + pitchMetrics.health_score + yawMetrics.health_score) / 3.0;
  safetyMonitor.trend_index = (safetyMonitor.trend_index + 1) % 10;
  
  //Check for emergency conditions
  bool emergency_error = rollMetrics.peak_error > 45.0 || pitchMetrics.peak_error > 45.0;
  bool emergency_oscillation = rollMetrics.oscillation_freq > 20.0 || pitchMetrics.oscillation_freq > 20.0;
  bool emergency_saturation = rollMetrics.control_effort > 0.95 || pitchMetrics.control_effort > 0.95;
  
  if (emergency_error || emergency_oscillation || emergency_saturation) {
    safetyMonitor.consecutive_bad_samples++;
    if (safetyMonitor.consecutive_bad_samples > 5) {
      safetyMonitor.emergency_freeze = true;
      learningState = LEARNING_FROZEN;
    }
  } else {
    safetyMonitor.consecutive_bad_samples = max(0, safetyMonitor.consecutive_bad_samples - 1);
  }
}

void adaptivePIDLearning() {
  //DESCRIPTION: Main adaptive learning function - replaces old version
  
  //Manage learning state machine
  manageAdaptiveLearning();
  
  //Perform performance-based adaptation
  performanceBasedAdaptation();
  
  //Store backup gains periodically
  static unsigned long backup_timer = 0;
  if (current_time - backup_timer > 5000000) { //Every 5 seconds
    safetyMonitor.backup_gains.Kp_roll = Kp_roll_angle;
    safetyMonitor.backup_gains.Ki_roll = Ki_roll_angle;
    safetyMonitor.backup_gains.Kd_roll = Kd_roll_angle;
    safetyMonitor.backup_gains.Kp_pitch = Kp_pitch_angle;
    safetyMonitor.backup_gains.Ki_pitch = Ki_pitch_angle;
    safetyMonitor.backup_gains.Kd_pitch = Kd_pitch_angle;
    safetyMonitor.backup_gains.Kp_yaw = Kp_yaw;
    safetyMonitor.backup_gains.Ki_yaw = Ki_yaw;
    safetyMonitor.backup_gains.Kd_yaw = Kd_yaw;
    backup_timer = current_time;
  }
}

void printAirspeedEstimation() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Airspeed Est:"));
    Serial.print(airspeedEst.estimated_airspeed);
    Serial.print(F(" m/s, Thrust:"));
    Serial.print(airspeedEst.current_thrust);
    Serial.print(F(" N, Confidence:"));
    Serial.print(airspeedEst.estimation_confidence);
    Serial.print(F(" Valid:"));
    Serial.println(airspeedEst.estimation_valid);
  }
}

void printDragCoefficients() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Drag Zones - Zone:"));
    Serial.print(airspeedEst.current_aoa_zone);
    Serial.print(F(" k:"));
    Serial.print(dragZones[airspeedEst.current_aoa_zone].k_coefficient);
    Serial.print(F(" Samples:"));
    Serial.print(dragZones[airspeedEst.current_aoa_zone].samples);
    Serial.print(F(" Converged:"));
    Serial.println(dragZones[airspeedEst.current_aoa_zone].converged);
  }
}

void printThrottleDither() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Throttle - Base:"));
    Serial.print(airspeedEst.base_throttle);
    Serial.print(F(" Dither:"));
    Serial.print(airspeedEst.dither_signal);
    Serial.print(F(" Final:"));
    Serial.print(airspeedEst.dithered_throttle);
    Serial.print(F(" Enabled:"));
    Serial.println(airspeedEst.dither_enabled);
  }
}

void printEKFStates() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("EKF - V:"));
    Serial.print(airspeedFilter.state[0]);
    Serial.print(F(" k:"));
    Serial.print(airspeedFilter.state[1]);
    Serial.print(F(" Innovation:"));
    Serial.print(airspeedFilter.innovation);
    Serial.print(F(" Converged:"));
    Serial.println(airspeedFilter.converged);
  }
}

void printWindAnalysis() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Wind - Gust:"));
    Serial.print(windAnalyzer.instant_gust_index);
    Serial.print(F(" Steady:"));
    Serial.print(windAnalyzer.steady_wind_index);
    Serial.print(F(" Condition:"));
    Serial.print(windAnalyzer.current_condition);
    Serial.print(F(" Frozen:"));
    Serial.println(windAnalyzer.learning_frozen);
  }
}

void printWindCompensation() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Wind FF - X:"));
    Serial.print(windAnalyzer.Kff_wind_x);
    Serial.print(F(" Y:"));
    Serial.print(windAnalyzer.Kff_wind_y);
    Serial.print(F(" Z:"));
    Serial.print(windAnalyzer.Kff_wind_z);
    Serial.print(F(" Conf:"));
    Serial.println(windAnalyzer.Kff_confidence);
  }
}

void printAdaptationRates() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Adapt Rate - Current:"));
    Serial.print(windAnalyzer.current_adaptation_rate);
    Serial.print(F(" Global:"));
    Serial.print(adaptation_gain);
    Serial.print(F(" Emergency:"));
    Serial.println(windControl.emergency_wind_mode);
  }
}

void printWindModel() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Wind Model - Mag:"));
    Serial.print(windAnalyzer.wind_magnitude);
    Serial.print(F(" Dir:"));
    Serial.print(windAnalyzer.wind_direction * 180.0 / 3.14159);
    Serial.print(F(" Persist:"));
    Serial.print(windModel.wind_persistence);
    Serial.print(F(" Converged:"));
    Serial.println(windAnalyzer.wind_model_converged);
  }
}

void printFixedWingControlSurfaces() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("Surfaces - LElevon:"));
    Serial.print(s1_command_scaled);
    Serial.print(F(" RElevon:"));
    Serial.print(s2_command_scaled);
    Serial.print(F(" Rudder:"));
    Serial.print(s3_command_scaled);
    Serial.print(F(" Throttle:"));
    Serial.println(m1_command_scaled);
  }
}

void printFixedWingWindBias() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("FW Wind Bias - X:"));
    Serial.print(windAnalyzer.wind_bias_x);
    Serial.print(F(" Y:"));
    Serial.print(windAnalyzer.wind_bias_y);
    Serial.print(F(" Z:"));
    Serial.print(windAnalyzer.wind_bias_z);
    Serial.print(F(" Airspeed:"));
    Serial.println(airspeedEst.airspeed_estimate);
  }
}

void printFixedWingWindEffects() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("FW Effects - Bank:"));
    Serial.print(RollAng);
    Serial.print(F(" Pitch:"));
    Serial.print(PitchAng);
    Serial.print(F(" WindMag:"));
    Serial.print(windAnalyzer.wind_magnitude);
    Serial.print(F(" EmergMode:"));
    Serial.println(windControl.emergency_wind_mode);
  }
}

// ------------------------------------------------------------------
// DEBUG: 4D Gain Schedule Interpolation Visualization
// ------------------------------------------------------------------
void printGainScheduleDebug() {
  static unsigned long lastGainDebug = 0;
  if (current_time - lastGainDebug < 200000) return;  //5 Hz max (200ms = 200,000 us)
  lastGainDebug = current_time;
  
  Serial.println(F("========== 4D GAIN SCHEDULE DEBUG =========="));
  
  // 1) Print current flight regime and indices
  Serial.print(F("Flight Regime - Throttle:"));
  Serial.print(thro_des, 3);
  Serial.print(F(" AOA:"));
  Serial.print(angle_of_attack, 3);
  Serial.print(F(" Phase:"));
  Serial.print(flightState.current_phase);
  Serial.print(F(" Airspeed:"));
  Serial.println(airspeedEst.airspeed_estimate, 3);
  
  // 2) Print base indices & interpolation factors
  Serial.print(F("Indices(t,a,p,s): "));
  Serial.print(flightState.throttle_idx);  Serial.print(',');
  Serial.print(flightState.aoa_idx);       Serial.print(',');
  Serial.print(flightState.phase_idx);     Serial.print(',');
  Serial.print(flightState.airspeed_idx);  Serial.print("   ");

  Serial.print(F("Factors(t,a,p,s): "));
  Serial.print(flightState.throttle_factor, 3); Serial.print(',');
  Serial.print(flightState.aoa_factor, 3);      Serial.print(',');
  Serial.print(flightState.phase_factor, 3);    Serial.print(',');
  Serial.println(flightState.airspeed_factor, 3);

  // 3) Walk through all 16 corner cells and show interpolation
  float totalKpR=0, totalKiR=0, totalKdR=0,
        totalKpP=0, totalKiP=0, totalKdP=0,
        totalKpY=0, totalKiY=0, totalKdY=0;
        
  Serial.println(F("16-Corner Interpolation Weights:"));
  for (int dt = 0; dt <= 1; dt++) {
    for (int da = 0; da <= 1; da++) {
      for (int dp = 0; dp <= 1; dp++) {
        for (int ds = 0; ds <= 1; ds++) {
          int ti = min(flightState.throttle_idx + dt, NUM_THROTTLE_ZONES-1);
          int ai = min(flightState.aoa_idx      + da, NUM_AOA_ZONES-1);
          int pi = min(flightState.phase_idx    + dp, NUM_PHASE_ZONES-1);
          int si = min(flightState.airspeed_idx + ds, NUM_AIRSPEED_ZONES-1);

          float wt = dt ? flightState.throttle_factor    : (1 - flightState.throttle_factor);
          float wa = da ? flightState.aoa_factor         : (1 - flightState.aoa_factor);
          float wp = dp ? flightState.phase_factor       : (1 - flightState.phase_factor);
          float ws = ds ? flightState.airspeed_factor    : (1 - flightState.airspeed_factor);
          float w  = wt * wa * wp * ws;

          PIDGains &g = gainSchedule[ti][ai][pi][si];
          
          if (w > 0.001) {  //Only show significant weights
            Serial.print(F("  Cell["));
            Serial.print(ti); Serial.print(',');
            Serial.print(ai); Serial.print(',');
            Serial.print(pi); Serial.print(',');
            Serial.print(si); Serial.print("] w=");
            Serial.print(w, 3);
            Serial.print(F(" Roll("));
            Serial.print(g.Kp_roll,3); Serial.print(',');
            Serial.print(g.Ki_roll,3); Serial.print(',');
            Serial.print(g.Kd_roll,3); Serial.print(F(") Confidence:"));
            Serial.println(g.confidence, 3);
          }

          // Accumulate weighted sums
          totalKpR += w * g.Kp_roll;  totalKiR += w * g.Ki_roll;  totalKdR += w * g.Kd_roll;
          totalKpP += w * g.Kp_pitch; totalKiP += w * g.Ki_pitch; totalKdP += w * g.Kd_pitch;
          totalKpY += w * g.Kp_yaw;   totalKiY += w * g.Ki_yaw;   totalKdY += w * g.Kd_yaw;
        }
      }
    }
  }

  // 4) Print final interpolated gains vs actual
  Serial.println(F("FINAL INTERPOLATED GAINS:"));
  Serial.print(F("→ Roll  PID: P="));
  Serial.print(totalKpR,4); Serial.print(F(" I="));
  Serial.print(totalKiR,4); Serial.print(F(" D="));
  Serial.println(totalKdR,4);

  Serial.print(F("→ Pitch PID: P="));
  Serial.print(totalKpP,4); Serial.print(F(" I="));
  Serial.print(totalKiP,4); Serial.print(F(" D="));
  Serial.println(totalKdP,4);

  Serial.print(F("→ Yaw   PID: P="));
  Serial.print(totalKpY,4); Serial.print(F(" I="));
  Serial.print(totalKiY,4); Serial.print(F(" D="));
  Serial.println(totalKdY,4);
  
  // 5) Compare with actual PID gains being used
  Serial.println(F("ACTUAL GAINS IN USE:"));
  Serial.print(F("→ Roll  PID: P="));
  Serial.print(Kp_roll_cmd,4); Serial.print(F(" I="));
  Serial.print(Ki_roll_cmd,4); Serial.print(F(" D="));
  Serial.println(Kd_roll_cmd,4);

  Serial.print(F("→ Pitch PID: P="));
  Serial.print(Kp_pitch_cmd,4); Serial.print(F(" I="));
  Serial.print(Ki_pitch_cmd,4); Serial.print(F(" D="));
  Serial.println(Kd_pitch_cmd,4);

  Serial.print(F("→ Yaw   PID: P="));
  Serial.print(Kp_yaw_cmd,4); Serial.print(F(" I="));
  Serial.print(Ki_yaw_cmd,4); Serial.print(F(" D="));
  Serial.println(Kd_yaw_cmd,4);
  
  Serial.println(F("=========================================="));
}

void printFlightRegimeVisualization() {
  static unsigned long lastRegimeDebug = 0;
  if (current_time - lastRegimeDebug < 500000) return;  //2 Hz (500ms = 500,000 us)
  lastRegimeDebug = current_time;
  
  Serial.println(F("======= FLIGHT REGIME VISUALIZATION ======="));
  
  // Flight phase detection
  Serial.print(F("Phase: "));
  switch(flightState.current_phase) {
    case 0: Serial.print(F("HOVER")); break;
    case 1: Serial.print(F("TRANSITION")); break;
    case 2: Serial.print(F("CRUISE")); break;
    case 3: Serial.print(F("LANDING")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F(" ("));
  Serial.print(flightState.current_phase);
  Serial.println(F(")"));
  
  // Create visual bar charts
  Serial.println(F("Throttle |0%      25%      50%      75%     100%|"));
  Serial.print(F("         |"));
  int throttle_bar = (int)(thro_des * 40);  //40 chars wide
  for (int i = 0; i < 40; i++) {
    if (i < throttle_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F("| "));
  Serial.print(thro_des * 100, 1);
  Serial.println(F("%"));
  
  Serial.println(F("AOA      |-30°    -15°      0°     +15°    +30°|"));
  Serial.print(F("         |"));
  int aoa_bar = (int)((angle_of_attack + 30) * 40 / 60);  //Map -30 to +30 → 0 to 40
  aoa_bar = constrain(aoa_bar, 0, 40);
  for (int i = 0; i < 40; i++) {
    if (i == 20) Serial.print(F("|"));  //Center line at 0°
    else if (i == aoa_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F("| "));
  Serial.print(angle_of_attack, 1);
  Serial.println(F("°"));
  
  Serial.println(F("Airspeed |0      10      20      30      40m/s|"));
  Serial.print(F("         |"));
  int airspeed_bar = (int)(airspeedEst.airspeed_estimate * 40 / 40);  //40 m/s max
  airspeed_bar = constrain(airspeed_bar, 0, 40);
  for (int i = 0; i < 40; i++) {
    if (i < airspeed_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F("| "));
  Serial.print(airspeedEst.airspeed_estimate, 1);
  Serial.println(F("m/s"));
  
  Serial.println(F("=========================================="));
}

void printPerformanceMetricsVisualization() {
  static unsigned long lastPerfDebug = 0;
  if (current_time - lastPerfDebug < 1000000) return;  //1 Hz (1000ms = 1,000,000 us)
  lastPerfDebug = current_time;
  
  Serial.println(F("====== PERFORMANCE METRICS VISUAL ======"));
  
  // Performance health scores
  Serial.print(F("Roll Health:  "));
  int roll_health_bar = (int)(rollMetrics.health_score * 20 / 100);  //20 chars, 0-100 scale
  for (int i = 0; i < 20; i++) {
    if (i < roll_health_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.print(rollMetrics.health_score, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Pitch Health: "));
  int pitch_health_bar = (int)(pitchMetrics.health_score * 20 / 100);
  for (int i = 0; i < 20; i++) {
    if (i < pitch_health_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.print(pitchMetrics.health_score, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Yaw Health:   "));
  int yaw_health_bar = (int)(yawMetrics.health_score * 20 / 100);
  for (int i = 0; i < 20; i++) {
    if (i < yaw_health_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.print(yawMetrics.health_score, 1);
  Serial.println(F("%"));
  
  // Learning state
  Serial.print(F("Learning State: "));
  switch(learningState) {
    case LEARNING_OBSERVING:  Serial.println(F("OBSERVING")); break;
    case LEARNING_ADAPTING:   Serial.println(F("ADAPTING")); break;
    case LEARNING_FROZEN:     Serial.println(F("FROZEN")); break;
    case LEARNING_ROLLBACK:   Serial.println(F("ROLLBACK")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
  
  // Adaptation gain visualization
  Serial.print(F("Adapt Rate:   "));
  int adapt_bar = (int)(adaptation_gain * 200);  //Scale up for visibility
  adapt_bar = constrain(adapt_bar, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < adapt_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.println(adaptation_gain, 6);
  
  Serial.println(F("=========================================="));
}

void printWindVisualization() {
  static unsigned long lastWindDebug = 0;
  if (current_time - lastWindDebug < 1000000) return;  //1 Hz
  lastWindDebug = current_time;
  
  Serial.println(F("========= WIND ANALYSIS VISUAL ========="));
  
  // Wind condition state
  Serial.print(F("Wind Condition: "));
  switch(windAnalyzer.current_condition) {
    case 0: Serial.println(F("CALM")); break;
    case 1: Serial.println(F("STEADY_WIND")); break;
    case 2: Serial.println(F("GUSTY")); break;
    case 3: Serial.println(F("SEVERE_GUST")); break;
    default: Serial.println(F("UNKNOWN")); break;
  }
  
  // Gust vs wind index visualization
  Serial.print(F("Gust Index:   "));
  int gust_bar = (int)(windAnalyzer.instant_gust_index * 20);  //20 chars
  gust_bar = constrain(gust_bar, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < gust_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.println(windAnalyzer.instant_gust_index, 3);
  
  Serial.print(F("Wind Index:   "));
  int wind_bar = (int)(windAnalyzer.steady_wind_index * 20);
  wind_bar = constrain(wind_bar, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i < wind_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.println(windAnalyzer.steady_wind_index, 3);
  
  // Wind compensation gains
  Serial.print(F("Wind FF X:    "));
  int ff_x_bar = (int)((windAnalyzer.Kff_wind_x + 0.4) * 25);  //Center around 0, ±0.4 range
  ff_x_bar = constrain(ff_x_bar, 0, 20);
  for (int i = 0; i < 20; i++) {
    if (i == 10) Serial.print(F("|"));  //Center line
    else if (i == ff_x_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.println(windAnalyzer.Kff_wind_x, 4);
  
  // Learning status
  Serial.print(F("Learning: "));
  if (windAnalyzer.learning_frozen) {
    Serial.print(F("FROZEN"));
  } else {
    Serial.print(F("ACTIVE"));
  }
  Serial.print(F("  Confidence: "));
  Serial.print(windAnalyzer.Kff_confidence, 3);
  Serial.print(F("  Converged: "));
  Serial.println(windAnalyzer.wind_model_converged ? F("YES") : F("NO"));
  
  Serial.println(F("=========================================="));
}

void printComprehensiveDebugOutput() {
  static unsigned long lastComprehensiveDebug = 0;
  if (current_time - lastComprehensiveDebug < 2000000) return;  //0.5 Hz (2 seconds)
  lastComprehensiveDebug = current_time;
  
  Serial.println(F(""));
  Serial.println(F("##################################################"));
  Serial.println(F("###    COMPREHENSIVE ADAPTIVE SYSTEM DEBUG    ###"));
  Serial.println(F("##################################################"));
  Serial.println(F(""));
  
  // System status overview
  Serial.println(F("======== SYSTEM STATUS OVERVIEW ========"));
  Serial.print(F("Loop Time: "));
  Serial.print(dt*1000000, 0);
  Serial.print(F("us  Armed: "));
  Serial.print(armed ? F("YES") : F("NO"));
  Serial.print(F("  Throttle: "));
  Serial.print(thro_des * 100, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Attitude - Roll:"));
  Serial.print(RollAng, 1);
  Serial.print(F("° Pitch:"));
  Serial.print(PitchAng, 1);
  Serial.print(F("° Yaw:"));
  Serial.print(YawAng, 1);
  Serial.println(F("°"));
  
  Serial.print(F("Errors   - Roll:"));
  Serial.print(error_roll, 3);
  Serial.print(F("° Pitch:"));
  Serial.print(error_pitch, 3);
  Serial.print(F("° Yaw:"));
  Serial.print(error_yaw, 3);
  Serial.println(F("°"));
  Serial.println(F(""));
  
  // Flight regime visualization (compact)
  Serial.println(F("======== FLIGHT REGIME ========"));
  Serial.print(F("Phase: "));
  switch(flightState.current_phase) {
    case 0: Serial.print(F("HOVER")); break;
    case 1: Serial.print(F("TRANSITION")); break;
    case 2: Serial.print(F("CRUISE")); break;
    case 3: Serial.print(F("LANDING")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  AOA: "));
  Serial.print(angle_of_attack, 1);
  Serial.print(F("°  Airspeed: "));
  Serial.print(airspeedEst.airspeed_estimate, 1);
  Serial.println(F("m/s"));
  
  Serial.print(F("Grid Position: ["));
  Serial.print(flightState.throttle_idx); Serial.print(',');
  Serial.print(flightState.aoa_idx); Serial.print(',');
  Serial.print(flightState.phase_idx); Serial.print(',');
  Serial.print(flightState.airspeed_idx); Serial.print(F("] Factors: ["));
  Serial.print(flightState.throttle_factor, 2); Serial.print(',');
  Serial.print(flightState.aoa_factor, 2); Serial.print(',');
  Serial.print(flightState.phase_factor, 2); Serial.print(',');
  Serial.print(flightState.airspeed_factor, 2); Serial.println(F("]"));
  Serial.println(F(""));
  
  // Current PID gains
  Serial.println(F("======== ACTIVE PID GAINS ========"));
  Serial.print(F("Roll:  P="));
  Serial.print(Kp_roll_cmd, 4); Serial.print(F(" I="));
  Serial.print(Ki_roll_cmd, 4); Serial.print(F(" D="));
  Serial.println(Kd_roll_cmd, 4);
  
  Serial.print(F("Pitch: P="));
  Serial.print(Kp_pitch_cmd, 4); Serial.print(F(" I="));
  Serial.print(Ki_pitch_cmd, 4); Serial.print(F(" D="));
  Serial.println(Kd_pitch_cmd, 4);
  
  Serial.print(F("Yaw:   P="));
  Serial.print(Kp_yaw_cmd, 4); Serial.print(F(" I="));
  Serial.print(Ki_yaw_cmd, 4); Serial.print(F(" D="));
  Serial.println(Kd_yaw_cmd, 4);
  Serial.println(F(""));
  
  // Performance metrics
  Serial.println(F("======== PERFORMANCE HEALTH ========"));
  Serial.print(F("Roll Health: "));
  int roll_bar = (int)(rollMetrics.health_score / 5);  //20 chars, scale /5
  for (int i = 0; i < 20; i++) {
    if (i < roll_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.print(rollMetrics.health_score, 1);
  Serial.println(F("%"));
  
  Serial.print(F("Pitch Health:"));
  int pitch_bar = (int)(pitchMetrics.health_score / 5);
  for (int i = 0; i < 20; i++) {
    if (i < pitch_bar) Serial.print(F("█"));
    else Serial.print(F("·"));
  }
  Serial.print(F(" "));
  Serial.print(pitchMetrics.health_score, 1);
  Serial.println(F("%"));
  Serial.println(F(""));
  
  // Learning system status
  Serial.println(F("======== ADAPTIVE LEARNING ========"));
  Serial.print(F("Learning State: "));
  switch(learningState) {
    case LEARNING_OBSERVING:  Serial.print(F("OBSERVING")); break;
    case LEARNING_ADAPTING:   Serial.print(F("ADAPTING")); break;
    case LEARNING_FROZEN:     Serial.print(F("FROZEN")); break;
    case LEARNING_ROLLBACK:   Serial.print(F("ROLLBACK")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  Adaptation Rate: "));
  Serial.print(adaptation_gain, 6);
  
  Serial.print(F("  Wind Frozen: "));
  Serial.println(windAnalyzer.learning_frozen ? F("YES") : F("NO"));
  Serial.println(F(""));
  
  // Wind analysis summary
  Serial.println(F("======== WIND ANALYSIS ========"));
  Serial.print(F("Condition: "));
  switch(windAnalyzer.current_condition) {
    case 0: Serial.print(F("CALM")); break;
    case 1: Serial.print(F("STEADY_WIND")); break;
    case 2: Serial.print(F("GUSTY")); break;
    case 3: Serial.print(F("SEVERE_GUST")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  Gust Index: "));
  Serial.print(windAnalyzer.instant_gust_index, 3);
  Serial.print(F("  Wind Index: "));
  Serial.println(windAnalyzer.steady_wind_index, 3);
  
  Serial.print(F("Wind Bias: X="));
  Serial.print(windAnalyzer.wind_bias_x, 3); Serial.print(F(" Y="));
  Serial.print(windAnalyzer.wind_bias_y, 3); Serial.print(F(" Z="));
  Serial.print(windAnalyzer.wind_bias_z, 3);
  Serial.print(F("  Magnitude: "));
  Serial.println(windAnalyzer.wind_magnitude, 3);
  
  Serial.print(F("Wind FF: X="));
  Serial.print(windAnalyzer.Kff_wind_x, 4); Serial.print(F(" Y="));
  Serial.print(windAnalyzer.Kff_wind_y, 4); Serial.print(F(" Z="));
  Serial.print(windAnalyzer.Kff_wind_z, 4);
  Serial.print(F("  Confidence: "));
  Serial.println(windAnalyzer.Kff_confidence, 3);
  Serial.println(F(""));
  
  // Control surface outputs (fixed-wing)
  Serial.println(F("======== CONTROL OUTPUTS ========"));
  Serial.print(F("Left Elevon: "));
  Serial.print(s1_command_scaled, 3); Serial.print(F("  Right Elevon: "));
  Serial.print(s2_command_scaled, 3); Serial.print(F("  Rudder: "));
  Serial.println(s3_command_scaled, 3);
  
  Serial.print(F("Throttle: "));
  Serial.print(m1_command_scaled, 3); Serial.print(F("  PID Outputs: R="));
  Serial.print(roll_PID, 3); Serial.print(F(" P="));
  Serial.print(pitch_PID, 3); Serial.print(F(" Y="));
  Serial.println(yaw_PID, 3);
  Serial.println(F(""));
  
  // Airspeed estimation
  Serial.println(F("======== AIRSPEED ESTIMATION ========"));
  Serial.print(F("EKF Airspeed: "));
  Serial.print(airspeedEst.airspeed_estimate, 2); Serial.print(F("m/s  Confidence: "));
  Serial.print(airspeedEst.estimate_confidence, 3);
  Serial.print(F("  Converged: "));
  Serial.println(airspeedFilter.converged ? F("YES") : F("NO"));
  
  Serial.print(F("Thrust: "));
  Serial.print(airspeedEst.estimated_thrust, 1); Serial.print(F("N  Dither: "));
  Serial.print(airspeedEst.dither_enabled ? F("ON") : F("OFF"));
  Serial.print(F("  Dithered Throttle: "));
  Serial.println(airspeedEst.dithered_throttle, 3);
  Serial.println(F(""));
  
  Serial.println(F("##################################################"));
  Serial.println(F("###          END COMPREHENSIVE DEBUG          ###"));
  Serial.println(F("##################################################"));
  Serial.println(F(""));
}

void printGettingStartedDebug() {
  static unsigned long lastSimpleDebug = 0;
  if (current_time - lastSimpleDebug < 500000) return;  //2 Hz (500ms)
  lastSimpleDebug = current_time;
  
  Serial.println(F(""));
  Serial.println(F("=============== FLIGHT STATUS ==============="));
  
  // Basic flight status
  Serial.print(F("Armed: "));
  Serial.print(armed ? F("YES") : F("NO"));
  Serial.print(F("  |  Throttle: "));
  Serial.print(thro_des * 100, 0);
  Serial.print(F("%  |  Loop: "));
  Serial.print(dt * 1000000, 0);
  Serial.println(F("us"));
  
  // Aircraft attitude
  Serial.print(F("Attitude: R="));
  Serial.print(RollAng, 1);
  Serial.print(F("° P="));
  Serial.print(PitchAng, 1);
  Serial.print(F("° Y="));
  Serial.print(YawAng, 1);
  Serial.print(F("°  |  AOA: "));
  Serial.print(angle_of_attack, 1);
  Serial.println(F("°"));
  
  // Control surfaces (fixed-wing)
  Serial.print(F("Controls: LE="));
  Serial.print((s1_command_scaled - 0.5) * 100, 0);  //Convert to ±50%
  Serial.print(F("% RE="));
  Serial.print((s2_command_scaled - 0.5) * 100, 0);
  Serial.print(F("% RUD="));
  Serial.print((s3_command_scaled - 0.5) * 100, 0);
  Serial.print(F("%  |  Airspeed: "));
  Serial.print(airspeedEst.airspeed_estimate, 1);
  Serial.println(F("m/s"));
  
  // Current gains being used
  Serial.print(F("PID Gains: Kp_R="));
  Serial.print(Kp_roll_cmd, 3);
  Serial.print(F(" Kp_P="));
  Serial.print(Kp_pitch_cmd, 3);
  Serial.print(F(" Kp_Y="));
  Serial.println(Kp_yaw_cmd, 3);
  
  // Flight regime and learning status
  Serial.print(F("Flight Regime: "));
  switch(flightState.current_phase) {
    case 0: Serial.print(F("HOVER")); break;
    case 1: Serial.print(F("TRANSITION")); break;
    case 2: Serial.print(F("CRUISE")); break;
    case 3: Serial.print(F("LANDING")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  |  Learning: "));
  switch(learningState) {
    case LEARNING_OBSERVING:  Serial.print(F("WATCHING")); break;
    case LEARNING_ADAPTING:   Serial.print(F("LEARNING")); break;
    case LEARNING_FROZEN:     Serial.print(F("FROZEN")); break;
    case LEARNING_ROLLBACK:   Serial.print(F("ROLLBACK")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  Rate: "));
  Serial.println(adaptation_gain, 5);
  
  // Wind analysis summary
  Serial.print(F("Wind: "));
  switch(windAnalyzer.current_condition) {
    case 0: Serial.print(F("CALM")); break;
    case 1: Serial.print(F("STEADY")); break;
    case 2: Serial.print(F("GUSTY")); break;
    case 3: Serial.print(F("SEVERE")); break;
    default: Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F("  |  Gust: "));
  Serial.print(windAnalyzer.instant_gust_index, 2);
  Serial.print(F("  Wind: "));
  Serial.print(windAnalyzer.steady_wind_index, 2);
  Serial.print(F("  |  FF Confidence: "));
  Serial.println(windAnalyzer.Kff_confidence, 2);
  
  // Performance health (simple)
  Serial.print(F("Health: Roll="));
  Serial.print(rollMetrics.health_score, 0);
  Serial.print(F("% Pitch="));
  Serial.print(pitchMetrics.health_score, 0);
  Serial.print(F("% Yaw="));
  Serial.print(yawMetrics.health_score, 0);
  Serial.print(F("%  |  EKF Converged: "));
  Serial.println(airspeedFilter.converged ? F("YES") : F("NO"));
  
  Serial.println(F("============================================="));
  Serial.println(F(""));
}

void printDebugHelp() {
  Serial.println(F(""));
  Serial.println(F("########################################"));
  Serial.println(F("###     ADAPTIVE FLIGHT CONTROLLER   ###"));
  Serial.println(F("###     DEBUG FUNCTION GUIDE         ###"));
  Serial.println(F("########################################"));
  Serial.println(F(""));
  Serial.println(F("QUICK START DEBUG FUNCTIONS:"));
  Serial.println(F(""));
  Serial.println(F("1. printGettingStartedDebug()"));
  Serial.println(F("   → Clean overview of flight status"));
  Serial.println(F("   → Shows: attitude, controls, gains, learning"));
  Serial.println(F("   → Perfect for first flights & basic debugging"));
  Serial.println(F("   → Rate: 2 Hz"));
  Serial.println(F(""));
  Serial.println(F("2. printComprehensiveDebugOutput()"));
  Serial.println(F("   → Complete system analysis"));
  Serial.println(F("   → Shows: everything in organized sections"));
  Serial.println(F("   → Use for detailed troubleshooting"));
  Serial.println(F("   → Rate: 0.5 Hz"));
  Serial.println(F(""));
  Serial.println(F("SPECIALIZED DEBUG FUNCTIONS:"));
  Serial.println(F(""));
  Serial.println(F("3. printGainScheduleDebug()"));
  Serial.println(F("   → Shows 4D gain interpolation in detail"));
  Serial.println(F("   → All 16 corner weights and final gains"));
  Serial.println(F("   → Compare interpolated vs actual"));
  Serial.println(F("   → Rate: 5 Hz"));
  Serial.println(F(""));
  Serial.println(F("4. printFlightRegimeVisualization()"));
  Serial.println(F("   → Visual bar charts of throttle, AOA, airspeed"));
  Serial.println(F("   → Shows flight phase detection"));
  Serial.println(F("   → Rate: 2 Hz"));
  Serial.println(F(""));
  Serial.println(F("5. printPerformanceMetricsVisualization()"));
  Serial.println(F("   → Performance health bars (roll/pitch/yaw)"));
  Serial.println(F("   → Learning state and adaptation rate"));
  Serial.println(F("   → Rate: 1 Hz"));
  Serial.println(F(""));
  Serial.println(F("6. printWindVisualization()"));
  Serial.println(F("   → Wind vs gust separation visualization"));
  Serial.println(F("   → Feedforward compensation gains"));
  Serial.println(F("   → Learning status and confidence"));
  Serial.println(F("   → Rate: 1 Hz"));
  Serial.println(F(""));
  Serial.println(F("FIXED-WING SPECIFIC:"));
  Serial.println(F(""));
  Serial.println(F("7. printFixedWingControlSurfaces()"));
  Serial.println(F("   → Elevon and rudder positions"));
  Serial.println(F("   → Throttle commands"));
  Serial.println(F(""));
  Serial.println(F("8. printFixedWingWindBias()"));
  Serial.println(F("   → Wind bias estimates in body frame"));
  Serial.println(F("   → Airspeed dependency"));
  Serial.println(F(""));
  Serial.println(F("9. printFixedWingWindEffects()"));
  Serial.println(F("   → Bank angle, pitch effects from wind"));
  Serial.println(F("   → Emergency mode status"));
  Serial.println(F(""));
  Serial.println(F("PHYSICS-BASED AIRSPEED:"));
  Serial.println(F(""));
  Serial.println(F("10. printAirspeedEstimation()"));
  Serial.println(F("    → EKF airspeed estimate with confidence"));
  Serial.println(F("    → Thrust estimation and calibration"));
  Serial.println(F(""));
  Serial.println(F("11. printEKFStates()"));
  Serial.println(F("    → Extended Kalman Filter states"));
  Serial.println(F("    → Innovation and convergence"));
  Serial.println(F(""));
  Serial.println(F("12. printDragCoefficients()"));
  Serial.println(F("    → Zone-based drag coefficient learning"));
  Serial.println(F("    → RLS adaptation status"));
  Serial.println(F(""));
  Serial.println(F("13. printThrottleDither()"));
  Serial.println(F("    → Throttle dithering for observability"));
  Serial.println(F("    → Sinusoidal signal status"));
  Serial.println(F(""));
  Serial.println(F("USAGE INSTRUCTIONS:"));
  Serial.println(F(""));
  Serial.println(F("1. Uncomment ONE function at a time in loop()"));
  Serial.println(F("2. Upload code and open Serial Monitor"));
  Serial.println(F("3. Set baud rate to 500000"));
  Serial.println(F("4. Arm aircraft and observe output"));
  Serial.println(F("5. Each function has its own update rate"));
  Serial.println(F(""));
  Serial.println(F("RECOMMENDED DEBUG SEQUENCE:"));
  Serial.println(F(""));
  Serial.println(F("1st Flight: printGettingStartedDebug()"));
  Serial.println(F("   → Verify basic operation"));
  Serial.println(F(""));
  Serial.println(F("Tuning: printGainScheduleDebug()"));
  Serial.println(F("   → Watch gain interpolation"));
  Serial.println(F(""));
  Serial.println(F("Wind Testing: printWindVisualization()"));
  Serial.println(F("   → Verify wind compensation"));
  Serial.println(F(""));
  Serial.println(F("Full Analysis: printComprehensiveDebugOutput()"));
  Serial.println(F("   → Complete system understanding"));
  Serial.println(F(""));
  Serial.println(F("########################################"));
  Serial.println(F(""));
}

//========================================================================================================================//
//                                      INTELLIGENT WIND DISTURBANCE REJECTION                                           //                           
//========================================================================================================================//

//Fixed-wing wind and gust detection parameters
#define WIND_FILTER_ALPHA 0.005       //Even slower low-pass for fixed-wing (more inertial)
#define GUST_THRESHOLD_HIGH 0.5        //Fixed-wing less sensitive to gusts due to inertia
#define GUST_THRESHOLD_MED 0.2         //Lower threshold for feedforward learning
#define WIND_THRESHOLD_MIN 0.03        //Lower minimum for fixed-wing wind adaptation
#define WIND_THRESHOLD_STEADY 0.08     //Fixed-wing shows wind effects more gradually
#define MAX_FEEDFORWARD_BIAS 0.4       //Higher max compensation for aerodynamic control

//Wind disturbance analysis
struct WindDisturbanceAnalyzer {
  //Turbulence indices
  float instant_gust_index;          //Immediate turbulence measurement (spiky)
  float steady_wind_index;           //Slow-moving average (steady trends)
  float prev_wind_index;             //Previous wind index for filtering
  
  //Wind direction and magnitude estimation
  float wind_bias_x;                 //Estimated wind bias in body X (forward)
  float wind_bias_y;                 //Estimated wind bias in body Y (right)
  float wind_bias_z;                 //Estimated wind bias in body Z (down)
  float wind_magnitude;              //Estimated wind magnitude
  float wind_direction;              //Estimated wind direction (radians)
  
  //Adaptation rate management
  float base_adaptation_rate;        //Base learning rate in calm conditions
  float current_adaptation_rate;     //Current learning rate based on wind conditions
  bool learning_frozen;              //True when learning is frozen due to gusts
  unsigned long freeze_start_time;   //When learning was frozen
  float freeze_duration_threshold;   //Minimum freeze duration (seconds)
  
  //Wind condition classification
  enum WindCondition {
    CALM = 0,           //Calm conditions - full learning
    STEADY_WIND = 1,    //Steady wind - reduced learning
    GUSTY = 2,          //Gusty conditions - minimal learning
    SEVERE_GUST = 3     //Severe gust - frozen learning
  } current_condition;
  
  //Historical wind data for trend analysis
  float gust_history[20];            //Recent gust index history
  float wind_history[20];            //Recent wind index history
  int history_index;                 //Current history buffer index
  float gust_variance;               //Variance in gust measurements
  float wind_trend;                  //Trend in wind measurements
  
  //Feedforward wind compensation
  float Kff_wind_x;                  //Feedforward gain for X-axis wind
  float Kff_wind_y;                  //Feedforward gain for Y-axis wind
  float Kff_wind_z;                  //Feedforward gain for Z-axis wind (vertical wind)
  float Kff_confidence;              //Confidence in feedforward estimates
  
  //Performance tracking
  float thrust_model_prediction;     //What motor model predicts we need
  float thrust_actual_command;       //What we actually commanded
  float wind_error_x;                //Wind disturbance error in X
  float wind_error_y;                //Wind disturbance error in Y
  float wind_error_z;                //Wind disturbance error in Z
  float wind_error_magnitude;        //Total wind error magnitude
  
  //Learning statistics
  unsigned long wind_learning_samples;  //Number of wind learning updates
  float wind_learning_quality;          //Quality metric for wind learning
  bool wind_model_converged;             //Has wind model converged?
  float wind_prediction_accuracy;       //How well we predict wind effects
};

WindDisturbanceAnalyzer windAnalyzer;

//Wind-aware adaptation control
struct WindAdaptiveControl {
  //Multi-rate adaptation system
  float adaptation_rate_calm;        //Full rate in calm air
  float adaptation_rate_wind;        //Reduced rate in steady wind  
  float adaptation_rate_gust;        //Minimal rate in gusty conditions
  float adaptation_rate_freeze;      //Zero rate during severe gusts
  
  //Adaptation strategy parameters
  float wind_learning_momentum;      //Momentum for wind feedforward learning
  float gust_rejection_factor;       //How much to suppress learning during gusts
  float wind_compensation_factor;    //How much to boost wind compensation
  
  //State management
  bool wind_compensation_enabled;    //Enable/disable wind compensation
  bool gust_rejection_enabled;       //Enable/disable gust rejection
  bool adaptive_rates_enabled;       //Enable/disable adaptive learning rates
  
  //Performance metrics
  float wind_rejection_performance;  //How well we're rejecting wind
  float gust_rejection_performance;  //How well we're rejecting gusts
  float adaptation_efficiency;       //Overall adaptation efficiency
  
  //Safety limits
  float max_wind_compensation;       //Maximum wind compensation allowed
  float max_adaptation_change;       //Maximum adaptation rate change per step
  bool emergency_wind_mode;          //Emergency mode for extreme conditions
};

WindAdaptiveControl windControl;

//Wind disturbance estimation and modeling
struct WindDisturbanceModel {
  //Physical wind model parameters
  float estimated_wind_velocity[3];  //Estimated wind velocity vector [x,y,z]
  float wind_acceleration[3];        //Rate of change of wind
  float wind_persistence;            //How persistent the wind is (0-1)
  float wind_predictability;         //How predictable wind changes are (0-1)
  
  //Disturbance frequency analysis
  float low_freq_content;            //Low frequency disturbance content (wind)
  float high_freq_content;           //High frequency disturbance content (gusts)
  float disturbance_spectrum_peak;   //Peak frequency of disturbances
  
  //Spatial wind characteristics
  bool horizontal_wind_dominant;     //Is horizontal wind the main disturbance?
  bool vertical_wind_present;        //Is there significant vertical wind?
  bool wind_shear_detected;          //Wind shear detection
  float wind_coherence_time;         //How long wind patterns persist
  
  //Gust characteristics
  float gust_intensity;              //Current gust intensity
  float gust_frequency;              //Frequency of gust occurrences
  float gust_duration_avg;           //Average gust duration
  bool gust_burst_detected;          //Burst of multiple gusts detected
  
  //Environmental adaptation
  float altitude_wind_factor;        //Wind scaling with altitude
  float temperature_wind_factor;     //Wind scaling with temperature
  float terrain_wind_factor;         //Terrain influence on wind
};

WindDisturbanceModel windModel;

//========================================================================================================================//
//                                 INTELLIGENT WIND DISTURBANCE REJECTION FUNCTIONS                                      //                           
//========================================================================================================================//

void initializeWindDisturbanceRejection() {
  //DESCRIPTION: Initialize the wind disturbance rejection system
  
  //Initialize wind analyzer
  windAnalyzer.instant_gust_index = 0.0;
  windAnalyzer.steady_wind_index = 0.0;
  windAnalyzer.prev_wind_index = 0.0;
  
  //Initialize wind bias estimates
  windAnalyzer.wind_bias_x = 0.0;
  windAnalyzer.wind_bias_y = 0.0;
  windAnalyzer.wind_bias_z = 0.0;
  windAnalyzer.wind_magnitude = 0.0;
  windAnalyzer.wind_direction = 0.0;
  
  //Initialize adaptation control
  windAnalyzer.base_adaptation_rate = 0.0005;
  windAnalyzer.current_adaptation_rate = 0.0005;
  windAnalyzer.learning_frozen = false;
  windAnalyzer.freeze_start_time = 0;
  windAnalyzer.freeze_duration_threshold = 2.0;  //2 seconds minimum freeze
  windAnalyzer.current_condition = windAnalyzer.CALM;
  
  //Initialize history buffers
  for (int i = 0; i < 20; i++) {
    windAnalyzer.gust_history[i] = 0.0;
    windAnalyzer.wind_history[i] = 0.0;
  }
  windAnalyzer.history_index = 0;
  windAnalyzer.gust_variance = 0.0;
  windAnalyzer.wind_trend = 0.0;
  
  //Initialize feedforward compensation
  windAnalyzer.Kff_wind_x = 0.0;
  windAnalyzer.Kff_wind_y = 0.0;
  windAnalyzer.Kff_wind_z = 0.0;
  windAnalyzer.Kff_confidence = 0.0;
  
  //Initialize performance tracking
  windAnalyzer.thrust_model_prediction = 0.0;
  windAnalyzer.thrust_actual_command = 0.0;
  windAnalyzer.wind_error_x = 0.0;
  windAnalyzer.wind_error_y = 0.0;
  windAnalyzer.wind_error_z = 0.0;
  windAnalyzer.wind_error_magnitude = 0.0;
  
  //Initialize learning statistics
  windAnalyzer.wind_learning_samples = 0;
  windAnalyzer.wind_learning_quality = 0.0;
  windAnalyzer.wind_model_converged = false;
  windAnalyzer.wind_prediction_accuracy = 0.0;
  
  //Initialize fixed-wing wind adaptive control (slower rates due to higher inertia)
  windControl.adaptation_rate_calm = 0.0003;    //Slower base rate for fixed-wing
  windControl.adaptation_rate_wind = 0.0001;    //33% of calm rate  
  windControl.adaptation_rate_gust = 0.00003;   //10% of calm rate
  windControl.adaptation_rate_freeze = 0.0;     //No learning during severe gusts
  
  windControl.wind_learning_momentum = 0.9;
  windControl.gust_rejection_factor = 0.8;
  windControl.wind_compensation_factor = 1.2;
  
  windControl.wind_compensation_enabled = true;
  windControl.gust_rejection_enabled = true;
  windControl.adaptive_rates_enabled = true;
  
  windControl.wind_rejection_performance = 0.0;
  windControl.gust_rejection_performance = 0.0;
  windControl.adaptation_efficiency = 0.0;
  
  windControl.max_wind_compensation = MAX_FEEDFORWARD_BIAS;
  windControl.max_adaptation_change = 0.001;
  windControl.emergency_wind_mode = false;
  
  //Initialize wind disturbance model
  for (int i = 0; i < 3; i++) {
    windModel.estimated_wind_velocity[i] = 0.0;
    windModel.wind_acceleration[i] = 0.0;
  }
  windModel.wind_persistence = 0.5;
  windModel.wind_predictability = 0.5;
  
  windModel.low_freq_content = 0.0;
  windModel.high_freq_content = 0.0;
  windModel.disturbance_spectrum_peak = 0.0;
  
  windModel.horizontal_wind_dominant = false;
  windModel.vertical_wind_present = false;
  windModel.wind_shear_detected = false;
  windModel.wind_coherence_time = 10.0;  //10 seconds default
  
  windModel.gust_intensity = 0.0;
  windModel.gust_frequency = 0.0;
  windModel.gust_duration_avg = 0.0;
  windModel.gust_burst_detected = false;
  
  windModel.altitude_wind_factor = 1.0;
  windModel.temperature_wind_factor = 1.0;
  windModel.terrain_wind_factor = 1.0;
}

void analyzeWindAndGustConditions() {
  //DESCRIPTION: Analyze turbulence to separate gusts from steady wind
  
  //Get instant gust index from existing turbulence detection
  windAnalyzer.instant_gust_index = flightDetector.turbulence_index;
  
  //Apply very slow low-pass filter to extract steady wind component
  windAnalyzer.steady_wind_index = (1.0 - WIND_FILTER_ALPHA) * windAnalyzer.prev_wind_index + 
                                   WIND_FILTER_ALPHA * windAnalyzer.instant_gust_index;
  windAnalyzer.prev_wind_index = windAnalyzer.steady_wind_index;
  
  //Update history buffers for trend analysis
  windAnalyzer.gust_history[windAnalyzer.history_index] = windAnalyzer.instant_gust_index;
  windAnalyzer.wind_history[windAnalyzer.history_index] = windAnalyzer.steady_wind_index;
  windAnalyzer.history_index = (windAnalyzer.history_index + 1) % 20;
  
  //Calculate gust variance (how spiky the turbulence is)
  float gust_mean = 0.0;
  for (int i = 0; i < 20; i++) {
    gust_mean += windAnalyzer.gust_history[i];
  }
  gust_mean /= 20.0;
  
  windAnalyzer.gust_variance = 0.0;
  for (int i = 0; i < 20; i++) {
    float diff = windAnalyzer.gust_history[i] - gust_mean;
    windAnalyzer.gust_variance += diff * diff;
  }
  windAnalyzer.gust_variance /= 20.0;
  
  //Calculate wind trend (is wind increasing or decreasing?)
  float wind_start = 0.0, wind_end = 0.0;
  for (int i = 0; i < 10; i++) {
    wind_start += windAnalyzer.wind_history[i];
    wind_end += windAnalyzer.wind_history[i + 10];
  }
  wind_start /= 10.0;
  wind_end /= 10.0;
  windAnalyzer.wind_trend = (wind_end - wind_start) / 10.0;  //Change per sample
  
  //Classify current wind condition
  if (windAnalyzer.instant_gust_index > GUST_THRESHOLD_HIGH) {
    windAnalyzer.current_condition = windAnalyzer.SEVERE_GUST;
  } else if (windAnalyzer.instant_gust_index > GUST_THRESHOLD_MED && windAnalyzer.gust_variance > 0.1) {
    windAnalyzer.current_condition = windAnalyzer.GUSTY;
  } else if (windAnalyzer.steady_wind_index > WIND_THRESHOLD_STEADY) {
    windAnalyzer.current_condition = windAnalyzer.STEADY_WIND;
  } else {
    windAnalyzer.current_condition = windAnalyzer.CALM;
  }
  
  //Update wind model characteristics
  windModel.gust_intensity = windAnalyzer.instant_gust_index;
  windModel.low_freq_content = windAnalyzer.steady_wind_index;
  windModel.high_freq_content = max(0.0, windAnalyzer.instant_gust_index - windAnalyzer.steady_wind_index);
  
  //Detect gust bursts (multiple consecutive high gusts)
  static int consecutive_gusts = 0;
  if (windAnalyzer.instant_gust_index > GUST_THRESHOLD_MED) {
    consecutive_gusts++;
  } else {
    consecutive_gusts = 0;
  }
  windModel.gust_burst_detected = (consecutive_gusts > 5);
  
  //Update wind persistence (how steady the wind component is)
  windModel.wind_persistence = constrain(1.0 - windAnalyzer.gust_variance * 2.0, 0.0, 1.0);
  
  //Analyze wind direction and magnitude from control surface deflections  
  updateFixedWingWindEstimate();
}

void updateFixedWingWindEstimate() {
  //DESCRIPTION: Estimate wind direction and magnitude for fixed-wing aircraft
  
  //Fixed-wing aircraft have different wind interaction than multirotor
  //Wind affects them through aerodynamic forces on wings and control surfaces
  
  //Get control surface deflections (normalized -1 to +1)
  float left_elevon_deflection = (s1_command_scaled - 0.5) * 2.0;
  float right_elevon_deflection = (s2_command_scaled - 0.5) * 2.0;
  float rudder_deflection = (s3_command_scaled - 0.5) * 2.0;
  
  //Extract elevator and aileron components from elevons
  float elevator_deflection = (left_elevon_deflection + right_elevon_deflection) / 2.0;
  float aileron_deflection = (left_elevon_deflection - right_elevon_deflection) / 2.0;
  
  //Low-pass filter control deflections to extract wind-induced bias
  static float wind_elevator_lp = 0.0, wind_aileron_lp = 0.0, wind_rudder_lp = 0.0;
  float wind_filter_rate = 0.0005;  //Even slower for fixed-wing (more inertial)
  
  wind_elevator_lp = (1.0 - wind_filter_rate) * wind_elevator_lp + wind_filter_rate * elevator_deflection;
  wind_aileron_lp = (1.0 - wind_filter_rate) * wind_aileron_lp + wind_filter_rate * aileron_deflection;
  wind_rudder_lp = (1.0 - wind_filter_rate) * wind_rudder_lp + wind_filter_rate * rudder_deflection;
  
  //Convert control surface bias to estimated wind forces
  //Account for airspeed dependency of control surface effectiveness
  float airspeed_factor = constrain(airspeedEst.airspeed_estimate / 15.0, 0.5, 2.0);
  
  //Elevator bias suggests pitch moment from head/tail wind or vertical wind
  windAnalyzer.wind_bias_x = wind_elevator_lp * 0.3 / airspeed_factor;  //Head/tail wind
  windAnalyzer.wind_bias_z = -wind_elevator_lp * 0.2 / airspeed_factor; //Vertical wind (updraft/downdraft)
  
  //Aileron bias suggests roll moment from crosswind
  windAnalyzer.wind_bias_y = wind_aileron_lp * 0.4 / airspeed_factor;   //Crosswind
  
  //Rudder bias confirms crosswind and adds yaw component
  float rudder_crosswind = wind_rudder_lp * 0.3 / airspeed_factor;
  windAnalyzer.wind_bias_y = (windAnalyzer.wind_bias_y + rudder_crosswind) / 2.0;  //Average aileron and rudder estimates
  
  //Fixed-wing specific wind analysis
  //Bank angle can indicate sustained crosswind compensation
  static float bank_angle_lp = 0.0;
  bank_angle_lp = (1.0 - wind_filter_rate) * bank_angle_lp + wind_filter_rate * RollAng;
  
  if (abs(bank_angle_lp) > 3.0) {  //Sustained bank angle > 3 degrees
    //This suggests crosswind compensation
    float crosswind_from_bank = sin(bank_angle_lp * 3.14159 / 180.0) * 0.5;
    windAnalyzer.wind_bias_y = (windAnalyzer.wind_bias_y + crosswind_from_bank) / 2.0;
  }
  
  //Throttle position analysis for headwind/tailwind
  static float throttle_lp = 0.0;
  throttle_lp = (1.0 - wind_filter_rate) * throttle_lp + wind_filter_rate * thro_des;
  
  //If throttle is consistently high/low, it suggests head/tail wind
  float expected_cruise_throttle = 0.5;  //Estimate based on aircraft
  float throttle_bias = throttle_lp - expected_cruise_throttle;
  if (abs(throttle_bias) > 0.1) {
    windAnalyzer.wind_bias_x = (windAnalyzer.wind_bias_x + throttle_bias * 0.5) / 2.0;
  }
  
  //Estimate total wind magnitude
  windAnalyzer.wind_magnitude = sqrt(windAnalyzer.wind_bias_x * windAnalyzer.wind_bias_x + 
                                     windAnalyzer.wind_bias_y * windAnalyzer.wind_bias_y);
  
  //Estimate wind direction (angle from body X-axis)
  if (windAnalyzer.wind_magnitude > 0.01) {
    windAnalyzer.wind_direction = atan2(windAnalyzer.wind_bias_y, windAnalyzer.wind_bias_x);
  }
  
  //Update wind model with current estimates
  windModel.estimated_wind_velocity[0] = windAnalyzer.wind_bias_x;
  windModel.estimated_wind_velocity[1] = windAnalyzer.wind_bias_y;
  windModel.estimated_wind_velocity[2] = windAnalyzer.wind_bias_z;
  
  //Fixed-wing wind characteristics
  windModel.horizontal_wind_dominant = (windAnalyzer.wind_magnitude > abs(windAnalyzer.wind_bias_z));
  windModel.vertical_wind_present = (abs(windAnalyzer.wind_bias_z) > 0.08);  //Higher threshold for fixed-wing
  
  //Wing loading effects - higher wind sensitivity at low speed
  float wing_loading_factor = constrain(15.0 / (airspeedEst.airspeed_estimate + 5.0), 0.5, 3.0);
  windAnalyzer.wind_magnitude *= wing_loading_factor;
}

void determineAdaptationRate() {
  //DESCRIPTION: Set adaptation rate based on wind conditions
  
  if (!windControl.adaptive_rates_enabled) {
    windAnalyzer.current_adaptation_rate = windAnalyzer.base_adaptation_rate;
    return;
  }
  
  float target_rate = windAnalyzer.base_adaptation_rate;
  
  //Determine target adaptation rate based on wind condition
  switch (windAnalyzer.current_condition) {
    case windAnalyzer.CALM:
      target_rate = windControl.adaptation_rate_calm;
      windAnalyzer.learning_frozen = false;
      break;
      
    case windAnalyzer.STEADY_WIND:
      target_rate = windControl.adaptation_rate_wind;
      windAnalyzer.learning_frozen = false;
      break;
      
    case windAnalyzer.GUSTY:
      target_rate = windControl.adaptation_rate_gust;
      windAnalyzer.learning_frozen = false;
      break;
      
    case windAnalyzer.SEVERE_GUST:
      target_rate = windControl.adaptation_rate_freeze;
      if (!windAnalyzer.learning_frozen) {
        windAnalyzer.learning_frozen = true;
        windAnalyzer.freeze_start_time = current_time;
      }
      break;
  }
  
  //Check if we should unfreeze learning
  if (windAnalyzer.learning_frozen && windAnalyzer.current_condition != windAnalyzer.SEVERE_GUST) {
    float freeze_duration = (current_time - windAnalyzer.freeze_start_time) / 1000000.0;
    if (freeze_duration > windAnalyzer.freeze_duration_threshold) {
      windAnalyzer.learning_frozen = false;
    }
  }
  
  //Override with freeze if still frozen
  if (windAnalyzer.learning_frozen) {
    target_rate = 0.0;
  }
  
  //Smoothly transition to target rate to avoid sudden changes
  float rate_change_limit = windControl.max_adaptation_change;
  float rate_diff = target_rate - windAnalyzer.current_adaptation_rate;
  rate_diff = constrain(rate_diff, -rate_change_limit, rate_change_limit);
  
  windAnalyzer.current_adaptation_rate += rate_diff;
  windAnalyzer.current_adaptation_rate = constrain(windAnalyzer.current_adaptation_rate, 0.0, 
                                                   windControl.adaptation_rate_calm);
  
  //Update global adaptation gain for the adaptive PID system
  adaptation_gain = windAnalyzer.current_adaptation_rate;
}

void updateWindFeedforwardCompensation() {
  //DESCRIPTION: Learn feedforward wind compensation gains
  
  if (!windControl.wind_compensation_enabled) return;
  
  //Only learn feedforward during steady wind conditions (not during gusts)
  bool suitable_for_learning = (windAnalyzer.steady_wind_index > WIND_THRESHOLD_MIN) && 
                               (windAnalyzer.instant_gust_index < GUST_THRESHOLD_MED) &&
                               !windAnalyzer.learning_frozen;
  
  if (!suitable_for_learning) return;
  
  //Calculate wind disturbance errors for fixed-wing aircraft
  //These represent the extra control effort needed to maintain attitude in wind
  
  //Fixed-wing uses control surface deflections as wind error indicators
  float left_elevon_deflection = (s1_command_scaled - 0.5) * 2.0;
  float right_elevon_deflection = (s2_command_scaled - 0.5) * 2.0;
  float rudder_deflection = (s3_command_scaled - 0.5) * 2.0;
  
  //Extract elevator and aileron components
  float elevator_deflection = (left_elevon_deflection + right_elevon_deflection) / 2.0;
  float aileron_deflection = (left_elevon_deflection - right_elevon_deflection) / 2.0;
  
  //Wind errors from control surface usage
  windAnalyzer.wind_error_x = elevator_deflection;  //Elevator deflection indicates pitch disturbance
  windAnalyzer.wind_error_y = aileron_deflection;   //Aileron deflection indicates roll disturbance
  windAnalyzer.wind_error_z = rudder_deflection;    //Rudder deflection indicates yaw disturbance
  
  //Throttle analysis for head/tail wind
  windAnalyzer.thrust_model_prediction = 0.5;  //Expected cruise throttle
  windAnalyzer.thrust_actual_command = m1_command_scaled;  //Actual throttle command
  
  windAnalyzer.wind_error_magnitude = sqrt(windAnalyzer.wind_error_x * windAnalyzer.wind_error_x + 
                                           windAnalyzer.wind_error_y * windAnalyzer.wind_error_y + 
                                           windAnalyzer.wind_error_z * windAnalyzer.wind_error_z);
  
  //Learn feedforward gains to cancel these errors
  float learning_rate = windAnalyzer.current_adaptation_rate * windControl.wind_compensation_factor;
  
  //Apply momentum for stability
  static float Kff_momentum_x = 0.0, Kff_momentum_y = 0.0, Kff_momentum_z = 0.0;
  
  Kff_momentum_x = windControl.wind_learning_momentum * Kff_momentum_x + 
                   (1.0 - windControl.wind_learning_momentum) * windAnalyzer.wind_error_x;
  Kff_momentum_y = windControl.wind_learning_momentum * Kff_momentum_y + 
                   (1.0 - windControl.wind_learning_momentum) * windAnalyzer.wind_error_y;
  Kff_momentum_z = windControl.wind_learning_momentum * Kff_momentum_z + 
                   (1.0 - windControl.wind_learning_momentum) * windAnalyzer.wind_error_z;
  
  //Update feedforward gains
  windAnalyzer.Kff_wind_x += learning_rate * Kff_momentum_x;
  windAnalyzer.Kff_wind_y += learning_rate * Kff_momentum_y;
  windAnalyzer.Kff_wind_z += learning_rate * Kff_momentum_z;
  
  //Apply safety limits
  windAnalyzer.Kff_wind_x = constrain(windAnalyzer.Kff_wind_x, -windControl.max_wind_compensation, 
                                      windControl.max_wind_compensation);
  windAnalyzer.Kff_wind_y = constrain(windAnalyzer.Kff_wind_y, -windControl.max_wind_compensation, 
                                      windControl.max_wind_compensation);
  windAnalyzer.Kff_wind_z = constrain(windAnalyzer.Kff_wind_z, -windControl.max_wind_compensation, 
                                      windControl.max_wind_compensation);
  
  //Update confidence based on consistency of wind estimates
  windAnalyzer.wind_learning_samples++;
  
  //Calculate prediction accuracy
  static float prev_wind_error_mag = 0.0;
  float error_reduction = max(0.0, prev_wind_error_mag - windAnalyzer.wind_error_magnitude);
  windAnalyzer.wind_prediction_accuracy = 0.95 * windAnalyzer.wind_prediction_accuracy + 
                                          0.05 * (error_reduction / (prev_wind_error_mag + 0.001));
  prev_wind_error_mag = windAnalyzer.wind_error_magnitude;
  
  //Update confidence
  windAnalyzer.Kff_confidence = constrain(windAnalyzer.wind_prediction_accuracy, 0.0, 1.0);
  
  //Check for convergence
  if (windAnalyzer.wind_learning_samples > 1000 && windAnalyzer.Kff_confidence > 0.8) {
    windAnalyzer.wind_model_converged = true;
  }
}

void applyFixedWingWindCompensation(float* control_commands) {
  //DESCRIPTION: Apply learned wind compensation to fixed-wing control surfaces
  
  if (!windControl.wind_compensation_enabled || !windAnalyzer.wind_model_converged) return;
  
  //Only apply compensation when we're confident in our wind model
  if (windAnalyzer.Kff_confidence < 0.3) return;
  
  //Scale wind compensation by current airspeed (control surfaces less effective at low speed)
  float airspeed_factor = constrain(airspeedEst.airspeed_estimate / 15.0, 0.3, 1.5);  //Normalize to ~15 m/s cruise
  
  //Calculate wind compensation for fixed-wing aerodynamics
  float wind_comp_elevator = windAnalyzer.Kff_wind_x * windAnalyzer.Kff_confidence * airspeed_factor;
  float wind_comp_aileron = windAnalyzer.Kff_wind_y * windAnalyzer.Kff_confidence * airspeed_factor;
  float wind_comp_rudder = windAnalyzer.Kff_wind_y * 0.3 * windAnalyzer.Kff_confidence * airspeed_factor; //Rudder for crosswind
  float wind_comp_throttle = windAnalyzer.Kff_wind_z * windAnalyzer.Kff_confidence;
  
  //Apply elevon mixing with wind compensation
  //Left elevon gets both elevator and aileron compensation
  control_commands[0] += wind_comp_elevator + wind_comp_aileron;  //Left elevon
  //Right elevon gets elevator compensation but opposite aileron
  control_commands[1] += wind_comp_elevator - wind_comp_aileron;  //Right elevon
  //Rudder compensation for crosswind coordination
  control_commands[2] += wind_comp_rudder;                       //Rudder
  //Throttle compensation for headwind/tailwind
  control_commands[3] += wind_comp_throttle;                     //Throttle
  
  //Additional fixed-wing specific compensation
  //Banking into crosswind for coordinated flight
  if (abs(windAnalyzer.wind_bias_y) > 0.1) {  //Significant crosswind
    float crosswind_bank = windAnalyzer.wind_bias_y * 0.2 * windAnalyzer.Kff_confidence;
    control_commands[0] += crosswind_bank;   //Bank left elevon into wind
    control_commands[1] -= crosswind_bank;   //Bank right elevon into wind
  }
  
  //Pitch compensation for vertical wind (updrafts/downdrafts)
  if (abs(windAnalyzer.wind_bias_z) > 0.1) {
    float vertical_wind_pitch = -windAnalyzer.wind_bias_z * 0.15 * windAnalyzer.Kff_confidence;
    control_commands[0] += vertical_wind_pitch;  //Pitch compensation both elevons
    control_commands[1] += vertical_wind_pitch;
  }
  
  //Update performance metrics
  windControl.wind_rejection_performance = constrain(1.0 - windAnalyzer.wind_error_magnitude * 2.0, 0.0, 1.0);
  windControl.gust_rejection_performance = constrain(1.0 - windAnalyzer.instant_gust_index, 0.0, 1.0);
  windControl.adaptation_efficiency = (windControl.wind_rejection_performance + 
                                       windControl.gust_rejection_performance) / 2.0;
}

void updateWindDisturbanceRejection() {
  //DESCRIPTION: Main wind disturbance rejection function - called every loop
  
  //Analyze current wind and gust conditions
  analyzeWindAndGustConditions();
  
  //Determine appropriate adaptation rate based on conditions
  determineAdaptationRate();
  
  //Update feedforward wind compensation learning
  updateWindFeedforwardCompensation();
  
  //Emergency wind mode detection (adjusted for fixed-wing characteristics)
  if (windAnalyzer.instant_gust_index > 1.0 || windAnalyzer.wind_magnitude > 1.5) {
    windControl.emergency_wind_mode = true;
    //In emergency mode, freeze all learning and use conservative control
    adaptation_gain = 0.0;
    windAnalyzer.learning_frozen = true;
  } else if (windAnalyzer.instant_gust_index < 0.2 && windAnalyzer.wind_magnitude < 0.4) {
    windControl.emergency_wind_mode = false;
  }
}

//========================================================================================================================//
//                                           EEPROM STORAGE FUNCTIONS                                                    //                           
//========================================================================================================================//

void loadGainScheduleFromEEPROM() {
  //DESCRIPTION: Load previously learned gain schedule from EEPROM
  
  #if ENABLE_EEPROM_STORAGE
    //TODO: Implement EEPROM loading when ready for persistent storage
    //For now, use default gains
    Serial.println(F("Loading gains from EEPROM..."));
    //Read gain schedule array from EEPROM
    //Validate checksum
    //Apply loaded gains
  #else
    //Use default initialized gains
    Serial.println(F("Using default gain schedule (EEPROM disabled)"));
  #endif
}

void saveGainScheduleToEEPROM() {
  //DESCRIPTION: Save current gain schedule to EEPROM for persistent storage
  
  #if ENABLE_EEPROM_STORAGE
    //TODO: Implement EEPROM saving when ready for persistent storage
    Serial.println(F("Saving gains to EEPROM..."));
    //Write gain schedule array to EEPROM
    //Write checksum
    //Verify write success
  #else
    //EEPROM storage disabled - gains are temporary
  #endif
}

void scaleCommands() {
  //DESCRIPTION: Scale normalized actuator commands to values for ESC/Servo protocol
  /*
   * mX_command_scaled variables from the mixer function are scaled to 125-250us for OneShot125 protocol. sX_command_scaled variables from
   * the mixer function are scaled to 0-180 for the servo library using standard PWM.
   * mX_command_PWM are updated here which are used to command the motors in commandMotors(). sX_command_PWM are updated 
   * which are used to command the servos.
   */
  //Scaled to 125us - 250us for oneshot125 protocol
  m1_command_PWM = m1_command_scaled*125 + 125;
  m2_command_PWM = m2_command_scaled*125 + 125;
  m3_command_PWM = m3_command_scaled*125 + 125;
  m4_command_PWM = m4_command_scaled*125 + 125;
  m5_command_PWM = m5_command_scaled*125 + 125;
  m6_command_PWM = m6_command_scaled*125 + 125;
  //Constrain commands to motors within oneshot125 bounds
  m1_command_PWM = constrain(m1_command_PWM, 125, 250);
  m2_command_PWM = constrain(m2_command_PWM, 125, 250);
  m3_command_PWM = constrain(m3_command_PWM, 125, 250);
  m4_command_PWM = constrain(m4_command_PWM, 125, 250);
  m5_command_PWM = constrain(m5_command_PWM, 125, 250);
  m6_command_PWM = constrain(m6_command_PWM, 125, 250);

  //Scaled to 0-180 for servo library
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

//========================================================================================================================//
//                                        BASIC FLIGHT CONTROLLER STUBS                                                 //                           
//========================================================================================================================//

//These are basic stub implementations for functions the debug code expects
//Replace these with your actual flight controller implementations

void armedStatus() {
  //DESCRIPTION: Check if aircraft is armed
  //This is a stub - implement your actual arming logic
  armed = true;  //Always armed for testing - change this
}

void getIMUdata() {
  //DESCRIPTION: Get IMU data and update attitude
  //This is a stub - implement your actual IMU reading
  static float time_step = 0;
  time_step += dt;
  
  //Simulate some attitude movement for testing
  RollAng = 5.0 * sin(time_step * 0.1);
  PitchAng = 3.0 * cos(time_step * 0.15);
  YawAng += 0.1 * dt;  //Slow yaw drift
  
  //Calculate angle of attack from pitch (simplified)
  angle_of_attack = PitchAng;
}

void getDesState() {
  //DESCRIPTION: Convert radio inputs to desired states
  //This is a stub - implement your actual radio input processing
  thro_des = 0.5;  //50% throttle for testing
}

void controlANGLE() {
  //DESCRIPTION: Angle control PID loops
  //This is a stub - implement your actual PID controllers
  
  //Calculate errors (desired - actual)
  error_roll = 0.0 - RollAng;    //Desired roll = 0
  error_pitch = 0.0 - PitchAng;  //Desired pitch = 0  
  error_yaw = 0.0 - YawAng;      //Desired yaw = 0
  
  //Simple P controller for testing
  roll_PID = Kp_roll_cmd * error_roll;
  pitch_PID = Kp_pitch_cmd * error_pitch;
  yaw_PID = Kp_yaw_cmd * error_yaw;
  
  //Constrain outputs
  roll_PID = constrain(roll_PID, -0.5, 0.5);
  pitch_PID = constrain(pitch_PID, -0.5, 0.5);
  yaw_PID = constrain(yaw_PID, -0.5, 0.5);
}

void controlMixer() {
  //DESCRIPTION: Mix PID outputs to actuator commands
  //This is already implemented in the adaptive section
}

void throttleCut() {
  //DESCRIPTION: Implement throttle cut safety
  //This is a stub - implement your actual throttle cut logic
  if (thro_cut) {
    m1_command_scaled = 0;
    m2_command_scaled = 0; 
    m3_command_scaled = 0;
    m4_command_scaled = 0;
  }
}

void commandMotors() {
  //DESCRIPTION: Send PWM commands to motors
  //This is a stub - implement your actual motor command code
}

void updateTime() {
  //DESCRIPTION: Update timing variables
  static unsigned long prev_time = 0;
  current_time = micros();
  dt = (current_time - prev_time) / 1000000.0;
  prev_time = current_time;
  
  //Constrain dt to reasonable values
  dt = constrain(dt, 0.0005, 0.02);  //0.5ms to 20ms
}
