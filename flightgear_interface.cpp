// Platform-specific includes
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <windows.h>
    #pragma comment(lib, "ws2_32.lib")
    // Undefine Windows macros that conflict with std::max/std::min
    #ifdef max
    #undef max
    #endif
    #ifdef min
    #undef min
    #endif
    #define sleep(x) Sleep((x)*1000)
    #define usleep(x) Sleep((x)/1000)
    typedef HANDLE pthread_t;
    typedef DWORD pthread_attr_t;
    #define pthread_create(thread, attr, func, arg) ((*thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)func, arg, 0, NULL)) == NULL)
    #define pthread_join(thread, retval) (WaitForSingleObject(thread, INFINITE), CloseHandle(thread))
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <pthread.h>
    #include <signal.h>
#endif

#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>

// SDL2 is optional - only include if available
#ifdef HAVE_SDL2
#include <SDL.h>
#endif

#include "src/core/FlightController.h"
#include "src/core/FlightGearHAL.h"

// Global control
volatile bool running = true;
volatile float rc_aileron = 0.0f;
volatile float rc_elevator = 0.0f;  
volatile float rc_rudder = 0.0f;
volatile float rc_throttle = 0.0f;

// UDP configuration
#define FG_INPUT_PORT 5000   // Port to receive FlightGear state
#define FG_OUTPUT_PORT 5010  // Port to send controls to FlightGear
#define UDP_BUFFER_SIZE 1024

// FlightGear UDP sockets
int sock_in = -1;   // Receive FlightGear state
int sock_out = -1;  // Send controls to FlightGear
struct sockaddr_in fg_addr_in, fg_addr_out;

// Flight controller components
FlightGearHAL* hal = nullptr;
FlightController* controller = nullptr;

// Function prototypes
bool initUDP();
void cleanup();
void signalHandler(int sig);
#ifdef _WIN32
DWORD WINAPI rcReaderThread(LPVOID arg);
#else
void* rcReaderThread(void* arg);
#endif
bool parseFlightGearData(const char* data, FlightGearHAL::FlightGearData* fg_data);
void sendControlsToFlightGear(const FlightGearHAL::ControlData& controls);

// Signal handler for clean shutdown
#ifdef _WIN32
BOOL WINAPI signalHandler(DWORD sig) {
    printf("\nShutdown signal received (%lu)\n", sig);
    running = false;
    return TRUE;
}
#else
void signalHandler(int sig) {
    printf("\nShutdown signal received (%d)\n", sig);
    running = false;
}
#endif

// Initialize UDP sockets for FlightGear communication
bool initUDP() {
    #ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup failed\n");
        return false;
    }
    #endif
    
    // Create input socket (receive FlightGear state)
    sock_in = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_in < 0) {
        perror("Failed to create input socket");
        return false;
    }
    
    // Configure input socket
    memset(&fg_addr_in, 0, sizeof(fg_addr_in));
    fg_addr_in.sin_family = AF_INET;
    fg_addr_in.sin_addr.s_addr = INADDR_ANY;
    fg_addr_in.sin_port = htons(FG_INPUT_PORT);
    
    // Bind input socket
    if (bind(sock_in, (struct sockaddr*)&fg_addr_in, sizeof(fg_addr_in)) < 0) {
        perror("Failed to bind input socket");
        #ifdef _WIN32
        closesocket(sock_in);
        #else
        close(sock_in);
        #endif
        return false;
    }
    
    // Create output socket (send controls to FlightGear)
    sock_out = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_out < 0) {
        perror("Failed to create output socket");
        #ifdef _WIN32
        closesocket(sock_in);
        #else
        close(sock_in);
        #endif
        return false;
    }
    
    // Configure output address
    memset(&fg_addr_out, 0, sizeof(fg_addr_out));
    fg_addr_out.sin_family = AF_INET;
    fg_addr_out.sin_addr.s_addr = inet_addr("127.0.0.1");
    fg_addr_out.sin_port = htons(FG_OUTPUT_PORT);
    
    printf("UDP sockets initialized:\n");
    printf("  Input:  Port %d (receive from FlightGear)\n", FG_INPUT_PORT);
    printf("  Output: Port %d (send to FlightGear)\n", FG_OUTPUT_PORT);
    printf("  Note: FlightGear should be configured to send TO port %d and listen ON port %d\n", FG_INPUT_PORT, FG_OUTPUT_PORT);
    
    return true;
}

// FlightGear Native FDM structure (simplified - key fields only)
struct FlightGearFDM {
    uint32_t version;           // Version number
    
    // Positions (double precision)
    double longitude;           // rad
    double latitude;            // rad
    double altitude;            // ft
    
    // Orientation (float precision) 
    float phi;                  // roll angle (rad)
    float theta;                // pitch angle (rad) 
    float psi;                  // yaw angle (rad)
    
    // Angular velocities (float precision)
    float phidot;               // roll rate (rad/s)
    float thetadot;             // pitch rate (rad/s)
    float psidot;               // yaw rate (rad/s)
    
    // Skip many fields we don't need...
    char padding[200];          // Simplified - real structure has many more fields
    
    // Accelerations (float precision) - these are at specific offsets
    float A_X_pilot;            // X acceleration (ft/s²)
    float A_Y_pilot;            // Y acceleration (ft/s²)  
    float A_Z_pilot;            // Z acceleration (ft/s²)
};

// Parse FlightGear Native FDM binary data
bool parseNativeFDM(const char* data, int length, FlightGearHAL::FlightGearData* fg_data) {
    // Native FDM packets are typically 1008+ bytes
    if (length < 100) {
        return false;  // Too small to be native FDM
    }
    
    // Check if this looks like binary data (contains null bytes)
    bool has_nulls = false;
    for (int i = 0; i < std::min(50, length); i++) {
        if (data[i] == 0) {
            has_nulls = true;
            break;
        }
    }
    
    if (!has_nulls) {
        return false;  // Probably text data
    }
    
    // For FlightGear native FDM, we need to extract data at specific byte offsets
    // This is a simplified parser - the real FDM structure is more complex
    
    if (length >= 408) {  // Native FDM packets are exactly 408 bytes
        // Based on analysis, the attitude data starts at byte offset 4
        float* attitude_data = (float*)(data + 4);
        
        // Extract attitude angles (convert from radians to degrees)
        fg_data->roll_deg = attitude_data[0] * 57.295779f;
        fg_data->pitch_deg = attitude_data[1] * 57.295779f;
        fg_data->heading_deg = attitude_data[2] * 57.295779f;
        
        // Normalize heading to 0-360 degrees
        while (fg_data->heading_deg < 0) fg_data->heading_deg += 360.0f;
        while (fg_data->heading_deg >= 360.0f) fg_data->heading_deg -= 360.0f;
        
        // Extract angular rates (convert from radians/sec to degrees/sec)
        fg_data->roll_rate_degps = attitude_data[3] * 57.295779f;
        fg_data->pitch_rate_degps = attitude_data[4] * 57.295779f;
        fg_data->yaw_rate_degps = attitude_data[5] * 57.295779f;
        
        // For now, set accelerations to reasonable defaults
        // We can find the correct offsets for these later if needed
        fg_data->x_accel_fps2 = 0.0f;
        fg_data->y_accel_fps2 = 0.0f;
        fg_data->z_accel_fps2 = -32.174f;  // Standard gravity in ft/s²
        
        return true;
    }
    
    return false;
}

// Parse FlightGear UDP data packet
bool parseFlightGearData(const char* data, FlightGearHAL::FlightGearData* fg_data) {
    if (!data || !fg_data) return false;
    
    // Get data length
    int data_length = strlen(data);
    if (data_length == 0) {
        // Might be binary data - check first 1000 bytes for binary patterns
        data_length = 1000;  // Assume reasonable max for binary packet
    }
    
    // Enhanced debugging: Print raw data with more detail
    static int debug_count = 0;
    if (++debug_count % 10 == 0) {  // Print every 10th packet for better debugging
        printf("\n=== FlightGear Data Debug (packet #%d) ===\n", debug_count);
        printf("Apparent string length: %d\n", (int)strlen(data));
        printf("Assumed data length: %d\n", data_length);
        
        // Check if this looks like binary data
        bool looks_binary = false;
        for (int i = 0; i < std::min(50, data_length); i++) {
            if (data[i] == 0 || (unsigned char)data[i] > 127) {
                looks_binary = true;
                break;
            }
        }
        
        if (looks_binary) {
            printf("Data type: BINARY (likely Native FDM)\n");
            printf("First 32 bytes (hex): ");
            for (int i = 0; i < std::min(32, data_length); i++) {
                printf("%02X ", (unsigned char)data[i]);
            }
            printf("\n");
        } else {
            printf("Data type: TEXT\n");
            printf("Raw data: '");
            // Print with visible tab characters and newlines
            for (int i = 0; data[i] != '\0' && i < 200; i++) {
                if (data[i] == '\t') {
                    printf("[TAB]");
                } else if (data[i] == '\n') {
                    printf("[NL]");
                } else if (data[i] == '\r') {
                    printf("[CR]");
                } else if (data[i] < 32 || data[i] > 126) {
                    printf("[%02X]", (unsigned char)data[i]);
                } else {
                    printf("%c", data[i]);
                }
            }
            printf("'\n");
        }
    }
    
    // Try binary (Native FDM) format first
    if (parseNativeFDM(data, data_length, fg_data)) {
        if (debug_count % 10 == 0) {
            printf("SUCCESS: Parsed Native FDM binary format\n");
            printf("Roll=%.2f° Pitch=%.2f° Heading=%.2f°\n", 
                   fg_data->roll_deg, fg_data->pitch_deg, fg_data->heading_deg);
            printf("Rates: %.2f°/s %.2f°/s %.2f°/s\n",
                   fg_data->roll_rate_degps, fg_data->pitch_rate_degps, fg_data->yaw_rate_degps);
            printf("==========================================\n\n");
        }
        return true;
    }
    
    // Fall back to text parsing if binary fails
    // First try to parse as full format (output_protocol.xml - 9 values)
    int parsed = sscanf(data, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f",
        &fg_data->roll_deg,
        &fg_data->pitch_deg, 
        &fg_data->heading_deg,
        &fg_data->roll_rate_degps,
        &fg_data->pitch_rate_degps,
        &fg_data->yaw_rate_degps,
        &fg_data->x_accel_fps2,
        &fg_data->y_accel_fps2,
        &fg_data->z_accel_fps2
    );
    
    if (debug_count % 10 == 0) {
        printf("Full format parse attempt: got %d values (need 9)\n", parsed);
        if (parsed > 0) {
            printf("Parsed values: ");
            if (parsed >= 1) printf("roll=%.3f ", fg_data->roll_deg);
            if (parsed >= 2) printf("pitch=%.3f ", fg_data->pitch_deg);
            if (parsed >= 3) printf("heading=%.3f ", fg_data->heading_deg);
            if (parsed >= 4) printf("roll_rate=%.3f ", fg_data->roll_rate_degps);
            if (parsed >= 5) printf("pitch_rate=%.3f ", fg_data->pitch_rate_degps);
            if (parsed >= 6) printf("yaw_rate=%.3f ", fg_data->yaw_rate_degps);
            if (parsed >= 7) printf("x_accel=%.3f ", fg_data->x_accel_fps2);
            if (parsed >= 8) printf("y_accel=%.3f ", fg_data->y_accel_fps2);
            if (parsed >= 9) printf("z_accel=%.3f ", fg_data->z_accel_fps2);
            printf("\n");
        }
    }
    
    if (parsed == 9) {
        if (debug_count % 10 == 0) printf("SUCCESS: Using full text format\n");
        return true;  // Successfully parsed full format
    }
    
    // If that failed, try IMU-only format (imu_protocol.xml - 6 values)
    parsed = sscanf(data, "%f\t%f\t%f\t%f\t%f\t%f",
        &fg_data->roll_rate_degps,
        &fg_data->pitch_rate_degps,
        &fg_data->yaw_rate_degps,
        &fg_data->x_accel_fps2,
        &fg_data->y_accel_fps2,
        &fg_data->z_accel_fps2
    );
    
    if (debug_count % 10 == 0) {
        printf("IMU-only format parse attempt: got %d values (need 6)\n", parsed);
        if (parsed > 0) {
            printf("Parsed values: ");
            if (parsed >= 1) printf("roll_rate=%.3f ", fg_data->roll_rate_degps);
            if (parsed >= 2) printf("pitch_rate=%.3f ", fg_data->pitch_rate_degps);
            if (parsed >= 3) printf("yaw_rate=%.3f ", fg_data->yaw_rate_degps);
            if (parsed >= 4) printf("x_accel=%.3f ", fg_data->x_accel_fps2);
            if (parsed >= 5) printf("y_accel=%.3f ", fg_data->y_accel_fps2);
            if (parsed >= 6) printf("z_accel=%.3f ", fg_data->z_accel_fps2);
            printf("\n");
        }
    }
    
    if (parsed == 6) {
        // For IMU-only format, we don't have attitude angles
        // Set them to zero or maintain previous values
        static bool first_imu_parse = true;
        if (first_imu_parse) {
            printf("Using IMU-only protocol (no attitude angles available)\n");
            fg_data->roll_deg = 0.0f;
            fg_data->pitch_deg = 0.0f;
            fg_data->heading_deg = 0.0f;
            first_imu_parse = false;
        }
        if (debug_count % 10 == 0) printf("SUCCESS: Using IMU-only format\n");
        return true;
    }
    
    if (debug_count % 10 == 0) {
        printf("FAILED: Could not parse data - expected 9 or 6 values, got %d\n", parsed);
        printf("==========================================\n\n");
    }
    return false;
}

// Send control commands to FlightGear
void sendControlsToFlightGear(const FlightGearHAL::ControlData& controls) {
    char send_data[256];
    
    // Clamp controls to safe range
    float aileron = std::max(-0.6f, std::min(0.6f, controls.aileron));
    float elevator = std::max(-0.6f, std::min(0.6f, controls.elevator));
    float rudder = std::max(-0.6f, std::min(0.6f, controls.rudder));
    float throttle = std::max(0.0f, std::min(1.0f, controls.throttle));
    
    // Format as tab-separated values per input_protocol.xml
    int len = snprintf(send_data, sizeof(send_data), "%.6f\t%.6f\t%.6f\t%.6f\n",
                      aileron, elevator, rudder, throttle);
    
    // Send to FlightGear
    #ifdef _WIN32
    int sent = sendto(sock_out, send_data, len, 0,
                     (struct sockaddr*)&fg_addr_out, sizeof(fg_addr_out));
    #else
    ssize_t sent = sendto(sock_out, send_data, len, 0,
                         (struct sockaddr*)&fg_addr_out, sizeof(fg_addr_out));
    #endif
    
    if (sent != len) {
        perror("Failed to send controls to FlightGear");
    }
}

// RC stick reader thread (optional - for manual control)
#ifdef _WIN32
DWORD WINAPI rcReaderThread(LPVOID arg) {
#else
void* rcReaderThread(void* arg) {
#endif
    printf("Starting RC reader thread...\n");
    
    #ifdef HAVE_SDL2
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        printf("SDL initialization failed: %s\n", SDL_GetError());
        #ifdef _WIN32
        return 0;
        #else
        return nullptr;
        #endif
    }
    #else
    printf("SDL2 not available - no joystick support\n");
    printf("System will run in autopilot mode only\n");
    #ifdef _WIN32
    return 0;
    #else
    return nullptr;
    #endif
    #endif
    
    #ifdef HAVE_SDL2
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks == 0) {
        printf("No joysticks detected - using keyboard/autopilot control\n");
        SDL_Quit();
        #ifdef _WIN32
        return 0;
        #else
        return nullptr;
        #endif
    }
    
    SDL_Joystick* joy = SDL_JoystickOpen(0);
    if (!joy) {
        printf("Failed to open joystick: %s\n", SDL_GetError());
        SDL_Quit();
        #ifdef _WIN32
        return 0;
        #else
        return nullptr;
        #endif
    }
    
    printf("Joystick connected: %s\n", SDL_JoystickName(joy));
    printf("  Axes: %d\n", SDL_JoystickNumAxes(joy));
    printf("  Buttons: %d\n", SDL_JoystickNumButtons(joy));
    
    static uint32_t debug_counter = 0;
    while (running) {
        SDL_PumpEvents();
        
        // Read stick inputs (adjust axis mapping as needed)
        int16_t rawA = SDL_JoystickGetAxis(joy, 0);  // Aileron (roll)
        int16_t rawE = SDL_JoystickGetAxis(joy, 1);  // Elevator (pitch)  
        int16_t rawR = SDL_JoystickGetAxis(joy, 3);  // Rudder (yaw)
        int16_t rawT = SDL_JoystickGetAxis(joy, 2);  // Throttle
        
        // Convert to -1...+1 range
        rc_aileron = rawA / 32767.0f;
        rc_elevator = -rawE / 32767.0f;  // Invert elevator
        rc_rudder = rawR / 32767.0f;
        rc_throttle = (-rawT + 32767) / 65534.0f;  // Convert -32767...32767 to 0...1
        
        // Debug output every 50 iterations (about 4 times per second)
        if (++debug_counter % 50 == 0) {
            printf("RC: A=%.2f E=%.2f R=%.2f T=%.2f\n", 
                   rc_aileron, rc_elevator, rc_rudder, rc_throttle);
        }
        
        usleep(5000);  // 200 Hz update rate
    }
    
    SDL_JoystickClose(joy);
    SDL_Quit();
    #else
    // No SDL2 - just maintain neutral controls
    while (running) {
        // Keep controls at neutral (autopilot mode)
        rc_aileron = 0.0f;
        rc_elevator = 0.0f;
        rc_rudder = 0.0f;
        rc_throttle = 0.1f;  // Small throttle for testing
        
        #ifdef _WIN32
        Sleep(50);  // 20 Hz update rate
        #else
        usleep(50000);
        #endif
    }
    #endif
    printf("RC reader thread terminated\n");
    #ifdef _WIN32
    return 0;
    #else
    return nullptr;
    #endif
}

// Cleanup function
void cleanup() {
    running = false;
    
    if (controller) {
        controller->disarm();
        delete controller;
        controller = nullptr;
    }
    
    if (hal) {
        delete hal;
        hal = nullptr;
    }
    
    if (sock_in >= 0) {
        #ifdef _WIN32
        closesocket(sock_in);
        #else
        close(sock_in);
        #endif
        sock_in = -1;
    }
    
    if (sock_out >= 0) {
        #ifdef _WIN32
        closesocket(sock_out);
        #else
        close(sock_out);
        #endif
        sock_out = -1;
    }
    
    #ifdef _WIN32
    WSACleanup();
    #endif
    
    printf("Cleanup completed\n");
}

int main(int argc, char* argv[]) {
    printf("=== FlightGear Flight Controller Interface ===\n");
    printf("Connecting FlightGear simulation to dRehm flight controller\n\n");
    
    // Setup signal handlers
    #ifdef _WIN32
    SetConsoleCtrlHandler(signalHandler, TRUE);
    #else
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    #endif
    
    // Initialize UDP communication
    if (!initUDP()) {
        printf("Failed to initialize UDP communication\n");
        return 1;
    }
    
    // Create hardware abstraction layer
    HardwareConfig hw_config;
    hw_config.use_magnetometer = false;  // No magnetometer available
    hal = new FlightGearHAL(hw_config);
    
    // Create flight controller
    FlightController::Config fc_config;
    fc_config.loop_frequency_hz = 100;  // 100 Hz for simulation
    fc_config.default_mode = ControlMode::STABILIZE;  // Stabilize mode works great without magnetometer
    fc_config.enable_debug_output = true;
    fc_config.debug_output_rate_hz = 2;  // 2 Hz debug output
    
    // Note: Without magnetometer, heading will drift over time in yaw axis
    // This is fine for stabilization modes but may affect navigation modes
    
    controller = new FlightController(hal, fc_config);
    
    // Initialize flight controller
    if (!controller->initialize()) {
        printf("Failed to initialize flight controller\n");
        cleanup();
        return 1;
    }
    
    printf("Flight controller initialized successfully\n");
    
    // Start RC reader thread (optional)
    pthread_t rc_thread;
    #ifdef _WIN32
    if (pthread_create(&rc_thread, nullptr, rcReaderThread, nullptr) != 0) {
    #else
    if (pthread_create(&rc_thread, nullptr, rcReaderThread, nullptr) != 0) {
    #endif
        printf("Warning: Failed to start RC reader thread\n");
    }
    
    // Main control loop
    printf("Starting main control loop...\n");
    printf("Waiting for FlightGear data on port %d...\n", FG_INPUT_PORT);
    printf("Make sure FlightGear is started with the correct UDP configuration!\n");
    
    char recv_buffer[UDP_BUFFER_SIZE];
    FlightGearHAL::FlightGearData fg_data;
    int loop_count = 0;
    
    while (running) {
        // 1) Read FlightGear state via UDP (blocking)
        #ifdef _WIN32
        int recv_len = recvfrom(sock_in, recv_buffer, sizeof(recv_buffer) - 1, 0, nullptr, nullptr);
        #else
        ssize_t recv_len = recvfrom(sock_in, recv_buffer, sizeof(recv_buffer) - 1, 0, nullptr, nullptr);
        #endif
        if (recv_len <= 0) {
            if (running) {
                perror("Failed to receive from FlightGear");
            }
            continue;
        }
        
        recv_buffer[recv_len] = '\0';  // Null terminate
        
        // 2) Parse FlightGear data
        if (!parseFlightGearData(recv_buffer, &fg_data)) {
            printf("Failed to parse FlightGear data: %s\n", recv_buffer);
            continue;
        }
        
        // 3) Update HAL with FlightGear data
        hal->setFlightGearData(fg_data);
        
        // 4) Update HAL with RC inputs (from joystick or autopilot)
        hal->setRadioInputs(rc_aileron, rc_elevator, rc_rudder, rc_throttle);
        
        // 5) Run flight controller update cycle
        controller->update();
        
        // 6) Get control outputs from flight controller
        FlightGearHAL::ControlData controls = hal->getControlOutputs();
        
        // 7) Send controls back to FlightGear
        sendControlsToFlightGear(controls);
        
        // 8) Status output every few seconds
        if (++loop_count % 200 == 0) {  // Every ~2 seconds at 100Hz
            printf("Loop %d: Roll=%.1f° Pitch=%.1f° → Ail=%.3f Elev=%.3f Thr=%.3f\n",
                   loop_count, fg_data.roll_deg, fg_data.pitch_deg,
                   controls.aileron, controls.elevator, controls.throttle);
        }
    }
    
    // Wait for RC thread to finish
    pthread_join(rc_thread, nullptr);
    
    cleanup();
    printf("FlightGear interface terminated\n");
    return 0;
} 