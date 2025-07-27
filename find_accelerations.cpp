#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <cstring>
#include <cmath>

#pragma comment(lib, "ws2_32.lib")

int main() {
    WSAData wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed" << std::endl;
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        WSACleanup();
        return 1;
    }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(5000);

    if (bind(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Bind failed - port 5000 already in use! Stop flightgear_interface_final.exe first" << std::endl;
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    std::cout << "Listening for FlightGear Native FDM on port 5000..." << std::endl;
    std::cout << "Looking for acceleration data in the binary stream..." << std::endl;
    std::cout << "Move the aircraft in FlightGear to generate acceleration!" << std::endl;

    char buffer[1024];
    int packetCount = 0;

    while (true) {
        int bytesReceived = recv(sock, buffer, sizeof(buffer), 0);
        if (bytesReceived <= 0) continue;

        packetCount++;
        if (packetCount % 20 != 0) continue; // Sample every 20th packet

        std::cout << "\n=== Packet " << packetCount << " - Acceleration Analysis ===" << std::endl;
        std::cout << "Data length: " << bytesReceived << " bytes" << std::endl;

        // Test potential acceleration offsets (looking for reasonable values)
        // Accelerations should be roughly -50 to +50 ft/s² for most maneuvers
        
        for (int offset = 20; offset < bytesReceived - 12; offset += 4) {
            float* float_data = (float*)(buffer + offset);
            
            float x_accel = float_data[0];
            float y_accel = float_data[1]; 
            float z_accel = float_data[2];
            
            // Check if these look like reasonable accelerations
            bool reasonable = (fabs(x_accel) < 100.0f && fabs(y_accel) < 100.0f && 
                              fabs(z_accel) < 100.0f && 
                              !std::isnan(x_accel) && !std::isnan(y_accel) && !std::isnan(z_accel));
            
            // Check for gravity in different units:
            // -32.174 ft/s² (standard gravity in ft/s²)
            // -9.81 m/s² (standard gravity in m/s²)
            bool gravity_ft = (fabs(z_accel + 32.174f) < 10.0f);
            bool gravity_m = (fabs(z_accel + 9.81f) < 5.0f);
            bool gravity_check = gravity_ft || gravity_m;
            
            if (reasonable && (fabs(x_accel) > 0.1f || fabs(y_accel) > 0.1f || gravity_check)) {
                std::cout << "Offset " << offset << ": X=" << x_accel 
                         << " Y=" << y_accel << " Z=" << z_accel;
                if (gravity_ft) std::cout << " ft/s² ← GRAVITY IN FT/S²";
                else if (gravity_m) std::cout << " m/s² ← GRAVITY IN M/S²";
                else if (gravity_check) std::cout << " ← POSSIBLE GRAVITY";
                std::cout << std::endl;
            }
        }
        
        // Look for better attitude data too
        bool found_attitude = false;
        for (int offset = 4; offset < 100; offset += 4) {
            float* attitude_test = (float*)(buffer + offset);
            float roll = attitude_test[0] * 57.295779f;
            float pitch = attitude_test[1] * 57.295779f;
            float heading = attitude_test[2] * 57.295779f;
            
            if (fabs(roll) < 180.0f && fabs(pitch) < 90.0f && fabs(heading) < 360.0f && 
                !std::isnan(roll) && !std::isnan(pitch) && !std::isnan(heading)) {
                std::cout << "Attitude at offset " << offset << ": Roll=" << roll 
                          << "° Pitch=" << pitch << "° Heading=" << heading << "°" << std::endl;
                found_attitude = true;
                break;
            }
        }
        if (!found_attitude) {
            std::cout << "No valid attitude data found in this packet" << std::endl;
        }
    }

    closesocket(sock);
    WSACleanup();
    return 0;
} 