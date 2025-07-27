#include <winsock2.h>
#include <cstdio>
#include <cmath>

#pragma comment(lib, "ws2_32.lib")

// Test different byte offsets to find reasonable attitude values
void analyzeBinaryData(const char* data, int length) {
    printf("=== Binary Data Analysis ===\n");
    printf("Data length: %d bytes\n", length);
    
    // Print first 64 bytes as hex
    printf("First 64 bytes: ");
    for (int i = 0; i < 64 && i < length; i++) {
        printf("%02X ", (unsigned char)data[i]);
        if ((i + 1) % 16 == 0) printf("\n                ");
    }
    printf("\n\n");
    
    // Try different starting positions and look for reasonable float values
    for (int offset = 4; offset < 100 && offset < length - 12; offset += 4) {
        float* floats = (float*)(data + offset);
        
        // Convert potential radians to degrees
        float val1_deg = floats[0] * 57.295779f;
        float val2_deg = floats[1] * 57.295779f;
        float val3_deg = floats[2] * 57.295779f;
        
        // Check if these look like reasonable attitude angles
        if (fabs(val1_deg) < 180.0f && fabs(val2_deg) < 90.0f && fabs(val3_deg) < 360.0f) {
            printf("Offset %d: Potential attitude data\n", offset);
            printf("  Value 1: %.3f° (could be roll)\n", val1_deg);
            printf("  Value 2: %.3f° (could be pitch)\n", val2_deg);
            printf("  Value 3: %.3f° (could be heading)\n", val3_deg);
            
            // Check next 3 values as potential rates
            float rate1 = floats[3] * 57.295779f;
            float rate2 = floats[4] * 57.295779f;
            float rate3 = floats[5] * 57.295779f;
            
            if (fabs(rate1) < 360.0f && fabs(rate2) < 360.0f && fabs(rate3) < 360.0f) {
                printf("  Rate 1: %.3f°/s (could be roll rate)\n", rate1);
                printf("  Rate 2: %.3f°/s (could be pitch rate)\n", rate2);
                printf("  Rate 3: %.3f°/s (could be yaw rate)\n", rate3);
            }
            printf("\n");
        }
    }
}

int main() {
    // Set up UDP to receive from FlightGear
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(5000);
    
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    
    printf("Listening for FlightGear Native FDM data on port 5000...\n");
    printf("Make sure FlightGear is running with --native-fdm=socket,out,10,127.0.0.1,5000,udp\n\n");
    
    char buffer[2000];
    for (int i = 0; i < 3; i++) {  // Analyze 3 packets
        int recv_len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
        printf("=== Packet %d ===\n", i + 1);
        analyzeBinaryData(buffer, recv_len);
        printf("\n");
    }
    
    closesocket(sock);
    WSACleanup();
    return 0;
} 