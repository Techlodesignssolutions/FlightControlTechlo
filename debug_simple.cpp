#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "ws2_32.lib")

#define FG_INPUT_PORT 5000  // This matches your FlightGear output port
#define UDP_BUFFER_SIZE 1024

int main() {
    printf("=== FlightGear UDP Debug Tool ===\n");
    printf("Listening on port %d for FlightGear data...\n", FG_INPUT_PORT);
    printf("Press Ctrl+C to exit\n\n");
    
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup failed\n");
        return 1;
    }
    
    // Create socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        printf("Failed to create socket: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }
    
    // Configure socket address
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(FG_INPUT_PORT);
    
    // Bind socket
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        printf("Failed to bind socket: %d\n", WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }
    
    printf("Socket bound successfully. Waiting for data...\n\n");
    
    char buffer[UDP_BUFFER_SIZE];
    int packet_count = 0;
    
    while (packet_count < 10) {  // Capture 10 packets then exit
        // Receive data
        int recv_len = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
        
        if (recv_len == SOCKET_ERROR) {
            printf("recvfrom failed: %d\n", WSAGetLastError());
            continue;
        }
        
        buffer[recv_len] = '\0';  // Null terminate
        packet_count++;
        
        printf("=== Packet #%d (length: %d) ===\n", packet_count, recv_len);
        
        // Print raw data with special characters visible
        printf("Raw data: '");
        for (int i = 0; i < recv_len; i++) {
            if (buffer[i] == '\t') {
                printf("[TAB]");
            } else if (buffer[i] == '\n') {
                printf("[NL]");
            } else if (buffer[i] == '\r') {
                printf("[CR]");
            } else if (buffer[i] == ' ') {
                printf("[SPACE]");
            } else if (buffer[i] < 32 || buffer[i] > 126) {
                printf("[0x%02X]", (unsigned char)buffer[i]);
            } else {
                printf("%c", buffer[i]);
            }
        }
        printf("'\n");
        
        // Count separators
        int tab_count = 0, space_count = 0, comma_count = 0;
        for (int i = 0; i < recv_len; i++) {
            if (buffer[i] == '\t') tab_count++;
            else if (buffer[i] == ' ') space_count++;
            else if (buffer[i] == ',') comma_count++;
        }
        
        printf("Separators: %d tabs, %d spaces, %d commas\n", tab_count, space_count, comma_count);
        
        // Try parsing as numbers
        if (tab_count > 0) {
            printf("Tab-separated values:\n");
            char temp_buffer[UDP_BUFFER_SIZE];
            strcpy(temp_buffer, buffer);
            
            char* token = strtok(temp_buffer, "\t\n\r");
            int field_num = 1;
            while (token != nullptr && field_num <= 12) {
                float value;
                if (sscanf(token, "%f", &value) == 1) {
                    printf("  Field %d: %.6f\n", field_num, value);
                } else {
                    printf("  Field %d: '%s' (not a number)\n", field_num, token);
                }
                token = strtok(nullptr, "\t\n\r");
                field_num++;
            }
        }
        
        printf("\n");
    }
    
    printf("Captured %d packets. Exiting...\n", packet_count);
    
    // Cleanup
    closesocket(sock);
    WSACleanup();
    
    return 0;
} 