#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <cstdio>
#include <cstring>

#pragma comment(lib, "ws2_32.lib")

int main() {
    printf("=== Simple UDP Receiver Test ===\n");
    
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, 0);
    
    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(5010);
    
    bind(sock, (struct sockaddr*)&addr, sizeof(addr));
    
    printf("Listening on port 5010...\n");
    
    char buffer[2000];
    for (int i = 0; i < 5; i++) {
        int recv_len = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
        printf("Packet %d: received %d bytes\n", i+1, recv_len);
        
        if (recv_len > 0) {
            printf("First 32 bytes (hex): ");
            for (int j = 0; j < 32 && j < recv_len; j++) {
                printf("%02X ", (unsigned char)buffer[j]);
            }
            printf("\n");
            
            printf("String length: %d\n", (int)strlen(buffer));
            printf("Contains nulls: %s\n", (memchr(buffer, 0, recv_len) ? "YES" : "NO"));
            printf("\n");
        }
    }
    
    closesocket(sock);
    WSACleanup();
    return 0;
} 