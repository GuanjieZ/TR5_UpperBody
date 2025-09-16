#include "udp_tool.hpp"

int init_broadcast_socket(struct sockaddr_in *broadcast_addr) {
    int sockfd;
    int broadcast_enable = 1;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        return -1;
    }

    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
        perror("setsockopt failed");
        close(sockfd);
        return -1;
    }

    memset(broadcast_addr, 0, sizeof(*broadcast_addr));
    broadcast_addr->sin_family = AF_INET;
    broadcast_addr->sin_port = htons(12345);
    broadcast_addr->sin_addr.s_addr = inet_addr("192.168.1.255");

    return sockfd;
};


