#ifndef LIDAR_COM_HPP
#define LIDAR_COM_HPP

#include <iostream>
#include <sys/socket.h>
#include <libnet.h>
#include <fcntl.h>
#include <netinet/in.h>

class DataSocket {
public:
	int server_socket=0;
	int client_socket=0;
	struct sockaddr_in server_address;
	DataSocket(const char *address_string, uint16_t server_port);
	int send_data(char* data);
    bool accept_client();
};


#endif
