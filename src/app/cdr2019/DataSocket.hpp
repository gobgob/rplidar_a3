#ifndef DATA_SOCKET_HPP
#define DATA_SOCKET_HPP

#ifdef _WIN32
#include <windows.h>
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

class DataSocket
{
public:
	DataSocket();
	int open(const char *address_string, uint16_t server_port);
	int send_data(const char* data);
    bool accept_client();
private:
	int server_socket;
	int client_socket;
	sockaddr_in server_address;
};

#endif
