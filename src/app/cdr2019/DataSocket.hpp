#ifndef DATA_SOCKET_HPP
#define DATA_SOCKET_HPP

#define DATA_SOCKET_MAX_CLIENT 4

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
	~DataSocket();
	int open(const char *address_string, uint16_t server_port);
	int send_data(const char* data);
    bool accept_client();
private:
	int server_socket;
	int clients_socket[DATA_SOCKET_MAX_CLIENT];
};

#endif
