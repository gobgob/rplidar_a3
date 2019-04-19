#include "DataSocket.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>


DataSocket::DataSocket()
{
	server_socket = 0;
	for (size_t i = 0; i < DATA_SOCKET_MAX_CLIENT; i++) {
		clients_socket[i] = 0;
	}
}

DataSocket::~DataSocket()
{
	shutdown(server_socket, SHUT_RDWR);
	for (size_t i = 0; i < DATA_SOCKET_MAX_CLIENT; i++) {
		shutdown(clients_socket[i], SHUT_RDWR);
	}
}

int DataSocket::open(const char *address_string, uint16_t server_port)
{
	// Create socket
	server_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (server_socket <= 0) {
		perror("Error at socket creation");
		return -1;
	}

	// Set socket as NON-BLOCKING
	if (fcntl(server_socket, F_SETFL, O_NONBLOCK) < 0) {
		perror("Error at set non-blocking");
		return -1;
	}

	// Address configuration
	sockaddr_in server_address;
	memset(&server_address, 0, sizeof(server_address));
	server_address.sin_family = AF_INET;
	server_address.sin_port = htons(server_port);
	if (inet_pton(AF_INET, address_string, &server_address.sin_addr) != 1) {
		perror("Error at address conversion");
		return -1;
	}

	// Set option: reusable addresses and ports
	int option_value = 1;
	if (setsockopt(server_socket, SOL_SOCKET, (SO_REUSEADDR | SO_REUSEPORT),
			&option_value, sizeof(option_value)) < 0) {
		perror("Error at setsockopt");
		return -1;
	}

	// Bind socket to address
	if (bind(server_socket, (sockaddr*)(&server_address), sizeof(server_address)) < 0) {
		perror("Error at bind");
		return -1;
	}

	// Start listening
	if (listen(server_socket, DATA_SOCKET_MAX_CLIENT) < 0) {
		perror("Error at listen");
		return -1;
	}

	return 0;
}

bool DataSocket::accept_client()
{
	int new_client = accept(server_socket, NULL, NULL);
	if (new_client > 0) {
		for (size_t i = 0; i < DATA_SOCKET_MAX_CLIENT; i++) {
			if (clients_socket[i] <= 0) {
				clients_socket[i] = new_client;
				printf("Client #%u connected\n", i);
				return true;
			}
		}
		shutdown(new_client, SHUT_RDWR);
		perror("Reached max number of clients");
	}
	return false;
}

int DataSocket::send_data(const char* data)
{
	int ret_code = 0;
	for (size_t i = 0; i < DATA_SOCKET_MAX_CLIENT; i++) {
		if (clients_socket[i] <= 0) continue;
		int ret = send(clients_socket[i], data, strlen(data), MSG_NOSIGNAL);
		if (ret < 0) {
			if (errno==EPIPE || errno==ECONNRESET) {
				printf("Client #%u disconnected\n", i);
			}
			else {
				ret_code = -1;
				fprintf(stderr, "Failed to send data to client #%u\n", i);
			}
			shutdown(clients_socket[i], SHUT_RDWR);
			clients_socket[i] = 0;
		}
	}
    return ret_code;
}
