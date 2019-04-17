#include "DataSocket.hpp"
#include "delay.h"

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>
#include <fcntl.h>


DataSocket::DataSocket()
{
	server_socket = 0;
	client_socket = 0;
	memset(&server_address, 0, sizeof(server_address));
}

int DataSocket::open(const char *address_string, uint16_t server_port)
{
	//Create socket
	sockaddr_in* server_address_p=&server_address;
	server_socket=socket(AF_INET, SOCK_STREAM, 0);
	if(server_socket<=0){
		perror("Error at socket creation");
		exit(EXIT_FAILURE);
	}

	fcntl(server_socket, F_SETFL, O_NONBLOCK); //NON BLOCKING
	//Address config
	server_address_p->sin_family=AF_INET;
	server_address_p->sin_port=htons(server_port);
	if(inet_pton(AF_INET, address_string, &server_address_p->sin_addr)<0){
		perror("Error at address conversion");
		exit(EXIT_FAILURE);
	}

	//Reusable addresses and ports
	int option_value=1;
	if(setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &option_value, sizeof(option_value))){
		perror("Error at setsockopt");
		exit(EXIT_FAILURE);
	}

	//Bind
	if(bind(server_socket,(struct sockaddr*)(server_address_p), sizeof(*server_address_p))){
		perror("Error at bind");
		exit(EXIT_FAILURE);
	}

	//Listen with queue of size 1
	if(listen(server_socket, 1)<0){
		perror("Error server listen");
		exit(EXIT_FAILURE);
	}
}

bool DataSocket::accept_client()
{
	struct sockaddr_in* server_address_p=&server_address;
	int addr_len=sizeof(*server_address_p);
	int new_socket = accept(server_socket, (struct sockaddr *) server_address_p, (socklen_t *) &addr_len);
	if(new_socket<=0){
		delay(50);
		return false;
	}
	else {
		client_socket = new_socket;
		return true;
	}
}

int DataSocket::send_data(const char* data)
{

	int result=send(client_socket, data, strlen(data), 0);
	if(result<0){
		if(errno==EPIPE || errno==ECONNRESET)
			std::cout<<"Client disconnected"<<std::endl;
		else {
			perror("Error sending data to client");
		}
	}

    return result;
}
