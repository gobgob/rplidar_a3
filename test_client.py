#!/usr/bin/python3
# coding: utf-8

import socket
import time
hote = "127.0.0.1"
port = 17685

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((hote, port))
print("Connection on {}".format(port))
while True:
    #socket.send(b"R")
    time.sleep(1)
    print(socket.recv(1024))

print("Close")
socket.close()
