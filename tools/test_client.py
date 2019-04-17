#!/usr/bin/python3
# coding: utf-8

import socket
import time
hote = "127.0.0.1"
port = 17685

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    socket.connect((hote, port))
except ConnectionRefusedError:
    print("Connection failed")
    quit(0)
print("Connection on {}".format(port))
with open("mesures.txt", "w") as f:
    try:
        while True:
            f.write(socket.recv(100000).decode("utf-8")+"\n")
    except KeyboardInterrupt:
        pass
print("Close")
socket.close()
