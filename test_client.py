#!/usr/bin/python3
# coding: utf-8

import socket
import time
hote = "127.0.0.1"
port = 17685

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
socket.connect((hote, port))
print("Connection on {}".format(port))
with open("mesures.txt", "w") as f:

    while True:
        #socket.send(b"R")
        #time.sleep(1)
        f.write(socket.recv(100000).decode("utf-8")+"\n")

print("Close")
socket.close()
