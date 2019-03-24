#!/usr/bin/env python3
import socket, struct, time
#import pygame
from modules.utils import *

UDP_IP = "127.0.0.1" # Vehicle IP address
#UDP_IP = "192.168.1.105" # Vehicle IP address
UDP_PORT = 51001 # This port match the ones using on other scripts
#[R, P, Y, T, mode, xxxxxxxx]
#message = [1450, 1450, 1450, 1000, 0,0,0,0, 0,0,0,0, 0,0,0,0,0,0] #mode=0 -> manual
message = [1500, 1500, 1500, 1050, 1,0,0,0, 0,0,0,0, 0,0,0,0,0,0] #mode=1 -> hold
formats = '>' + 'd' * len(message)
#print formats
buf = struct.pack(formats, *message)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(buf, (UDP_IP, UDP_PORT))

