import os
import time
from socket import *
sockobj = socket(AF_INET, SOCK_STREAM)

DISCONNECT_STATE = -1
STABALIZED_STATE = 0
GUIDED_STATE = 1

EMPTY_FILE = "9"+"0"*55

TIMEOUT = 50

LANDING_COMMAND = 0
TAKEOFF_COMMAND = 1
GOTO_COMMAND = 2
ORBIT_COMMAND = 3
LISTEN_COMMAND = 9

344138295
initial_lat = -353631909
initial_lon = 1491652374


def receive_command():
    command_type = input("command_type:")
    target_lat = str(initial_lat+1796).zfill(11)
    target_lon = str(initial_lon).zfill(11)
    target_alt = "001000"
    orbit_radius = "010"
    phone_lat = "1"*11
    phone_lon = "2"*11
    return command_type+target_lat+target_lon+target_alt+orbit_radius+phone_lat+phone_lon

def write_command(command):
    f = open("command.txt", "w")
    f.write(command)
    f.close()
    
if __name__ == "__main__":
    while(1):
        write_command(receive_command())