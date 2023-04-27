import os
import time
from time import sleep

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

def receive_command():
    f = open("command.txt", "r")
    command = f.read()
    return command
    
def write_command(command):
    f = open("//192.168.56.101/share/command.txt", "w")
    f.write(command+"0"*6+str(int(time.time()*10)))
    f.close()
    
def read_status():
    f = open("//192.168.56.101/share/log.txt", "r")
    status = f.read()
    f.close()
    ack = status[0]
    if(int(ack) == 1 ):
        print("takeoff fail")
    elif(int(ack) == 2 ):
        print("takeoff success")
        
    return "0"+status[32:38]

def write_log(drone_log,connection):
        
    f = open("log.txt", "w")
    f.write(drone_log + str(connection))
    f.close()

drone_log = ""

while(1):
    sleep(0.5)
    try:
        command = receive_command()
        write_command(command)
        drone_log = read_status()
        write_log(drone_log, 0)
        #print("read_status()")
    except:
        write_log(drone_log, 1)
        print("lost connection")