import socket
from pymavlink import mavutil
import numpy as np
import os
import time
import math
from threading import Thread

port = "udpin:localhost:14551"
DEAFAULT_ALT = 10
R = 6373.0
T = 0.1
P = 60

HOST = '192.168.56.106' # Enter IP or Hostname of your server
PORT = 12345 # Pick an open Port (1000+ recommended), must match the server port



the_connection = mavutil.mavlink_connection(port)
the_connection.wait_heartbeat()


def getGPS():
    while True:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 33, 10000 , 0, 0, 0, 0, 0)
        global msg_gps
        msg_gps = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
        f = open("/home/yifanpan/share/telemetry.txt", "w")
        f.write(str(msg_gps.lat)+" "+str(msg_gps.lon)+" "+str(msg_gps.vx)+" "+str(msg_gps.vy))
        f.close()
        
t1 = Thread(target = getGPS)
t1.start()

def arm():
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
	return (msg.result)

def disarm():
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)


def guided():
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)

	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
	return (msg.result)

def takeoff():
	# last param: altitude
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, DEAFAULT_ALT)

	msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
	return (msg.result)
def land():
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0)
def takeoff_procedure():
	if (guided() > 0):
		return 1
	if (arm() > 0):
		return 1
	if (takeoff() > 0):
		return 1
	return 0

def go_pos(target_latitude, target_longitude, target_yaw = 0):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                        the_connection.target_component, 
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                         int(0b110111111000), int(target_latitude), 
                        int(target_longitude), DEAFAULT_ALT, 0, 0, 0, 0, 0, 0, target_yaw, 0))
    
def go_vel(vx, vy, target_yaw):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                            the_connection.target_component, 
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                            2503, 0, 
                            0, 0, vx, vy, 0, 0, 0, 0, target_yaw, 0))
    
def rotate(period):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                            the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
							int(0b011111000111), 0, 0, -DEAFAULT_ALT, 0, 0, 0, 0, 0, 0, 0, 2*math.pi/period))

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST,PORT))

#Lets loop awaiting for your input
while True:
    try:
        reply = s.recv(1024)
        if reply is not None:
            command = reply.decode()
            print(command)
            if(command[0] == '0'):
                print("land")
                land()
            elif(command[0] == '1'):
                takeoff_result = takeoff_procedure()
                if(takeoff_result!= 0):
                      print("takeoff fail")
                else:
                      print("takeoff success")
            elif(command[0] == '2'):
                #go position command
                target_lat = int(command[1:12])
                target_lon = int(command[12:23])
                target_yaw = float(command[23:29])
                go_pos(target_lat, target_lon, target_yaw)
            elif(command[0] == '3'):
                #go position command
                vx = float(command[1:12])
                vy = float(command[12:23])
                target_yaw = float(command[23:29])
                go_vel(vx, vy, target_yaw)
            elif(command[0] == '4'):
                #go position command
                period = int(command[1:3])
                rotate(period)
    except Exception as e:
        print(e)
        break
