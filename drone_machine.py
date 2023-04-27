from pymavlink import mavutil
import numpy as np
import os
import time
import math
from threading import Thread

DISCONNECT_STATE = -1
STABALIZED_STATE = 0
GUIDED_STATE = 1

EMPTY_FILE = "9"+"0"*85

TIMEOUT = 100
DRONE1_START = 0
LANDING_COMMAND = 0
TAKEOFF_COMMAND = 1
GOTO_COMMAND =2
ORBIT_COMMAND = 3
LISTEN_COMMAND = 9

DEAFAULT_ALT = 10

TAKEOFF_SUCCESS = 2
TAKEOFF_FAIL = 1

#current drone is at its orbit point,wait the other drone to be ready
WAIT_ORBIT = 0

#The other drone is in explore
ONE_ORBIT = 1

#Two drones orbit
TWO_ORBIT = 2

#Go to explore desitnation (on its way)
EXPLRE_ORBIT = 3

#On its way to go back to the orbit point
WAIT_MOVEMENT = 4

#Arrive at the explore point, start rotate
ARR_ROTATE  = 5

#On its way to the initiial point of the orbit
START_ORBIT = 16

port = "udpin:localhost:14551"

R = 6373.0
T = 0.1
P = 60

Kp = 0.5
Kd = 2

the_connection = mavutil.mavlink_connection(port)
the_connection.wait_heartbeat()


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

def takeoff_procedure(altitude):
	if (guided() > 0):
		return 1
	if (arm() > 0):
		return 1
	if (takeoff() > 0):
		return 1
	return 0

def land():
	the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
									mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0)

def writeTelemetry(ack):
	battery_info = msg_battery.battery_remaining

	gps_info = msg_gps
	gps_lat, gps_long, alt = gps_info.lat, gps_info.lon, gps_info.alt/1000

	yaw = gps_info.hdg
	telemetry =(str(ack)+
	      str(battery_info).zfill(3)+
		  str(gps_lat).zfill(11)+
		  str(gps_long).zfill(11)+
		  str(alt).zfill(6)+
		  str(yaw).zfill(6)+
		  str(orbit_index).zfill(4)+
		  str(current_state)+
		  str(orbit_state)+
		  str(time.time()*1000))
	
	f = open("/home/yifanpan/share/log.txt", "w")
	f.write(telemetry)
	f.close()

def writeBlankCommand():
	f = open("/home/yifanpan/share/command.txt", "w")
	f.write(EMPTY_FILE)
	f.close()


def readcommand():
	content = ""
	while(len(content)<1):
		f = open("/home/yifanpan/share/command.txt", "r")
		content = f.read()
		f.close()
	return content

def getGPS():
    while True:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 33, 10000 , 0, 0, 0, 0, 0)
        global msg_gps
        msg_gps = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)


def getBattery():
    while True:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, 1, 10000 , 0, 0, 0, 0, 0)
        global msg_battery
        msg_battery = the_connection.recv_match( type='SYS_STATUS', blocking=True)
	
def generate_traj(initial_lat, initial_lon, radius):
    total_steps = int(P/T)
    rad = [0]
    lat = [initial_lat+radius*10000/111.32]
    lon = [initial_lon]
    for i in range(total_steps-1):
        rad = rad + [(i+1)*math.pi/(P/T/2)]
        lat = lat + [(initial_lat+radius*math.cos(rad[i+1]) *10000/111.32)]
        lon = lon + [(initial_lon+radius*math.sin(rad[i+1]) *10000/111.32)]
    target_vx = [0]*total_steps
    target_vy = [0]*total_steps
    for i in range(total_steps-1):
        target_vx[i] = (lat[i+1]-lat[i])/T
        target_vy[i] = (lon[i+1]-lon[i])/T
    target_vy[-1] = target_vy[-2]
    return rad, lat, lon, target_vx, target_vy 

def calculate_speed(target_lat, target_lon, target_vx, target_vy):
    current_vx = msg_gps.vx
    current_vy = msg_gps.vy
    current_y = msg_gps.lon
    current_x = msg_gps.lat

    new_vx = Kp*(target_lat-current_x)+Kd*(target_vx-current_vx)
    new_vy = Kp*(target_lon-current_y)+Kd*(target_vy-current_vy)
    return new_vx, new_vy

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
    
writeBlankCommand()

current_state = DISCONNECT_STATE
orbit_state = START_ORBIT
prev_timestamp = 0
current_timestamp = 0
prev_time = 0
current_time = 0
ack = 0
prev_message = ""
prev_command = ""
current_command = ""
msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)

initial_lat = msg.lat
initial_lon = msg.lon

target_rad, target_lat, target_lon, target_vx, target_vy = generate_traj(initial_lat, initial_lon, 10)

msg_gps = ""
msg_battery = ""

t1 = Thread(target = getGPS)
t2 = Thread(target = getBattery)

t1.start()
t2.start()

time.sleep(1)
prev_time = current_time = int(time.time()*10)
orbit_index = DRONE1_START

short_period = False
explore_lat = 0
explore_lon = 0
while (1):
	start_time = time.time()
	message_received = readcommand()

	# get the timestamp for new command
	current_timestamp = int(message_received[58:])
	current_command = message_received[:58]
	#print("current_timestamp: "+str(current_timestamp))
	
	current_time = int(time.time()*10)

	# if not connected

	command_type = int(message_received[0])

	# if in disconnected state, once receive timestamp, go to stabilized state
	if (current_state == DISCONNECT_STATE):
		print("currently in DISCONNECT_STATE")
		if (current_timestamp != 0):
			current_state = STABALIZED_STATE
			prev_message = message_received
			print("receive timestamp, go to STABALIZED_STATE")
		else:
			print("haven't receive timestamp, stay in DISCONNECT_STATE")
		writeTelemetry(0)

	# if in stabilized state
	elif (current_state == STABALIZED_STATE):
		print("currently in STABALIZED_STATE")
		# if the current command is the same as the previous one, check if lost connection

		if(current_time - prev_time > TIMEOUT):
			current_state = DISCONNECT_STATE
			print(current_time - prev_time)
			writeTelemetry(0)
			writeBlankCommand()
		else:
			prev_message =  message_received			
			# if command is take off
			if (command_type == 1):

				print("command is takeoff")
				target_alt = int(message_received[23:29])
				ack = takeoff_procedure(target_alt)
				print(ack)
				# takeoff success, switch to guided mode
				if (ack == 0):
					current_state = GUIDED_STATE
					orbit_state = START_ORBIT
					print("takeoff success, goto GUIDED_STATE")
					ack = 2
				# takeoff failed, remain in stabilized mode
				else:
					ack = 1
			writeTelemetry(ack)
		

	elif (current_state == GUIDED_STATE):
		print("currently in GUIDED_STATE")
		if(current_time - prev_time > TIMEOUT):
			land()
			current_state = DISCONNECT_STATE
			writeTelemetry(0)
			writeBlankCommand()
			
		elif (command_type == 0):
			land()
			current_state = STABALIZED_STATE
			writeTelemetry(0)

		else:
			prev_message =  message_received

			
			#16
			if(orbit_state == START_ORBIT):
				print("in START_ORBIT state")
				orbit_index = DRONE1_START
				if (command_type == 3):
					go_pos(int(target_lat[orbit_index]), int(target_lon[orbit_index]), target_rad[orbit_index])
					orbit_state = WAIT_MOVEMENT
			#14
			elif(orbit_state == WAIT_MOVEMENT):
				print("in WAIT_MOVEMENT state")
				print((msg_gps.lat-int(target_lat[orbit_index]))**2 +(msg_gps.lon-int(target_lon[orbit_index]))**2)
				print(msg_gps.lat)
				print(target_lat[orbit_index]) 

				if ((msg_gps.lat-int(target_lat[orbit_index]))**2 +(msg_gps.lon-int(target_lon[orbit_index]))**2 <300):
					orbit_state = WAIT_ORBIT
				else: 
					go_pos(int(target_lat[orbit_index]), int(target_lon[orbit_index]), target_rad[orbit_index])
			#12
			elif(orbit_state == TWO_ORBIT):
				short_period = True
				print("in TWO_ORBIT state")
				if (command_type == 2):
					if (int(message_received[1:12]) == 0):
						orbit_state = ONE_ORBIT
					else: 
						orbit_state = EXPLRE_ORBIT
						explore_lat = int(message_received[1:12])
						explore_lon = int(message_received[12:23])
						go_pos(explore_lat, explore_lon, 0)
				else:
					print(orbit_index)
					vx, vy = calculate_speed(target_lat[orbit_index%(int(P/T))], 
									target_lon[orbit_index%(int(P/T))], 
									target_vx[orbit_index%(int(P/T))], 
									target_vy[orbit_index%(int(P/T))])
					go_vel( vx, vy, target_rad[orbit_index%(int(P/T))])
					orbit_index = (orbit_index+1)%(int(P/T))
			#13
			elif(orbit_state == EXPLRE_ORBIT):
				print("in EXPLRE_ORBIT state")
				print((msg_gps.lat-explore_lat)**2 +(msg_gps.lon-explore_lon)**2)
				if((msg_gps.lat-explore_lat)**2 +(msg_gps.lon-explore_lon)**2 <300):
					orbit_state = ARR_ROTATE
				else:
					go_pos(explore_lat, explore_lon, 0)
			#15
			elif(orbit_state == ARR_ROTATE):
				print("in ARR_ROTATE state")
				if (command_type == 3):	
					orbit_index = int(message_received[54:58])
					go_pos(int(target_lat[orbit_index]), int(target_lon[orbit_index]), target_rad[orbit_index])
					orbit_state = WAIT_MOVEMENT
				else:
					rotate(10)
			#10
			elif(orbit_state == WAIT_ORBIT):
				print("in WAIT_ORBIT state")
				if (command_type == 4):
					orbit_state = TWO_ORBIT
			#11
			elif(orbit_state == ONE_ORBIT):
				print("in ONE_ORBIT state")
				if(command_type == 3):
					orbit_index = int(message_received[54:58])
					go_pos(int(target_lat[orbit_index]), int(target_lon[orbit_index]), target_rad[orbit_index])
					orbit_state = WAIT_MOVEMENT
				else:
					vx, vy = calculate_speed(target_lat[orbit_index%(int(P/T))], 
									target_lon[orbit_index%(int(P/T))], 
									target_vx[orbit_index%(int(P/T))], 
									target_vy[orbit_index%(int(P/T))])
					go_vel( vx, vy, target_rad[orbit_index%(int(P/T))])
					orbit_index = (orbit_index+1)%(int(P/T))
		writeTelemetry(2)
	prev_command = current_command
	if (short_period):
		time.sleep(T-(time.time()-start_time))
	else:
		time.sleep(0.25)
	if(prev_timestamp != current_timestamp):
		prev_time = current_time
	prev_timestamp = current_timestamp