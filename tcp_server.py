import socket
import os
import time
import math
from threading import Thread

STABALIZED_STATE = 0
GUIDED_STATE = 1

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
WAIT_ORBIT = 7

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
START_ORBIT = 6

R = 6373.0
T = 0.1
P = 60

Kp = 0.5
Kd = 2

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
    current_vx = vx
    current_vy = vy
    current_y = lon
    current_x = lat

    new_vx = Kp*(target_lat-current_x)+Kd*(target_vx-current_vx)
    new_vy = Kp*(target_lon-current_y)+Kd*(target_vy-current_vy)
    return new_vx, new_vy

HOST = '192.168.56.106' # Server IP or Hostname
PORT = 12345 # Pick an open Port (1000+ recommended), must match the client sport
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print ('Socket created')

#managing error exception
try:
	s.bind((HOST, PORT))
except socket.error:
    print ('Bind failed ')

s.listen(2)
print ('Socket awaiting messages')
(conn, addr) = s.accept()
print ('Connected')
print (addr)
# awaiting for message
lat = 0
lon = 0
vx = 0
vy = 0
def readTelemetry():
    global lat, lon, vx, vy
    while(1):
        telemetry = ""
        while(len(telemetry)<5):
            f = open("//192.168.56.101/share/telemetry.txt", "r")
            telemetry = f.read()
            f.close()
        telemetry = telemetry.split()
        lat = int(telemetry[0])
        lon = int(telemetry[1])
        vx = int(telemetry[2])
        vy = int(telemetry[3])
        time.sleep(0.02)
t1 = Thread(target = readTelemetry)

t1.start()
time.sleep(1)

initial_lat = lat
initial_lon = lon

target_rad, target_lat, target_lon, target_vx, target_vy = generate_traj(initial_lat, initial_lon, 10)

orbit_index = DRONE1_START
current_state = STABALIZED_STATE

def receive_command():
    command = ""
    while(len(command)<1):
        f = open("command.txt", "r")
        command = f.read()
        f.close()
    return command

while True:
    start_time = time.time()
    command = receive_command()
    print(command)
    if (command[0] == "0"):
        conn.send(command.encode())
        current_state = STABALIZED_STATE

    elif(current_state == STABALIZED_STATE and command[0] == "1"):
        conn.send(command.encode())
        print("send takeoff")
        current_state = GUIDED_STATE
        orbit_state = START_ORBIT

    elif (current_state == GUIDED_STATE):
        if(orbit_state == START_ORBIT):
            print("in START_ORBIT state")
            orbit_index = DRONE1_START
            if (command[0] == '3'):
                conn.send(("2"+str(int(target_lat[orbit_index])).zfill(11) +
                           str(int(target_lon[orbit_index])).zfill(11)+str(target_rad[orbit_index]).zfill(6)).encode())
                orbit_state = WAIT_MOVEMENT

        elif(orbit_state == WAIT_MOVEMENT):
            print("in WAIT_MOVEMENT state")
            print((lat-int(target_lat[orbit_index]))**2 +(lon-int(target_lon[orbit_index]))**2)
            print(lat)
            print(target_lat[orbit_index]) 

            if ((lat-int(target_lat[orbit_index]))**2 +(lon-int(target_lon[orbit_index]))**2 <3000):
                orbit_state = WAIT_ORBIT
            else: 
                conn.send(("2"+str(int(target_lat[orbit_index])).zfill(11) +
                           str(int(target_lon[orbit_index])).zfill(11)+str(target_rad[orbit_index]).zfill(6)).encode())
        elif(orbit_state == TWO_ORBIT):
            short_period = True
            print("in TWO_ORBIT state")
            if (command[0] == "2"):
                if (int(command[1:12]) == 0):
                    orbit_state = ONE_ORBIT
                else: 
                    orbit_state = EXPLRE_ORBIT
                    explore_lat = int(command[1:12])
                    explore_lon = int(command[12:23])
                    conn.send(("2"+str(explore_lat).zfill(11) +
                           str(explore_lon).zfill(11)+str(0).zfill(6)).encode())
            else:
                print(orbit_index)
                vx, vy = calculate_speed(target_lat[orbit_index%(int(P/T))], 
                                target_lon[orbit_index%(int(P/T))], 
                                target_vx[orbit_index%(int(P/T))], 
                                target_vy[orbit_index%(int(P/T))])
                conn.send(("3"+ str(int(vx*1000)).zfill(11)+
                            str(int(vy*1000)).zfill(11)+str(target_rad[orbit_index%(int(P/T))]).zfill(6)).encode())
                orbit_index = (orbit_index+1)%(int(P/T))

        elif(orbit_state == EXPLRE_ORBIT):
            print("in EXPLRE_ORBIT state")
            print((lat-explore_lat)**2 +(lon-explore_lon)**2)
            if((lat-explore_lat)**2 +(lon-explore_lon)**2 <3000):
                orbit_state = ARR_ROTATE
            else:
                conn.send(("2"+str(explore_lat).zfill(11) +
                        str(explore_lon).zfill(11)+str(0).zfill(6)).encode())
        elif(orbit_state == ARR_ROTATE):
            print("in ARR_ROTATE state")
            if (command[0] == '3'):	
                orbit_index = int(command[54:58])
                conn.send(("2"+str(int(target_lat[orbit_index])).zfill(11) +
                           str(int(target_lon[orbit_index])).zfill(11)+str(target_rad[orbit_index]).zfill(6)).encode())
                orbit_state = WAIT_MOVEMENT
            else:
                conn.send(("2"+str(10).zfill(2)).encode())
        elif(orbit_state == WAIT_ORBIT):
            print("in WAIT_ORBIT state")
            
            orbit_state = TWO_ORBIT
        elif(orbit_state == ONE_ORBIT):
            print("in ONE_ORBIT state")
            if(command[0] == '3'):
                orbit_index = int(command[54:58])
                conn.send(("2"+str(int(target_lat[orbit_index])).zfill(11) +
                           str(int(target_lon[orbit_index])).zfill(11)+str(target_rad[orbit_index]).zfill(6)).encode())
                orbit_state = WAIT_MOVEMENT
            else:
                vx, vy = calculate_speed(target_lat[orbit_index%(int(P/T))], 
                                target_lon[orbit_index%(int(P/T))], 
                                target_vx[orbit_index%(int(P/T))], 
                                target_vy[orbit_index%(int(P/T))])
                conn.send(("3"+ str(int(vx*1000)).zfill(11)+
                            str(int(vy*1000)).zfill(11)+str(target_rad[orbit_index%(int(P/T))]).zfill(6)).encode())
                orbit_index = (orbit_index+1)%(int(P/T))
    time.sleep(T-(time.time()-start_time))
conn.close() # Close connections