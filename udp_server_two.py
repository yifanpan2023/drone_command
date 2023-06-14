import socket
import os
import time
import math
from threading import Thread
import matplotlib.pyplot as plt

STABALIZED_STATE = 0
GUIDED_STATE = 1

R = 6373.0
T = 0.5
P = 60

DRONE1_START = 0
DRONE2_START = int(P/T/2)
LANDING_COMMAND = "0"
TAKEOFF_COMMAND = "1"
EXPLORE_COMMAND = "2"
ORBIT_COMMAND = "3"
CONFIRM_ORBIT_COMMAND = "4"
RADAR_COMMAND = "5"
BIRD_COMMAND = "6"

DEAFAULT_ALT = 3

TAKEOFF_SUCCESS = 2
TAKEOFF_FAIL = 1

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

#current drone is at its orbit point,wait the other drone to be ready
WAIT_ORBIT = 7

#rotate at current orbit point
RADAR_MODE = 8

#One drone goes high rotate, and the other drone keep orbiting 
BIRD_MODE = 9

#On its way to the desired height
WAIT_VERTICLE = 10

Kp = 0.5
Kd = 2

RADIUS = 10

def generate_traj(initial_lat, initial_lon, radius):
    total_steps = int(P/T)
    rad = [0.0]
    lat = [initial_lat+radius*10000/111.32]
    lon = [initial_lon]
    for i in range(total_steps-1):
        rad = rad + [(i+1)*math.pi/(P/T/2)]
        lat = lat + [(initial_lat+radius*math.cos(rad[i+1]) *10000/111.32)]
        lon = lon + [(initial_lon+radius*math.sin(rad[i+1]) *10000/111.32)]

    return rad, lat, lon


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print ('Socket created')


lat_1 = 0
lon_1 = 0
yaw_1 = 0
alt_1 = 0 
lat_2 = 0
lon_2 = 0
yaw_2 = 0
alt_2 = 0
def readTelemetry1():
    while(1):
        telemetry = ""
        while(len(telemetry)<5):    
            f = open("//192.168.56.101/share/telemetry.txt", "r")
            telemetry = f.read()
            f.close()
        telemetry = telemetry.split()
        global lat_1, lon_1, alt_1, yaw_1
        lat_1 = int(telemetry[0])
        lon_1 = int(telemetry[1])
        alt_1 = float(telemetry[2])
        yaw_1 = float(telemetry[3])
        time.sleep(0.02)

def readTelemetry2():
    while(1):
        telemetry = ""
        while(len(telemetry)<5):    
            f = open("//192.168.56.103/share/telemetry.txt", "r")
            telemetry = f.read()
            f.close()
        telemetry = telemetry.split()
        global lat_2, lon_2, alt_2, yaw_2
        lat_2 = int(telemetry[0])
        lon_2 = int(telemetry[1])
        alt_2 = float(telemetry[2])
        yaw_2 = float(telemetry[3])
        time.sleep(0.02)
t1 = Thread(target = readTelemetry1)
t2 = Thread(target = readTelemetry2)
t1.start()
t2.start()
time.sleep(1)

initial_lat = lat_1
initial_lon = lon_1

target_rad, target_lat, target_lon = generate_traj(initial_lat, initial_lon, 10)

orbit_index = DRONE1_START
current_state = STABALIZED_STATE

def receive_command():
    command = ""
    while(len(command)<1):
        f = open("command.txt", "r")
        command = f.read()
        f.close()
    return command

def send_position(lat, lon, alt, rad,  ip, port):
    s.sendto(("2"+str(int(lat)).zfill(11) + 
              str(int(lon)).zfill(11)+str(round(rad, 3)).zfill(6) + str(int(alt)).zfill(2)).encode(), (ip, port))
    
class drone_machine:
    def __init__(self, drone_id, phone_lat, phone_lon):
        self.drone_id = drone_id
        if drone_id == 1:
            self.orbit_index = 0
            self.ip = '192.168.56.101'
            self.port = 1111
            self.lat = lat_1
            self.lon = lon_1
            self.yaw = yaw_1
            self.alt = alt_1
        else:
            self.orbit_index = int(P/T/2)
            self.ip = '192.168.56.103'
            self.port = 2222
            self.lat = lat_2
            self.lon = lon_2
            self.yaw = yaw_2
            self.alt = alt_2
        self.current_state = STABALIZED_STATE
        self.target_rad, self.target_lat, self.target_lon = generate_traj(phone_lat, phone_lon, RADIUS)
        self.orbit_state = START_ORBIT
        self.explore_lat = self.lat
        self.explore_lat = self.lon
        self.explore = False
        self.target_alt = DEAFAULT_ALT
        self.phone_lat = phone_lat
        self.phone_lon = phone_lon

    def update_telemetry(self):
        if self.drone_id == 1:
            self.lat = lat_1
            self.lon = lon_1
            self.yaw = yaw_1
            self.alt = alt_1
        else:
            self.lat = lat_2
            self.lon = lon_2
            self.yaw = yaw_2
            self.alt = alt_2

    def go_orbit_pos(self):
        print("drone "+str(self.drone_id)+" go orbit")
        print(self.orbit_index)
        send_position(self.target_lat[self.orbit_index],
                      self.target_lon[self.orbit_index],
                      self.target_alt,
                      self.target_rad[self.orbit_index],
                      self.ip, self.port)
        
    def send_explore_pos(self):
        send_position(self.explore_lat, self.explore_lon, self.target_alt, 0.0, self.ip, self.port)

    def send_rotate(self, period):
        s.sendto(("4"+str(period).zfill(3)+str(self.target_alt).zfill(2)).encode(), (self.ip, self.port))

    def check_arrive_orbit(self):
        
        print("target:")
        print(int(self.target_lat[self.orbit_index]))
        print(int(self.target_lon[self.orbit_index]))
        print("current:")
        print(int(self.lat))
        print(int(self.lon))
        
        return (self.lat-int(self.target_lat[self.orbit_index]))**2 + (self.lon-int(self.target_lon[self.orbit_index]))**2
    
    def check_arrive_explore(self):
        """
        print("target:")
        print(int(self.explore_lat))
        print(int(self.explore_lon))
        print("current:")
        print(int(self.lat))
        print(int(self.lon))
        """
        return (self.lat-self.explore_lat)**2 + (self.lon-int(self.explore_lon))**2
    
    def check_arrive_height(self):
        """
        print("target:")
        print(int(self.explore_lat))
        print(int(self.explore_lon))
        print("current:")
        print(int(self.lat))
        print(int(self.lon))
        """
        return (self.alt-self.target_alt)**2
    
    def handle_command(self, command):
        self.update_telemetry()
        if (command[0] == "0"):
            #land command
            s.sendto(command.encode(),(self.ip, self.port))
            self.current_state = STABALIZED_STATE

        elif(self.current_state == STABALIZED_STATE and command[0] == "1"):
            # takeoff
            s.sendto(command.encode(),(self.ip, self.port))
            self.current_state = GUIDED_STATE
            self.orbit_state = START_ORBIT

        elif (self.current_state == GUIDED_STATE):
            if(self.orbit_state == START_ORBIT):
                #start orbit, go to the initial point
                print("in START_ORBIT state")
                if (command[0] == '3'):
                    self.go_orbit_pos()
                    self.orbit_state = WAIT_MOVEMENT

            elif(self.orbit_state == WAIT_MOVEMENT):
                # wait until arrive
                print("in WAIT_MOVEMENT state")
                if (self.check_arrive_orbit() <3000):
                    self.orbit_state = WAIT_ORBIT
                else: 
                    self.go_orbit_pos()

            elif(self.orbit_state == TWO_ORBIT):
                self.explore = False
                print("in TWO_ORBIT state")
                if (command[0] == EXPLORE_COMMAND):
                    #no target, keep orbiting
                    if (int(command[1:11]) == 0):
                        self.orbit_state = ONE_ORBIT
                    else: 
                        print(command)
                        print(str(self.drone_id)+"go explore!!!!!!!!!!!!!!!!!!!!")
                        #receive target, start explore
                        self.orbit_state = EXPLRE_ORBIT
                        self.explore_lat = int(command[1:11])
                        #print(self.explore_lat)
                        self.explore_lon = int(command[12:23])
                        #print(self.explore_lon)
                        self.send_explore_pos()

                elif (command[0] == RADAR_COMMAND):
                    self.go_orbit_pos()
                    self.orbit_state = RADAR_MODE

                elif (command[0] == BIRD_COMMAND):
                    if (self.drone_id == 1):
                        print(command)
                        print(str(self.drone_id)+"go explore!!!!!!!!!!!!!!!!!!!!")
                        #receive target, start explore
                        self.orbit_state = EXPLRE_ORBIT
                        self.explore_lat = self.target_lat[self.orbit_index]
                        #print(self.explore_lat)
                        self.explore_lon = self.target_lon[self.orbit_index]
                        self.target_alt = 5
                        #print(self.explore_lon)
                        self.send_explore_pos()
                    else:
                        self.orbit_state = ONE_ORBIT
                else:
                    #no explore command, stay in orbit
                    print(self.orbit_index)
                    self.go_orbit_pos()
                    self.orbit_index = (self.orbit_index+1)%(int(P/T))

            elif(self.orbit_state == EXPLRE_ORBIT):
                #if arrive, start to rotate
                if(self.check_arrive_explore() <3000):
                    self.orbit_state = ARR_ROTATE
                else:
                    self.send_explore_pos()

            elif(self.orbit_state == ARR_ROTATE):
                print("in ARR_ROTATE state")
                if (command[0] == ORBIT_COMMAND):	
                    #if command is orbit, go to orbit position
                    self.target_alt = DEAFAULT_ALT
                    self.orbit_index = int(command[55:59])
                    self.go_orbit_pos()
                    self.orbit_state = WAIT_MOVEMENT
                else:
                    self.send_rotate(10)
            elif(self.orbit_state == WAIT_ORBIT):
                print("in WAIT_ORBIT state")
                if(command[0] == CONFIRM_ORBIT_COMMAND):
                    self.orbit_state = TWO_ORBIT
                else:
                    self.go_orbit_pos()

            elif(self.orbit_state == ONE_ORBIT):
                print("in ONE_ORBIT state")
                if(command[0] == ORBIT_COMMAND):
                    self.orbit_index = int(command[55:59])
                    print("drone "+str(self.drone_id))
                    print(orbit_index)
                    self.go_orbit_pos()
                    self.orbit_state = WAIT_MOVEMENT
                else:
                    self.go_orbit_pos()
                    self.orbit_index = (self.orbit_index+1)%(int(P/T))
            elif(self.orbit_state == RADAR_MODE):
                if(command[0] == ORBIT_COMMAND):
                    self.go_orbit_pos()
                    self.orbit_state = TWO_ORBIT
                else:
                    print("radar rotating")
                    self.send_rotate(10)

command = receive_command()
print(int(command[32:42]))
print(int(command[43:54]))
d1 = drone_machine(1, int(command[32:42]), int(command[43:54]))
d2 = drone_machine(2, int(command[32:42]), int(command[43:54]))

print(d1.lat)
print(d2.lat)
print(d1.lon)
print(d2.lon)

plt.plot(d1.target_lat, d1.target_lon)
plt.plot(d2.target_lat, d2.target_lon)
plt.show()

while True:
    start_time = time.time()
    command = receive_command()
    print(len(command)) 

    if(d1.orbit_state == d2.orbit_state and d1.orbit_state == WAIT_ORBIT):
        drone1_command = '4' +command[1:]
        drone2_command = drone1_command
    elif (command[0] == '2' and d1.orbit_state == d2.orbit_state and d1.orbit_state == TWO_ORBIT):
        explore_lat = int(command[1:11])
        explore_lon = int(command[12:23])
        if((d1.lat - explore_lat)**2 + (d1.lon - explore_lon)**2 <
           (d2.lat - explore_lat)**2 + (d2.lon - explore_lon)**2):
            print("drone1 explore")
            d1.explore = True
            drone1_command = command
            drone2_command = '2' + '0'*30
        else:
            print("drone2 explore")
            d2.explore = True
            drone2_command = command
            drone1_command = '2' + '0'*30    
    elif (command[0] == '3' and (d1.orbit_state == ONE_ORBIT or d2.orbit_state == ONE_ORBIT)):
        if (d1.orbit_state == ONE_ORBIT):
            print("drone1 goes back")
            
            orbit_index = d2.orbit_index
            print(orbit_index)
            drone1_command = command + str(int((orbit_index+P/T/2)%(int(P/T)))).zfill(4)
            drone2_command = command + str(int(orbit_index)).zfill(4)
        else:
            print("drone2 goes back")
            orbit_index = d1.orbit_index
            drone2_command = command + str(int((orbit_index+P/T/2)%(int(P/T)))).zfill(4)
            drone1_command = command + str(int(orbit_index)).zfill(4)
    else:
        print("normal state")
        drone1_command = command
        drone2_command = command
    
    d1.handle_command(drone1_command)
    d2.handle_command(drone2_command)
    time.sleep(T-(time.time()-start_time))
