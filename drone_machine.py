from pymavlink import mavutil
import numpy as np
import os
import time

DISCONNECT_STATE = -1
STABALIZED_STATE = 0
GUIDED_STATE = 1

EMPTY_FILE = "9"+"0"*55

TIMEOUT = 100

LANDING_COMMAND = 0
TAKEOFF_COMMAND = 1
GOTO_COMMAND =2
ORBIT_COMMAND = 3
LISTEN_COMMAND = 9

DEAFAULT_ALT = 3

TAKEOFF_SUCCESS = 2
TAKEOFF_FAIL = 1


class Drone:
    def __init__(self):
        self.phase = 0
        self.conn = mavutil.mavlink_connection('udpin:localhost:14551')
        self.conn.wait_heartbeat()
        writeBlankCommand()

    def arm(self):
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
        return (msg.result)

    def disarm(self):
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)


    def guided(self):
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)

        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
        return (msg.result)

    def takeoff(self, altitude):
        # last param: altitude
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, int(altitude))

        msg = self.conn.recv_match(type='COMMAND_ACK', blocking=True)
        return (msg.result)

    def takeoff_procedure(self, altitude):
        if (self.guided() > 0):
            return 1
        if (self.arm() > 0):
            return 1
        if (self.takeoff(altitude) > 0):
            return 1
        return 0

    def land(self):
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 9, 0, 0, 0, 0, 0)

    def writeTelemetry(self, ack):

        battery_info = self.conn.recv_match(
            type='BATTERY_STATUS', blocking=True).battery_remaining

        gps_info = self.conn.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True)
        gps_lat, gps_long, alt = gps_info.lat, gps_info.lon, gps_info.relative_alt

        yaw = self.conn.recv_match(type='ATTITUDE', blocking=True).yaw

        telemetry = str(ack)+str(battery_info)+str(gps_lat).zfill(11)+str(
            gps_long).zfill(11)+str(alt).zfill(6)+str(yaw).zfill(6)+str(time.time()*1000)
        
        f = open("log.txt", "w")
        f.write(telemetry)
        f.close()

    def set_target(self, lat, lon, alt):
        self.conn.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                10, self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                int(0b110111111000), int(lat), int(lon), int(alt), 0, 0, 0, 0, 0, 0, 1.57, 0.5))
        
def writeBlankCommand():
    f = open("/share/command.txt", "w")
    f.write(EMPTY_FILE)
    f.close()



    
def writeBlankCommand():
    f = open("/share/command.txt", "w")
    f.write(EMPTY_FILE)
    f.close()


def readcommand():
    f = open("/share/command.txt", "r")
    content = f.read()
    f.close()
    return content

if __name__ == "__main__":
    from time import sleep
    d = Drone()
    print(time.time())
    current_state = DISCONNECT_STATE
    prev_timestamp = 0
    current_timestamp = 0
    prev_time = 0
    current_time = 0
    ack = 0
    prev_message = ""
    prev_command = ""
    current_command = ""

    while (1):
        sleep(1)
        message_received = readcommand()

        # get the timestamp for new command
        current_timestamp = int(message_received[54:])
        current_command = message_received[:54]
        print("current_timestamp: "+str(current_timestamp))
        
        current_time = int(time.time()*10)

        # if not connected

        command_type = int(message_received[0])

        # if in disconnected state, once receive timestamp, go to stabilized state
        if (current_state == DISCONNECT_STATE):
            print("currently in DISCONNECT_STATE")
            if (current_timestamp != 0):
                current_state = STABALIZED_STATE
                prev_message = message_received
                prev_time = time.time()
                print("receive timestamp, go to STABALIZED_STATE")
            else:
                print("haven't receive timestamp, stay in DISCONNECT_STATE")
            d.writeTelemetry(0)

        # if in stabilized state
        elif (current_state == STABALIZED_STATE):
            print("currently in STABALIZED_STATE")
            # if the current command is the same as the previous one, check if lost connection
            if (message_received == prev_message):

                d.writeTelemetry(0)
            else:
                prev_message =  message_received
                prev_time =  current_time
                
                # if command is take off
                if (command_type == 1):

                    print("command is takeoff")
                    target_alt = int(message_received[23:29])
                    ack = d.takeoff_procedure(target_alt)
                    # takeoff success, switch to guided mode
                    if (ack == 0):
                        current_state = GUIDED_STATE
                        print("takeoff success, goto GUIDED_STATE")
                        ack = 2
                    # takeoff failed, remain in stabilized mode
                    else:
                        ack = 1
                d.writeTelemetry(ack)
            

        elif (current_state == GUIDED_STATE):
            print("currently in GUIDED_STATE")
            if (message_received == prev_message):
                d.writeTelemetry(0)
            else:
                prev_message =  message_received
                prev_time =  current_time
                if(current_command != prev_command):
                    if (command_type == 2):
                        target_lat = message_received[1:12]
                        target_lon = message_received[12:23]
                        target_alt = message_received[23:29]
                        d.set_target(target_lat, target_lon, target_alt)
                        
                    elif (command_type == 3):
                        print("orbit")
                        
                    elif (command_type ==0):
                        d.land() 
                        current_state = STABALIZED_STATE
       
                
            d.writeTelemetry(0)
        prev_command = current_command
