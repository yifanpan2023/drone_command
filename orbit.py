from pymavlink import mavutil
import math
import time
R = 6373.0
# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))
time_run = time.time()
msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
initial_lat = (msg.lat)
initial_lon = (msg.lon)

# this function retrun a list of rad, list of lat and list of lon about target location
def generate_traj(initial_lat, initial_lon, radius):
    rad = [0]
    lat = [initial_lat+radius*10000/111.32]
    lon = [initial_lon]
    for i in range(119):
        rad = rad + [(i+1)*math.pi/60]
        lat = lat + [(initial_lat+radius*math.cos(rad[i+1]) *10000/111.32)]
        lon = lon + [(initial_lon+radius*math.sin(rad[i+1]) *10000/111.32)]
    
    return rad, lat, lon

# this funciton will calculate the velocity by v = dx/t
def calculate_speed(start_lat, start_lon, target_lat, target_lon, interval, offset_lon, offset_lat):
    diff_x = (target_lat - start_lat + offset_lat)/10000*111.32
    diff_y = (target_lon - start_lon + offset_lon)/10000*111.32
    print(target_lat, start_lat)
    print(target_lon, start_lon)
    return diff_x/interval, diff_y/interval

traj_yaw, traj_lat, traj_lon = (generate_traj(initial_lat,initial_lon,10))

print(calculate_speed(traj_lat[0],traj_lon[0],traj_lat[1],traj_lon[1],1,0,0))

# go to a point on circle, while leaving some space to accelerate
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                         the_connection.target_component, 
                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                         int(0b110111111000), int(traj_lat[0]), 
                        int(traj_lon[0]-1.5/10000*111.32), 10, 0, 0, 0, 0, 0, 0, 0, 0))
time.sleep(10)

# orbit by sending command to go to targt points
for i in range(120):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                            the_connection.target_component, 
                            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                            int(0b110111111000), int(traj_lat[i]), 
                            int(traj_lon[i]), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
    time.sleep(0.8)
msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)


print(int(traj_lat[0]),traj_lon[0]-1.5/10000*111.32)

# go to a point on circle, while leaving some space to accelerate

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                         the_connection.target_component, 
                         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                         int(0b110111111000), int(traj_lat[0]), 
                        int(traj_lon[0]-1.5/10000*111.32), 10, 0, 0, 0, 0, 0, 0, 0, 0))
start_time = time.time()
while(time.time()<start_time+3):
    msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
    print(msg.lat, msg.lon)
    print(int(traj_lat[0]), int(traj_lon[0]-1.5/10000*111.32))
    print("------------------")
print(msg.lat, msg.lon)
print("==============")

# start orbiting by velocity control
t = 0
while 1:
    if(t == 0):
        the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                            the_connection.target_component, 
                            6, 
                            3527, 0, 
                            0, 0, 0, 5, 0, 0, 0, 0, 0, 0))
        time.sleep(0.7)
        t = t+1
        msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
        print(msg.vx, msg.vy)
    print(t)
    time_run = time.time()
    msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)

    v_x, v_y = calculate_speed(msg.lat, msg.lon, traj_lat[t%120], traj_lon[t%120],1,0,0 )
    print(v_x, v_y)

    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                            the_connection.target_component, 
                            7, 
                            3527, 0, 
                            0, 0, v_x, v_y, 0, 0, 0, 0, 0, 0))
    #print(time.time()-time_run)
    msg = the_connection.recv_match( type='GLOBAL_POSITION_INT', blocking=True)
    print(msg.vx, msg.vy)
    t = t+1
    time.sleep(1-time.time()+time_run)
    

