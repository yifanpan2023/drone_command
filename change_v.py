from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

import time

the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                        the_connection.target_component, 
                        6, 
                        3527, 0, 
                        0, 0, 0.1, 10, 0, 0, 0, 0, 0, 0))

for i in range(20):
      the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                              the_connection.target_component, 
                              6, 
                              3527, 0, 
                              0, 0, i/10, i/5, 0, 0, 0, 0, 0, 0))

      time.sleep(1)

for i in range(20):
      the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                              the_connection.target_component, 
                              6, 
                              3527, 0, 
                              0, 0, 2-i/10, 4-i/5, 0, 0, 0, 0, 0, 0))
      time.sleep(1)

for i in range(20):
      the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                              the_connection.target_component, 
                              6, 
                              3527, 0, 
                              0, 0, -i/10, -i/5, 0, 0, 0, 0, 0, 0))
      time.sleep(1)

for i in range(20):
      the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                              the_connection.target_component, 
                              6, 
                              3527, 0, 
                              0, 0, -2+i/10, -4+i/5, 0, 0, 0, 0, 0, 0))
      time.sleep(1)