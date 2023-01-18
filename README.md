# drone_command
install SITL Simulator: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
install pymavlink: https://mavlink.io/en/mavgen_python/
install qground control: http://qgroundcontrol.com/
enviroment: https://www.youtube.com/watch?v=1FpJvUVPxL0&t=0s

# how to run code:
1. start sim_viechle.py -v arducopter to create a simulated drone
2. run QgroundControl to start the simulator
3. run code in this repo, starting from guided, and takeoff

# movement command
follow instruction on https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED for local movement (based on starting point)
and https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT for global (gps based) movement

type_mask for movement command can be found at https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK

# recv_match
GLOBAL_POSITION_INT: GPS information of drone 
LOCAL_POSITION_NED: position reated to starting point
BATTERY_STATUS: battery info

more filters, see https://mavlink.io/en/messages/common.html#MESSAGE_INTERVAL

# change speed, yaw, altidude
MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_CONDITION_YAW, MAV_CMD_DO_CHANGE_ALTITUDE 
https://mavlink.io/en/messages/common.html#MAV_FRAME_LOCAL_NED

change speed after the destination is set. In set target, velocity will be set to default (10)

# for changing of flight mode,
see https://ardupilot.org/copter/docs/parameters.html#fltmode1
