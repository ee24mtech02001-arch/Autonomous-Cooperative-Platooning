#::::::::::::::::::::::::::::::::::::  LIDAR NAVIGATION CODE FOR MAINI 2 ::::::::::::::::::::::::::::::::::::#
#!/usr/bin/env python3
from pymodbus.client import ModbusTcpClient as ModbusClient
import math 
import numpy as np    
import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, UInt16, Float32, Float64, Float32MultiArray
import os
import subprocess
import signal
import sys

current_x = 0
current_y = 0
heading = 0
wp = 0
vehicle_velocity = 0
velocity_value = 0
control_effort = 0
current_steering_angle = 0
steering_output = 0
obstacle_distance = 100
global reverse_flag
reverse_flag = False
# Time tracking
start_time = time.time()
waypoints_file_1 = "/home/tihan/aroi_avoid_small/wp/waypoints.txt"
#waypoints_file_2 = "/home/infy/testbed_demo/waypoints_R.txt"  # New waypoints file for case 2
waypoints = []

# ---------------------------
# Modbus / DBW configuration
# ---------------------------
HOST = '192.168.0.5'
PORT = 502
UNIT = 0x1



# connect modbus
try:
    client = ModbusClient(HOST, PORT)
    client.connect()
    print("Modbus connected to %s:%s" % (HOST, PORT))
except Exception as e:
    rospy.logerr("Modbus connection failed: %s" % str(e))
    sys.exit(1)

emg_status = client.read_coils(60, 1, unit=UNIT)
print("Coil 60 state =", emg_status.bits[0])

# ---------------------------
# DBW utility functions (from second file)
# ---------------------------

def read_angle():
    try:
        read1 = client.read_holding_registers(address=30, count=1, unit=UNIT)
        current_angle = read1.registers[0]
        return current_angle
    except Exception as e:
        rospy.logwarn("read_angle error: %s" % str(e))
        return 0

def set_steer(angle):
    try:
        if angle < 0:
            y = abs(angle)
            client.write_coil(20, True, unit=UNIT)   # direction = negative
            client.write_registers(400, y, unit=UNIT)
        else:
            client.write_coil(20, False, unit=UNIT)  # direction = positive
            client.write_registers(400, int(angle), unit=UNIT)

        # Read back coil state for debug
        coil_state = client.read_coils(20, 1, unit=UNIT)
        #if coil_state.isError():
            #print("Error reading coil 20")
        #else:
            #print("Coil 20 state =", coil_state.bits[0])

    except Exception as e:
        rospy.logwarn("set_steer error: %s" % str(e))


    
def accelerate(value):
    try:
        client.write_registers(800, [int(value)], unit=UNIT)
    except Exception as e:
        rospy.logwarn("accelerate error: %s" % str(e))
        
def control_eff(value):      
    try:
        client.write_registers(22, int(value), unit=UNIT)
    except Exception as e:
        rospy.logwarn("control_eff error: %s" % str(e))

# -------------------------------
# Steering Test Loop (for debug)
# -------------------------------
'''try:
    while True:
        print("\nSending -30 (Left)")
        set_steer(0)
        control_eff(3500)
        time.sleep(2)

        print("\nSending +30 (Right)")
        set_steer(450)
        time.sleep(2)

except KeyboardInterrupt:
    print("Test stopped by user.")
    set_steer(0)
    control_eff(0)
    accelerate(0)
    client.close()'''

      
      
def extract_waypoints_from_file(file_path):
    try:
        with open(file_path, 'r') as file:
            file_contents = file.read()
    except FileNotFoundError:
        print("File not found.")
        return []
    except Exception as e:
        print("An error occurred: ", str(e))
        return []
    waypoints = []
    waypoints_data = file_contents.strip().split('\n')
    for line in waypoints_data:
        x, y, _ = map(float, line.strip("[],").split(','))
        waypoints.append([x, y])
    return waypoints

def callback_ndt_pose(data):
    global current_x
    global current_y
    current_x = data.pose.position.x
    current_y = data.pose.position.y
    print("Current co-ordinates : ", current_x, ", ", current_y)

def callback_cur_steer(data):
    global current_steering_angle
    current_steering_angle = data.data

def callback_vehicle_speed(data):
    global vehicle_velocity
    vehicle_velocity = data.data

def callback_min_distance(data):
    global obstacle_distance
    obstacle_distance = data.data

def callback_eular_angle(data):
    global heading
    heading = math.degrees(data.data[2]) 

def listener():
    rospy.init_node("topic_subscriber")
    rospy.Subscriber("/ndt_pose", PoseStamped, callback_ndt_pose)
    rospy.Subscriber("/eular_angle", Float32MultiArray, callback_eular_angle)
    rospy.Subscriber("/currentSteerAngle", Float64, callback_cur_steer)
    rospy.Subscriber("/lidar_min_distance_topic", Float32, callback_min_distance)
    rospy.Subscriber("/vehicle_speed", Float64, callback_vehicle_speed)

def calc_steer_output(required_bearing, Ld_steer):
    global steering_output
    current_bearing = float(heading)
    bearing_diff  = required_bearing - current_bearing

    if (bearing_diff < -180):
        bearing_diff = bearing_diff + 360

    if (bearing_diff > 180):
        bearing_diff = bearing_diff - 360

    steering_output = 40 * np.arctan(-1 * 2 * 3.5 * np.sin(np.pi * bearing_diff / 180) / Ld_steer)

def calc_velocity():
   # if reverse_flag == True:
    #    scale1 = -0.70
     #   scale2 = -0.70
    #else:
    scale1 = 0.9
    scale2 = 0.9
    velocity_steer = 55 + 45 * (30 - abs(steering_output)) / 30
    #velocity_steer = 2 * (30 - abs(steering_output)) + 40
    print(" velocity for steering           ",velocity_steer)
    velocity_brake = max(0, 100 - 100 * math.exp(-2.5 * (obstacle_distance - 2.5) / (15)))
    velocity_steer1 = velocity_steer * scale2
    velocity_brake = velocity_brake * scale1
    #return min(velocity_steer, velocity_brake)
    return velocity_steer1

def calc_control_effort():
    global control_effort
    control_effort = 2100 + abs(steering_output) * 60
      
def find_nearest_waypoint_index(waypoints):
    distance_threshold = 2
    min_distance = float("inf")
    next_index = -1
    for i in range(len(waypoints) - 1):
        waypoint = waypoints[i]
        waypoint_x, waypoint_y = waypoint
        distance = ((waypoint_x - current_x) ** 2 + (waypoint_y - current_y) ** 2) ** 0.5
        if distance < min_distance and distance < distance_threshold: 
            min_distance = distance
            next_index = i
    print("The next nearest waypoint index is : ", next_index)
    return(next_index)

# Load initial waypoints

#wp = find_nearest_waypoint_index(waypoints)
print("                                             ", wp)
# Function to switch waypoints based on elapsed time
'''def switch_waypoints(case):
    global waypoints, wp, reverse_flag
    if case == 1:
        waypoints = extract_waypoints_from_file(waypoints_file_1)
    elif case == 2:
        waypoints = extract_waypoints_from_file(waypoints_file_2)
        reverse_flag = True
    wp = find_nearest_waypoint_index(waypoints)'''

# Cleanup function to be called on termination signals
def cleanup(signum, frame):
    """
    Signal handler to set control effort and velocity to zero
    when the script is terminated.
    """
    print("\nCtrl+C detected! Stopping the vehicle...")
    accelerate(0)
    control_eff(0)
    set_steer(0)  # Center the steering for safety.
    client.close()
    print("Vehicle stopped. Exiting.")
    sys.exit(0)

# Register the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, cleanup)

# Main loop
listener()
rospy.wait_for_message('/ndt_pose', PoseStamped)


waypoints = extract_waypoints_from_file(waypoints_file_1)
wp = find_nearest_waypoint_index(waypoints)
print("Waypoint Index:  ", wp)
while not rospy.is_shutdown():

    
    distance = math.sqrt((current_x - waypoints[wp][0]) ** 2 + (current_y - waypoints[wp][1]) ** 2)
    emg_status = client.read_coils(60, 1, unit=UNIT)
    print("Coil 60 state =", emg_status.bits[0])

    print("Current_x : ", current_x)
    print("Current_y : ", current_y)
    print("Distance between current position and next waypoint is : ", distance)

    if (wp == len(waypoints) - 1) :
        accelerate(0)
        control_eff(0)
        set_steer(0)
        break
 
    if (emg_status.bits[0] == False):
        accelerate(0)
        control_eff(0)
        set_steer(0)

    elif (wp < len(waypoints) - 1):
        print("Obstacle distance : ", obstacle_distance)

        off_x = waypoints[wp][0] - current_x
        off_y = waypoints[wp][1] - current_y

        bearing_ppc = math.degrees(math.atan2(off_y, off_x))

        if bearing_ppc < 0:
            bearing_ppc += 360

        Ld_steer = 7
        calc_steer_output(bearing_ppc, Ld_steer)
        #steer_output= -180
        print("                                                 ",steering_output)
        set_steer(int(-steering_output))

        #calc_control_effort()
        velocity_value = calc_velocity()
        print(" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",velocity_value)
        control_effort=2100
        control_eff(int(control_effort))
        print(velocity_value)
        accelerate(int(velocity_value))

        Ld = 3.5

        if (distance < Ld) and (wp < len(waypoints)):
            wp = wp + 1
  
    print("Waypoint Index : ", wp, "..........................................................")
  
    time.sleep(0.1)

#:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::#
def send_signal_to_processes(signal_type):
    """Send the specified signal to all processes running the script."""
    try:
        result = subprocess.check_output(["pgrep", "-f", "python3 stop.py"])
        pids = result.decode().strip().split("\n")
        
        # Send the specified signal to each matching process
        for pid in pids:
            os.kill(int(pid), signal_type)
            print(f"Signal {signal_type} sent to process with PID {pid}.")
    except subprocess.CalledProcessError:
        print("Process not found.")

# The following lines are not reached if the script exits gracefully or by Ctrl+C
# They are part of the original code but will not execute in a standard run.

# Sending Ctrl+C (SIGINT) to the process
#send_signal_to_processes(signal.SIGINT)

# Sending Ctrl+Z (SIGTSTP) to the process
#send_signal_to_processes(signal.SIGTSTP)

'''
Updated by Rakshith on 3-10-24
'''
