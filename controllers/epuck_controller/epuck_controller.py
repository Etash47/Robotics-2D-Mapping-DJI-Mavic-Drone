"""epuck_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import sys
import math
import numpy as np
import copy

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.125669815
MAX_SPEED = 6.28

# Ishika and I found that 1.5 was most optimal for smoothness
SPEED_DIVISOR = 1

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Set wheel velocities as a function of radians
vL = MAX_SPEED / SPEED_DIVISOR
vR = MAX_SPEED / SPEED_DIVISOR

# Useful constants
pi = math.pi
heading_threshold = pi/8

# Odometry initialization
pose_x     = 0
pose_y     = 0
pose_theta = 0

# Goals
goal_x = 0
goal_y = 0

# Sleep function from lab 1
def sleep(duration):
    global robot
    end_time = robot.getTime() + duration

    while robot.step(timestep) != -1 and robot.getTime() < end_time:
        pass


import numpy as np


def get_meter_from_pixel(pixel, pixel_height, pixel_width, meter_height, meter_width):
    # Function that converts pixel to meter coordinates

    pixel_y = pixel[0]
    pixel_x = pixel[1]

    meter_y = pixel_y * (meter_height / pixel_height)
    meter_x = pixel_x * (meter_width / pixel_width)

    return (meter_x, -meter_y)

def retrieve_waypoints():

    pixel_path = np.load(sys.path[0] + '/../mavic2proPython/path.npy')
    cropped_map = np.load(sys.path[0] + '/../mavic2proPython/map.npy')

    pixel_height = cropped_map.shape[0]
    pixel_width = cropped_map.shape[1]

    meter_height = 3.3  # Based upon distance between green circles on map
    meter_width = 3.3

    # Determining path relative to start point
    start_pixel = pixel_path[0]
    relative_pixel_path = pixel_path - start_pixel

    # Last pixel of relative path
    goal = relative_pixel_path[-1]

    sample_rate = 20
    # Sampling every _ pixels
    sampled_pixel_path = relative_pixel_path[0::sample_rate]

    # Ensuring goal is the last point
    pixel_waypoints = list(sampled_pixel_path)
    pixel_waypoints.append(goal)
    # print(pixel_waypoints)

    # Applying function to each pixel to translate to meters
    meter_waypoints = [get_meter_from_pixel(px, pixel_height, pixel_width, meter_height, meter_width) for px in
                       pixel_waypoints]

    return meter_waypoints

waypoints = []
waypoint_index = 0
num_waypoints = 0
goal_waypoint_index = 0

# Function that provides theta dot and r dot based on error sources
def get_theta_dot_and_r_dot(euclid_error, bearing_error):
  # Case that we are far away and not pointed in the right direction of travel
  if (abs(bearing_error) > math.pi/5 and euclid_error > 0.1):
    #print("======== ALIGNING ============")
    X_dot_R = 0
    theta_dot = bearing_error # Prioritize correcting bearing error
    return(X_dot_R, theta_dot)

  # Case that we are very close, but not oriented in the right direction
  #elif(euclid_error < 0.05):
    #print("======== ORIENTING ============")
   # X_dot_R = 0
    #theta_dot = heading_error # Prioritize correcting heading error
    #return(X_dot_R, theta_dot)

  else :
    #print("======== DRIVING ============")
    X_dot_R = euclid_error
    theta_dot = bearing_error     # Equally balance distance and bearing error
    return(X_dot_R, theta_dot)

# Intial state is wait
state = 'wait'

while robot.step(timestep) != -1:

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commasnds, like:
    #  motor.setPosition(10.0)

    if state == 'wait':

        f = open(sys.path[0] + '/../communicator.txt', 'r')

        f.seek(0)

        file_value = int(f.read())

        if file_value == 0:
            #print("ePuck is Waiting")
            sleep(1)
            continue

        if file_value == 1:
            state = 'running'
            wf = open(sys.path[0] + '/../communicator.txt', 'w')
            wf.write("0")
            print("------Starting up ePuck Navigation!-------")
            waypoints = retrieve_waypoints()
            

            startpoint = waypoints[0]
            endpoint = waypoints[-1]

            pose_x = startpoint[0]
            pose_y = startpoint[1]

            goal_x = endpoint[0]
            goal_y = endpoint[1]

            waypoint_index = 1
            num_waypoints = len(waypoints)
            goal_waypoint_index = num_waypoints - 1

    if state == 'running':

        #print("ePuck is Running!")
        #print("Current Waypoint:",waypoints[waypoint_index])

        goal_x = waypoints[waypoint_index][0]
        goal_y = waypoints[waypoint_index][1]
        goal_theta = 0

        # STEP 2: Calculate sources of error
        euclid_error = math.sqrt((goal_x - pose_x) ** 2 + (goal_y - pose_y) ** 2)  # Euclidean distance
        bearing_error = (math.atan2((goal_y - pose_y), (goal_x - pose_x)) - pose_theta)
        #heading_error = goal_theta - pose_theta

        # Ensuring value is between -pi and pi
        if bearing_error > 6.28+3.14/2: bearing_error -= 6.28
        if bearing_error < -3.14: bearing_error += 6.28

        # Ensuring value is between -pi and pi
        #if heading_error > 6.28+3.14/2: heading_error -= 6.28
        #if heading_error < -3.14: heading_error += 6.28

        # Feedback Controller get X, theta dot values
        (X_dot_R, theta_dot) = get_theta_dot_and_r_dot(euclid_error, bearing_error)

        # Inverse Kinematics Equations
        phi_r = X_dot_R + (EPUCK_AXLE_DIAMETER * theta_dot) / 2
        phi_l = X_dot_R - (EPUCK_AXLE_DIAMETER * theta_dot)/ 2

        vL = (phi_l) / (max(abs(phi_l), abs(phi_r))) * MAX_SPEED / SPEED_DIVISOR
        vR = (phi_r) / (max(abs(phi_l), abs(phi_r))) * MAX_SPEED / SPEED_DIVISOR

        if vL > MAX_SPEED:
            vL = MAX_SPEED
        if vR > MAX_SPEED:
            vR = MAX_SPEED
        if vL < -MAX_SPEED:
            vL = -MAX_SPEED
        if vR < -MAX_SPEED:
            vR = -MAX_SPEED
        
        #print("Pose x:", pose_x, "Goal x:", goal_x)
        #print("Pose y:", pose_y, "Goal y:", goal_y)
        
        if (waypoint_index == goal_waypoint_index):
            distance_threshold = 0.05
        else:
            distance_threshold = 0.1
            

        if (euclid_error < distance_threshold):  # we are close to waypoint_index

            if (waypoint_index < goal_waypoint_index):  # waypoint is not final goal
                #print("Waypoint x:", goal_x, "Pose x:", pose_x)
                #print("Waypoint y:", goal_y, "Pose y:", pose_y)
                waypoint_index += 1  # update waypoint index

            elif (waypoint_index == goal_waypoint_index):  #
                print("------Reached Final Destination-------")
                print("Final Goal x:", goal_x, "Pose x:", pose_x)
                print("Final Goal y:", goal_y, "Pose y:", pose_y)
                print("Final Goal theta:", goal_theta, "Pose theta:", pose_theta)
                break

        distL = vL / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED * timestep / 1000.0
        distR = vR / MAX_SPEED * EPUCK_MAX_WHEEL_SPEED * timestep / 1000.0
        pose_x += (distL + distR) / 2.0 * math.cos(pose_theta)
        pose_y += (distL + distR) / 2.0 * math.sin(pose_theta)
        pose_theta += (distR - distL) / EPUCK_AXLE_DIAMETER
        # Bound pose_theta between [-pi, 2pi+pi/2]
        # Important to not allow big fluctuations between timesteps (e.g., going from -pi to pi)
        if pose_theta > 6.28 + 3.14 / 2: pose_theta -= 6.28
        if pose_theta < -3.14: pose_theta += 6.28

        leftMotor.setVelocity(vL)
        rightMotor.setVelocity(vR)

leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Enter here exit cleanup code.
