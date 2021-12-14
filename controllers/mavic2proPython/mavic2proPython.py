# LIST OF RESOURCES USED FOR UNDERSTANDING DRONE CONTROLS!
# https://github.com/cyberbotics/webots/blob/released/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c
# https://cyberbotics.com/doc/reference/motor?tab-language=python#wb_motor_set_position
# https://cyberbotics.com/doc/reference/camera?tab-language=python#wb_camera_get_image

from controller import *
from simple_pid import PID
import csv
import struct
import numpy as np
from maze_detector import *
from path_planning import *
import os.path


### WRITE TO A FILE TO TELL THE EPUCK TO WAIT

f = open(sys.path[0] + '/../communicator.txt', 'w')
f.write("0")


def clamp_value(val, low, high):
    # Function that clamps given value between low and high thresholds
    if val < low:
        return low
    elif val > high:
        return high
    else:
        return val

def set_motor_speeds(front_left, front_right, rear_left, rear_right):
    
    frontLeftMotor.setVelocity(front_left)
    frontRightMotor.setVelocity(front_right)
    backLeftMotor.setVelocity(rear_left)
    backRightMotor.setVelocity(rear_right)
 

params = dict() # Using parameters used in generic webots code: https://github.com/cyberbotics/webots/blob/released/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c
with open("../params.csv", "r") as f:
    lines = csv.reader(f)
    for line in lines:
        params[line[0]] = line[1]

TIME_STEP = int(params["QUADCOPTER_TIME_STEP"])
TAKEOFF_THRESHOLD_VELOCITY = int(params["TAKEOFF_THRESHOLD_VELOCITY"])
M_PI = np.pi
img_filename = "maze_img.jpg"

robot = Robot() 

# Attaching motors
frontLeftMotor = robot.getDevice('front left propeller')
frontRightMotor = robot.getDevice('front right propeller')
backLeftMotor = robot.getDevice('rear left propeller')
backRightMotor = robot.getDevice('rear right propeller')

frontLeftMotor.setPosition(float('inf'))
frontRightMotor.setPosition(float('inf'))
backLeftMotor.setPosition(float('inf'))
backRightMotor.setPosition(float('inf'))

set_motor_speeds(TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY, TAKEOFF_THRESHOLD_VELOCITY)


gps = GPS("gps")
gps.enable(TIME_STEP)
imu = InertialUnit("inertial unit")
imu.enable(TIME_STEP)
compass = Compass("compass")
compass.enable(TIME_STEP)
gyro = Gyro("gyro")
gyro.enable(TIME_STEP)

camera = Camera("camera")
camera.enable(TIME_STEP)

camera_roll_motor =  robot.getDevice('camera roll')
camera_pitch_motor =  robot.getDevice('camera pitch')
camera_yaw_motor = robot.getDevice('camera yaw')

keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

yaw_setpoint=-1

pitchPID = PID(float(params["pitch_Kp"]), float(params["pitch_Ki"]), float(params["pitch_Kd"]), setpoint=0.0)
rollPID = PID(float(params["roll_Kp"]), float(params["roll_Ki"]), float(params["roll_Kd"]), setpoint=0.0)
throttlePID = PID(float(params["throttle_Kp"]), float(params["throttle_Ki"]), float(params["throttle_Kd"]), setpoint=1)
yawPID = PID(float(params["yaw_Kp"]), float(params["yaw_Ki"]), float(params["yaw_Kd"]), setpoint=float(yaw_setpoint))


# More parameters from Webots: https://github.com/cyberbotics/webots/blob/released/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c
k_vertical_thrust = 68.5  
k_vertical_offset = 0.6   
k_vertical_p = 3.0        
k_roll_p = 50.0           
k_pitch_p = 30.0          
target_altitude = 8.0

camera_roll = 0
camera_pitch = 0
camera_yaw = 0

camera_pitch_motor.setPosition(camera_yaw)
camera_yaw_motor.setPosition(camera_pitch)
camera_roll_motor.setPosition(camera_roll)

state = "fly"

slept = 0

print("------DRONE MOVEMENT CONTROLS-------")
print("Keypad Up: Tilt Up")
print("Keypad Down: Tilt Down")
print("Keypad Left: Rotate Left")
print("Keypad Right: Rotate Right")

print(" ")

print("------DRONE CAMERA CONTROLS-------")
print("W: Tilt Camera Up")
print("S: Tilt Camera Down")
print("A: Rotate Camera Left")
print("D: Rotate Camera Right")

print(" ")

while (robot.step(TIME_STEP) != -1):
    if state == "fly":
	
        roll = imu.getRollPitchYaw()[0] + M_PI / 2.0
        pitch = imu.getRollPitchYaw()[1]
        altitude = gps.getValues()[1]
        roll_acceleration = gyro.getValues()[0]
        pitch_acceleration = gyro.getValues()[1]
        
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            #print("LEFT")
            yaw_disturbance = -1.3
        elif key == keyboard.RIGHT:
            #print("RIGHT")
            yaw_disturbance = 1.3
        elif key == keyboard.UP:
            #print("UP")
            pitch_disturbance = 2.0
        elif key == keyboard.DOWN:
            #print("DOWN")
            pitch_disturbance = -2.0
        elif key == keyboard.SHIFT + keyboard.DOWN:
            #print("DECREASING ALTITUDE")
            target_altitude -+ 0.05
        elif key == keyboard.SHIFT + keyboard.UP: 
            #print("INCREASING ALTITUDE", target_altitude)
            target_altitude += 0.1
        elif key == ord('A'):
            #print("Camera Left")
            camera_pitch += 0.01
        elif key == ord('D'):
            #print("Camera Left")
            camera_pitch -= 0.01
        elif key == ord('W'):
            #print("Camera Left")
            camera_yaw -= 0.01
        elif key == ord('S'):
            #print("Camera Left")
            camera_yaw += 0.01
        elif key == ord(' '):
            #print("taking image")
            image = camera.getImage()
            camera.saveImage(img_filename, quality = 100) # Saving image
    
            
            # Detecting image from maze
            red_centre, blue_centre = detect_maze(img_filename)
            
            # Running path planning algorithm
            path = plan_path(red_centre, blue_centre)
            np.save("path.npy", path)
            
            state = "Die"
            
            
            #set_motor_speeds(0, 0, 0, 0)
            
           
            #set_motor_speeds(10, 10, 2, 2)
        # Utilizing generic motion control provided in webots documentation: https://github.com/cyberbotics/webots/blob/released/projects/robots/dji/mavic/controllers/mavic2pro/mavic2pro.c
        roll_input = k_roll_p * clamp_value(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance;
        pitch_input = k_pitch_p * clamp_value(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance;
        yaw_input = yaw_disturbance;
        clamped_difference_altitude = clamp_value(target_altitude - altitude + k_vertical_offset, -1.0, 1.0);
        vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0);
    
        front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input;
        front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
        rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
        rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
        
        
        camera_pitch = clamp_value(camera_pitch, -1.7, 1.7)
        camera_yaw = clamp_value(camera_yaw, -0.5, 1.7)
        
        camera_pitch_motor.setPosition(camera_yaw)
        camera_yaw_motor.setPosition(camera_pitch)
        
        #print(front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
        
        set_motor_speeds(front_left_motor_input, -front_right_motor_input, -rear_left_motor_input, rear_right_motor_input)
            
         
    
    else:
    
        print(".", end = '')
        
        set_motor_speeds(70, -70, -69, 69)
        slept += 1
        
        if slept > 400:
            f2 = open(sys.path[0] + '/../communicator.txt', 'w')
            f2.write("1")
            f2.close()
            print(" ")
            
            break
            

    
    
    
