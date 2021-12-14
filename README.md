# Robotics-2D-Mapping-DJI-Mavic-Drone
CSCI 3302 Final Project (Contributors: Etash Kalra, Ishika Patel, Ethan Meyer)

## Project Description

We have created a multi-robot simulation that leverages color detection, image analysis, shortest path Algorithms, and robot odometry to allow two robots to work together to complete a maze in uncharted territory.

The Mavic robot has the responsibility of flying over and mapping the built out maze as well as identifying both obstacles and target objects. The Mavic takes an image of our map, and then we use this pixel representation to perform pixel analysis to better understand the maze. We implemented color detection on an aerial image in order to transform our image into a workable matrix to perform path planning on. While the drone is flying, the ground robot (ePuck) is in a waiting state until prompted by the Mavik to begin navigating the maze. The ground robot then navigates along waypoints produced by our path finding algorithm.


## Applications

Applications of this final project could include autonomous robot driving based on real life obstacle data such as in military applications. In unsafe areas we could minimize human interaction by allowing robots to communicate and traverse unknown landscapes. This project started with us pitching a completely different idea, but with some creativity we are proud to deliver this Drone/Ground-Robot system implementation!


## Software Prerequisites

 Python 3
 
 Webots
 

## List of Relevant Python Packages

```bash
import pdb
import pickle
import random
import copy
import cv2  
import numpy as np  
from simple_pid import PID
```

```bash
pip install
pip3 install opencv-python
pip install numpy
```

## Webots Instructions

1. Lauch Webots Maze File
2. Press Play
3. Click Within the Simulation Window
4. The Drone should be starting up.

The following are helpful keyboard commands to navigating the Drone Flight and Camera Position:

------DRONE MOVEMENT CONTROLS-------

Keypad Up: Tilt Up

Keypad Down: Tilt Down

Keypad Left: Rotate Left

Keypad Right: Rotate Right

------DRONE CAMERA CONTROLS-------

W: Tilt Camera Up

S: Tilt Camera Down

A: Rotate Camera Left

D: Rotate Camera Right


6. Shift the Mavic Camera and Position to hover over the fill maze to capture in the camera
7. Once the map is displayed in the center of the camera simulation: Press Space Bar to take Image of the Full Map, this prompts 3 images to populate your computer to show pixel anaysis (These may pop up behind your Webots tab)
8. Press any key on those images to move past the analysis images and to start the E-Puck Simulation
9. E-puck should be processing and moving along the planned shortest path map by A*

## Project Demonstration
