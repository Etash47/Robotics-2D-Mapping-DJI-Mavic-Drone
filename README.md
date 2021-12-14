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

## Project Demonstration
