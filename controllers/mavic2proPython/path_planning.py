'''
Authors: Ethan Meyer, Etash Kalra, Ishika Patel
Purpose: Path planning functions for use in CSCI 3302 Final Profect - Fall 2021
Resources Used: 
  - CSCI 3302 Lab 5
  - A-Star Pseudocode: https://en.wikipedia.org/wiki/A*_search_algorithm

'''

import numpy as np
from matplotlib import pyplot as plt
from numpy.lib.arraypad import pad
import cv2

def path_planner(grid, start, goal):
      print("Planning...")
      '''
      :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
      :param start: A tuple of indices representing the start cell in the map
      :param end: A tuple of indices representing the end cell in the map
      :return: A list of tuples as a path from the given start to the given end in the given maze
      '''
      def euclidian_dist(a, b): # heuristic to be used in Astar
        return np.linalg.norm(a - b)

      def is_point_valid(point, num_rows, num_cols): # Function that determines if a given point is valid or not
        if (0 <= point[0] and point[0] < num_rows - 1):
          if(0 <= point[1] and point[1] < num_cols - 1):
            return True
        return False
      
      # All the possible moves from a cell including diagonals
      neighbor_transform = [np.array([-1,-1]), np.array([-1,0]), np.array([-1,1]), np.array([0,-1]), np.array([0,1]), np.array([1,-1]), np.array([1,0]), np.array([1,1])]

      # Function that returns path from goal to start
      def backtrack(parent_tracker, start, goal):
        path = [goal]
        current = goal
        while  not np.array_equal(current, start):
          current = parent_tracker[current[0]][current[1]]
          path.insert(0, current)
        
        return path
      
      
      
      num_rows = len(grid)
      num_cols = len(grid[0])
      
      frontier = []
      frontier.append((euclidian_dist(start, goal), start))
      

      # parent_tracker[i][j] = [i][j]'s parent
      parent_tracker = []
      for row in range(num_rows):
        parent_tracker.append([None] * num_cols)
      
      # array that keeps track of running best cost from start
      cost_from_start = np.full((num_rows, num_cols), np.inf)

      # cost from start to start is obviously 0
      cost_from_start[start[0]][start[1]] = 0

      # array that estimates cost from node to goal
      est_cost_to_goal = np.full((num_rows, num_cols), np.inf)

      # estimated cost from start to goal
      est_cost_to_goal[start[0]][start[1]] = euclidian_dist(start, goal)

      
      while(len(frontier) != 0):
        current = frontier.pop(0)[1]
        
        if (np.array_equal(current, goal) == True):
          return backtrack(parent_tracker, start, goal)

        neighbors = []

        for shift in neighbor_transform:
          neighbor = current + shift

          if(is_point_valid(neighbor, num_rows, num_cols) and grid[neighbor[0]][neighbor[1]] == 0 ):
            neighbors.append(neighbor) # Determine all valid and visitable neighbors

        
        for n in neighbors: #iterate through all current's neighbors
          dist = cost_from_start[current[0]][current[1]] + 1

          if dist < cost_from_start[n[0]][n[1]]: # Found a better way to get to cell
            parent_tracker[n[0]][n[1]] = current # Update parent
            cost_from_start[n[0]][n[1]] = dist # Update cost from start
            est_cost_to_goal[n[0]][n[1]] = dist + euclidian_dist(n, goal) 

            frontier.append((euclidian_dist(n, goal), n))
            frontier.sort(key=lambda y: y[0])

      print("Failed to reach goal") # If frontier is empty and haven't reached goal, cannot get there
      return []

def pad_map(map, pad_amount):
    # Function that takes in 
    height = map.shape[0]
    width = map.shape[1]

    config_map = map.copy()

    for x in range(0, height-1):
      for y in range(0, width-1):
        if map[x][y] == 1:
          map[x][y] = 1
          for i in range(-pad_amount + x, pad_amount + x):
            for j in range(-pad_amount + y, pad_amount + y):
              if i >= 0 and i <= height-1 and j >= 0 and j <= width-1:
                config_map[i][j] = 1
    
    return config_map


def plan_path(red_centre, blue_centre):
    

    # print("Starting main")
    mappy = np.load("map.npy")
    img_height = mappy.shape[0]
    img_width = mappy.shape[1]

    mappp = mappy.copy()

    # Part 2.2: Compute an approximation of the “configuration space”
    config_map = pad_map(mappy, pad_amount=4)

    
    start = np.array(red_centre)
    end = np.array(blue_centre)

    # Part 2.3 continuation: Call path_planner
    path = path_planner(config_map,start,end)
      

    for arr in path:
      x = arr[0]
      y = arr[1]
      mappp[x][y] = 0.5
      config_map[x][y] = 0.5
    
    # print("Map Shown")
    cv2.imshow('Maze w/ Path', mappp)
    #plt.imshow(np.rot90(np.fliplr(mappp)))
    #plt.show() 

    # print("Config Map Shown")
    cv2.imshow('Padded Maze w/ Path', config_map)
    #plt.imshow(np.rot90(np.fliplr(config_map)))
    #plt.show() 

    cv2.waitKey(-1)  # Wait until a key is pressed to exit the program
    cv2.destroyAllWindows()  # Close all the windows

    return path

