import numpy as np
import sys
import cv2
import math
from ctypes import c_int64
import time
import heapq as hq
#check12
# Clearance as mentioned of 5 mm
clearance = 5

# Size of canvas
w = 1200                          # Canvas width
h = 500                           # Canvas height

# Colour definition to identify different parameters
red = [0 , 0 , 255]                 #Obstacle
green = [0 , 255 , 0]               #Node movement
blue = [255 , 0 , 0]                #Boundary
white = [255 , 255 , 255]           #Goal

# Writing function to display the obstacles and clearance on the canvas
def define_obstacle(obs):
    for x in range(w):
        for y in range(h):
            
            # Defining clearance on the borders
            if x<=clearance or x>=1200-clearance or y>=500-clearance or y<=clearance:
                obs[499-y][x] = blue
            
          
            #Equation for Hexagon
            if x>=(530-clearance) and (y-0.573*x-32.467)<=0 and (y+0.573*x-777.367)<=0 and x<=(770+clearance) and (y-0.573*x+277.367)>=0 and (y+0.573*x-467.533)>=0:
                obs[499-y][x] = blue
            
            if x>=530 and (y-0.573*x-26.467)<=0 and (y+0.573*x-771.367)<=0 and x<=770 and (y-0.573*x+271.367)>=0 and (y+0.573*x-473.533)>=0:
                obs[499-y][x] = red


            if (x >= (900-clearance) and x <= (1100+clearance) and y >= (50-clearance) and y <= (450+clearance) ) and not (x >= (900-clearance) and x <= (1020-clearance) and y >= (125+clearance) and y <= (375-clearance)):
                obs[499-y][x] = blue
            if (x >= 900 and x <= 1100 and y >= 50 and y <= 450 ) and not (x >= 900 and x <= 1020 and y >= 125 and y <= 375):
                obs[499-y][x] = red
            # The point (x, y) is inside the outer rectangle but not inside the inner cutout


            # Rectangle for the Top
            if (100-clearance)<=x and x<=(175+clearance) and (100-clearance)<=y and y<=h:
                obs[499-y][x] = blue   
            if x>=100 and x<=175 and y>=100 and y<=h: 
                obs[499-y][x] = red
                
            # Rectangle for the Bottom
            if (275-clearance)<=x and x<=(350+clearance) and y>=0 and y<=(400+clearance):
                obs[499-y][x] = blue
            if (275)<=x and x<=(350) and 0<=y and y<=(400): 
                obs[499-y][x] = red
    return obs


# Defing the movement of the robot
revolution_robot = 499

# Movement for downwards
def move_down(current_node, canvas):
    # Creating a new list to store current nodes
    new_node = list(current_node)
    # substracting 1 from y co-ordinate
    new_node[1] -= 1 
    if new_node[1]>=0 and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Movement for upwards
def move_up(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    if new_node[1]<=h and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None
    
# Movement for left
def move_left(current_node, canvas):
    new_node = list(current_node)
    new_node[0] -= 1 
    if new_node[0] >= 0 and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Movement for right
def move_right(current_node, canvas):
    new_node= list(current_node)
    new_node[0] += 1  
    if new_node[0] < w and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Right-down Movement
def down_right(current_node, canvas):
    new_node = list(current_node)
    new_node[1] -= 1 
    new_node[0] += 1
    if new_node[1] >= 0 and new_node[0] < w and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None
    
# Left-down Movement
def down_left(current_node, canvas):
    new_node = list(current_node)
    new_node[1] -= 1 
    new_node[0] -= 1
    if new_node[1]>=0 and new_node[0]>=0 and (canvas[(revolution_robot-new_node[1]), new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Right-up Movement
def up_right(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    new_node[0] += 1
    if new_node[1]<h and new_node[0]<w and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: return None

# Left-up Movement
def up_left(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    new_node[0] -= 1
    if new_node[1] < h and new_node[0]>=0 and (canvas[revolution_robot-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Defining function to explore the nodes 
def explore(new_node, current_node, temp, c_idx, explored_list, cost):
    chk = False
    new_node = tuple(new_node)
    # Check if the node is already explored
    if new_node not in explored_list:
        new_node = [current_node[0], current_node[1], new_node, current_node[2]]
        for i in temp:   
            if i[2] == new_node[2]:
                chk = True
                new_node[0] = new_node[0] + cost
                if i[0] > new_node[0]:
                    i[0] = new_node[0]
                    i[3] = new_node[3]
        if chk == False:
            new_node[0] = new_node[0] + cost 
            c_idx.value += 1
            new_node[1] = c_idx.value
            hq.heappush(temp, new_node)

############# Implementation of Dijkstra Algorithm ###################################################

def dijkstra_algorithm(current_node, temp, c_idx, explored_list, canvas):
    new_node = move_up(current_node[2], canvas)
    if new_node != None:
        cost = 1           #Cost of motion for linear movement
        explore(new_node, current_node, temp, c_idx, explored_list, cost)
    
    new_node2 = move_down(current_node[2], canvas)
    if new_node2 != None:
        cost = 1
        explore(new_node2, current_node, temp, c_idx, explored_list, cost)
    
    new_node3 = move_left(current_node[2], canvas)
    if new_node3 != None:
        cost = 1
        explore(new_node3, current_node, temp, c_idx, explored_list, cost)

    new_node4 = move_right(current_node[2], canvas)
    if new_node4!= None:
        cost = 1
        explore(new_node4, current_node, temp, c_idx, explored_list, cost)

    new_node5 = up_right(current_node[2], canvas)
    if new_node5 != None:
        cost = 1.4           #Cost of motion for diagonal movement
        explore(new_node5, current_node, temp, c_idx, explored_list, cost)

    new_node6 = up_left(current_node[2], canvas)
    if new_node6 != None:
        cost = 1.4
        explore(new_node6, current_node, temp, c_idx, explored_list, cost)
    
    new_node7 = down_right(current_node[2], canvas)
    if new_node7 != None:
        cost = 1.4
        explore(new_node7, current_node, temp, c_idx, explored_list, cost)
    
    new_node8 = down_left(current_node[2], canvas)
    if new_node8 != None:
        cost = 1.4
        explore(new_node8, current_node, temp, c_idx, explored_list, cost)

################################### Backtracking Implementation ###########################################
def backtrack(explored_list, start, goal, canvas):
    # Save the results as a video file
    out2 = cv2.VideoWriter_fourcc(*'mp4v')
    result = cv2.VideoWriter('Animation_Video.mp4', out2 ,100,(1200,500))
    # Create a list to store the back track path
    track = []
    # Add goal node to the list
    track.append(goal)
    # Display the exploration of nodes on screen
    for current_node in explored_list:
        canvas[499 - current_node[1], current_node[0]] = green
        cv2.waitKey(1)
        cv2.imshow("Path_Animation", canvas)
        # Write the changes in the video
        result.write(canvas)
    # Backtracking from goal node to start node
    current_node = tuple(goal)
    while(current_node != start):
        current_node = explored_list[current_node]
        track.append(current_node)
    track.reverse()
    for p in range (len(track)):
        canvas[499 - track[p][1], track[p][0]] = red
        # Writing the back tracked path in the video
        result.write(canvas)
    #cv2.imshow("canvas",(start_node , goal_node))
    # Display the back tracked path on screen
    cv2.imshow("Path_Animation", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    result.release()

def main():
    check = False
    canvas_size = np.ones((h, w, 3), dtype=np.uint8)
    
    # Create a canvas to display output on screen with obstacles
    canvas = define_obstacle(canvas_size)
    
    # Taking input for the Start Node
    x1 = int(input('Input: Value of x for the start node (between 0 and 1200)'))
    y1 = int(input('Input: Value of y for the start node (between 0 and 500)'))
    if (x1<0) or x1>1200 or y1<0 or y1>500:
        print("Invalid input. Please enter a valid input.")
    
    # Taking input for Goal Node
    x2 = int(input('Input: Value of x for the goal node (between 0 and 1200)'))
    y2 = int(input('Input: Value of y for the goal node(between 0 and 1200)'))
    if (x1<0) or x1>1200 or y1<0 or y1>500:
        print("\nInvalid input. Please enter a valid input")
    
    # storing the start and goal nodes
    start_node =(x1, y1)
    goal_node = (x2, y2)
    c_idx = c_int64(0)
    n_s = [0.0, 0, start_node , start_node]
    
    #creating a list to stores the nodes
    temp = []
    
    # Creating a list to store the explored nodes
    explored_list = {}
    hq.heappush(temp, n_s)
    hq.heapify(temp)
    
    # Start timer to record the time taken for the process
    starting_time = time.time()

    while (len((temp)) > 0):
        current_node = hq.heappop(temp)
        
        # Adding to the explored list
        explored_list[current_node[2]] = current_node[3]
        # Checking if the search reached the goal node
        if current_node[2] == tuple(goal_node):
            print("Goal Reached")
            # Print the time taken to compute the path
            print("Time taken to reach the goal is ", round(time.time() - starting_time,2)," seconds.")
            # Performing back tracking to find optimal path
            backtrack(explored_list, start_node, goal_node, canvas)
            check = True
            break
        # Call the function to run the algorithm
        dijkstra_algorithm(current_node, temp , c_idx, explored_list, canvas)

    if check == False:
        # Display the message if the nodes entered are in the obstacle space
        print("Goal cannot be reached because input is in obstacle space. Try again.")


if __name__ == '__main__':
    main()