import math
import numpy as np
import heapq
import pygame
import cv2
import time

# The program has been written according to the flowchart and steps given in the powerpoint slide for this project.
# The goal of this program is to find the optimal path in a 2D obstacle space with a circle, a hexagon, and a concave quadrilateral as the obstacles.
# Various functions have been written in order to achieve this goal. The functions are called in the main() function as needed and a graphical output is shown at the end.

# Step 1: defining actions in a mathematical format. Since, the coordinate system is inverted, we will define positive x facing botton. positive y in right direction
def move_up(current_node):
    new_node_x = current_node[0]
    new_node_x -=1
    new_node = (new_node_x, current_node[1])
    if new_node[0]>0 and new_node[1]>0:
        return(new_node, True)
    else:
        return(current_node, False)
def move_down(current_node):
        new_node_x = current_node[0]
        new_node_x += 1
        new_node = (new_node_x, current_node[1])
        if new_node[0] >= 0 and new_node[1] >= 0:
            return (new_node, True)
        else:
            return (current_node, False)
def move_left(current_node):
    new_node_y = current_node[1]
    new_node_y -=1
    new_node = (current_node[0], new_node_y)
    if new_node[0]>0 and new_node[1]>0:
        return(new_node, True)
    else:
        return(current_node, False)
def move_righ(current_node):
    new_node_y = current_node[1]
    new_node_y +=1
    new_node = (current_node[0], new_node_y)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(current_node, False)
def move_top_left(current_node):
    new_node_y = current_node[1]
    new_node_y -= 1
    new_node_x = current_node[0]
    new_node_x -= 1
    new_node = (new_node_x, new_node_y)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(current_node, False)
def move_top_right(current_node):
    new_node_y = current_node[1]
    new_node_y += 1
    new_node_x = current_node[0]
    new_node_x -= 1
    new_node = (new_node_x, new_node_y)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(current_node, False)
def move_bottom_left(current_node):
    new_node_y = current_node[1]
    new_node_y -= 1
    new_node_x = current_node[0]
    new_node_x += 1
    new_node = (new_node_x, new_node_y)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(current_node, False)
def move_down_right(current_node):
    new_node_y = current_node[1]
    new_node_y += 1
    new_node_x = current_node[0]
    new_node_x += 1
    new_node = (new_node_x, new_node_y)
    if new_node[0]>=0 and new_node[1]>=0:
        return(new_node,True)
    else:
        return(current_node, False)

# Step 2: representing the obstacle space using half planes and semi-algebraic models
def generate_robot_world(width, height):
    # creating a tuple with the coordinate of each node in the robot world
    all_nodes = []
    for i in range(0, 401):
        for j in range(251):
            all_nodes.append((i, j))
    print('\nTotal number of nodes in the obstacle space is: '+str(len(all_nodes)))

    # using half planes and semi algebraic equations to define the obstacles and add the points to the obstacle list.
    obstacle_nodes = []
    for i in all_nodes:
        x = i[0]
        y = i[1]
        # circle obstacle
        if ((x - 300) ** 2 + (y - 185) ** 2 <= (45) ** 2):
            obstacle_nodes.append((x, y))
        # hexagon obstacle
        if y - (0.57725) * x <= 30.7 and y + (0.57725) * x <= 261.6 and x <= 240 and y - (0.57725) * x >= - 61.6 and y + (0.57725) * x >= 169.3 and x >= 160:
            obstacle_nodes.append((x, y))
        # concave arrow obstacle
        if x >= 27.5 and x <= 136.9 and y >= 76.6 and y <= 222.2:
            if y + (123 / 100) * x >= 221.4:
                if y - (31 / 100) * x <= 178.9:
                    if y + (3.2003) * x <= 452.8 or y - (85 / 100) * x >= 104.8:
                        obstacle_nodes.append((x, y))
    return obstacle_nodes

# Step 3: Generating a complete graph of 8-connected space and storing in a dictionary data structure
def generate_action_set(start, len_x, len_y):  # this creates action sets based on location of the node
    x = start[0]
    y = start[1]
    if x < len_x and y < len_y:
        action_set = {}
        if x == 0 and y == 0:
            action_set[(x, y)] = {(x + 1, y + 1), (x + 1, y), (x, y + 1)}
        elif x == len_x - 1 and y == len_y - 1:
            action_set[(x, y)] = {(x - 1, y), (x - 1, y - 1), (x, y - 1)}
        elif x == len_x - 1 and y == 0:
            action_set[(x, y)] = {(x - 1, y), (x - 1, y + 1), (x, y + 1)}
        elif y == len_y - 1 and x == 0:
            action_set[(x, y)] = {(x, y - 1), (x + 1, y - 1), (x + 1, y)}
        elif x == 0 and y != 0 and y != len_y - 1:
            action_set[(x, y)] = {(x, y - 1), (x, y + 1), (x + 1, y - 1), (x + 1, y), (x + 1, y + 1)}
        elif x == len_x - 1 and y != 0 and y != len_y - 1:
            action_set[(x, y)] = {(x, y - 1), (x, y + 1), (x - 1, y - 1), (x - 1, y), (x - 1, y + 1)}
        elif y == 0 and x != 0 and x != len_x - 1:
            action_set[(x, y)] = {(x - 1, y), (x + 1, y), (x + 1, y + 1), (x, y + 1), (x - 1, y + 1)}
        elif y == len_y - 1 and x != 0 and x != len_x - 1:
            action_set[(x, y)] = {(x - 1, y), (x + 1, y), (x + 1, y - 1), (x, y - 1), (x - 1, y - 1)}
        else:
            action_set[(x, y)] = {(x - 1, y), (x - 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x + 1, y), (x + 1, y + 1),
                             (x, y - 1), (x, y + 1)}
        return (action_set)
    else:
        pass
def generate_action_graph(height, width, obstacle_nodes):
    initial_graph = {}
    for i in range(height - 1, -1, -1):
        for j in range(width - 1, -1, -1):
            graph = generate_action_set((i, j), height, width)
            initial_graph[(i, j)] = graph[(i, j)]
    # looping through all nodes dictionary of action sets and deleting obstacle points and actions that are invalid
    for key, value in initial_graph.items():
        value_copy = value.copy()
        for coordinates in value_copy:
            if coordinates in obstacle_nodes:
                value.remove(coordinates)
    base_graph_copy = initial_graph.copy()
    for key, value in base_graph_copy.items():
        if key in obstacle_nodes:
            del initial_graph[key]
    print('\nTotal number of nodes in the obstacle space after removing the obstacle nodes: '+str(len(initial_graph)))
    return initial_graph
def calculate_cost(graph, start):
    # initializing a key = node, value = cost associated with each action
    dic = {}
    for key, value in graph.items():
        dic[key] = {}
        for neighbour in value:
            Right = move_righ(key)
            Left = move_left(key)
            Up = move_up(key)
            Down = move_down(key)
            Top_left = move_top_left(key)
            Top_right = move_top_right(key)
            Bottom_left = move_bottom_left(key)
            Bottom_right = move_down_right(key)
            if (neighbour == Right[0]) or (neighbour == Left[0]) or (neighbour == Up[0]) or (neighbour == Down[0]):
                dic[key][neighbour] = 1
            elif (neighbour == Top_left[0]) or (neighbour == Top_right[0]) or (neighbour == Bottom_left[0]) or (neighbour == Bottom_right[0]):
                dic[key][neighbour] = 1.414
    return dic

# Step 4: Finding the optimal path using Dijktra's algorithm
def dijkstraAlgorithm(graph, start, goal):
    global exit, closed_list
    # getting the start node and assigning distance = 0 to it
    distances[start] = 0
    # adding the start node to the closed list
    closed_list.append(start)
    for vertex, edge in graph.items():
        distances[vertex] = math.inf
    # using heapq to cycle through elements of the open list
    open_list_queue = [(0, start)]
    # starting the loop to cycle through open list until it the final node is popped
    while len(open_list_queue) > 0 and exit != []:
        current_distance, current_node = heapq.heappop(open_list_queue)
        # if the calcualted distance is less than the existing distance, then updating the distance to the lesser value
        if current_distance > distances[current_node]:
            continue
        for neighbour, cost in graph[current_node].items():
            distance = current_distance + cost
            if distance < distances[neighbour]:
                backtr[neighbour] = {}
                backtr[neighbour][distance] = current_node
                distances[neighbour] = distance
                heapq.heappush(open_list_queue, (distance, neighbour))
                if neighbour not in closed_list:
                    closed_list.append(neighbour)
                    if neighbour == goal:
                        print('The Goal node has been popped! ')
                        exit = []
                        break
    return (distances, closed_list, backtr)
def backtracker(backtrack_dict, goal, start):
    backtrack_list = []
    backtrack_list.append(start)
    while goal != []:
        for key, value in backtr.items():
            for key1, value1 in value.items():
                if key == start:
                    if value1 not in backtrack_list:
                        backtrack_list.append(start)
                    start = value1
                    if value1 == goal:
                        goal = []
                        break
    return (backtrack_list)


closed_list = []
exit = 0
distances = {}
backtr = {}

# This is the main function which controls the flow of the program by calling various functions as needed.
# After the completion of the algorithm, this function represents the optimal path. So, Step 5 is within this main function.

# Step 5: Representing the optimal path.
def main():
    start_time = time.time()
    global distances, backtr, closed_list, exit

    print('\nPlease enter the start and end states: ')
    start_x = int(input("Start x: "))
    start_y = int(input("Start y: "))
    goal_x = int(input("Goal x: "))
    goal_y = int(input("Goal y: "))

    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    width = 400
    height = 250

    width += 1
    height += 1
    print("\nGenerating obstacle map...")
    obstacle_nodes = generate_robot_world(width, height)

    if goal in obstacle_nodes:
        print('Error! The goal state falls within an obstacle. Exiting program...')
    print('Number of nodes falling inside obstacles is: '+ str(len(obstacle_nodes)))

    print("Generating graph of all nodes and action set...")
    initial_graph = generate_action_graph(width,height,obstacle_nodes)

    print("Calculating all cost...")
    all_costs = calculate_cost(initial_graph, start)
    final_graph = all_costs
    distances = {}
    backtr = {}
    closed_list = []

    print("Now running Dijkstra's algorithm...")
    distances, closed_list, backtr = dijkstraAlgorithm(final_graph, start, goal)

    distances_copy = distances.copy()
    for k, v in distances_copy.items():
        if distances_copy[k] == math.inf:
            del distances[k]

    backtrack = backtr

    print("Algorithm ran succesfully. Now running the backtracking function...\n")
    backtracked_final = backtracker(backtrack, start, goal)
    print("The program has found the optimal path based on Dijkstra's algorithm. Here is the backtrack list: ")
    print(backtracked_final)
    print("Time taken to complete the search: ", (time.time() - start_time)/60, "minutes")


    # Step 5: Visualization
    black = (0, 0, 0)
    white = (0, 255, 255)
    visuals = np.zeros((251, 401, 3), np.uint8)
    for c in obstacle_nodes:
        visuals[(c[1], c[0])] = [255, 128, 128] # color for obstacles
    visuals = np.flipud(visuals)
    # copies created for later use
    visuals_copy_backtrack = visuals.copy()
    visuals_copy_visited = visuals.copy()
    visuals_copy_visited = cv2.resize(visuals_copy_visited, (800, 500))

    cv2.imshow('Visuals', visuals)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    pygame.init()

    screen = pygame.display.set_mode((400, 250),pygame.FULLSCREEN)

    last_frame = False
    while not last_frame:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                last_frame = True

        screen.fill(black)
        for path in closed_list:
            if path not in visuals_copy_visited:
                x = path[0]
                y = abs(250 - path[1])
                pygame.draw.rect(screen, white, [x, y, 1, 1])
                pygame.display.flip()
        for path in backtracked_final:
            pygame.time.wait(5)
            x = path[0]
            y = abs(250 - path[1])
            pygame.draw.rect(screen, (51, 51, 51), [x, y, 1, 1])
            pygame.display.flip()
        last_frame = True
    pygame.quit()

    # showing the visited nodes
    for path in closed_list:
        x = path[0]
        y = path[1]
        visuals_copy_backtrack[(250 - y, x)] = [153, 204, 0] # color for visited nodes
    backtracked_image = cv2.resize(visuals_copy_backtrack, (800, 500))
    cv2.imshow('Visited Nodes', backtracked_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    for path in backtracked_final:
        x = path[0]
        y = path[1]
        visuals_copy_backtrack[(250 - y, x)] = [255, 102, 0]  # color for backtracked nodes
    # showing the final backtracked nodes
    backtracked_image = cv2.resize(visuals_copy_backtrack, (800, 500))
    cv2.imshow('Backtraked Nodes', backtracked_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()