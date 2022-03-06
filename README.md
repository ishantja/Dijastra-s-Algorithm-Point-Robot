# Dijastra-s-Algorithm-Point-Robot

ENPM 661 Project 2
Implementation of Dijkstra's Algorithm to find the optimal path for a point robot in a 2D obstacle space

The program is a single python file that was created using pycharm in windows environment
The following is the dependencies list

Python 3.9
pygame 2.1.2
numpy 1.22.2
opencv-python 4.5.5.62
math library
heapq library

When the file is run, the program will require input from the user for the start and goal node locations

The map is a 400x250 grid, so the user can enter any location within this grid. 

The user is required to enter the start node's x location first. For example, if start node = (10,21), user should enter 10, then press enter
then enter 21. 

Continue in similar fashion and enter all the location. When complete, the program should automatically begin execting the required commands to find the optimal path. 

Program will output the progression of the algorithm as it reaches certain checkpoints. 

Finally a visuals window will show up showing the map generated. You should close this to continue. 

Then video will play in full screen showing the exploration of the map. 

Again subsequent windows will show visited nodes, and backtracked nodes.
