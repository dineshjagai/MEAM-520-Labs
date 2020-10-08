import heapq
import numpy as np
import math
from OccupancyMap import OccupancyMap
import matplotlib.pyplot as plt
    

class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1]
    
    def __repr__(self):
      return f"{self.position} - g: {self.g} h: {self.h} f: {self.f}"

    def __lt__(self, other):
      return self.f < other.f
    
    def __gt__(self, other):
      return self.f > other.f


def return_path(current_node):
    path = []
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    path = path[::-1]  # Reverse path
    return np.array(path)


def Astar(filepath, start, goal, radius):

    occ_map = OccupancyMap(filepath, radius)
    bounds = occ_map.get_bounds()
    x_min = bounds[0]
    x_max = bounds[1]
    y_min = bounds[2]
    y_max = bounds[3]

    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, goal)
    end_node.g = end_node.h = end_node.f = 0

    open_list = []
    closed_list = []

    heapq.heapify(open_list) 
    heapq.heappush(open_list, start_node)

    adjacent_squares = ((0, -0.5), (0, 0.5), (-0.5, 0), (0.5, 0), (-0.5, -0.5), (-0.5, 0.5), (0.5, -0.5), (0.5, 0.5),)
    print(occ_map.get_blocks())
    while len(open_list) > 0:
        
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)
        
        print(current_node.position)
        if current_node == end_node:
            return return_path(current_node)

        children = []
        
        for new_position in adjacent_squares:

            node_position = [current_node.position[0] + new_position[0], current_node.position[1] + new_position[1]]

            if node_position[0] < x_min or node_position[0] > x_max or node_position[1] < y_min or node_position[1] > y_max:
                continue

            blocked = False
            for block in occ_map.get_blocks():
                if node_position[0] >= block[0] and node_position[0] <= block[1] and node_position[1] >= block[2] and node_position[1] <= block[3]:
                    blocked = True
                    break
            
            if blocked:
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # check if this is node is diagonal
            if (abs(current_node.position[0] - child.position[0]) + abs(current_node.position[1] - child.position[1]) > 0.5):
                child.g = current_node.g + np.sqrt(2) / 2
            else:
                child.g = current_node.g + 0.5
            
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            if len([open_node for open_node in open_list if child.position == open_node.position and child.g > open_node.g]) > 0:
                continue

            heapq.heappush(open_list, child)

    return None
