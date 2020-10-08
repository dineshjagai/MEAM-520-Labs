# uncompyle6 version 3.5.0
# Python bytecode 3.6 (3379)
# Decompiled from: Python 2.7.5 (default, Aug  7 2019, 00:51:29) 
# [GCC 4.8.5 20150623 (Red Hat 4.8.5-39)]
# Embedded file name: /home/meam520/code/Lab3/Astar.py
# Compiled at: 2020-07-08 02:30:14
# Size of source mod 2**32: 6278 bytes
from heapq import heappush, heappop
import numpy as np, math
from OccupancyMap import OccupancyMap
import matplotlib.pyplot as plt

def Astar(filepath, start, goal, radius):
    """
    Parameters:
        filepath,   json file representing the environment obstacles and infomation
        start,      xy position in meters, shape=(2,)
        goal,       xy position in meters, shape=(2,)
        radius,     radius of the robot
    Output:
        path,       xy position coordinates along the path in meters with
                    shape=(N,2). These are typically the centers of visited
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
    """
    occ_map = OccupancyMap(filepath, radius)
    start_index = tuple(occ_map.metric_to_index(start))
    goal_index = tuple(occ_map.metric_to_index(goal))
    path = [goal]
    i, j = occ_map.occ.shape
    g = np.full((i, j), np.inf)
    p = np.zeros((i, j, 2))
    G = []
    coeff = 10
    h1 = max(abs(start_index[0] - goal_index[0]) * coeff, abs(start_index[1] - goal_index[1]) * coeff)
    g[start_index] = h1
    heappush(G, (g[start_index], start_index))
    heappush(G, (g[goal_index], goal_index))
    state = np.zeros((i, j))
    state[start_index] = 1
    while np.min(g) < np.inf and not state[goal_index] == 2:
        u = heappop(G)[1]
        state[u] = 2
        for i in range(u[0] - 1, u[0] + 2):
            for j in range(u[1] - 1, u[1] + 2):
                v = (i, j)
                if occ_map.is_valid_index(v):
                    if not occ_map.is_occupied_index(v):
                        if not state[v] == 2:
                            h = max(abs(goal_index[0] - v[0]) * coeff, abs(goal_index[1] - v[1]) * coeff)
                            if abs(v[0] - u[0]) + abs(v[1] - u[1]) == 1:
                                c = 10
                            else:
                                c = 14
                            d = g[u] + c + h
                            if state[v] == 1:
                                if g[v] > d:
                                    G.remove((g[v], v))
                                    g[v] = d
                                    p[v] = u
                                    heappush(G, (g[v], v))
                                elif state[v] == 0:
                                    g[v] = d
                                    p[v] = u
                                    heappush(G, (g[v], v))
                                    state[v] = 1

    if (p[goal_index] == (0, 0)).all():
        path = None
        raise Exception('No Path Found. ')
    else:
        pointer = goal_index
        while pointer != start_index:
            par = p[pointer]
            path.append(occ_map.index_to_metric_negative_corner(par))
            ind1 = int(par[0])
            ind2 = int(par[1])
            pointer = (ind1, ind2)

        path.reverse()
        path = np.array(path)
    return path


if __name__ == '__main__':
    # import json
    # from matplotlib.patches import Rectangle
    # path = 'test_maze_2.json'
    # start = [0, 0]
    # goal = [9, 9]
    # a, b = Astar(path, start, goal, 0.2)
    # print(a)
    # fig, ax = plt.subplots()
    # t = np.arange(0, len(a))
    # ax.set(xlabel='x (m)', ylabel='y (m)', title='position of robot')
    # ax.plot(a[:, 0], a[:, 1])
    # ax.grid()
    # for block in b:
    #     obstacle = Rectangle((block[0], block[2]), (np.diff(block[0:2])),
    #       (np.diff(block[2:])),
    #       color='red',
    #       fill=True)
    #     ax.add_patch(obstacle)

    # plt.show()
    result = Astar('tests/map1.json', [0,0], [2,5], 1)
    print(result)