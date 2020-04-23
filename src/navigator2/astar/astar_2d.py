#encoding=utf-8
import astar_config
import numpy as np
import heapq

from heapq import heappush, heappop
from nav_msgs.msg import OccupancyGrid
import rospy


class A_star_2D(object):
    def __init__(self, end_pos):
        self.end_pos = end_pos

        self.map_array = []
        self.map_width = 0
        self.map_height = 0
        self.map_obstacle_indexed = None
        self.map_unknown_indexed  = None
        self.map_free_indexed     = None
        self.resolution = 0

        self.openlist = []
        self.closed = []
        self.movement_list = astar_config.astar_config['movement_list_2d']
        self.func_h = astar_config.astar_config['func_h_2d']
        #self.horiontal_radius, self.vertical_radius = self.__aircraft_radius(aircraft_obj['aircraft_points'])
        # print ('aircraft_obj',aircraft_obj)


    def heuristic_cost_estimate(self, neighbor, goal):
        x = neighbor[0] - goal[0]
        y = neighbor[1] - goal[1]
        return abs(x) + abs(y)


    def dist_between(self, a, b):
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path


    # astar function returns a list of points (shortest path)
    def find_path(self, start, goal):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]  # 8个方向

        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic_cost_estimate(start, goal)}

        openSet = []
        heappush(openSet, (fscore[start], start))  # 往堆中插入一条新的值

        # while openSet is not empty
        while openSet:
            # current := the node in openSet having the lowest fScore value
            current = heappop(openSet)[1]  # 从堆中弹出fscore最小的节点

            if current == goal:
                return self.reconstruct_path(came_from, current)

            close_set.add(current)

            for i, j in directions:  # 对当前节点的 8 个相邻节点一一进行检查
                neighbor = current[0] + i, current[1] + j

                ## 判断节点是否在地图范围内，并判断是否为障碍物
                if 0 <= neighbor[1] < self.map_height:
                    if 0 <= neighbor[0] < self.map_width:
                        if self.map_array[neighbor[1]][neighbor[0]] == 100:  # 100为障碍物
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                # Ignore the neighbor which is already evaluated.
                if neighbor in close_set:
                    continue

                #  The distance from start to a neighbor via current
                tentative_gScore = gscore[current] + self.dist_between(current, neighbor)

                if neighbor not in [i[1] for i in openSet]:  # Discover a new node
                    heappush(openSet, (fscore.get(neighbor, np.inf), neighbor))
                elif tentative_gScore >= gscore.get(neighbor, np.inf):  # This is not a better path.
                    continue

                    # This path is the best until now. Record it!
                came_from[neighbor] = current
                gscore[neighbor] = tentative_gScore
                fscore[neighbor] = tentative_gScore + self.heuristic_cost_estimate(neighbor, goal)

        return None

    def update_map(self, occupancy_grid_raw):
        width  = occupancy_grid_raw.info.width
        height = occupancy_grid_raw.info.height

        free_list     = []
        occupied_list = []
        unknown_list  = []
        for y in range(0, height):
            row = []
            for x in range(0, width):
                data = occupancy_grid_raw.data[y * width + x]
                if data == -1:
                    unknown_list.append((x, y))
                    row.append(-1)
                elif data > 45:
                    occupied_list.append((x, y))
                    row.append(100)
                else:
                    free_list.append((x, y))
                    row.append(0)
            self.map_array.append(row)

        self.map_obstacle_indexed = set(occupied_list)
        self.map_unknown_indexed  = set(unknown_list)
        self.map_free_indexed     = set(free_list)
        self.map_width  = width
        self.map_height = height
        self.resolution = occupancy_grid_raw.info.resolution

    # 放在这以后可以参考，同时验证地图正确性
    # 下面可以屏蔽任意一个列表，以验证其中的数据对不对
    # usage: data = self.algo.map_list_to_occupancy_grid()
    def map_list_to_occupancy_grid(self):
        data = OccupancyGrid()
        data.info.resolution = self.resolution
        data.info.width = self.map_width
        data.info.height = self.map_height

        for y in range(0, data.info.width):
            for x in range(0, data.info.height):
                data.data.append(-1)

        for (x, y) in self.map_obstacle_indexed:
            data.data[y * self.map_width + x] = 100
        for (x, y) in self.map_unknown_indexed:
            data.data[y * self.map_width + x] = -1
        for (x, y) in self.map_free_indexed:
            data.data[y * self.map_width + x] = 0
        data.header.stamp = rospy.Time.now()

        return data
