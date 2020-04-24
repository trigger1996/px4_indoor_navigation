#encoding=utf-8
import astar_config
import DiscreteGridUtils
import numpy as np
import heapq

from heapq import heappush, heappop
from nav_msgs.msg import OccupancyGrid
import rospy

DEFAULT_UAV_ALT = 1.5

class A_star_2D(object):
    def __init__(self, end_pos, vehicle_width = 0.8, vehicle_length = 0.8, resolution = 0.2):
        self.end_pos = end_pos

        # basic map info
        self.map_array = []
        self.map_width = 0                  # in grids
        self.map_height = 0                 # in grids
        self.map_obstacle_indexed = None
        self.map_unknown_indexed  = None
        self.map_free_indexed     = None
        self.resolution = resolution        # in meters
        self.map_origin = []                # where grid map point (0, 0, 0) in real world

        # map scale parameters
        self.dg = DiscreteGridUtils.DiscreteGridUtils_wOffset(grid_size=0.2)

        # A* algorithm parameters
        self.came_from = {}

        self.openSet   = None
        self.close_set = None
        self.movement_list = astar_config.astar_config['movement_list_2d']
        self.func_h = astar_config.astar_config['func_h_2d']

        # vehicle_info
        self.vehicle_width  = vehicle_width         # in meters
        self.vehicle_length = vehicle_length
        self.vehicle_R_grid = int((max(self.vehicle_width, self.vehicle_length) / 2
                                   + (self.resolution * 0.5)) / self.resolution) - 1
        print ('vehicle Radius/Grids: ', self.vehicle_R_grid)

    def heuristic_cost_estimate(self, neighbor, goal):
        #x = neighbor[0] - goal[0]
        #y = neighbor[1] - goal[1]
        #return abs(x) + abs(y)
        return self.func_h(neighbor, goal)


    def dist_between(self, a, b):
        return (b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2


    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append((current[0], current[1], DEFAULT_UAV_ALT / self.resolution))
        return path[::-1]


    # astar function returns a list of points (shortest path)
    # start, goal in meters
    def find_path(self, start, goal):
        #directions = [(0, 1), (0, -1), (1, 0), (-1, 0) , (1, 1), (1, -1), (-1, 1), (-1, -1)]  # 8个方向
        directions = self.movement_list

        start = self.dg.continuous_to_discrete((start[0], start[1], start[2]))  # meters to grids
        goal  = self.dg.continuous_to_discrete((goal[0],  goal[1],  goal[2]))
        start = (start[0], start[1])                                            # 3D to 2D
        goal  = (goal[0], goal[1])

        self.close_set = set()
        self.came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic_cost_estimate(start, goal)}

        self.openSet = []
        heappush(self.openSet, (fscore[start], start))  # 往堆中插入一条新的值

        # while openSet is not empty
        while self.openSet:
            # current := the node in openSet having the lowest fScore value
            current = heappop(self.openSet)[1]  # 从堆中弹出fscore最小的节点

            if current == goal:
                return self.reconstruct_path(self.came_from, current)

            self.close_set.add(current)

            for i, j in directions:  # 对当前节点的 8 个相邻节点一一进行检查
                neighbor = current[0] + i, current[1] + j

                ## 判断节点是否在地图范围内，并判断是否为障碍物
                if 0 <= neighbor[1] < self.map_height:
                    if 0 <= neighbor[0] < self.map_width:
                        # 1 目标点不是障碍物
                        if self.map_array[neighbor[1]][neighbor[0]] > 45:  # 100为障碍物
                            continue
                        # 2 目标点不是未知点
                        if self.map_array[neighbor[1]][neighbor[0]] == -1:
                            continue
                        # 3 目标点周围的点不是障碍物
                        if not self.is_valid(neighbor[0], neighbor[1]):
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue

                # Ignore the neighbor which is already evaluated.
                if neighbor in self.close_set:
                    continue

                # The distance from start to a neighbor via current
                tentative_gScore = gscore[current] + self.dist_between(current, neighbor)

                if neighbor not in [i[1] for i in self.openSet]:  # Discover a new node
                    heappush(self.openSet, (fscore.get(neighbor, np.inf), neighbor))
                elif tentative_gScore >= gscore.get(neighbor, np.inf):  # This is not a better path.
                    continue

                # This path is the best until now. Record it!
                self.came_from[neighbor] = current
                gscore[neighbor] = tentative_gScore
                fscore[neighbor] = tentative_gScore + self.heuristic_cost_estimate(neighbor, goal)


        return None

    def is_valid(self, _x, _y):
        for y in range(_y - self.vehicle_R_grid, _y + self.vehicle_R_grid):
            for x in range(_x - self.vehicle_R_grid, _x + self.vehicle_R_grid):
                if 0 <= x < self.map_width and 0 <= y < self.map_height:
                    # the point is within map range
                    if self.map_array[y][x] > 45:
                        return False
                else:
                    continue
        return True

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
        self.map_origin = ([occupancy_grid_raw.info.origin.position.x,
                            occupancy_grid_raw.info.origin.position.y,
                            occupancy_grid_raw.info.origin.position.z])
        self.resolution = occupancy_grid_raw.info.resolution

        # update map scale converter
        self.dg = DiscreteGridUtils.DiscreteGridUtils_wOffset(offset=self.map_origin, grid_size=self.resolution)
        # update vehicle by the way
        self.vehicle_R_grid = int((max(self.vehicle_width, self.vehicle_length) / 2
                                   + (self.resolution * 0.5)) / self.resolution) - 1

    # 放在这以后可以参考，同时验证地图正确性
    # 下面可以屏蔽任意一个列表，以验证其中的数据对不对
    # usage: data = self.algo.map_list_to_occupancy_grid()
    def map_list_to_occupancy_grid(self):
        data = OccupancyGrid()
        data.info.resolution = self.resolution
        data.info.width = self.map_width
        data.info.height = self.map_height
        data.info.origin.position.x = self.map_origin[0]
        data.info.origin.position.y = self.map_origin[1]
        data.info.origin.position.z = self.map_origin[2]

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

    def find_alternative_closest_point(self, final_pos):
        dst = 10 ** 6

        for node in self.close_set:
            x = node[0]
            y = node[1]
            # is_valid is confirmed in a*
            dst_temp = ((x - final_pos[0]) ** 2 + (y - final_pos[1]) ** 2) ** 0.5
            if dst_temp < dst and self.is_valid(x, y):
                target_pt = [x, y]
                dst = dst_temp
        return target_pt

    def find_alternative_path(self, final_pos):
        final_pos = self.dg.continuous_to_discrete((final_pos[0], final_pos[1], 5))
        final_pos = (final_pos[0], final_pos[1])

        target_pt = self.find_alternative_closest_point(final_pos)
        path_in_grid = self.reconstruct_path(self.came_from, (target_pt[0], target_pt[1]))

        path = []
        for nav_pt_in_grid in path_in_grid:
            nav_pt = self.dg.discrete_to_continuous_target((nav_pt_in_grid[0],  nav_pt_in_grid[1], DEFAULT_UAV_ALT / self.resolution))
            path.append(nav_pt)
        #print("path", path)
        return path

