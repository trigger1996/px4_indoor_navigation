#encoding=utf-8
import astar_config
import heapq

from nav_msgs.msg import OccupancyGrid
import rospy

#from simulator.visualization.config.aircraft_config import aircraft_obj

def gen_aircraft_obj():
    ret_val = []
    K = 3
    for i in range(-K, K):
        for j in range(-K, K):
            for k in range(-K, K):
                ret_val.append((i, j, k))

    ret_val.append((0, 0, -3))


    return ret_val


aircraft_obj = {
    "aircraft_points": gen_aircraft_obj(),
    "init_center": (0, 0, 0)
}


# 节点类
class Node_2D(object):
    def __init__(self, point):
        self.point = point
        self.parent = None
        self.G = 0
        self.H = 0

    def get_point(self):
        return self.point

    def move_cost(self, p):
        if self.point[0] != p[0] and self.point[1] != p[1]:
            return 1.414
        else:
            return 1.0

    def __str__(self):
        return "point : %s ,  G: %.1f,  H: %.1f,  F: %.1f" % (str(self.point), self.G, self.H, self.G + self.H)

# end_pos        : 结束点(x,y,z)
# openlist       : 开列表
# closed         : 关列表
# is_valid       : 判断点是否允许通过
# movement_list  : 允许转向的方向
# func_h         : 计算H值的算法
class A_star_2D(object):
    def __init__(self, end_pos):
        self.end_pos = end_pos

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
        self.horiontal_radius, self.vertical_radius = self.__aircraft_radius(aircraft_obj['aircraft_points'])
        # print ('aircraft_obj',aircraft_obj)

    # usage: self.algo.update_map(self.occupancy_grid_raw)
    #        self.algo.find_path(self.get_current_pose())
    def find_path(self, start_pos):
        self.openlist = []
        self.closed = set()
        start_obj = Node_2D(start_pos)
        #self.openlist.append(start_obj)

        heapq.heappush(self.openlist, ((start_obj.G+start_obj.H), start_obj))
        center_point_can_go = self.is_valid(self.end_pos, self.map_obstacle_indexed)   #判断中心点能否到达，如果中心点不能到达，则判断机身能否到达
        path = []
        count = 0

        if self.end_pos in self.map_obstacle_indexed:
            print('Target Position Found inside Obstacle, Finding Path Failed!')
            return None

        while self.openlist:
            #print ('obstacle_pos_list', self.map_obstacle_indexed)
            self.debug_get_openlist_info()
            count += 1

            current_obj = heapq.heappop(self.openlist)[1]

            if self.__success(current_obj, center_point_can_go):   #判断是否到达

                while current_obj.parent:
                    path.append(current_obj.get_point())
                    current_obj = current_obj.parent

                path.append(current_obj.get_point())

                print('count:', count)
                print("extend:", len(self.openlist)+len(self.closed))

                return path[::-1]


            self.closed.add(current_obj)

            self.extend_round(current_obj, self.map_obstacle_indexed)
            print("current_pt: ", current_obj.get_point())

        print('Open List Run Out, No Path Found.')
        return None

    def __distance(self, pt1, pt2):
        return ((pt1[0] - pt2[0]) ** 2 + (pt1[1] - pt2[1]) ** 2) ** 0.5

    def __aircraft_radius(self, aircraft_points):
        horiontal_radius = 0
        vertical_radius = 0
c        #center_point = aircraft_points[0]
        center_point = (0, 0, 0)

        print("center point is: ", center_point)

        for point in aircraft_points:
            if point[2] == 0:
                r = self.__distance(point,center_point)
                if horiontal_radius < r:
                    horiontal_radius = r
            if point[2] < 0:
                r = self.__distance(point,center_point)
                if vertical_radius < r:
                    vertical_radius = r

        print("horizontal radius and vertical: ", horiontal_radius, vertical_radius)
        return horiontal_radius, vertical_radius

    def __success(self, current_obj, center_point_can_go):
        # if reached target point, return True
        if center_point_can_go:
            if current_obj.get_point() == self.end_pos:
                return True
        # if distance small than body radius, return True
        elif not center_point_can_go:
            if self.__distance(current_obj.get_point(), self.end_pos) <= self.horiontal_radius:
                return True

        return False


    def debug_get_openlist_info(self):
        first_element = heapq.heappop(self.openlist)
        node = first_element[1]
        #print ('First element of openlist:',node.get_point())
        heapq.heappush(self.openlist, ((node.G+node.H), node))

    # 遍历周围节点，检测碰撞，寻找路径
    def extend_round(self, current_obj, obstacle_pos_list):

        # 8 direction
        for x, y in self.movement_list:

            new_point = (x + current_obj.get_point()[0],
                         y + current_obj.get_point()[1])

            node_obj = Node_2D(new_point)

            if not new_point in (self.map_obstacle_indexed | self.map_unknown_indexed):
                continue

            if not self.is_valid(new_point, obstacle_pos_list):
                continue

            if self.is_in_closedlist(new_point):
                continue

            if self.is_in_openlist(new_point):
                new_g = current_obj.G + current_obj.move_cost(new_point)
                if node_obj.G > new_g:
                    node_obj.G = new_g
                    node_obj.parent = current_obj
            else:
                node_obj = Node_2D(new_point)
                node_obj.G = current_obj.G + current_obj.move_cost(new_point)
                node_obj.H = self.func_h(node_obj.get_point(), self.end_pos)
                node_obj.parent = current_obj
                # self.openlist.append(node_obj)
                heapq.heappush(self.openlist, ((node_obj.G + node_obj.H), node_obj))

    def update_map(self, occupancy_grid_raw):
        width  = occupancy_grid_raw.info.width
        height = occupancy_grid_raw.info.height

        free_list     = []
        occupied_list = []
        unknown_list  = []
        for y in range(0, height):
            for x in range(0, width):
                data = occupancy_grid_raw.data[y * width + x]
                if data == -1:
                    unknown_list.append((x, y))
                elif data > 45:
                    occupied_list.append((x, y))
                else:
                    free_list.append((x, y))

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

    def is_in_closedlist(self, p):
        for node in self.closed:
            if node.point[0] == p[0] and node.point[1] == p[1]:
                return True
        return False

    def is_in_openlist(self, p):
        for heap_obj in self.openlist:
            node = heap_obj[1]
            if node.point[0] == p[0] and node.point[1] == p[1]:
                return True
        return False

    def is_valid(self, pt, obstacle_map):
        global aircraft_obj
        obstacle_map_indexed = set(obstacle_map)
        aircraft_points = []

        for item in aircraft_obj['aircraft_points']:
            aircraft_points.append(
                (item[0] + pt[0],
                 item[1] + pt[1])
            )

        aircraft_indexed = set(aircraft_points)
        # import pdb;pdb.set_trace()
        if obstacle_map_indexed & aircraft_indexed:
            return False
        else:
            return True

    def path_is_valid(self, path_points, obstacle_map):
        #    return False
        for pt in path_points:
            if not self.is_valid(pt, obstacle_map):
                print('Path is invalid.')
                return False

        print('Path is valid.')

        # print 'path_points:',path_points
        # print 'obstacle map:',obstacle_map
        return True

def find_alternative_cloest_point(final_pos, closed_list):
    dst = 10^6
    target_pt = [final_pos[0], final_pos[1]]
    for node in closed_list:
        x = node.point[0]
        y = node.point[1]
        # is_valid is confirmed in a*
        dst_temp = ((x - final_pos[0]) ** 2 + (y - final_pos[1]) ** 2) ** 0.5
        if dst_temp < dst:
            target_pt = [x, y]
            dst = dst_temp
    return target_pt
