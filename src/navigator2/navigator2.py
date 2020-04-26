#!/usr/bin/env python
#encoding=utf-8

'''
        project overview:

	Subscribe:
		1.slam pose(global/local pose) *
		2.octomap_server/global map
		3.local pointcloud/local octomap
		4.target input(semantic target/visual pose target/gps target)
	Publish:
		1.Mavros(amo) Command
		2.Navigator status

	Algorithms:
		1.D*
		2.state transfer
		3.position->position PID controller
		4.global/semantic/visual target to local pose
'''

import threading
import time
from path_optimization.path_pruning import PathPruning, PathPruning_wScale
# for ros
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32, String
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu, NavSatFix, PointCloud, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker,MarkerArray
# for mavros
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget#, Command
from mavros_msgs.srv import CommandBool, SetMode


# for octomap
from octomap_msgs.msg import Octomap, OctomapWithPose, octomap_msgs


# other useful utilities
#from pyquaternion import Quaternion
import pyquaternion

import astar.astar
import astar.astar_2d
import astar.driver
import occupancy_grid_helper

import time
import math
from enum import Enum
import thread
#from queue import Queue

#from Pos2PosController import Pos2PosController as Controller  # TODO:re-implement this.
from SimController import Controller as Controller
from SimController import atan2_yaw as atan2_yaw
import DiscreteGridUtils

import numpy as np

from navigator2_config  import navigator2_config as navigator2_config

# define system status
class status(Enum):
    INITIALIZED = 1

    LOOKING_FOR_PATH = 2
    LOOKING_FOR_PATH_SUCCEED = 3
    LOOKING_FOR_PATH_FAILED = 4

    EXPLORATION = 5
    GOING_TO_TARGET = 6
    GOING_TO_VISION_TARGET = 7
	

def dist(pos1,pos2):
    if not pos1 or not pos2:
        return False, 0
    else:
        return True, reduce(lambda x,y:x+y,map(lambda i:(pos1[i]-pos2[i])**2,[0,1,2]))

class Navigator:
    
    def __init__(self,config_file_path = None):
        if config_file_path:
            pass

        # Parameters for debugging
        self.is_disable_waiting_time      = navigator2_config['disable_waiting_time']
        self.is_enable_testmap_publishing = navigator2_config['enable_publishing_test_map']

        rospy.init_node("gi_navigator_node")
        self.dg = DiscreteGridUtils.DiscreteGridUtils(grid_size=0.2)#0.2)
        self.rate = rospy.Rate(50)
        self.driver = astar.driver.Driver()
        self.controller = Controller()
        self.mavros_state = "OFFBOARD"
        self.set_status(status.INITIALIZED)

        self.cur_command_id = 0
        self.prev_command_id = 0
        self.cur_target_position=None           # in grids
        self.cur_target_position_raw = None     # in meters

        self.task_id = -1
        self.obstacle_set_mutex = threading.Lock()  # mutex.acquire(timeout);mutex.release()
        self.occupancy_grid_mutex = threading.Lock()
        self.nav_command_mutex = threading.Lock()  # for nav command in dstar and ros high level command.
        self.local_pose = None                  # in meters
        self.local_pose_raw = None              # in meters
        t1 = threading.Thread(target=self.ros_thread)
        t1.start()

        self.obs_set_raw = None

        self.is_local_pose_updated = False
        self.is_obstacle_set_updated = False

        self.navigator_status_pub = rospy.Publisher('/gi/navigator_status', String, queue_size=10)
        self.path_plan_pub = rospy.Publisher('/gi/navi_path_plan',MarkerArray,queue_size=10)
        #t2 = thread.start_new_thread(self.Dstar_thread, ())

        # Parameters
        self.is_use_bresenham = navigator2_config['is_use_bresenham']                   # 这个开了以后是用3D地图避障
        self.is_remove_collinear_pts = navigator2_config['is_remove_collinear_pts']     # 这个开了以后会飞的更快，关了有更好的避障效果
                                                                                        # 避障可能会让飞机在某些地方“飞不动”
        self.is_rotate_uav = navigator2_config['is_rotate_uav']                         # 飞行过程中机头指向目标，建议开启

        self.path = []
        self.path_prune = PathPruning(obstacle_distance=8)
        self.path_prune_2D = PathPruning_wScale(obstacle_distance=navigator2_config['pathpruning_obstacle_dst'],
                                                resolution=navigator2_config['pathpruning_resolution'])

        time.sleep(2)


    '''
    Navigating thread    
    '''
    def keep_navigating(self):

        ### Added, wait for several seconds for mavros switching to offboard
        for i in range(0, 25):
            if self.mavros_state == "OFFBOARD":
                break
            time.sleep(1)

        # wait a second for better initial mapping
        if not self.is_disable_waiting_time:
            print("wait for initial mapping...")
            time.sleep(15)

        while self.mavros_state == "OFFBOARD" and not(rospy.is_shutdown()):

            # print ('Inside outer loop!')
            #print ("Navigator state: ", self.STATUS.data, "Mavros state: ", self.mavros_state)
            relative_pos = (0, 0, 0)

            end_pos = self.get_latest_target()

            current_pos = self.get_current_pose() # TODO:fix this.
            if current_pos is None:
                #print ('current pose not valid!')
                continue

            while current_pos != end_pos and not self.navi_task_terminated() and not(rospy.is_shutdown()):  # Till task is finished:

                '''
                    几个比较坑的问题
                    1 地图提供的origin是地图(0, 0, 0)点对应实际坐标系的位置
                    2 2D地图和3D地图的分辨率是不同的                    
                    
                    核心编程思想：
                    1 这个函数内所有的量纲都是公制，在各个模块内换成栅格，输出的时候再换回公制
                    2 打印一定要清晰                    
                
                '''

                if not self.is_local_pose_updated:
                    time.sleep(1)
                    continue
                else:
                    self.is_local_pose_updated = False

                if not self.is_obstacle_set_updated:
                    time.sleep(1)
                    continue
                else:
                    self.is_obstacle_set_updated = False

                current_pos = self.get_current_pose()       # in grids
                end_pos = self.dg.continuous_to_discrete((self.cur_target_position_raw[0],
                                                          self.cur_target_position_raw[1],
                                                          self.cur_target_position_raw[2]))   # in_grids

                # A* algorithm
                self.algo = astar.astar_2d.A_star_2D(self.cur_target_position_raw,
                                                     vehicle_width=2, vehicle_length=2,
                                                     resolution=self.occupancy_grid_raw.info.resolution)

                occupancy_map_resized = occupancy_grid_helper.resize(self.occupancy_grid_raw, 0.2)
                self.algo.update_map(occupancy_map_resized)

                # for debugging
                if self.is_enable_testmap_publishing:
                    data = self.algo.map_list_to_occupancy_grid()
                    self.occupancy_grid_pub.publish(data)

                path = self.algo.find_path(self.local_pose_raw, self.cur_target_position_raw)
                if path != None:
                    print("2D path found!")

                    # publish raw path plan.
                    m_arr = MarkerArray()
                    marr_index = 0
                    for point in alternative_path:
                        mk = Marker()
                        mk.header.frame_id = "map"
                        mk.action = mk.ADD
                        mk.id = marr_index
                        marr_index += 1
                        mk.color.r = 1.0
                        mk.color.a = 1.0
                        mk.type = mk.CUBE
                        mk.scale.x = 0.6
                        mk.scale.y = 0.6
                        mk.scale.z = 0.6
                        mk.pose.position.x = point[0]
                        mk.pose.position.y = point[1]
                        mk.pose.position.z = point[2]
                        m_arr.markers.append(mk)
                    self.path_plan_pub.publish(m_arr)


                    if self.is_remove_collinear_pts:
                        path = self.path_prune_2D.remove_collinear_points(path)
                    if self.is_use_bresenham:
                        path = self.path_prune_2D.path_pruning_bresenham3d(path, self.obs_set_raw)
                    print("Path: ", path)

                else:
                    print("2D path NOT found!")
                    alternative_path = self.algo.find_alternative_path([self.cur_target_position_raw[0], self.cur_target_position_raw[1]])

                    # publish raw path plan.
                    m_arr = MarkerArray()
                    marr_index = 0
                    for point in alternative_path:
                        mk = Marker()
                        mk.header.frame_id = "map"
                        mk.action = mk.ADD
                        mk.id = marr_index
                        marr_index += 1
                        mk.color.r = 1.0
                        mk.color.a = 1.0
                        mk.type = mk.CUBE
                        mk.scale.x = 0.6
                        mk.scale.y = 0.6
                        mk.scale.z = 0.6
                        mk.pose.position.x = point[0]
                        mk.pose.position.y = point[1]
                        mk.pose.position.z = point[2]
                        m_arr.markers.append(mk)
                    self.path_plan_pub.publish(m_arr)

                    # 路径简化
                    if self.is_remove_collinear_pts:
                        alternative_path = self.path_prune_2D.remove_collinear_points(alternative_path)
                    if self.is_use_bresenham:
                        alternative_path = self.path_prune_2D.path_pruning_bresenham3d(alternative_path, self.obs_set_raw)

                    # 因为在闭列表内，所以不可能找不到备用路径的
                    print("Alternative path found: ", alternative_path)

                    #move_iter = 0
                    #move_iter_max = 3
                    for next_move in alternative_path:
                        if self.navi_task_terminated():
                            break

                        current_pos = self.get_current_pose()  # in grids
                        print ('current_pos:', current_pos)
                        next_pos = next_move
                        relative_pos = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1],
                                        next_pos[2] - current_pos[2])

                        self.current_pos = next_pos

                        #axis transform                        # TODO
                        if not self.algo.is_valid(next_pos[0], next_pos[1], True):
                            print ('Path not valid!')
                            break
                        relative_pos_new = (-relative_pos[0], -relative_pos[1], relative_pos[2])

                        #self.controller.mav_move(*relative_pos_new,abs_mode=False) # TODO:fix this.
                        print ('mav_move() input: relative pos=',next_pos)

                        if self.is_use_bresenham:
                            while self.distance(self.local_pose_raw, next_pos) >= 0.66:
                                self.controller.mav_move(next_pos[0],next_pos[1],next_pos[2], abs_mode=True)  # TODO:fix this.
                                time.sleep(2)
                        else:
                            self.controller.mav_move(next_pos[0], next_pos[1], next_pos[2], abs_mode=True)  # TODO:fix this.
                            time.sleep(2)

                        # rotate vehicle, the position is no use
                        if self.is_rotate_uav:
                            target_yaw = atan2_yaw(relative_pos[1], relative_pos[0]) * 180. / math.pi
                            self.controller.mav_move(next_pos[0], next_pos[1], next_pos[2], yaw = target_yaw)

                    continue


                for next_move in path:
                    if self.navi_task_terminated():
                        break

                    current_pos = self.get_current_pose()  # in grids
                    print ('current_pos:', current_pos)
                    next_pos = next_move
                    relative_pos = (next_pos[0] - current_pos[0], next_pos[1] - current_pos[1],
                                    next_pos[2] - current_pos[2])

                    # TODO
                    if not self.algo.is_valid(next_pos[0], next_pos[1], True):
                        print ('Path not valid!')
                        break

                    self.current_pos = next_pos

                    #axis transform
                    relative_pos_new = (-relative_pos[0], -relative_pos[1], relative_pos[2])



                    #self.controller.mav_move(*relative_pos_new,abs_mode=False) # TODO:fix this.
                    print ('mav_move() input: relative pos=',next_pos)
                    if self.is_use_bresenham:
                        while self.distance(self.local_pose_raw, next_pos) >= 0.66:
                            self.controller.mav_move(next_pos[0], next_pos[1], next_pos[2],
                                                     abs_mode=True)  # TODO:fix this.
                            time.sleep(2)
                    else:
                        self.controller.mav_move(next_pos[0], next_pos[1], next_pos[2], abs_mode=True)  # TODO:fix this.
                        time.sleep(2)

                        # rotate vehicle, the position is no use
                        if self.is_rotate_uav:
                            target_yaw = atan2_yaw(relative_pos[1], relative_pos[0]) * 180. / math.pi
                            self.controller.mav_move(next_pos[0], next_pos[1], next_pos[2], yaw = target_yaw)

                current_pos = self.get_current_pose()
                time.sleep(0.05) # wait for new nav task.

            return

        print("Mavros not in OFFBOARD mode, Disconnected!")


    '''
    move quad in body frame
    '''

    def distance(self, p1, p2):
        x_distance = (p2[0] - p1[0])**2
        y_distance = (p2[1] - p1[1])**2
        z_distance = (p2[2] - p1[2])**2

        return np.sqrt(x_distance + y_distance + z_distance)


    def terminate_navigating(self):
        #TODO
        pass

    def resume_navigating(self):
        #TODO
        pass

    def do_hover(self):
        #TODO
        pass


    def set_target_postion(self, target_position):
        self.found_path = True
        self.cur_target_position_raw = target_position
        self.cur_target_position     = self.dg.continuous_to_discrete(target_position)
        print("Current target position in grid: ", self.cur_target_position)
        #print("Set Current Position to: ", target_position[0], target_position[1], target_position[2])

    def get_latest_target(self):
        return self.cur_target_position

    def set_vision_target(self):
        self.set_status(status.GOING_TO_VISION_TARGET)
        self.set_target_position(xxxxx) #TODO
        pass

    def navi_task_terminated(self):
        if dist(self.local_pose,self.cur_target_position) <0.25:  #TODO: or stop flag is set.
            return True
        else:
            return False

    '''
    Dstar Thread
    
    def Dstar_thread(self):
        while not rospy.is_shutdown():
            while status!= xxx:# TODO
                next_move = xxx
                return next_move'''

    '''##For test:
        target = [0.5, 0.5, 0.5]
        self.set_target_postion(target)
        pass'''


    '''
    ROS thread
    responsible for subscribers and publishers
    '''
    def ros_thread(self):
        print('ros_thread spawn!!!!')
        self.octomap_msg = None

        # subscribers
        self.slam_sub = rospy.Subscriber("/gi/slam_output/pose", PoseStamped, self.slam_pose_callback)
        self.vision_target_sub = rospy.Subscriber("/gi/visual_target/pose", PoseStamped, self.vision_target_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera/left/point_cloud", PointCloud, self.point_cloud_callback)
        self.occupancy_grid_sub = rospy.Subscriber("/map", OccupancyGrid, self.occupancy_grid_callback)
        self.octomap_cells_vis = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_update_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)


        # publishers
        #self.mavros_control_pub = rospy.Publisher('mavros/Command', Command, queue_size=10)
        if self.is_enable_testmap_publishing:
            self.occupancy_grid_pub = rospy.Publisher('/map_test', OccupancyGrid, queue_size=10)

        self.set_status(status.INITIALIZED)

        rospy.spin()


    '''
    ROS callbacks
    '''
    def slam_pose_callback(self, msg):
        self.slam_pose = msg

    def vision_target_callback(self, msg):
        self.vision_target = msg
        #print("Received New Vision Target!")

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        #print(msg.mode, type(msg.mode))
        self.navigator_status_pub.publish(self.STATUS)

    def point_cloud_callback(self, msg):
        self.current_point_cloud = msg

    def occupancy_grid_callback(self, msg):
        self.occupancy_grid_raw = msg

    def octomap_update_callback(self, msg):  # as pointcloud2.
        obs_set     = set()
        obs_set_raw = set()
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #print " x : %f  y: %f  z: %f" % (p[0], p[1], p[2])
            point = self.dg.continuous_to_discrete((p[0],p[1],p[2]))
            #print ('point:',point)
            obs_set.add(point)
            obs_set.add(p)
        acquired = self.obstacle_set_mutex.acquire(True)  # blocking.
        if acquired:
            #print('octomap updated!')
            self.driver.set_obstacle_set(obs_set)
            self.obs_set_raw = obs_set_raw
            self.obstacle_set_mutex.release()

            self.is_obstacle_set_updated = True
            return
        else:
            print ('Lock not acquired!')

    def local_pose_callback(self, msg):
        pose_ = msg.pose.position #TODO:do fusion with visual slam.
        self.local_pose_raw = (pose_.x,pose_.y,pose_.z)
        self.local_pose = self.dg.continuous_to_discrete((pose_.x,pose_.y,pose_.z))
        #print ('local_pose set!!!')
        self.is_local_pose_updated = True

    def get_local_pose(self): # in mavros axis.for command.
        #print ('self.local_pose',self.local_pose)
        return self.local_pose

    def get_current_pose(self): # current pose:slam pose(in world axis)
        return self.get_local_pose() # TODO:do transform T1 ^-1 * T2.



    '''
    helper functions
    '''
    def set_status(self, status):
        self.STATUS = String(status.name)

    '''
    TODO
    '''
    def reachTargetPosition(self, target, threshold = 0.1):

        delta_x = math.fabs(self.local_pose.pose.position.x - target.pos_sp[0])
        delta_y = math.fabs(self.local_pose.pose.position.y - target.pos_sp[1])
        delta_z = math.fabs(self.local_pose.pose.position.z - target.pos_sp[2])

        distance = (delta_x ** 2 + delta_y ** 2 + delta_z ** 2)  ** 0.5

        print("distance: ", distance, "threshold: ", threshold)
        if distance < threshold:
            return True
        else:
            return False


    def setMavMode(self, msg):
        pass


if __name__ == '__main__':
    nav = Navigator()

    # in FLU meters
    target_x = rospy.get_param('~target_x', 30)     # 10, 0, 2.5, modified here for debugging
    target_y = rospy.get_param('~target_y', 6)
    target_z = rospy.get_param('~target_z', 1.5)

    print("[Px4 indoor] target position: ", [target_x, target_y, target_z])

    #FLU meters.
    nav.set_target_postion((target_x, target_y, target_z))
    nav.keep_navigating()
