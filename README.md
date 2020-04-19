#	使用大致流程

	0 
		git submodule update --init --recursive

	1 完成到GAAS教程3，注意PCL版本：1.8.1，protobuf最好别装，要装的话版本3.0.0
		https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen
	2 安装cartographer_ros，版本1.0.0
		https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
	3 复制Firmware覆盖原始Firmware

#	启动方式

	1 大致思路
		一个Terminal开gazebo
		一个Terminal开SLAM		如果是Cartographer可以用roslaunch和gazebo整合，如果是VSLAM没招
		一个Terminal开按键控制		按键控制必须和其他分开，当然按键控制也可以用GAAS的命令Python脚本代替
	2 文件结构
		Firmware			~/catkin_ws/src/Firmware
		Cartographer_ros		~/catkin_ws_src/src/内
		px4_indoor(本Project）		~/catkin_ws_ros/px4_indoor
		GAAS				~/catkin_ws_ros/px4_indoor/GAAS		（submodule）

		因为cartographer用catkin_make_isolated编译，和本工程冲突

##	3 GAAS课程3

		Terminal 1 (Gazebo)
			cd ~/catkin_ws/src/Firmware
			roslaunch launch/simple_tests/mavros_posix_sitl_rplidar_no_gps.launch

		Terminal 2 (VSLAM)
			~/catkin_ws_ros/src/px4_indoor/GAAS/software/SLAM/ygz_slam_ros

		Terminal 3 (Key manpulation)
			# cd catkin_ws_ros/
			# source devel/setup.bash
			rosrun px4_indoor px4_indoor_key_control

		注意：原来要用QGC改EKF2_AID_MASK，从1改到8，这个整合进启动脚本了

##	4 Cartographer Only

		改~/.bashrc最后几行为
			source /opt/ros/melodic/setup.bash
			#source ~/catkin_ws/devel/setup.bash
			source ~/catkin_ws_ros/devel/setup.bash
			#source ~/catkin_ws_src/devel/setup.bash
			source ~/catkin_ws/src/Firmware/Tools/setup_gazebo.bash ~/catkin_ws/src/Firmware/ ~/catkin_ws/src/Firmware/build/posix_sitl_default
			export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Firmware
			export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/Firmware/Tools/sitl_gazebo
		主要是上面几个source，如果出问题了，这没有好办法，就是一个个屏蔽了试过去 
			https://blog.csdn.net/xiaodingqq/article/details/88136323?depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3&utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-3
		
		
		Terminal 1 (Gazebo)
			cd ~/catkin_ws/src/Firmware
			roslaunch launch/indoor_cartographer/mavros_posix_sitl_rplidar_no_gps.launch

		Terminal 2 (Key manpulation)
			rosrun px4_indoor px4_indoor_key_control

##	5 Cartographer + VSLAM

		Terminal 1 (Gazebo)
			cd ~/catkin_ws/src/Firmware
			roslaunch launch/indoor_hybrid/mavros_posix_sitl_rplidar_no_gps.launch

		Terminal 2 (VSLAM)
			~/catkin_ws_ros/src/px4_indoor/GAAS/software/SLAM/ygz_slam_ros

		Terminal 3 (Key manpulation)
			rosrun px4_indoor px4_indoor_key_control

##	6 GAAS课程4

		下载Gazebo模型
                    https://gitee.com/trigger1996/gazebo_models
		这里完成下载后，model在~/src/Gazebo/model内

                现在默认整合到submodule里了，所以直接通过
                    git submodule update --init --recursive
                更新就可以不用做上一步
		
		修改～/.bashrc，在最后加一行
			export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws_ros/src/px4_indoor/gazebo_models

		Terminal 1
			cd ~/catkin_ws/src/Firmware
			roslaunch launch/GAAS/path_planning.launch


		本次实验是在有GPS情况下完成，以减少复杂度
                如果gazebo出问题，如spawn_vehicle啥的失败
                直接删一下~/.gazebo内所有log文件


##	7 自定 gps-denied path planning

		TODO:
			px4_mavros_run的默认起飞高度对激光来说太高了
			激光点云从2D投影改为3D投影
			视觉移植到ROS
			开启视觉
			开启自动导航

#	如何调试
	这份代码的整合度很高，原来GAAS等等其他需要3-4个终端才能开完的代码在这边一个终端全部开完
	如何确认这份代码能用？
	按照上面的数字顺序一点点运行代码以确定各个模块的好坏
	如果某个roslaunch文件有问题，则可分模块注释以检查性能

#	已知问题
	1 当前只有Ubuntu 18.04.4通过测试，老的18.04.2不知道为什么不行，18.04.3没有测试，因为自动登录有问题，不适合机载计算机使用（截止2020.4.14）

        2 Firmware源码只能放在~/catkin_ws/src/下，别的地方都不行，测了两天只得到这个结果

        3 model的显示有问题，激光雷达的蓝线不显示但是有数据，回学校以后可以看下以前是怎么做的

        4 下下来以后最好工程文件夹改名，从px4_indoor_navigation改成px4_indoor，不然得进文件一个个改

        5 关于激光-视觉复合建立3D地图，也就是match_pointcloud_and_slam_pose2这个node，现在代码有点问题
          因为cartographer得到的激光点云是全局坐标系的，而这个模块发布的是本地坐标系的，所以必须在这个node内
          做坐标变换
          里面那个is_project_laser本身是说用来添加z轴数据和做旋转的，可以不打开
          但是现在代码改为在xy方向上平移，所以必须打开
          必须打开
          必须打开

          为什么没有去掉这个？因为在真机上可能会有位姿数据/mavros/local_position/pose这个数据读不出来的情况，这个数据不去掉主要是为了提醒调真机的时候注意这个问题


#	几个参考
	为什么在ubuntu 18.04.2上编译会失败
		估计是模型没有编译通过
		模型通过gazebo和mavros交互，所以模型没过自然会造成系统爆炸

	在gazebo中给无人机添加激光雷达和光流传感器并开始仿真	
		https://zhuanlan.zhihu.com/p/31878534?from_voters_page=true

	如何确定激光雷达数据是否正常
		rostopic echo
		激光雷达数据是
			/laser/scan
	如何确定双目相机数据是否正常
		双目相机数据是
			/gi/simulation/
		然后后面一堆

	ROS常用操作
	rostopic
		rostopic list
		rostopic echo
                rostopic type
		rqt_graph
	TF
		https://www.cnblogs.com/CZM-/p/5879845.html
		rosrun tf tf_echo <坐标系1> <坐标系2>
		rosrun rqt_tf_tree rqt_tf_tree
		rosrun tf view_frames  

		<node pkg="tf" type="static_transform_publisher" name="join_laser" args="0 0 0 0 0 0 base_link laser 10" />

	如何使用rosparam
		https://blog.csdn.net/u014695839/article/details/78348600
		编程看网页
		命令看下面
		Commands:
			rosparam set	    set parameter                 设置参数
			rosparam get	    get parameter                 获得参数值
			rosparam load	    load parameters from file     从文件中加载参数到参数服务器
			rosparam dump       dump parameters to file       将参数服务器中的参数写入到文件
			rosparam delete     delete parameter              删除参数
			rosparam list       list parameter names          列出参数服务器中的参数


		http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters
		Python下
			# get a global parameter
			rospy.get_param('/global_param_name')

			# get a parameter from our parent namespace
			rospy.get_param('param_name')

			# get a parameter from our private namespace
			rospy.get_param('~private_param_name')

			You can also specify a default value if the parameter doesn't exist:

			rospy.get_param('foo', 'default_value')

		注意~后绝对不能加/

	如何使用cartographer
		https://ardupilot.org/dev/docs/ros-cartographer-slam.html
		https://blog.csdn.net/qq_38649880/article/details/88372390

		仿真：
			robot_pose_publisher
			cartographer的lua文件
			cartographer的launch文件
				注意tf
				有条件可以开rviz
		实体机：
			多一个rplidar_ros

        如何用rosrun或者roslaunch启动python代码
            https://answers.ros.org/question/56640/calling-a-python-script-with-rosrun/

            First, make sure the python script is executable
                (chmod +x mypythonscript.py).

            Next, make sure you have a shebang line at the top of the script that looks like
                #!/usr/bin/env python.

	如何使用gazebo_plugin自己设计传感器
		https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_range.cpp
		gazebosim.org/tutorials?tut=ros_gzplugins


		其实核心就是看着cpp写xml就行
		那个高度计就是自己做的


	如何使用gazebo自增加模型
		参考里面做好的
		就一个细节这里需要强调：
			以iris_lidar.sdf为例
			iris_lidar/iris_lidar.sdf内
			    <include>
      				<uri>model://range_finder</uri>
      				<pose>-0.12 0 -0.05 0 0 0</pose>
    				</include>
    				<joint name="range_finder" type="fixed">
      				<child>range_finder::link</child>
      				<parent>iris::base_link</parent>
      				<axis>

			range_finder/model.sdf内	
				<?xml version="1.0" ?>
				<sdf version="1.5">
  					<model name="range_finder">
    					<link name="link">

			这里的range_finder::link必须和range_finder和link对应
			不然传感器加不进来

##	加一个猜测
	为什么真机飞不起来？
		今天试了一把，误将pose类型（而非posestamped）类型的数据传入/mavros/vision_pose/pose内
		可以收，但是反馈的/mavros/altitude高度数据有问题，导致飞机可以解锁，但是无法起飞
		所以实体机到时候要注意一下高度

#	关于真机设计
	https://www.flyeval.com/
	https://www.flyeval.com/recalc.html

#	SPECAL THANKS to GAAS Team for their intellgence and inspiration for this project !


