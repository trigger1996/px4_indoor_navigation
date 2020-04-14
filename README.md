#使用大致流程
	1 完成到GAAS教程3，注意PCL版本：1.8.1，protobuf最好别装，要装的话版本3.0.0
		https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen
	2 安装cartographer_ros，版本1.0.0
		https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
	3 复制Firmware覆盖原始Firmware

#启动方式
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
			cd catkin_ws_ros/
			source devel/setup.bash
			rosrun px4_indoor px4_indoor_key_control

		注意：原来要用QGC改EKF2_AID_MASK，从1改到8，这个整合进启动脚本了

##	4 Cartographer Only
		TODO

##	5 Cartographer + VSLAM
		TODO

##	6 GAAS课程4
		TODO

##	7 自定path planning
		TODO

#已知问题
	1 当前只有Ubuntu 18.04.4通过测试，老的18.04.2不知道为什么不行，18.04.3没有测试，因为自动登录有问题，不适合机载计算机使用（截止2020.4.14）
	2 Firmware源码只能放在~/catkin_ws/src/下，别的地方都不行，测了两天只得到这个结果
	3 model的显示有问题，激光雷达的蓝线不显示但是有数据，回学校以后可以看下以前是怎么做的

#几个参考
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

#SPECAL THANKS to GAAS Team for their intellgence and inspiration for this project !

