使用大致流程
	1 完成到GAAS教程3，注意PCL版本：1.8.1，protobuf最好别装，要装的话版本3.0.0
		
	2 安装cartographer_ros，版本1.0.0
		https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html
	3 复制Firmware覆盖原始Firmware
		https://gaas.gitbook.io/guide/software-realization-build-your-own-autonomous-drone/wu-ren-ji-zi-dong-jia-shi-xi-lie-offboard-kong-zhi-yi-ji-gazebo-fang-zhen

已知问题
	1 当前只有Ubuntu 18.04.4通过测试，老的18.04.2不知道为什么不行，18.04.3没有测试，因为自动登录有问题，不适合机载计算机使用（截止2020.4.14）
	2 Firmware源码只能放在~/catkin_ws/src/下，别的地方都不行，测了两天只得到这个结果
	3 model的显示有问题，激光雷达的蓝线不显示但是有数据，回学校以后可以看下以前是怎么做的

几个参考
	在gazebo中给无人机添加激光雷达和光流传感器并开始仿真	
	https://zhuanlan.zhihu.com/p/31878534?from_voters_page=true

