#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

sensor_msgs::LaserScan laser;
bool is_laser_updated = false;
void laser_raw_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser = *msg;
    is_laser_updated = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "topic_relay_node");
    ros::NodeHandle nh;

    ros::Subscriber laser_raw_sub = nh.subscribe<sensor_msgs::LaserScan>
            ("/laser/scan", 10, laser_raw_cb);
    ros::Publisher laser_relay_pub = nh.advertise<sensor_msgs::LaserScan>
            ("/scan", 10);

    ros::Rate rate(20.0);

    cout << "[Px4 topic relay] Relay Started, content: " << endl;
    cout << "\t LaserScan" << endl;

    while (ros::ok()) {

        if (is_laser_updated) {
            // update header and frame_id
            laser.header.stamp = ros::Time::now();
            laser.header.frame_id = "laser";
            laser_relay_pub.publish(laser);

            is_laser_updated = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    cout << "[Px4 topic relay] Relay Stopped" << endl;
    return 0;
}
