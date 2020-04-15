#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/Altitude.h>
#include <geometry_msgs/Pose.h>

using namespace std;

sensor_msgs::LaserScan laser;
bool is_laser_updated = false;
void laser_raw_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser = *msg;
    is_laser_updated = true;
}

mavros_msgs::Altitude fcu_alt;
bool is_fcu_alt_updated = false;
void fcu_alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    fcu_alt = *msg;
    is_fcu_alt_updated = true;
}

geometry_msgs::Pose laser_slam_pose;
bool is_laser_slam_pose_updated = false;
void laser_slam_pose_cb(const geometry_msgs::Pose::ConstPtr& msg){
    laser_slam_pose = *msg;
    is_laser_slam_pose_updated = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "topic_relay_node");
    ros::NodeHandle nh;

    // laser relay
    ros::Subscriber laser_raw_sub = nh.subscribe<sensor_msgs::LaserScan>
            ("/laser/scan", 10, laser_raw_cb);
    ros::Publisher laser_relay_pub = nh.advertise<sensor_msgs::LaserScan>
            ("/scan", 10);

    // robot pose integration
    ros::Subscriber fcu_alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("/mavros/altitude", 10, fcu_alt_cb);
    ros::Subscriber laser_slam_pos_sub = nh.subscribe<geometry_msgs::Pose>
            ("/robot_pose", 10, laser_slam_pose_cb);
    ros::Publisher slam_pose_3d_pub = nh.advertise<geometry_msgs::Pose>
            ("/mavros/vision_pose/pose", 10);

    ros::Rate rate(20.0);

    cout << "[Px4 topic relay] Relay Started, content: " << endl;
    cout << "\t LaserScan" << endl;
    cout << "\t SLAMPose" << endl;

    while (ros::ok()) {

        if (is_laser_updated) {
            // update header and frame_id
            laser.header.stamp = ros::Time::now();
            laser.header.frame_id = "laser";
            laser_relay_pub.publish(laser);

            is_laser_updated = false;
        }

        if (is_fcu_alt_updated && is_laser_slam_pose_updated) {
            // integrate 2D pose and alt to 3D
            laser_slam_pose.position.z = fcu_alt.relative;
            slam_pose_3d_pub.publish(laser_slam_pose);

            is_fcu_alt_updated = false;
            is_laser_slam_pose_updated = false;
        }

        ros::spinOnce();
        rate.sleep();
    }

    cout << "[Px4 topic relay] Relay Stopped" << endl;
    return 0;
}
