#include <iostream>
#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>

using namespace std;

geometry_msgs::PoseStamped slam_pose;
bool is_slam_pose_updated = false;
void slam_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    slam_pose = *msg;
    is_slam_pose_updated = true;
}

gazebo_msgs::ModelStates gazebo_pose_all;
bool is_gazebo_pose_updated = false;
void gazebo_pose_cb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    gazebo_pose_all = *msg;
    is_gazebo_pose_updated = true;
}


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "trajectory_recorder_node");
    ros::NodeHandle nh;

    ros::Subscriber slam_pose_sub   = nh.subscribe("/mavros/local_position/pose", 10, slam_pose_cb);
    ros::Subscriber gazebo_pose_sub = nh.subscribe("/gazebo/model_states", 10, gazebo_pose_cb);

    ros::Publisher slam_pos_pub = nh.advertise<nav_msgs::Path>
            ("debug/slam_trajectory", 10);
    ros::Publisher gazebo_pos_pub = nh.advertise<nav_msgs::Path>
            ("debug/gazebo_trajectory", 10);

    nav_msgs::Path slam_path_to_pub;
    nav_msgs::Path gazebo_path_to_pub;

    slam_path_to_pub.header.frame_id   = "map";
    gazebo_path_to_pub.header.frame_id = "map";

        cout << "[Trajectory Recorder] record started" << endl;

    while (ros::ok()) {
        if (is_slam_pose_updated) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position    = slam_pose.pose.position;
            pose.pose.orientation = slam_pose.pose.orientation;

            slam_path_to_pub.header.stamp = ros::Time::now();
            slam_path_to_pub.poses.push_back(pose);
            slam_pos_pub.publish(slam_path_to_pub);

            is_slam_pose_updated = false;
        }

        if (is_gazebo_pose_updated) {
            int iris_index = -1;
            int gazebo_pose_arr_len = gazebo_pose_all.name.size();
            for (int i = 0; i < gazebo_pose_arr_len; i++) {
                if (gazebo_pose_all.name[i].find(string("iris"), 0) != string::npos)
                    iris_index = i;
            }
            geometry_msgs::PoseStamped pose;
            pose.pose.position    = gazebo_pose_all.pose[iris_index].position;
            pose.pose.orientation = gazebo_pose_all.pose[iris_index].orientation;

            gazebo_path_to_pub.header.stamp = ros::Time::now();
            gazebo_path_to_pub.poses.push_back(pose);
            gazebo_pos_pub.publish(gazebo_path_to_pub);


            is_gazebo_pose_updated = false;
        }

        ros::spinOnce();
        usleep(500e3);      // 500ms
    }

    cout << "[Trajectory Recorder] record stopped" << endl;
    return 0;
}

