/**
 * @file key_main.cpp
 * @brief Offboard control node with key manpulation, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <tf/transform_datatypes.h>

#define TTY_PATH            "/dev/tty"
#define STTY_US             "stty raw -echo -F "
#define STTY_DEF            "stty -raw echo -F "

static int get_char();

static int get_char()       // https://blog.csdn.net/qq_14835443/article/details/50446099
{
    fd_set rfds;
    struct timeval tv;
    int ch = 0;

    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间

    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0)
    {
        ch = getchar();
    }

    return ch;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "key_rc_node");
    ros::NodeHandle nh;

    int ch = 0;
    system(STTY_US TTY_PATH);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ch = get_char();
        if (ch == 3)    //ctrl+c
            { system(STTY_DEF TTY_PATH); return 0; }

        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    double roll = 0., pitch = 0., yaw = 0.;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0.45;     // in meters

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // calculate current yaw
        //tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
        //tf::Matrix3x3 m(q);
        //m.getRPY(roll, pitch, yaw);

        // give control order
        ch = get_char();
        if (ch) {
            //printf("key = %d(%c)\n\r", ch, ch);
            switch (ch) {
                case 3://ctrl+c
                    { system(STTY_DEF TTY_PATH); return 0; }
                case 'w':
                case 'W':
                    pose.pose.position.x += 0.025;   break;      // 0.05
                case 'a':
                case 'A':
                    pose.pose.position.y += 0.025;   break;      // ENU
                case 's':
                case 'S':
                    pose.pose.position.x -= 0.025;   break;
                case 'd':
                case 'D':
                    pose.pose.position.y -= 0.025;   break;
                case 'q':
                case 'Q':
                    yaw -= 5. * M_PI / 180.; break;             // 旋转
                case 'e':
                case 'E':
                    yaw += 5. * M_PI / 180.; break;
                case 'x':
                case 'X':
                    pose.pose.position.z += 0.075;   break;      // 上升/下降
                case 'z':
                case 'Z':
                    pose.pose.position.z -= 0.075;   break;
            }
        }

        // set desired orintation
        tf::Quaternion q_;
        q_.setRPY(roll, pitch, yaw);
        //printf("yaw: %f, q_: %f %f %f %f\n\n", yaw, q_.x(), q_.y(), q_.z(), q_.w());

        // update orientation in desired pose
        pose.pose.orientation.x = q_.x();
        pose.pose.orientation.y = q_.y();
        pose.pose.orientation.z = q_.z();
        pose.pose.orientation.w = q_.w();

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
