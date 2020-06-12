#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <mavros_msgs/Altitude.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

mavros_msgs::Altitude external_alt;
bool is_external_alt_updated = false;
void external_alt_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    external_alt = *msg;
    is_external_alt_updated = true;
}

geometry_msgs::Pose laser_slam_pose;
bool is_laser_slam_pose_updated = false;
void laser_slam_pose_cb(const geometry_msgs::Pose::ConstPtr& msg){
    laser_slam_pose = *msg;
    is_laser_slam_pose_updated = true;
}

geometry_msgs::PoseStamped vslam_pose;
bool is_vslam_pose_updated = false;
void vslam_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    vslam_pose = *msg;
    is_vslam_pose_updated = true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "topic_relay_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("px4_indoor_topic_relay");

    int is_use_vslam_data = false;
    private_nh.getParam("is_use_vslam_data", is_use_vslam_data);

    double laser_lpf_K_xy       = 0.85;
    double external_alt_lpf_K_z = 1.00;
    double vision_lpf_K_xy      = 0.33;
    double vision_lpf_K_z       = 0.2;
    double data_fusion_K_xy = 0.66;
    double data_fusion_K_z  = 0.66;
    private_nh.getParam("laser_lpf_K_xy",       laser_lpf_K_xy);
    private_nh.getParam("external_alt_lpf_K_z", external_alt_lpf_K_z);
    private_nh.getParam("vision_lpf_K_xy",      vision_lpf_K_xy);
    private_nh.getParam("vision_lpf_K_z",       vision_lpf_K_z);

    // robot pose integration
    ros::Subscriber fcu_alt_sub = nh.subscribe<mavros_msgs::Altitude>
            ("/mavros/altitude", 10, external_alt_cb);
    ros::Subscriber laser_slam_pos_sub = nh.subscribe<geometry_msgs::Pose>
            ("/robot_pose", 10, laser_slam_pose_cb);
    ros::Publisher slam_pose_3d_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10);

    // slam fusion
    ros::Subscriber vslam_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 10, vslam_pose_cb);

    ros::Rate rate(20.0);

    cout << "[Px4 topic relay] Relay Started, content: " << endl;
    cout << "\t LaserScan" << endl;
    cout << "\t SLAMPose" << endl;
    cout << "\t SLAM Fusion: " << is_use_vslam_data << endl;

    while (ros::ok()) {

        if (is_use_vslam_data) {
            if (is_vslam_pose_updated && is_external_alt_updated && is_laser_slam_pose_updated) {
                // integrate 2D pose and alt to 3D
                geometry_msgs::PoseStamped pos_3d_to_pub;
                pos_3d_to_pub.header.stamp = ros::Time::now();
                pos_3d_to_pub.header.frame_id = "base_link";

                pos_3d_to_pub.pose.position.x = -laser_slam_pose.position.y;
                pos_3d_to_pub.pose.position.y = laser_slam_pose.position.x;
                pos_3d_to_pub.pose.position.z = external_alt.local;

                pos_3d_to_pub.pose.orientation.x = pos_3d_to_pub.pose.orientation.y =
                pos_3d_to_pub.pose.orientation.z = pos_3d_to_pub.pose.orientation.w = 0.;

                // data fusion
                pos_3d_to_pub.pose.position.x = pos_3d_to_pub.pose.position.x * data_fusion_K_xy +
                                                vslam_pose.pose.position.x * (1. - data_fusion_K_xy);
                pos_3d_to_pub.pose.position.y = pos_3d_to_pub.pose.position.y * data_fusion_K_xy +
                                                vslam_pose.pose.position.y * (1. - data_fusion_K_xy);
                pos_3d_to_pub.pose.position.z = pos_3d_to_pub.pose.position.z * data_fusion_K_z +
                                                vslam_pose.pose.position.z * (1. - data_fusion_K_z);

                slam_pose_3d_pub.publish(pos_3d_to_pub);

                is_vslam_pose_updated      = false;
                is_external_alt_updated    = false;
                is_laser_slam_pose_updated = false;

            }
        }
        else {
            if (is_external_alt_updated && is_laser_slam_pose_updated) {
                // integrate 2D pose and alt to 3D
                geometry_msgs::PoseStamped pos_3d_to_pub;
                pos_3d_to_pub.header.stamp = ros::Time::now();
                pos_3d_to_pub.header.frame_id = "base_link";

                pos_3d_to_pub.pose.position.x = -laser_slam_pose.position.y;
                pos_3d_to_pub.pose.position.y = laser_slam_pose.position.x;
                pos_3d_to_pub.pose.position.z = external_alt.local;

                pos_3d_to_pub.pose.orientation.x = pos_3d_to_pub.pose.orientation.y =
                pos_3d_to_pub.pose.orientation.z = pos_3d_to_pub.pose.orientation.w = 0.;

                slam_pose_3d_pub.publish(pos_3d_to_pub);

                is_external_alt_updated    = false;
                is_laser_slam_pose_updated = false;
            }
        }

//        if (is_use_vslam_data) {
//            if (is_external_alt_updated && is_laser_slam_pose_updated) {
//                // integrate 2D pose and alt to 3D
//                geometry_msgs::PoseStamped pos_3d_to_pub;
//                pos_3d_to_pub.header.stamp = ros::Time::now();
//                pos_3d_to_pub.header.frame_id = "map";

//                // LPF
//                pos_3d_to_pub.pose.position.x += (laser_slam_pose.position.x - pos_3d_to_pub.pose.position.x) * laser_lpf_K_xy;
//                pos_3d_to_pub.pose.position.y += (laser_slam_pose.position.y - pos_3d_to_pub.pose.position.y) * laser_lpf_K_xy;
//                pos_3d_to_pub.pose.position.z += (external_alt.relative      - pos_3d_to_pub.pose.position.z) * external_alt_lpf_K_z;

//                pos_3d_to_pub.pose.orientation.x = pos_3d_to_pub.pose.orientation.y =
//                pos_3d_to_pub.pose.orientation.z = pos_3d_to_pub.pose.orientation.w = 0.;

//                slam_pose_3d_pub.publish(pos_3d_to_pub);

//                is_external_alt_updated    = false;
//                is_laser_slam_pose_updated = false;
//            }

//            if (is_vslam_pose_updated) {
//                // integrate 2D pose and alt to 3D
//                geometry_msgs::PoseStamped pos_3d_to_pub;
//                pos_3d_to_pub.header.stamp = ros::Time::now();
//                pos_3d_to_pub.header.frame_id = "map";

//                // Vision LPF
//                pos_3d_to_pub.pose.position.x += (vslam_pose.pose.position.x - pos_3d_to_pub.pose.position.x) * vision_lpf_K_xy;
//                pos_3d_to_pub.pose.position.y += (vslam_pose.pose.position.y - pos_3d_to_pub.pose.position.y) * vision_lpf_K_xy;
//                pos_3d_to_pub.pose.position.z += (vslam_pose.pose.position.z - pos_3d_to_pub.pose.position.z) * vision_lpf_K_z;

//                pos_3d_to_pub.pose.orientation.x = pos_3d_to_pub.pose.orientation.y =
//                pos_3d_to_pub.pose.orientation.z = pos_3d_to_pub.pose.orientation.w = 0.;

//                slam_pose_3d_pub.publish(pos_3d_to_pub);

//                is_vslam_pose_updated      = false;
//            }
//        }
//        else {
//            if (is_external_alt_updated && is_laser_slam_pose_updated) {
//                // integrate 2D pose and alt to 3D
//                geometry_msgs::PoseStamped pos_3d_to_pub;
//                pos_3d_to_pub.header.stamp = ros::Time::now();
//                pos_3d_to_pub.header.frame_id = "map";

//                pos_3d_to_pub.pose.position.x = laser_slam_pose.position.x;
//                pos_3d_to_pub.pose.position.y = laser_slam_pose.position.y;
//                pos_3d_to_pub.pose.position.z = 0.;
//                //pos_3d_to_pub.pose.position.z = external_alt.local;

//                pos_3d_to_pub.pose.orientation.x = pos_3d_to_pub.pose.orientation.y =
//                pos_3d_to_pub.pose.orientation.z = pos_3d_to_pub.pose.orientation.w = 0.;

//                slam_pose_3d_pub.publish(pos_3d_to_pub);

//                is_external_alt_updated    = false;
//                is_laser_slam_pose_updated = false;
//            }
//        }

        ros::spinOnce();
        rate.sleep();
    }

    cout << "[Px4 topic relay] Relay Stopped" << endl;
    return 0;
}
