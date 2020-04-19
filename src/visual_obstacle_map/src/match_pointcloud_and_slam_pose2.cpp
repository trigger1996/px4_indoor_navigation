#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <stereo_dense_reconstruction/CamToRobotCalibParamsConfig.h>
#include <fstream>
#include <ctime>
#include <opencv2/opencv.hpp>
#include "popt_pp.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

#include "pcl_helper.h"


using namespace cv;
using namespace std;


geometry_msgs::PoseStamped cur_pose;
sensor_msgs::PointCloud cur_pointcloud;
ros::Publisher output_cloud_pub;

pcl_helper* mpPCL_helper;

int is_integrate_laser = false;
int is_display_pose = false;
string helper_file_name = "config.yaml";
string src_prefix = "";

void Pose_callback(const geometry_msgs::PoseStamped& temp_pose)
{
    cur_pose = temp_pose;
}

sensor_msgs::PointCloud2 laser_pc;
bool is_laser_pc_updated = false;
void laser_pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    laser_pc = *msg;
    is_laser_pc_updated = true;
}

void PC_callback(const sensor_msgs::PointCloud2& temp_pc)
{

    sensor_msgs::PointCloud pc;
    bool result = sensor_msgs::convertPointCloud2ToPointCloud(temp_pc, pc);

    if (is_display_pose)
      cout<<"point cloud callback"<<endl;

    cur_pointcloud = pc;

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(cur_pose.pose.orientation.w, cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z);
    Eigen::Matrix3d mat = q.toRotationMatrix();

    for (int i = 0;i<3;i++)
    {
        for(int j =0 ;j<3;j++)
        {
            transform(i,j) = mat(i,j);
            if (is_display_pose)
                std::cout<<transform(i,j)<<"	|	";
        }
    }

    transform(0,3) = cur_pose.pose.position.x;
    transform(1,3) = cur_pose.pose.position.y;
    transform(2,3) = cur_pose.pose.position.z;

    if (is_display_pose) {
        for(int k=0;k<3;k++)
        {
          std::cout<<transform(k,3)<<" |";
        }
        std::cout<<"ENDL"<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto iter = cur_pointcloud.points.begin(); iter!=cur_pointcloud.points.end(); ++iter)
    {
        pcl::PointXYZ p;
        p.x = (*iter).x;
        p.y = (*iter).y;
        p.z = (*iter).z;
        original_cloud->push_back(p);
    }


    // do_some filtering here

    // option 1: PCLRadiusOutlierRemoval
    int UseRadiusOutlierRemoval = 1;
    if(UseRadiusOutlierRemoval)
    {
        if (is_display_pose)
            cout<<"original_cloud before: "<<*original_cloud<<endl;

        original_cloud = mpPCL_helper->PCLRadiusOutlierRemoval(original_cloud);

        if (is_display_pose)
            cout<<"original_cloud after: "<<*original_cloud<<endl;
    }

    // option 2: PCLStatisticalOutlierFilter

    // option 3: PCLConditionalRemoval


    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
    sensor_msgs::PointCloud  pc_out;

    for(auto iter = original_cloud->points.begin(); iter!=original_cloud->points.end(); ++iter)
    {
        geometry_msgs::Point32 p;
        p.x=(*iter).x;
        p.y=(*iter).y;
        p.z=(*iter).z;
        pc_out.points.push_back(p);
    }

    /// ADDED FOR POINT CLOUD UPDATED
    if (is_integrate_laser) {
        if (is_laser_pc_updated) {
            sensor_msgs::PointCloud laser_pc1;
            sensor_msgs::convertPointCloud2ToPointCloud(laser_pc, laser_pc1);
            int point_num = laser_pc1.points.size();

            // Assumed that laser point cloud is filtered
            // so it is added to point cloud directly
            for (int i = 0; i < point_num; i++) {
                pcl::PointXYZ pt(laser_pc1.points[i].x,
                                 laser_pc1.points[i].y,
                                 laser_pc1.points[i].z + cur_pose.pose.position.z);
                transformed_cloud->push_back(pt);
            }

            is_laser_pc_updated = false;
        }
    }

    sensor_msgs::PointCloud2 pc_out_pointcloud2;
    pcl::toROSMsg(*transformed_cloud, pc_out_pointcloud2);
    pc_out_pointcloud2.header.frame_id = "map";
    output_cloud_pub.publish(pc_out_pointcloud2);

//    pcl::PointCloud<pcl::PointXYZ>::Ptr  transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::transformPointCloud(*original_cloud, *transformed_cloud, transform);
//    sensor_msgs::PointCloud  pc_out;
//
//    for(auto iter = original_cloud->points.begin(); iter!=original_cloud->points.end(); ++iter)
//    {
//        geometry_msgs::Point32 p;
//        p.x=(*iter).x;
//        p.y=(*iter).y;
//        p.z=(*iter).z;
//        pc_out.points.push_back(p);
//    }
//
//
//    sensor_msgs::PointCloud2 pc_out_pointcloud2;
//
//    //NOTE filter pointcloud
//    transformed_cloud = mpPCL_helper->PCLStatisticalOutlierFilter(transformed_cloud);
//
//    pcl::toROSMsg(*transformed_cloud, pc_out_pointcloud2);
//    pc_out_pointcloud2.header.frame_id = "map";
//
//    output_cloud_pub.publish(pc_out_pointcloud2);
}


int main(int argc,char** argv)
{
    string node_name = "pointcloud_to_global_frame";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("match_pointcloud_and_slam_pose2");

    private_nh.getParam("is_integrate_laser", is_integrate_laser);
    private_nh.getParam("is_display_pose", is_display_pose);
    private_nh.getParam("src_prefix", src_prefix);

    helper_file_name = src_prefix + helper_file_name;
    mpPCL_helper = new pcl_helper(helper_file_name);

    cout << "Node initialized as: " << node_name << endl;
    cout << "[Px4 indoor] is_integrate_laser:  " << is_integrate_laser  << endl;
    cout << "[Px4 indoor] is_display_pose:     " << is_display_pose  << endl;
    cout << "[Px4 indoor] helper_file_path:    " << helper_file_name << endl;

    output_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in",1);

    ros::Subscriber pc_sub = nh.subscribe("/camera/left/point_cloud2", 1, PC_callback);
    ros::Subscriber pose_sub = nh.subscribe("/mavros/local_position/pose", 100, Pose_callback);

    ros::Subscriber laser_pc_sub = nh.subscribe("/scan_matched_points2", 10, laser_pc_callback);

    cout<<"sync_pc_callback 2"<<endl;

    ros::spin();
    return 0;
}


