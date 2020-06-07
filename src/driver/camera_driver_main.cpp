#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace std::chrono;
using namespace cv;

double fps();

// https://blog.csdn.net/github_30605157/article/details/50990493?locationNum=13&fps=1

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "camera_driver");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("px4_indoor_cam_driver");

    // ros params
    bool is_show_img = false;
    bool is_calculate_fps = false;
    int  resolution_x = 1280;
    int  resolution_y = 960;
    string img_l_topic = "/gi/simulation/left/image_raw";
    string img_r_topic = "/gi/simulation/right/image_raw";

    private_nh.getParam("is_show_img",       is_show_img);
    private_nh.getParam("is_calculate_fps",  is_calculate_fps);
    private_nh.getParam("resolution_x",      resolution_x);
    private_nh.getParam("resolution_y",      resolution_y);
    private_nh.getParam("image_left_topic",  img_l_topic);
    private_nh.getParam("image_right_topic", img_r_topic);
    resolution_x *= 2;


    // ROS varibles
    image_transport::ImageTransport it(nh);
    image_transport::Publisher img_l_pub = it.advertise(img_l_topic,  1);
    image_transport::Publisher img_r_pub = it.advertise(img_r_topic, 1);

    // OpenCV varibles
    VideoCapture cap(0);

    Mat frame;
    double fps_val = 0, fps_val_last = 0;

    if (!cap.isOpened()) {
        cout << "[Camera Driver] Camera Open Failed!" << endl;
        return -1;
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH,  resolution_x);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, resolution_y);

    cap >> frame;
    cout << "[Camera Driver] Image Width:" << frame.cols / 2 << "\t Height: " << frame.rows << endl;
    cout << "[Camera Driver] is_show_img: "      << is_show_img      << endl;
    cout << "[Camera Driver] is_calculate_fps: " << is_calculate_fps << endl;

    while (ros::ok()) {
        cap >> frame;

        int img_width  = frame.cols;
        int img_height = frame.rows;

        CvRect rect_R = cvRect(0, 0, img_width / 2, img_height);
        CvRect rect_L = cvRect(img_width / 2, 0, img_width / 2, img_height);
        Mat frame_L = frame(rect_L);
        Mat frame_R = frame(rect_R);

        if (is_show_img) {
            //imshow("raw", frame);
            imshow("image_L", frame_L);
            imshow("image_R", frame_R);
        }

        if (is_calculate_fps) {
            fps_val = fps();
            if (fps_val_last != fps_val) {
                cout << "[Camera Driver] FPS: " << fps_val << endl;
                fps_val_last = fps_val;
            }
        }

        sensor_msgs::ImagePtr msg_L = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_L).toImageMsg();
        sensor_msgs::ImagePtr msg_R = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_R).toImageMsg();
        img_l_pub.publish(msg_L);
        img_r_pub.publish(msg_R);


        waitKey(1);
        ros::spinOnce();

    }

    cout << 233 << endl;
    return 0;
}

double fps() {
    static double fps = 0.0;
    static int frameCount = 0;
    static auto lastTime = system_clock::now();
    static auto curTime = system_clock::now();

    curTime = system_clock::now();
    auto duration = duration_cast<microseconds>(curTime - lastTime);
    double duration_s = double(duration.count()) * microseconds::period::num / microseconds::period::den;

    if (duration_s > 2) { //2秒之后开始统计FPS
        fps = frameCount / duration_s;
        frameCount = 0;
        lastTime = curTime;
    }
    ++frameCount;
    return fps;
}

