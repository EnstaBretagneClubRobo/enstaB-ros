#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <sstream>
#include "sensor_msgs/Image.h" 

#define CAM_REC_BUFFER_SIZE 25

typedef boost::shared_ptr< cv_bridge::CvImage > Image;
Image bridge_;

bool start=false;

using namespace std;

void imageCB(const sensor_msgs::Image msg_ptr)
{
  sensor_msgs::Image frame = msg_ptr;
  
  bridge_ = cv_bridge::toCvCopy(frame, "bgr8");
  start = true;
  ROS_INFO("callback");
}

int main(int argc,char **argv)
{   
    ros::init(argc,argv,"movie_record");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    int fps;
    string filepath;

    if (argc == 2)
    {
      fps = atoi(argv[1]);
    }
    else if (argc == 3)
    {
      filepath = argv[2];
    }
    else {
      if (!nh_private_.getParam ("fps", fps))
        fps = 1;
      if (!nh_private_.getParam ("filepath", filepath))
        filepath = "./test/";
    }


    time_t t_now = time(0);
    struct tm* now = localtime(&t_now);
    stringstream path_ss;
    path_ss	<< filepath 
        << (now->tm_year + 1900) << "-"
        << setfill('0') << setw(2) << (now->tm_mon + 1) << "-"
        << setfill('0') << setw(2) << now->tm_mday << "--"
        << setfill('0') << setw(2) << now->tm_hour << ":"
        << setfill('0') << setw(2) << now->tm_min << ":"
        << setfill('0') << setw(2) << now->tm_sec << "--Video/";
    string path_record = (path_ss).str();
    string mkdir_command = "mkdir \"" + path_record + "\"";
    system(mkdir_command.c_str());
    
    ROS_INFO("de1");

    path_ss << "veo1.avi";

    ros::Subscriber image_sub = nh.subscribe("/camera/rgb/image_color",1,imageCB);

    ros::Rate rate(fps);
    cv::VideoWriter record(path_ss.str(), CV_FOURCC('D','I','V','X'), fps,cv::Size(320,240), true);

    if( !record.isOpened() ) {
        ROS_ERROR("VideoWriter failed to open!\n");
        return -1;
    }

    rate.sleep();
    ros::spinOnce();
    while(ros::ok())
    {   rate.sleep();
        ros::spinOnce();
        // add frame to recorded video
        //ROS -> OPENCV
        if (start){
        cv::Mat framecv =bridge_->image;
        bridge_->image.release();
        record << framecv;
        }
    }
    
    return 0;
}


