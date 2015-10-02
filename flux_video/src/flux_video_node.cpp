#include <ros/ros.h>
#include "sensor_msgs/Image.h" 


bool start=false;
sensor_msgs::Image frame;
using namespace std;

void imageCB(const sensor_msgs::Image msg_ptr)
{
  frame = msg_ptr;
  start = true;
}

int main(int argc,char **argv)
{   
    ros::init(argc,argv,"flux_video");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_("~");

    int fps = 9;

    ros::Subscriber image_sub = nh.subscribe("/camera/rgb/image_color/",1,imageCB);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/ground_station_flux",1);

    ros::Rate rate(fps);

    rate.sleep();
    ros::spinOnce();
    while(ros::ok())
    {   rate.sleep();
        ros::spinOnce();
        // add frame to recorded video
        //ROS -> OPENCV
        if (start){
           image_pub.publish(frame);
        }
    }
    
    return 0;
}


