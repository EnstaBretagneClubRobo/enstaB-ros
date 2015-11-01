#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <stdio.h>
#include <stdlib.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"


int main(int argc,char **argv)
{   
    ros::init(argc,argv,"imgfile_to_pub");

    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh;
    
    std::string filepath;
    if (argc == 2){
      filepath=argv[1];
    }
    else{
       if (!nh_private_.getParam ("filepath", filepath))
        filepath = "test.png"; 
    }

    ros::Rate rate(0.5);
    
    cv::Mat image;
    image = cv::imread(filepath, CV_LOAD_IMAGE_COLOR);   // Read the file

    if(! image.data )    // Check for invalid input
    {
        ROS_ERROR("Could not open or find the image %s",filepath.c_str());
        return -1;
    }

    ros::Publisher image_pub_= nh_private_.advertise<sensor_msgs::Image>("img", 5);

    cv_bridge::CvImage newIma;
    sensor_msgs::Image newIm;
    newIm.header.frame_id = "/img_file";
    newIm.header.stamp = ros::Time::now();
    newIma.image = image;
    newIma.encoding = sensor_msgs::image_encodings::BGR8;
    newIma.header = newIm.header;
    
    newIma.toImageMsg(newIm);


    while (ros::ok()){
     newIm.header.stamp = ros::Time::now();
     image_pub_.publish(newIm);
     rate.sleep();
    }

    return 0;
}

