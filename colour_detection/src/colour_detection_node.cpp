
#include "colour_detection/colour_detection_node.h"

namespace colour_detect {
//constructor ColourVision
ColourVision::ColourVision():seuilH(150),
    seuilB(0),
    seuilHh(150),
    seuilHs(150),
    seuilHv(150),
    seuilBh(0),
    seuilBs(0),
    seuilBv(0),
    nbIter(0),
    nbIterD(0)
{   
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh;

    if (!nh_private_.getParam ("publish_image", publish_image_))
        publish_image_ = false; 
    image_sub_ = nh_private_.subscribe("/camera/rgb/image_color",1,&ColourVision::imageCB,this);
    image_pub_= nh_private_.advertise<sensor_msgs::Image>("image_processed", 5);
    //thresholds_server_ = nh_private_.advertiseService("set_thresholds",/*temp*/,this);//temp
    cvNamedWindow("test", CV_WINDOW_AUTOSIZE );
};

//destructor Astar
ColourVision::~ColourVision(){
};

void ColourVision::imageCB(const sensor_msgs::ImageConstPtr& msg_ptr){
  CvMat cv_image;
  
  bridge_ = cv_bridge::toCvCopy(msg_ptr, "bgr8");
  cv_image = bridge_->image ;
  IplImage *ipl_image ;
  IplImage *sub = cvCreateImage(cvSize(bridge_->image.cols, bridge_->image.rows), IPL_DEPTH_8U,3); //I want to release this image 
  
  ipl_image = cvGetImage( &cv_image, sub);   
  ROS_INFO("3");
  fluxWebBE(ipl_image);
  ROS_INFO("4");
  cv_bridge::CvImage newIma;
  newIma.image = cv::cvarrToMat(frame);
  newIma.encoding = bridge_->encoding;
  newIma.header = bridge_->header;
  sensor_msgs::Image newIm;
  newIma.toImageMsg(newIm);
  image_pub_.publish(newIm);
}

}

using namespace colour_detect;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"colour_detect");
    
    ColourVision node;

    ros::spin();

    return 0;
}


