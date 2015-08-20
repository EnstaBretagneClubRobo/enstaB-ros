
#include "colour_detection/colour_detection_node.h"


namespace colour_detect 
{
//constructor ColourVision
ColourVision::ColourVision():
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
    thresholds_server_ = nh_private_.advertiseService("set_thresholds",&ColourVision::thresholdCB,this);//temp
    cvNamedWindow("test", CV_WINDOW_AUTOSIZE );
};

//destructor ColourVision
ColourVision::~ColourVision(){};

void ColourVision::imageCB(const sensor_msgs::ImageConstPtr& msg_ptr)
{
  //ROS -> OPENCV
  CvMat cv_image;
  bridge_ = cv_bridge::toCvCopy(msg_ptr, "bgr8");
  cv_image = bridge_->image ;
  IplImage *ipl_image ;
  IplImage *sub = cvCreateImage(cvSize(bridge_->image.cols, bridge_->image.rows), IPL_DEPTH_8U,3); 
  ipl_image = cvGetImage( &cv_image, sub);  
 
  //Processing
  fluxWebBE(ipl_image);

  //OPENCV -> ROS
  cv_bridge::CvImage newIma;
  newIma.image = cv::cvarrToMat(frame);
  newIma.encoding = bridge_->encoding;
  newIma.header = bridge_->header;
  sensor_msgs::Image newIm;
  newIma.toImageMsg(newIm);
  image_pub_.publish(newIm);
}


bool ColourVision::thresholdCB(colour_detection::Threshold::Request  &req,
         colour_detection::Threshold::Response &res)
{ //1:Bh 2:Bs 3:Bv 4:Hh 5:Hs 6:Hv 7:Iter 8:IterD
  if (req.channel <1 || req.channel >8){
    ROS_WARN("channel 1-8");
    res.success = false; 
    return true;
  }
  if (req.value > 250 || req.value < 0)
  {
    ROS_WARN("value should be between 0 and 250");
    res.success = false; 
    return true;
  }
  if ((req.channel == 1 || req.channel == 4)&& req.value > 179) {
    ROS_WARN("value should be between 0 and 179 - Hue parameter");
    res.success = false;
    return true; 
  }
  switch(req.channel)
  {
    case 1:
    {
      seuilBh = req.value;
      break;
    }
    case 2:
    {
      seuilBs = req.value;
      break;
    }
    case 3:
    {
      seuilBv = req.value;
      break;
    }
    case 4:
    {
      seuilHh = req.value;
      break;
    }
    case 5:
    {
      seuilHs = req.value;
      break;
    }
    case 6:
    {
      seuilHv = req.value;
      break;
    }
    case 7:
    {
      nbIter = req.value;
      break;
    }
    case 8:
    {
      nbIterD = req.value;
      break;
    }
  }

  res.success =true;
  return true;
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


