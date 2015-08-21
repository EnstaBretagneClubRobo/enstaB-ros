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
#include "colour_detection/Threshold.h"

namespace colour_detect {


class ColourVision 
{ 

  public:
    ColourVision();
    ~ColourVision();



    
  private:
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::ServiceServer thresholds_server_;

    bool publish_image_;
    typedef boost::shared_ptr< cv_bridge::CvImage > Image;
    
    void imageCB(const sensor_msgs::Image msg_ptr);

    IplImage *imGlobal;
    IplImage *imGloDiff;
    //IplImage *imGloBin;
    IplImage *moyenne;
    int seuilHh;
    int seuilHs;
    int seuilHv;
    int seuilBh;
    int seuilBs;
    int seuilBv;
    int nbIter;
    int nbIterD;
    

    int fluxWebBE(cv::Mat &newFrame);

    bool thresholdCB(colour_detection::Threshold::Request  &req,
         colour_detection::Threshold::Response &res);
};

}
