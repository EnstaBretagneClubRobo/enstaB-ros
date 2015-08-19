#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <stdio.h>
#include <stdlib.h>

namespace colour_detect {


class ColourVision 
{ 

  public:
    ColourVision();
    ~ColourVision();
  private:
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    bool publish_image_;


    void imageCB(const sensor_msgs::Image);

    IplImage *imGlobal;
    IplImage *imGloDiff;
    //IplImage *imGloBin;
    IplImage *moyenne;
    int seuilH;
    int seuilB;
    int seuilHh;
    int seuilHs;
    int seuilHv;
    int seuilBh;
    int seuilBs;
    int seuilBv;
    int nbIter;
    int nbIterD;
    IplImage *frame;
    char *nomFenet;
    IplImage *smooth;
    IplImage *hsv;
    IplImage *imGloBin;

    CvMemStorage *memCon;


    int fluxWebBE(char *nomFenetre,char *nameSaved);

    void trackbarBE(int k);
};

}
