
#include "colour_detection/colour_detection_node.h"

namespace colour_detect {
//constructor Astar
ColourVision::ColourVision()
{   
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh;


    if (!nh_private_.getParam ("publish_image", publish_image_))
        publish_image_ = false; 
    
    image_sub_ = nh_private_.subscribe("/map",1,&ColourVision::imageCB,this);
    image_pub_= nh_private_.advertise<sensor_msgs::Image>("image_processed", 5);
    //thresholds_server_ = nh_private_.advertiseService("set_thresholds",/*temp*/,this);//temp
};

//destructor Astar
ColourVision::~ColourVision(){
};

void ColourVision::imageCB(const sensor_msgs::Image){


}

}

using namespace colour_detect;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"astar_path");
    
    ColourVision node;

    ros::spin();

    return 0;
}


