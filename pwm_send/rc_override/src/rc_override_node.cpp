#include <ros/ros.h>
#include "mavros/OverrideRCIn.h"
#include "rc_override/Override.h"
#include "rc_override/Over_int.h"

ros::Publisher override_pub;
mavros::OverrideRCIn msg;

bool sendOverrideCB(rc_override::Override::Request  &req,rc_override::Override::Response &res){
   msg = req.over_msg; 
   return res.result = true;
}

bool sendOver_intCB(rc_override::Over_int::Request  &req,rc_override::Over_int::Response &res){
   
   msg.channels = req.over_msg;
   
   return res.result = true;
}

int main(int argc,char **argv)
{   
    ros::init(argc,argv,"rc_override");
    
    ros::NodeHandle nh;

    override_pub =  nh.advertise<mavros::OverrideRCIn>("override", 10);

    ros::ServiceServer service = nh.advertiseService("override",&sendOverrideCB);
    ros::ServiceServer service2 = nh.advertiseService("override_int",&sendOver_intCB);
    ROS_INFO("Ready to send override RC signal");

    while (ros::ok()){
	// Do non-callback stuff
	override_pub.publish(msg); 
        ros::spinOnce();
        usleep(10000);//0.1 sec
    }
    

    return 0;
}
