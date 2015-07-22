// ros stuff
#include <ros/ros.h>

// messages
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Transform.h"
#include <boost/thread/mutex.hpp>

#include "matrice_from_imu/matFromImu.h"

int main(int argc,char **argv)
{   
    ros::init(argc,argv,"mat_from_imu");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<matrice_from_imu::matFromImu>("matr_from_imu/imu_cov");

    ROS_INFO("pp");
    matrice_from_imu::matFromImu srv;
 

    srv.request.a=true;

    if (client.call(srv))
    {
      ROS_INFO_STREAM("worked");
    }
    else
    {
      ROS_ERROR("Failed to call service imu_cov");
      return 1;
    }

    return 0;
}

