// ros stuff
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "astar_path/CasePath.h"


class CarController
{
 public:
   int h;
   CarController();
   ~CarController();
 private:
   double k1_;
   double k2_;
   std::string input_data_type_;
   std::string map_frame_;
   std::string base_frame_;
   
   ros::Subscriber path_sub_;

   tf::TransformListener tf_listener_;

   void pathCB(astar_path::CasePath);

   bool checkPosSegment(double *a,double *b);

   bool checkPosSegment(double ax,double ay,double bx,double by);
};
