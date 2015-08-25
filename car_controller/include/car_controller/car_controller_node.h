// ros stuff
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "astar_path/CasePath.h"
#include "pwm_serial_py/Over_int.h"
#include "car_controller/Start_Control.h"
#include "car_controller/Double_Control.h"

class CarController
{
 public:
   int h;
   CarController();
   ~CarController();
   
   void spin();
 private:
   double k1_;
   double k2_;
   

   std::string input_data_type_;
   std::string map_frame_;
   std::string base_frame_;
   
   ros::Subscriber path_sub_;
   ros::ServiceClient client_;

   ros::ServiceServer service_;
   ros::ServiceServer serviceSpeed_;
   ros::ServiceServer servicekUv_;

   pwm_serial_py::Over_int srv_;
   int channel_k1_;
   int channel_k2_;
   double speed_;//this value only make sens in pwm (we do 1500(neutral)+ x to drive)
   double kUv_;


   tf::TransformListener tf_listener_;

   tf::StampedTransform transformX; //only use for pose of base 
  
   bool starterControl(car_controller::Start_Control::Request &req,car_controller::Start_Control::Response &res);

   bool speedControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res);

   bool kUvControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res);

   void pathCB(astar_path::CasePath);

   bool checkPosSegment(double *a,double *b);

   bool checkPosSegment(double ax,double ay,double bx,double by);
   
   bool start;
   bool receivedData;
   bool startCheck;

   void spinPath();

   int sendCommand(double k1,double k2);

   astar_path::CasePath cases;
   astar_path::CasePath waitCases;                  
};
