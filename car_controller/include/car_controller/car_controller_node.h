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

namespace car_controller
{

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
   int ros_rate;

   std::string input_data_type_;
   std::string map_frame_;
   std::string base_frame_;
   
   ros::Subscriber path_sub_;
   ros::Subscriber map_sub_;
   ros::ServiceClient client_;

   ros::ServiceServer service_;
   ros::ServiceServer serviceSpeed_;
   ros::ServiceServer servicekUv_;

   pwm_serial_py::Over_int srv_;
   int channel_k1_;
   int channel_k2_;
   double speed_;//this value only make sens in pwm (we do 1500(neutral)+ x to drive)
   

   tf::TransformListener tf_listener_;

   tf::StampedTransform transformX; //only use for pose of base 
  
   bool starterControl(car_controller::Start_Control::Request &req,car_controller::Start_Control::Response &res);

   bool speedControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res);
//////////////////////////////////////
   bool kUvControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res);

   void pathCB(astar_path::CasePath);

   bool checkPosSegment(double *a,double *b);

   bool checkPosSegment(double ax,double ay,double bx,double by);

   void spinPath();

   double kUv_;
   double ku_;
   astar_path::CasePath cases;
   astar_path::CasePath waitCases;   
//////////////////////////////////////   
   void mapCB(const nav_msgs::OccupancyGrid newMap);
   
   void spinOccupancyGrid();

   void transformToCase(int *pos);
   void calculPotentiel(int const *pos,double *command);

   int nFromXY(int const *coord);
   
   double xp;//speed
   tf::StampedTransform transformXOld;

   nav_msgs::OccupancyGrid map;
   nav_msgs::OccupancyGrid waitMap;
/////////////////////////////////////
   bool start;
   bool receivedData;
   bool startCheck;

   typedef void (CarController::*spin_ptr)();
   spin_ptr spinCommand;
   

   int sendCommand(double k1,double k2);

               
};

}
