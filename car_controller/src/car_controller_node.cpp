
#include "car_controller/car_controller_node.h"


namespace car_controller 
{

CarController::CarController():startCheck(false),
                               start(false),
                               receivedData(false),
                               xp(0)
{
    ros::NodeHandle nh_private_("~");
    

    if (!nh_private_.getParam ("k1", k1_))
        k1_ = 0.1;
    if (!nh_private_.getParam ("k2", k2_))
        k2_ = 0.1;
    if (!nh_private_.getParam ("feedback_data_type", input_data_type_))
        input_data_type_ = "CasePath";
    if (!nh_private_.getParam ("map_frame", map_frame_))
        map_frame_ = "/map";
    if (!nh_private_.getParam ("base_frame", base_frame_))
        base_frame_ = "/base_link";
    if (!nh_private_.getParam ("channel_k1", channel_k1_))
        channel_k1_ = 0;
    if (!nh_private_.getParam ("channel_k2", channel_k2_))
        channel_k2_ = 2;
    if (!nh_private_.getParam ("speed",speed_))
        speed_ = 150;
    if (!nh_private_.getParam ("kUv", kUv_))
        kUv_ = 0.5;
    if (!nh_private_.getParam ("ku", ku_))
        ku_ = 0.5;
    if (!nh_private_.getParam ("ros_rate", ros_rate))
        ros_rate = 30;

    if (input_data_type_.compare("CasePath") == 0)
      spinCommand = &CarController::spinPath;
    if (input_data_type_.compare("OccupancyGrid") == 0)
      spinCommand = &CarController::spinOccupancyGrid;

    
    client_ = nh_private_.serviceClient<pwm_serial_py::Over_int>("pwm_serial_send");
    service_ = nh_private_.advertiseService("starter", &CarController::starterControl,this);

    //for CasePath
    path_sub_= nh_private_.subscribe("astar_path/path",1, &CarController::pathCB,this);
    serviceSpeed_ = nh_private_.advertiseService("speed", &CarController::speedControl,this);//TODO transform this service to a subcriber ?
    servicekUv_ = nh_private_.advertiseService("kUv", &CarController::kUvControl,this);
 
    //for OccupancyGrid potentiel method ( unknow parts of the map are attractive and obstacles repulsive)
    map_sub_ = nh_private_.subscribe("/map",1, &CarController::mapCB,this);

    transformXOld.setOrigin(tf::Vector3(999, 999, 999));
}


CarController::~CarController(){} 


int CarController::sendCommand(double k1,double k2){
  srv_.request.over_msg.fill(0);
  srv_.request.over_msg[channel_k1_] = k1;
  srv_.request.over_msg[channel_k2_] = k2;

  if (!client_.call(srv_))
  {
    ROS_ERROR("Failed to call service pwm_serial_send");
    return 1;
  }
  return 0;
}

void CarController::spin(){
  ros::Rate loop_rate(ros_rate);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (start)
       (*this.*spinCommand)();
  }

}

bool CarController::starterControl(car_controller::Start_Control::Request &req,car_controller::Start_Control::Response &res){
  start = req.demand; 
  res.result = true;
  return  true;
}

}

using namespace car_controller;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"car_controller");
    
    CarController node;

    node.spin();

    return 0;
}
