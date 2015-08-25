
#include "car_controller/car_controller_node.h"


CarController::CarController():startCheck(false),
                               start(false),
                               receivedData(false)
{
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh;

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
    if (!nh_private_.getParam ("vitesse",speed_))
        speed_ = 150;
    if (!nh_private_.getParam ("kUv", kUv_))
        kUv_ = 0.1;

    path_sub_= nh_private_.subscribe("astar_path/map_path",1, &CarController::pathCB,this);
    client_ = nh_private_.serviceClient<pwm_serial_py::Over_int>("pwm_serial_send");
    service_ = nh_private_.advertiseService("starter", &CarController::starterControl,this);
    serviceSpeed_ = nh_private_.advertiseService("speed", &CarController::speedControl,this);//TODO transform this service to a subcriber ?
    servicekUv_ = nh_private_.advertiseService("kUv", &CarController::kUvControl,this);
}


CarController::~CarController(){} 

void CarController::pathCB(const astar_path::CasePath newCases)
{
 
  receivedData = true;
  waitCases = newCases;
  if (!startCheck && receivedData)
    cases = newCases;
  
}

bool CarController::checkPosSegment(double ax,double ay,double bx,double by){
  double a[2] = {ax,ay};
  double b[2] = {bx,by};
  checkPosSegment(a,b);
}

bool CarController::checkPosSegment(double *a,double *b)
{
  
  double vOrthoAB[2],vMB[2];
  vOrthoAB[0] = -(b[1]-a[1]);
  vOrthoAB[1] = (b[0]-a[1]);
  vMB[0] = b[0]-transformX.getOrigin().x();
  vMB[1] = b[1]-transformX.getOrigin().y();
  // we do MB^BC BC is orthogonol to AB M the position of base_link
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]<0; //true if we didn't pass through B
}


void CarController::spinPath(){
  
  int size = cases.x.size();
  int i=0;
  
  cases = waitCases;

  // check Drone Position in regards of points 
  startCheck  = true;//TODO should be multithreading protected
  tf_listener_.lookupTransform(map_frame_, base_frame_,  
                               ros::Time(0), transformX);
  
  while ( !checkPosSegment(cases.x[i],cases.y[i],cases.x[i+1],cases.y[i+1]) && i < size-1) {
    i++;
  }
  double vABx = cases.x[i+1]-cases.x[i];
  double vABy = cases.y[i+1]-cases.y[i];
  double vAMx = transformX.getOrigin().x()-cases.x[i];
  double vAMy = transformX.getOrigin().y()-cases.y[i];
  double roll,pitch,theta;
  transformX.getBasis().getRPY(roll, pitch, theta);
  startCheck  = false;

  //follow abstract ligne
  double phi = atan2(vABy,vABx);
  double eL = (vABx*vAMy-vABy*vAMx)/sqrt(pow(vABx,2)+pow(vABy,2));//det([b-a,m-a])/norm(b-a);//distance a la ligne
  double thetabar = phi-atan(eL);
  double e = thetabar-theta;
  
  double u=atan(tan(e/2));//atan for modulo 2*pi*/
  double v = speed_ - kUv_*abs(u) ;//TODO find the right parameters
  sendCommand(1500+v,1500+u);
}

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
  ros::Rate loop_rate(30);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (start)
      spinPath();
  }

}

bool CarController::starterControl(car_controller::Start_Control::Request &req,car_controller::Start_Control::Response &res){

  start = req.demand; 
  res.result = true;
  return  true;
}

bool CarController::speedControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res){

  speed_ = req.demand; 
  res.result = true;
  return  true;
}

bool CarController::kUvControl(car_controller::Double_Control::Request &req,car_controller::Double_Control::Response &res){

  kUv_ = req.demand; 
  res.result = true;
  return  true;
}

int main(int argc,char **argv)
{   
    ros::init(argc,argv,"astar_path");
    
    CarController node;

    node.spin();

    return 0;
}
