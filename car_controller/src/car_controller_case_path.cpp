#include "car_controller/car_controller_node.h"
#define PI 3.14159265
namespace car_controller
 {
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
  if (!receivedData)
    return;  

  cases = waitCases;

  int size = cases.x.size();
  int i=0;
  


  // check Drone Position in regards of points 
  startCheck  = true;//TODO should be multithreading protected
  tf_listener_.lookupTransform(map_frame_, base_frame_,  
                               ros::Time(0), transformX);

  double distFromGoalSq = pow(transformX.getOrigin().x()-cases.x[size-1],2)+pow(transformX.getOrigin().y()-cases.y[size-1],2);

  if (distFromGoalSq < 1) {
     sendCommand(1500,1500);//Stop Motor
     ros::Publisher pub = nh.advertise<std_msgs::Empty>("/astar_arrived",1);
     pub.publish(std_msgs::Empty());
     start = false;
  }

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
  
  double u=ku_*(2/PI)*atan(tan(e/2));//atan for modulo 2*pi*/
  double v = speed_ - kUv_*abs(u) ;//TODO find the right parameters
  sendCommand(1500+v,1500+u);
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
}

