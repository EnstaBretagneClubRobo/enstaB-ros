#include "car_controller/car_controller_node.h"

namespace car_controller {

void CarController::mapCB(const nav_msgs::OccupancyGrid newMap)
{
 
  receivedData = true;
  waitMap = newMap;
  if (!startCheck && receivedData)
    map = newMap;
  
}

void CarController::spinOccupancyGrid(){
  if (!receivedData)
    return;  


  map = waitMap;

  // check Drone Position in regards of points 
  startCheck  = true;//TODO should be multithreading protected
  tf_listener_.lookupTransform(map_frame_, base_frame_,  
                               ros::Time(0), transformX);
  int pose[2];
  transformToCase(pose);
  double command[2];
  calculPotentiel(pose,command);
  startCheck = false;

  sendCommand(1500+speed_*command[0],1500+ku_*command[1]);
}

void CarController::transformToCase(int *pos)
{
  pos[0]=transformX.getOrigin().x()/map.info.resolution;
  pos[1]=transformX.getOrigin().y()/map.info.resolution;
}

void CarController::calculPotentiel(int const *pos,double *command)
{
  ROS_INFO("start");
  int mask = 100;
  int dx,dy;
  int coord[2];
  double p_repusl[2] = {0,0};
  double p_attrac[2] = {0,0};
  int value;
  double w[2];
  for (dy= -mask ; dy<=mask;dy++)
  {
     for (dx = -mask ; dx<=mask;dx++)
     {
       if(dy*dy+dx*dx <= mask*mask)
       {
         coord[0] = pos[0]+dx;
         coord[1] = pos[1]+dy;
         if (!(coord[0]<0 || coord[1] < 0 ||coord[0]>map.info.width|| coord[1] > map.info.height))
         {
           value = map.data[nFromXY(coord)];
           if (value<0){
             w[0] += -2*(-dx);
             w[1] += -2*(-dy);
           }
           else if (value>0){
             if (dx != 0 || dy != 0){
               w[0] += -value*dx/(dx*dx+dy*dy);
               w[1] += -value*dy/(dx*dx+dy*dy);
             }
             else//we should avoid to be in this situation
             {
               w[0] += 200;
               w[1] += 200;
             }
           }
         }
  /*%champ de vecteur
    nq=x(1:2)-qhat;
    w=vhat-2*(x(1:2)-phat)+nq/(norm(nq)^2);
    vbar=norm(w); thetabar = atan2(w(2),w(1));
    u = [vbar-x(3);10*atan(tan((thetabar-x(4))/2))];*/
       }
     }
  }
  double vbar = sqrt(w[0]*w[0]+w[1]*w[1]);
  double thetabar = atan2(w[1],w[0]); 
  //
  if (transformXOld.getOrigin().getX() == 999){
    xp = 0 ;
  }
  else{
    double x2=pow(transformX.getOrigin().getX()-transformXOld.getOrigin().getX(),2)+pow(transformX.getOrigin().getY()-transformXOld.getOrigin().getY(),2);
    xp =sqrt(x2)*ros_rate;
    transformXOld = transformX;
  }
  //
  command[0] = vbar - xp;
  double roll,pitch,theta;
  transformX.getBasis().getRPY(roll, pitch, theta);
  command[1] = 10*atan(tan((thetabar-theta)/2));
  ROS_INFO("end"); 
}

int CarController::nFromXY(int const *coord){
	return coord[0]*map.info.width+coord[1];
}

}
