
#include "car_controller/car_controller_node.h"


CarController::CarController()
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
   /* map_sub = nh_private_.subscribe("/map",1,&AstarPath::mapCB,this);
    path_pub_= nh_private_.advertise<astar_path::CasePath>("path", 5);
    map_path_pub_= nh_private_.advertise<nav_msgs::OccupancyGrid>("map_path", 1);
    goal_server_ = nh_private_.advertiseService("set_goal",&AstarPath::setGoalService,this);*/
    path_sub_= nh_private_.subscribe("astar_path/map_path",1, &CarController::pathCB,this);


}


CarController::~CarController(){}

void CarController::pathCB(astar_path::CasePath cases)
{
  // check Drone Position in regards of points 
  int size = cases.x.size();
  int i=0;
  while ( !checkPosSegment(cases.x[i],cases.y[i],cases.x[i+1],cases.y[i+1]) && i < size-1) {
    i++;
  }
}

bool CarController::checkPosSegment(double ax,double ay,double bx,double by){
  double a[2] = {ax,ay};
  double b[2] = {bx,by};
  checkPosSegment(a,b);
}

bool CarController::checkPosSegment(double *a,double *b)
{
  tf::StampedTransform transform;
  tf_listener_.lookupTransform(map_frame_, base_frame_,  
                               ros::Time(0), transform);
  double vOrthoAB[2],vMB[2];
  vOrthoAB[0] = -(b[1]-a[1]);
  vOrthoAB[1] = (b[0]-a[1]);
  vMB[0] = b[0]-transform.getOrigin().x();
  vMB[1] = b[1]-transform.getOrigin().y();
  // we do MB^BC BC is orthogonol to AB M the position of base_link
  return vMB[0]*vOrthoAB[1]-vMB[1]*vOrthoAB[0]<0; //true if we didn't pass through B
}






int main(int argc,char **argv)
{   
    ros::init(argc,argv,"astar_path");
    
    CarController node;

    ros::spin();

    return 0;
}
