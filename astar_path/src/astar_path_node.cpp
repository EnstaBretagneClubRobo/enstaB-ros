

/*
* @author : Elouan Autret elouan.autret@ensta-bretagne.org
* @source A * : Auguste Bourgeois (translated from Java)
*/
#include "astar_path/astar_path_node.h"


using namespace std;

namespace astar
{
//constructor Astar
AstarPath::AstarPath()
{   
    ros::NodeHandle nh_private_("~");
    ros::NodeHandle nh;

    if (!nh_private_.getParam ("map_frame", map_frame_))
        map_frame_ = "/map_star";
    if (!nh_private_.getParam ("base_frame", base_frame_))
        base_frame_ = "/base_star";
    start = false;
    start_sub = nh_private_.subscribe("/start_astar",1,&AstarPath::setStart,this);
    map_sub = nh_private_.subscribe("/map",1,&AstarPath::mapCB,this);
    path_pub_= nh_private_.advertise<astar_path::CasePath>("path", 5);
    map_path_pub_= nh_private_.advertise<nav_msgs::OccupancyGrid>("map_path", 1);
    goal_server_ = nh_private_.advertiseService("set_goal",&AstarPath::setGoalService,this);
    goal[0]=100000;//to check if goal is set (would not work with map > 100,000)
    goal[1] = 100000;
    waitgoal[0] = 100000;
    waitgoal[1] = 100000;
};

//destructor Astar
AstarPath::~AstarPath(){
};



void AstarPath::mapCB( const nav_msgs::OccupancyGrid newMap)
{ 
  if (asleep)
  {
    map = newMap;
    waitMap = newMap;
    findPath();
  }
  else
  {
    waitMap = newMap;
  }
};
//deprecated
bool AstarPath::setGoalService(astar_path::GoalSet::Request  &req,astar_path::GoalSet::Response &res)
{
   int x = req.goal[0]/map.info.resolution;
   int y = req.goal[1]/map.info.resolution;
   
   if (x>map.info.width || y>map.info.height || x<0 || y<0)
   {
     cerr << "Error goal not in map cannot compute path" << endl;
     res.res=false;
     return true;
   }
   if (asleep)
   {
     goal[0]=x;
     goal[1]=y;
     waitgoal[0]=x;
     waitgoal[1]=y;
     findPath();
   }
   else
   {
     waitgoal[0]=x;
     waitgoal[1]=y;
   }
   
   return res.res= true;
}

void AstarPath::setStart(const std_msgs::Bool mes){
    start=mes.data;
}

void AstarPath::findPath()
{
  asleep = false;
  if (goal[0] != 100000 && start){
    ROS_INFO("Start A* algorithm"); 
    tf_lis.lookupTransform(base_frame_, map_frame_,ros::Time(0), transform);
    tf_lis.lookupTransform("/pointtofollow", map_frame_,ros::Time(0), transform2);
    int x = transform2.getOrigin().getX()/map.info.resolution;
    int y = transform2.getOrigin().getY()/map.info.resolution;
    goal[0]=x;
    goal[1]=y;
    waitgoal[0]=x;
    waitgoal[1]=y;
    int pose[2];
    transformToXY(transform,pose);
    ROS_INFO("Goal x: %d , y : %d ",goal[0],goal[1]); 
    findPath(pose, goal);//compute closedList
    list<int> l2 ;
    reFindPath(closedList, pose, goal,l2);
    list<vector<int>> l3 ;
    toList(l2,l3);
    ROS_INFO("Calc ended now publishing,Size closed: %d ",(int) closedList.size()); 
    publishMapPath(l3);
    
    //create a Map for only path
    path_map = map;
    openList.clear();
    closedList.clear();
    transformFromXY(l3);  //publisher inside this function
    ROS_INFO("end A* algorithm\n");
    cases.clear(); 
  }
    if (!isSameMap())
    {
      map = waitMap;
      findPath();
    }
  ros::Duration(1).sleep();
  asleep = true;
};

void AstarPath::findPath(int *start, int *end)
   {
    int d = distance(start,end);
    //Add start node to openList and closedList 
    Case Start(start[0],start[1], d,0,d);
    Start.id = 0;
    cases.push_back(Start);
    int curr = Start.id;
    ROS_INFO("Start * curr x: %d y: %d",cases[curr].x,cases[curr].y);
    openList.push_back(curr);
    addCloseList(curr);
    //Add near available case of start case to openList.
    addAdjOpenList(curr, end);
    //While openList is not empty (empty = no solution);
    //or we are not at the end.
    while ((cases[curr].x != end[0] || cases[curr].y != end[1] ) && openList.size()!=0)
    {
         curr = bestNode();
         //best node is added to closedList.
         addCloseList(curr);
         //we continue with this node.
         addAdjOpenList(curr, end);
    }
    //closedList computed.
};

void AstarPath::reFindPath(list<int> const& l, int *start, int *end,list<int>& path){
	int d = distance(start,end);
	//We start from end we climb up the path to start case
	int tmp = l.front();
        bool nullit = cases[tmp].prev!=99999999 ;
	while(tmp != 0 && nullit )//while tmp different from start
        {
	    path.push_back(tmp);
	    tmp = cases[tmp].prev;
            nullit = cases[tmp].prev!=99999999;
	}
	path.push_back(tmp);
};

int AstarPath::distance(int const *curr, int const *targ){
      return abs(targ[0]-curr[0]) + abs(targ[1]-curr[1]);//manhattan
     //  return max(abs(targ[0]-curr[0]),abs(targ[1]-curr[1]));//chebyshev longer but better
};

int AstarPath::distance(int const *curr, std::vector<int> targ){
     return abs(targ[0]-curr[0]) + abs(targ[1]-curr[1]);//manhattan
      //return max(abs(targ[0]-curr[0]),abs(targ[1]-curr[1]));//chebyshev
};

int AstarPath::bestNode(){
    int f = 100000;
    int res;
    for (list<int>::const_iterator n=openList.cbegin(); n != openList.cend(); ++n){
       if(cases[*n].f<f)
       {
	  f=cases[*n].f;
	  res = *n;
       }
    }
   return res;
};

bool AstarPath::contains(list<int> const& l,vector<int> const& coord){
	for (list<int>::const_iterator n=openList.cbegin(); n != openList.cend(); ++n){
	    if( cases[*n].x==coord[0] && cases[*n].y==coord[1]){
		return true;
	    }
	}
	return false;
};


int AstarPath::search(list<int> const& l,vector<int> const& coord){
	for (list<int>::const_iterator n=openList.cbegin(); n != openList.cend(); ++n){
	    if( cases[*n].x==coord[0] && cases[*n].y==coord[1]){
		return *n;
	    }
	}
	return 99999999;//it sort of the null pointer for Case
}

void  AstarPath::addCloseList(int n){
          
	openList.remove(n);
	closedList.push_front(n);
};


void AstarPath::addAdjOpenList(int n, int const *end){
	vector<vector<int>> lAdj;
        getAdj(cases[n] ,  lAdj );
	for (vector<vector<int>>::const_iterator coord=lAdj.cbegin(); coord != lAdj.cend(); ++coord)
        {
	    //we only look at unregistered case in closedList
	    //that we are sure that they are not in final path 
            
	    if (!contains(closedList, *coord))
            {
		int g = cases[n].g + 1;
		int h = distance(end, *coord);
		int f = g+h;
                
		Case tmp((*coord)[0],(*coord)[1], f, g, h);
                tmp.id = cases.size();
		tmp.prev = n;
		//update specs of node if better than before
		if(contains(openList, *coord))
                {
		    if(f < cases[search(openList, *coord)].f)
                    {
                        cases.push_back(tmp);
			openList.remove(search(openList, *coord));
			openList.push_back(tmp.id);
		    }
		}
		//else we add case to openList
		else{
                    cases.push_back(tmp);
		    openList.push_back(tmp.id);
		}
	    }
	}
};

void AstarPath::getAdj(Case n , vector<vector<int>>& lAdj ) {
   int mask = 0.40/map.info.resolution+1;//TODO
   //m = (0,1) (1,1) (1,0) (-1,1) (-1,0) (-1,-1) (0,-1) (1,-1) m is static constexpr
   
   for (int i = 0 ; i<8;i++)
   {
    if (checkMask(n.x+m[i][0],n.y+m[i][1],mask))
     {
	vector<int> coord;
        coord.push_back(n.x+m[i][0]);
        coord.push_back(n.y+m[i][1]);
        lAdj.push_back(coord);
     }
   }
};


void AstarPath::toList(list<int> const& l,list<vector<int>>& res){
	//a list of only coordonates is better suited for other apllication
	for (list<int>::const_iterator n=l.cbegin(); n != l.cend(); ++n){
            vector<int> dot;
            dot.push_back(cases[*n].x);
            dot.push_back(cases[*n].y);
            res.push_front(dot);
	}
};

bool AstarPath::isSameMap()
{
  return (map.header.stamp == waitMap.header.stamp && goal[0] == waitgoal[0] && goal[1] == waitgoal[1] ) ;
};

void AstarPath::transformToXY(tf::StampedTransform transform, int *pose){
   int x = transform.getOrigin().x()/map.info.resolution;
   int y = transform.getOrigin().y()/map.info.resolution;
   ROS_INFO("Pose x: %d , y : %d ",x,y);  
   if (x>map.info.width || y>map.info.height || x<0 || y<0)
   {
     cerr << "Error robot not in map cannot compute path" << endl;
     pose[0]=goal[0];
     pose[1]=goal[1];
   }
   pose[0]=x;
   pose[1]=y;
};


void AstarPath::transformFromXY(list<vector<int>> const& l){
   //optimize straight line then publish the points in map frame in meter
   astar_path::CasePath msg_path;
   list<vector<int>> shortPath;
   int dx=0;
   int dy=0;
   int ndx=0;
   int ndy=0;
   for (list<vector<int>>::const_iterator n=l.cbegin(); n != l.cend(); ++n)
   { list<vector<int>>::const_iterator d1 = l.cbegin();
     d1++;
     if (n != d1 && n!=l.cbegin())//optimization of path
     {
        ndx=(*n)[0];
        ndy=(*n)[1];
        n--;
        ndx-=(*n)[0];
        ndy-=(*n)[1];
        n++;
        if (ndx!=dx || ndy!= dy) {
            dx=ndx;
            dy=ndy;
            n--;
            vector<int> oldVect = *n;
	    n++;
            if (oldVect!=shortPath.back())
            	shortPath.push_back(oldVect);
            shortPath.push_back(*n);
        }
     }
     else if (n != l.cbegin())
     {
	dx=(*n)[0];
        dy=(*n)[1];
        n--;
        dx-=(*n)[0];
        dy-=(*n)[1];
        n++;
        shortPath.push_back(*n);
     }
     else
     {
        shortPath.push_back(*n);
     }
     
   }
   int size = shortPath.size();
   int i=0;
   for (list<vector<int>>::const_iterator n=shortPath.cbegin(); n != shortPath.cend(); ++n)
   {
      msg_path.x.push_back(((*n)[0]+1/2)*map.info.resolution);
      msg_path.y.push_back(((*n)[1]+1/2)*map.info.resolution);
      i++;
   }
   path_pub_.publish(msg_path);
};

void AstarPath::publishMapPath(list<vector<int>> const& l){

	path_map.data.assign(map.info.width*map.info.height,-1);
        ROS_INFO("size of path %d",(int) l.size());
        for (list<vector<int>>::const_iterator n=l.cbegin(); n != l.cend(); ++n){
            path_map.data[nFromXY(*n)]=100;
        }
        map_path_pub_.publish(path_map);
}

bool AstarPath::checkMask(int x,int y,int mask){

  for (int dy= -mask ; dy<=mask;dy++)
  {
     for (int dx = -(mask-dy) ; dx<=(mask-dy);dx++)
     {
       int coord[2]={x+dx,y+dy};
       if (!checkFree(coord))
         return false;
     }
  } 
  return true;
}

int AstarPath::nFromXY(int const *coord){
	return coord[0]*map.info.width+coord[1];
}

int AstarPath::nFromXY(vector<int> const& coord){
	return coord[0]*map.info.width+coord[1];
}

bool AstarPath::checkFree(int const *coord){
   if (coord[0]>map.info.width || coord[1]>map.info.height || coord[0]<0 || coord[1]<0 )//if we are still inside the map
       return false;
   return (map.data[nFromXY(coord)] ==-1 || map.data[nFromXY(coord)] ==0);
}

};//end astar

using namespace astar;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"astar_path");
    
    AstarPath node;

    ros::spin();

    return 0;
}
