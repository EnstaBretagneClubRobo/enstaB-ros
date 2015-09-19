#ifndef __ASTAR_PATH_NODE__
#define __ASTAR_PATH_NODE__

// ros stuff
#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "astar_path/CasePath.h"
#include "astar_path/GoalSet.h"
#include "std_msgs/Bool.h"

namespace astar
{

class Case
{
 public:
  Case(){prev=99999999;}
  Case(int nx,int ny, int nf, int ng, int nh) {x=nx,y=ny, g=ng, h=nh, f=nf, prev=99999999;}
  Case(int nx,int ny ) {x=nx,y=ny, g=0, h=0, f=0, prev=99999999;}
  int x;
  int y;
  int g;
  int h;
  int f;
  int prev;
  int id;
  virtual ~Case(){}
}; //class Case


class  AstarPath
{

public:
  /// constructor
  AstarPath();
  tf::TransformListener tf_lis; 
 
  /// destructor
  virtual ~AstarPath();

private:
  ros::Subscriber map_sub;
  ros::ServiceServer goal_server_;
  ros::Subscriber start_sub;
  ros::Publisher path_pub_;
  ros::Publisher map_path_pub_;

  std::string base_frame_; ///< Fixed frame parameter
  std::string map_frame_;  ///< Moving frame parameter

  bool setGoalService(astar_path::GoalSet::Request  &req,astar_path::GoalSet::Response &res);

  void mapCB( const nav_msgs::OccupancyGrid );
  bool start;
  void findPath();

  bool isSameMap();
  void setStart(const std_msgs::Bool mes);

  void transformToXY(tf::StampedTransform transform, int *pose);
  int distance(int const *curr, int const *targ);
  int distance(int const *curr, std::vector<int> targ);
  int bestNode();
  bool contains(std::list<int> const& l,std::vector<int> const& coord);
  void addCloseList(int n);
  void addAdjOpenList(int n, int const *end);
  int search(std::list<int> const& l,std::vector<int> const& coord);
  void findPath(int *start, int *end);
  void reFindPath(std::list<int> const& l, int *start, int *end,std::list<int>& path);
  void toList(std::list<int> const& l, std::list<std::vector<int>>& res);
  void transformFromXY(std::list<std::vector<int>> const& l3);
 
  void publishMapPath(std::list<std::vector<int>> const& l);
  void getAdj(Case n, std::vector<std::vector<int>>& lAdj );
  int nFromXY(int const *coord);
  int nFromXY(std::vector<int> const& coord);
  bool checkFree(int const *coord);
  bool checkMask(int x,int y,int mask);

  nav_msgs::OccupancyGrid map;
  nav_msgs::OccupancyGrid waitMap;
  nav_msgs::OccupancyGrid path_map;
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  bool asleep;

  int goal[2];
  int waitgoal[2];
  std::list<int> openList;
  std::list<int> closedList;
  std::vector<Case> cases;

}; // class AstarPath


static constexpr int m[8][2] = {{0,1}, {1,1}, {1,0}, {-1,1}, {-1,0}, {-1,-1}, {0,-1},{1,-1}};

}; // namespace

#endif
