#ifndef __STATE_INTEGRATEUR_NODE__
#define __STATE_INTEGRATEUR_NODE__

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// messages
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mavros/RCOut.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "math.h"
#define PI 3.14159265

#include "vector.h"

typedef nav_msgs::Path PathMsg;
namespace stateint
{

class StateInt
{
public:
  /// constructor
  StateInt();
  tf::TransformBroadcaster tf_pub;
  
 
  /// destructor
  virtual ~StateInt();

private:
  ros::Publisher pose_pub_;
  ros::Subscriber rc_sub;
  ros::Subscriber imu_sub_;

  std::string fixed_frame_; ///< Fixed frame parameter
  std::string base_frame_;  ///< Moving frame parameter

  void rcCallback(const mavros::RCOut& msg);

  typedef void (StateInt::*eq_state_pointer)(int, int);
  eq_state_pointer state_ptr;
  //functions of equations of state
  void integrateCar(int u1, int u2);
  
  bool initialized;
// vectors
  geometry_msgs::PoseStamped pose_;



//integration variable
  float xp;
  float yp; 
  float thetap; 
  double k1;
  double k2;
  float u1;
  float u2;
  tf::Quaternion q;
  double roll, pitch, yaw;
// Time
  ros::Time old_time_;
  ros::Time new_time_;
  ros::Time begin_;
  double dt;

//cal
  int maxRCYaw;
  int minRCYaw;    
  int maxRCSpeed;
  int minRCSpeed;
  int c1;
  int c2;
  vector<float> v_yaw;
  vector<float> v_speed;
  int n_cal;
  double moyRCspeed_;
  double moyRCyaw_;
  bool calibration;

  int thrMaxRCYaw;
  int thrMinRCYaw;    
  int thrMaxRCSpeed;
  int thrMinRCSpeed;
  double tmpK1;
  double tmpK2;
  bool threshold_Use;
//path
  ros::Publisher path_pub_; 
  PathMsg path_msg_;
}; // class


}; // namespace

#endif
