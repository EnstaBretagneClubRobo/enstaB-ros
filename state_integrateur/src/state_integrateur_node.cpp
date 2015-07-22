
#include "state_integrateur/state_integrateur_node.h"

using namespace tf;
using namespace std;


namespace stateint

{
//constructor
StateInt::StateInt():initialized(false),
		     n_cal(0),
                     calibration(false)
{   
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    if (!nh_private.getParam ("fixed_frame", fixed_frame_))
        fixed_frame_ = "/world";
    if (!nh_private.getParam ("base_frame", base_frame_))
        base_frame_ = "/robot";

    if (!nh_private.getParam ("k1", k1))
        k1 = 0.01;
    if (!nh_private.getParam ("k2", k2))
        k2 = 0.01;

    //limits PWM
    if (!nh_private.getParam ("maxRCYaw", maxRCYaw))
        maxRCYaw = 1846;
    if (!nh_private.getParam ("minRCYaw", minRCYaw))
        minRCYaw = 1073;    
    if (!nh_private.getParam ("maxRCSpeed", maxRCSpeed))
        maxRCSpeed = 1926;
    if (!nh_private.getParam ("minRCSpeed", minRCSpeed))
        minRCSpeed = 1130;

    
    
    // threshold PWM before robot moves
    if (!nh_private.getParam ("threshold_Use", threshold_Use))
        threshold_Use = false;

    if (!nh_private.getParam ("thrMaxRCYaw", thrMaxRCYaw))
        thrMaxRCYaw = 1445;
    if (!nh_private.getParam ("thrMinRCYaw", thrMinRCYaw))
        thrMinRCYaw = 1320;    
    if (!nh_private.getParam ("thrMaxRCSpeed", thrMaxRCSpeed))
        thrMaxRCSpeed = 1586;
    if (!nh_private.getParam ("thrMinRCSpeed", thrMinRCSpeed))
        thrMinRCSpeed = 1480;

    tmpK1 = k1;
    tmpK2 = k2;

    //channels to monitor
    if (!nh_private.getParam ("channelYaw", c1))
        c1 = 0;
    if (!nh_private.getParam ("channelSpeed", c2))
        c2 = 2;

    // calibration recommended if use of an rc remote (do not touch it while it calibrate)
    if (!nh_private.getParam ("calibrate", calibration))
        calibration = false;

    moyRCspeed_ = (maxRCSpeed+minRCSpeed)/2.0;
    moyRCyaw_   = (maxRCYaw+minRCYaw)/2.0;
   
    begin_=ros::Time::now();

    state_ptr = &StateInt::integrateCar;

    pose_pub_ = nh_private.advertise<geometry_msgs::PoseStamped>("odom", 10);
    path_pub_ = nh_private.advertise<PathMsg>("path", 10);
    rc_sub = nh_private.subscribe("/mavros/rc/out",10,&StateInt::rcCallback,this);
    
    //initiate pose_
    q.setRPY(0, 0, 0); //todo change for compass information
    geometry_msgs::Quaternion odom_quat;
    tf::quaternionTFToMsg(q, odom_quat);
    pose_.pose.position.x = 0;
    pose_.pose.position.y = 0;
    pose_.pose.position.z = 0;
    ROS_INFO("Ready to integrate pose");
};

//destructor
StateInt::~StateInt(){
  delete &pose_;
  delete &path_msg_;
};


/*
 * 
 */
void StateInt::rcCallback(const mavros::RCOut& msg){
    new_time_ = msg.header.stamp;
    if (new_time_-begin_>ros::Duration(5) || !calibration) //false while in calibration
    { 
      if (calibration){// only do at end of calibration
         double return_value = 0.0;
         int n = v_speed.size();
       
         for ( int i=0; i < n; i++)
         {
            return_value += v_speed[i];
         }
         moyRCspeed_ = return_value/v_speed.size();
         return_value = 0.0;
         n = v_yaw.size();
       
         for ( int i=0; i < n; i++)
         {
            return_value += v_yaw[i];
         }
         moyRCyaw_ = return_value/v_yaw.size();
         calibration = false;
         ROS_INFO_STREAM("RC Calibration done");
      }
      dt = new_time_.toSec() - old_time_.toSec();
      u1 = msg.channels[c2]-moyRCspeed_;
      u2 = msg.channels[c1]-moyRCyaw_;

      if (!initialized)
      {
	dt=0;
        initialized = true;
      } 
      if (threshold_Use)
      {
        if (msg.channels[c2]<= thrMaxRCSpeed && msg.channels[c2]>= thrMinRCSpeed)
        {
          if (u1>= 0)
            tmpK1 = k1 * atan (0.1*u1/(double)(thrMaxRCSpeed-moyRCspeed_)) * 2/PI;
          else
            tmpK1 = k1 * atan (0.1*u1/(double)(thrMinRCSpeed-moyRCspeed_)) * 2/PI;
        }
        else
        {
          tmpK1 = k1;
          if (u1>= 0)
            u1 = msg.channels[c2]-thrMaxRCSpeed;
          else
            u1 = msg.channels[c2]-thrMinRCSpeed;
        }

        if (msg.channels[c1]<= thrMaxRCYaw && msg.channels[c1]>= thrMinRCYaw)
        {
          if (u2>= 0)
            tmpK2 = k2 * atan (u2/(double)(thrMaxRCYaw-moyRCyaw_)) * 2/PI;
          else
            tmpK2 = k2 * atan (u2/(double)(thrMinRCYaw-moyRCyaw_)) * 2/PI;
        }
        else
        {
          tmpK2=k2;
          if (u2>= 0)
            u2 = msg.channels[c1]-thrMaxRCYaw;
          else
            u2 = msg.channels[c1]-thrMinRCYaw;
         }
      }

      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      (*this.*state_ptr)(u1,u2);

      geometry_msgs::Quaternion odom_quat;
      tf::quaternionTFToMsg(q, odom_quat);
      pose_.header = msg.header;
      
      pose_.header.frame_id.assign(base_frame_);
      pose_.pose.position.x += xp;
      pose_.pose.position.y += yp;
      pose_.pose.position.z = 0;
      pose_.pose.orientation = odom_quat;
      pose_pub_.publish(pose_);
      path_msg_.poses.push_back(pose_);
      path_msg_.header = pose_.header;
      path_msg_.header.frame_id = fixed_frame_;
      path_pub_.publish(path_msg_);
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(pose_.pose.position.x, pose_.pose.position.y, 0.0) );
      transform.setRotation(q);
      tf_pub.sendTransform(tf::StampedTransform(transform, new_time_, fixed_frame_, base_frame_));
      old_time_ = new_time_;
    }
    else {
        v_yaw.push_back(msg.channels[c1]);
        v_speed.push_back(msg.channels[c2]);
    }

};



/*
* xp = k1*u1*cos(theta)
* yp = k1*u1*sin(theta)
* thetap = k2*u2
*/
void StateInt::integrateCar(int u1,int u2){
      thetap = tmpK2*u2*dt;
      q.setRPY(0, 0, yaw+thetap);
      xp = tmpK1*u1*cos(yaw+thetap)*dt;
      yp = tmpK1*u1*sin(yaw+thetap)*dt;
};

};//end stateint

using namespace stateint;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"mat_from_imu");
    
    StateInt node;

    ros::spin();

    return 0;
}

