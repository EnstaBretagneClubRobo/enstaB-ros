#ifndef __MAT_FROM_IMU__
#define __MAT_FROM_IMU__

// ros stuff
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// messages
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Transform.h"
#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <boost/thread/mutex.hpp>

#include "matrice_from_imu/matFromImu.h"


namespace matimu
{

/** \mainpage
 *  \htmlinclude manifest.html
 * 
 * <b>Package Summary</b>
*/



//typedef boost::array<double, 9ul> ImuCov;


class MatFromIMU
{
public:
  /// constructor
  MatFromIMU();
  
  ros::Subscriber imu_sub_;
 
  typedef boost::shared_ptr<sensor_msgs::Imu const> ImuConstPtr;
  /// destructor
  virtual ~MatFromIMU();

private:
  
  bool sendCovImu(matrice_from_imu::matFromImu::Request  &req,
          matrice_from_imu::matFromImu::Response &res);
  
  void imuCallback(const ImuConstPtr& imu);
  

  ros::ServiceServer service;
  tf::Transform imu_meas_,imu_meas_old_;
    
  tf::TransformBroadcaster robot_broadcaster_;

// vectors
  ros::Time imu_time_;
  ros::Time imu_stamp_;
  ros::Time imu_init_stamp_;
  MatrixWrapper::SymmetricMatrix imu_covariance_;
    
  std::string output_frame_, base_footprint_frame_, tf_prefix_;
  bool initialized;
  
}; // class

}; // namespace

#endif
