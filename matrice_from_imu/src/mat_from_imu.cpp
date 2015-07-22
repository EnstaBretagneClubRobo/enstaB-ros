
#include "matrice_from_imu/mat_from_imu.h"

using namespace tf;
namespace matimu

{

MatFromIMU::MatFromIMU():imu_covariance_(3),
    initialized(false)
{   ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
    base_footprint_frame_="world";

    
    imu_sub_ = nh.subscribe("/android/imu", 10,  &MatFromIMU::imuCallback,this);
    
    service = nh_private.advertiseService("imu_cov",&MatFromIMU::sendCovImu,this);
    ROS_INFO("Ready to send Matrice from Imu");
    ROS_INFO_STREAM(""+service.getService());
};

MatFromIMU::~MatFromIMU(){


};



bool MatFromIMU::sendCovImu(matrice_from_imu::matFromImu::Request  &req,
          matrice_from_imu::matFromImu::Response &res){
    geometry_msgs::Transform m;
    if (!initialized){
       tf::Transform tmp;
       
       tmp.setIdentity();
       tf::transformTFToMsg(tmp,m);
       res.transform=m;
       imu_meas_old_ = imu_meas_;
       initialized=true;
       return true;
    }

    tf::transformTFToMsg(imu_meas_*imu_meas_old_.inverse(),m);
    res.transform =m ;
    imu_meas_old_=imu_meas_;
    return true;
};

void MatFromIMU::imuCallback(const ImuConstPtr& imu)
  {

    
    imu_stamp_ = imu->header.stamp;
    tf::Quaternion orientation;
    quaternionMsgToTF(imu->orientation, orientation);
    imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
    //orientation_covariance = imu->orientation_covariance;
    for (unsigned int i=0; i<3; i++)
      for (unsigned int j=0; j<3; j++)
        imu_covariance_(i+1, j+1) = imu->orientation_covariance[3*i+j];

    /*/ Transforms imu data to base_footprint frame
    if (!robot_state_.waitForTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, ros::Duration(0.5))){
        ROS_ERROR("Could not transform imu message from %s to %s", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
        ROS_DEBUG("Could not transform imu message from %s to %s. Imu will not be activated yet.", imu->header.frame_id.c_str(), base_footprint_frame_.c_str());
      return;
    }
    tf::StampedTransform base_imu_offset;
    robot_state_.lookupTransform(base_footprint_frame_, imu->header.frame_id, imu_stamp_, base_imu_offset);
    imu_meas_ = imu_meas_ * base_imu_offset;
*/
   // imu_time_  = ros::Time::now();

    // manually set covariance untile imu sends covariance
    if (imu_covariance_(1,1) == 0.0){
      MatrixWrapper::SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
      measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
      measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
      imu_covariance_ = measNoiseImu_Cov;
    }

     robot_broadcaster_.sendTransform(tf::StampedTransform(imu_meas_.inverse(), imu_stamp_, base_footprint_frame_, "imuTF"));
 };


};//end matimu

using namespace matimu;
int main(int argc,char **argv)
{   
    ros::init(argc,argv,"mat_from_imu");
    
    MatFromIMU node;

    ros::spin();

    return 0;
}

