#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "astar_path/CasePath.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h" 
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

using namespace std;

typedef boost::shared_ptr< cv_bridge::CvImage > Image;

bool save_int_srv_asked;
bool notGo;
bool hardRecord;

ros::Subscriber map_sub;
ros::Subscriber key_sub;
ros::Subscriber pcl_sub;
ros::Subscriber image_sub;
ros::Subscriber pose_sub;
ros::Subscriber odom_sub;

bool received_pose;
bool received_key;
bool received_odom;
bool received_image;
bool received_pcl;
bool received_map;
string path_record
int time;

bool save_inst_srv_cb(save_inst_srv::Save_inst_srv::Request &req,save_inst_srv::Save_inst_srv::Response &res){
 //send request received
   save_int_srv_asked = true;
   notGo = true;
   hardRecord = req.hardRecord;
   req.response = true;
   return true;
}

void mapCB(nav_msgs::OccupancyGrid nMap){
     map = nMap;
     received_Map = true;
}

void keyCB(nav_msgs::pcl nKey){
   key = nKey;
   received_key = true;
}

void pclCB(nav_msgs::pcl nPcl){
   pcl = nPcl;
   received_pcl = true;
}

void odomCB(nav_msgs::Odometry nOdom){
   odom = nOdom;
   received_odom = true;
}

void imageCB(sensor_msgs::Image nImage){
   Image bridge_ = cv_bridge::toCvCopy(frame, "bgr8");
   cv::Mat framecv =bridge_->image;
   bridge_->image.release();
   received_image = true;
}

void poseCB(geometry_msgs::PoseStamped nPose){
   pose = nPose;
   received_pose = true;
}


int main(int argc,char **argv)
{   
    ros::init(argc,argv,"save_node");
    
    ros::NodeHandle nh;
    if (!nh.getParam ("filepath", filepath))
        filepath = "./test/";
  
    ros::Service serv = nh.advertiseService("/save_shot",&save_inst_srv_cb);
    while (ros::ok())
    {  
       ros::spinOnce();
       if (save_int_srv_asked)
       {
         if (notGo){
          time_t t_now = time(0);
          struct tm* now = localtime(&t_now);
          stringstream path_ss;
          path_ss	<< filepath 
                        << (now->tm_year + 1900) << "-"
                        << setfill('0') << setw(2) << (now->tm_mon + 1) << "-"
                        << setfill('0') << setw(2) << now->tm_mday << "--"
                        << setfill('0') << setw(2) << now->tm_hour << ":"
                        << setfill('0') << setw(2) << now->tm_min << ":"
                        << setfill('0') << setw(2) << now->tm_sec << "--Video/";
          string path_record = (path_ss).str();
          string mkdir_command = "mkdir \"" + path_record + "\"";
          system(mkdir_command.c_str());

          map_sub = nh.subscribe("/map",1,mapCB);
          key_sub = nh.subscribe("/keyframes",1,keyCB);
          pcl_sub = nh.subscribe("/pointcloud",1,pclCB);
          image_sub = nh.subscribe("/camera/rgb/image_color",1,imageCB);
          pose_sub = nh.subscribe("/slam_out_pose",1,poseCB);
          odom_sub = nh.subscribe("/voodometry",1,odomCB);
          time = now->tm_sec;
          notGo = false;
        }

          if received_map;
             map_sub.unregister()
          if received_key;
             key_sub.unregister()
          if received_pcl;
             pcl_sub.unregister()
          if received_image;
             image_sub.unregister()
          if received_pose;
             pose_sub.unregister()
          if received_odom;
             odom_sub.unregister()
          //save tf
          tf::StampedTransform transform;
          try{
            tf_listener_.lookupTransform("map","base_link" ,ros::Time(0), transform);
          }
          catch(tf::TransformException e){}

          try{
            tf_listener_.lookupTransform("odom","camera_link" ,ros::Time(0), transform);
          }
          catch(tf::TransformException e){}

          try{
            tf_listener_.lookupTransform("map","laser" ,ros::Time(0), transform);
          }
          catch(tf::TransformException e){}

          try{
            tf_listener_.lookupTransform("local_origin","fcu" ,ros::Time(0), transform);
          }
          catch(tf::TransformException e){}
          
          bool result = received_map && received_key && received_pcl && received_image && received_pose && received_odom;
          if (result || (time(0)->tm_sec-time > 10)){
             save_int_srv_asked = false;
             received_pose = false;
             received_key = false;
             received_odom = false;
             received_image = false;
             received_pcl = false;
             received_map = false;
             if (hardRecord)
             {
                //faire appel service record octomap et pcl 
               //rosservice call /save_octomap path_record.ot  /save_pcd_map.pcd
             }
           }
       }
       ross::time::sleep(ros::Duration(1));
    }
}


