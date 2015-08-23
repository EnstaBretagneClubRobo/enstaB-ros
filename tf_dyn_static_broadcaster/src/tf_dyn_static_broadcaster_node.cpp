#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tf_dyn_static");

  ros::NodeHandle node;

  std::string target_frame;
  std::string source_frame;
  std::string new_frame_source;
  std::string new_frame_target;

  if (argc == 5)
  {
    target_frame = argv[1];
    source_frame = argv[2];
    new_frame_source = argv[3];
    new_frame_target = argv[4];
  }
  else {
    if (!node.getParam ("source_frame", source_frame))
        source_frame = "/odom";
    if (!node.getParam ("target_frame", target_frame))
        target_frame = "/camera_link";
    if (!node.getParam ("new_frame_source", new_frame_source))
        new_frame_source = "/odom";
    if (!node.getParam ("new_frame_target", new_frame_target))
        new_frame_target = "/map";
  }

  static tf::TransformBroadcaster br;
  tf::TransformListener listener;

  ros::Rate rate(10.0);
  bool start = false;
  tf::StampedTransform transform;
  transform.setOrigin( tf::Vector3(0, 0, 0) );
  while (node.ok()){
    
    try
    {
      if (!start) 
      {
        listener.lookupTransform(source_frame, target_frame,  
                               ros::Time(0), transform);
        start = true;
      }
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), new_frame_source, new_frame_target));
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      if (transform.getOrigin().x() == 0 && transform.getOrigin().y() ==0 && transform.getOrigin().z() == 0){ //if we have not started broadcast we still try to get transform
         start=false;
      } 
    }
    rate.sleep();
  }
  return 0;
};
