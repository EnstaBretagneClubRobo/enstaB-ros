#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <fstream>
#include <iostream>
#include <signal.h>
#include "ros/xmlrpc_manager.h"

using namespace std;

ofstream fichier;
sig_atomic_t volatile g_request_shutdown = 0;
ros::Subscriber sub;
bool initialized = false;

void writerCallback(const sensor_msgs::LaserScan& msg)
{ 
  if (!initialized) {
    fichier << "Angle min :" << msg.angle_min <<endl;
    fichier << "Angle max :" << msg.angle_max <<endl;
    fichier << "Angle inc :" << msg.angle_increment <<endl;
    fichier << "range min :" << msg.range_min <<endl;
    fichier << "range max :" << msg.range_max <<endl;
    initialized = true;
  }
  fichier << msg.header.stamp <<  ';';
  int n=sizeof (msg.ranges)/sizeof (float);
  int n2=(msg.angle_max-msg.angle_min)/msg.angle_increment+1;
  for (int i=0 ; i<n2;i++){
    fichier << msg.ranges[i] << ';'; 
  }
  fichier << endl;
  fichier.flush();              
}


void closingFunc(int sig){ // can be called asynchronously
  
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "writerLaser",ros::init_options::NoSigintHandler);
  
  std::string filepath = "../data/test.txt";
  if (argc==2)
    filepath = argv[1];

  fichier.open(filepath.c_str(), ios::out | ios::trunc);
 
  if(!fichier)
  {
    cerr << "File could not be openned" << endl;
    exit(1);
  }
  signal(SIGINT, closingFunc);
  //signal(SIGCHLD,closingFunc);
  
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);


  ros::NodeHandle n;


  sub = n.subscribe("scan", 100, &writerCallback);

  ROS_INFO("Initialized");

  while (!g_request_shutdown)
  {
    // Do non-callback stuff

    ros::spinOnce();
    usleep(100000);//0.1 sec
  }

  // Do pre-shutdown tasks
  ROS_INFO("Termination signal received, writing file");
  sub.shutdown();
  usleep(100000);//1 sec
  fichier.close();
  ROS_INFO("File written");
  ros::shutdown();

  return 0;
}

