#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/depth.h"

//float64 eta[6];
std_msgs::String msg;

void depth_state_read(const std_msgs::String::ConstPtr& eta)
{
  ROS_INFO("I heard: [%s]", eta->data.c_str());
  msg.data = eta->data.c_str();
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "depth_sensor");

  ros::NodeHandle depth_sensor;

  ros::Subscriber depth_sub = depth_sensor.subscribe("state_real", 1, depth_state_read);
  ros::Publisher depth_pub = depth_sensor.advertise<modellazione::depth>("sensor/depth", MAX_QUEUE_LENGTH);


  ros::Rate loop_rate(10);

  while(ros::ok()){

  	ros::spinOnce();
 
    ROS_INFO("sto per pubblicare: %s", msg.data.c_str());

    depth_pub.publish(msg);

    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}