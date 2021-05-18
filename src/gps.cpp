#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"


//TODO: Conversion between NED to LL

std_msgs::String msg;

void gps_state_read(const std_msgs::String::ConstPtr& eta)
{
  ROS_INFO("I heard: [%s]", eta->data.c_str());
  msg.data = eta->data.c_str();
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "gps_sensor");

  ros::NodeHandle gps_sensor;

  ros::Subscriber gps_sub = gps_sensor.subscribe("state_real", 1, gps_state_read);
  ros::Publisher gps_pub = gps_sensor.advertise<std_msgs::String>("sensor/gps", MAX_QUEUE_LENGTH);


  ros::Rate loop_rate(10);

  while(ros::ok()){

  	ros::spinOnce();
 
    ROS_INFO("sto per pubblicare: %s", msg.data.c_str());

    gps_pub.publish(msg);

    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}