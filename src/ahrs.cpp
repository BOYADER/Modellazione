#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"


std_msgs::String msg;

void ahrs_state_read(const std_msgs::String::ConstPtr& eta)
{
  ROS_INFO("I heard: [%s]", eta->data.c_str());
  msg.data = eta->data.c_str();
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "ahrs_sensor");

  ros::NodeHandle ahrs_sensor;

  ros::Subscriber ahrs_sub = ahrs_sensor.subscribe("state_real", 1, ahrs_state_read);
  ros::Publisher ahrs_pub = ahrs_sensor.advertise<std_msgs::String>("sensor/ahrs", MAX_QUEUE_LENGTH);


  ros::Rate loop_rate(10);

  while(ros::ok()){

  	ros::spinOnce();
 
    ROS_INFO("sto per pubblicare: %s", msg.data.c_str());

    ahrs_pub.publish(msg);

    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}