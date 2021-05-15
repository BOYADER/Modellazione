#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"

//float64 eta[6];
std_msgs::String msg;

void dvl_state_read(const std_msgs::String::ConstPtr& eta)
{
  ROS_INFO("I heard: [%s]", eta->data.c_str());
  msg.data = eta->data.c_str();
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "dvl_sensor");

  ros::NodeHandle dvl_sensor;

  ros::Subscriber dvl_sub = dvl_sensor.subscribe("state_real", 1, dvl_state_read);
  ros::Publisher dvl_pub = dvl_sensor.advertise<std_msgs::String>("sensor/dvl", MAX_QUEUE_LENGTH);


  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
 
    ROS_INFO("sto per pubblicare: %s", msg.data.c_str());

    dvl_pub.publish(msg);

    loop_rate.sleep();

  }

  ros::spin();

  return 0;
}