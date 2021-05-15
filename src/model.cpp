#include <iostream>

#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "constant.h"
 
/*  Model node must subscribe /tau topic in order to obtain control actions.
    After solving dynamics differential equations it has to publish on 
    /state_real topic, from which every sensor will subscribe and take the 
    actual informations needed.
*/      
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "model");

  ros::NodeHandle n;

  ros::Publisher model_pub = n.advertise<std_msgs::String>("state_real", MAX_QUEUE_LENGTH);

  ros::Rate loop_rate(1000);

  
  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    
    model_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }



return 0;
}



