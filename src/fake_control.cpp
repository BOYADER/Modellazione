#include <iostream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "modellazione/state_real.h"
#include "modellazione/tau.h"
#include <sstream>
#include "constant.h"
#include <math.h>


//QUESTO NODO DEVE SIMULARE LA PUBBLICAZIONE SUL TOPIC "tau"
//DEL MSG tau.msg (geometry_msgs/Vector3/Wrench tau)

modellazione::tau control_tau;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_control");

  ros::NodeHandle control;

  ros::Publisher control_pub = control.advertise<modellazione::tau>("tau", MAX_QUEUE_LENGTH);

  ros::Rate loop_rate(1);
  
    
  while (ros::ok()){
    
    ros::spinOnce();

    control_pub.publish(control_tau);

    //ros::spinOnce();

    loop_rate.sleep();
  }

return 0;
}
