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
#include "math_utility.h"



//QUESTO NODO DEVE SIMULARE LA PUBBLICAZIONE SUL TOPIC "tau"
//DEL MSG tau.msg (geometry_msgs/Wrench tau)

modellazione::tau control_tau;
void init_tau (){
control_tau.tau.force.x= frand(-5,5);
control_tau.tau.force.y= frand(-5,5);
control_tau.tau.force.z= frand(-5,5);
control_tau.tau.torque.x= frand(-5,5);
control_tau.tau.torque.y= frand(-5,5);
control_tau.tau.torque.z= frand(-5,5);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_control");

  ros::NodeHandle control;

  ros::Publisher control_pub = control.advertise<modellazione::tau>("tau", MAX_QUEUE_LENGTH);

  ros::Rate loop_rate(5);
  
    
  while (ros::ok()){
    
    ros::spinOnce();
    init_tau();

    control_pub.publish(control_tau);

    //ros::spinOnce();

    loop_rate.sleep();
  }

return 0;
}
