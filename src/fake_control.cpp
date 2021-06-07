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



modellazione::tau control_tau;
void init_tau (){
control_tau.tau.force.x = 0;
control_tau.tau.force.y = 0;
control_tau.tau.force.z = 0;
control_tau.tau.torque.x = 0;
control_tau.tau.torque.y = 0;
control_tau.tau.torque.z = 0;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_control");

  ros::NodeHandle control;

  ros::Publisher control_pub = control.advertise<modellazione::tau>("tau", MAX_QUEUE_LENGTH);

  ros::Rate loop_rate(5);
  int count = 0;
  init_tau();

  while (ros::ok()){
    
    ros::spinOnce();

    if(count > 25)
      control_tau.tau.force.x = 100;
      
    control_pub.publish(control_tau);
    


    loop_rate.sleep();
    count++;
  }

return 0;
}
