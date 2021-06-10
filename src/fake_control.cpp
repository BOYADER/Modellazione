#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "/usr/include/eigen3/Eigen/Dense"
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

    /*-------------------------SIMULAZIONE IMPULSO---------------------------*/
    //INSERIRE force o torque e la direzione su cui si vuole testare l'impulso 

    /*if(count > 20 && count <= 23)
      control_tau.tau.force.x = 100;        
    else
      control_tau.tau.force.x = 0; 
    /*-----------------------------------------------------------------------*/


    /*----------------------------SIMULAZIONE GRADINO -----------------------*/
    //INSERIRE force o torque e la direzione su cui si vuole testare il gradino

    /*if(count > 20)
      control_tau.tau.force.x = 100;        
    else
      control_tau.tau.force.x = 0; 
    /*-----------------------------------------------------------------------*/


    /*---------------------VALIDAZIONE UNDER_WATER GPS-----------------------*/
    /*if((count > 10 && count <= 15) || (count > 80 && count <=85))
      control_tau.tau.force.z = 50;
    else
      control_tau.tau.force.z = 0;*/
    /*-----------------------------------------------------------------------*/


    /*---------------------------VALIDAZIONE DVL-----------------------------*/
    /*if(count > 10){
      control_tau.tau.force.x = 100;
      control_tau.tau.force.y = 100;
    }
    /*-----------------------------------------------------------------------*/


    control_pub.publish(control_tau);
    loop_rate.sleep();
    count++;
  }

return 0;

}
