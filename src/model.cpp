#include <iostream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "modellazione/state_real.h"
#include "modellazione/tau.h"
#include <sstream>
#include "constant.h"
 
/*  Model node must subscribe /tau topic in order to obtain control actions.
    After solving dynamics differential equations it has to publish on 
    /state_real topic, from which every sensor will subscribe and take the 
    actual informations needed.
*/      
 
using namespace Eigen;

geometry_msgs::Vector3 dyn_force;
geometry_msgs::Vector3 dyn_torque;

modellazione::state_real resolveDynamics(){
  modellazione::state_real state;
  state.eta_1.x = 0;
  state.eta_1.y = 50;
  state.eta_1.z = 50;
  state.eta_2.x = 0;
  state.eta_2.y = 0;
  state.eta_2.z = 0;
  state.ni_1.x = 0;
  state.ni_1.y = 0;
  state.ni_1.z = 0;
  state.ni_2.x = 0;
  state.ni_2.y = 0;
  state.ni_2.z = 0;
  return state;
}


void tau_read(const modellazione::tau &wrench){/*
  dyn_force = wrench.tau.force;
  dyn_torque = wrench.tau.torque;
  ROS_INFO("I heard force = [%f %f %f] \n torque = [%f %f %f] \n", 
            dyn_force.x, dyn_force.y, dyn_force.z,
            dyn_torque.x, dyn_torque.y, dyn_torque.z);*/

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "model");

  ros::NodeHandle model;

  ros::Subscriber model_sub = model.subscribe("tau", 1000, tau_read);
  ros::Publisher model_pub = model.advertise<modellazione::state_real>("state_real", MAX_QUEUE_LENGTH);

  ros::Rate loop_rate(1000);
  
  float count = 1;
    
  while (ros::ok()){
    
    ros::spinOnce();

    modellazione::state_real state;

    state = resolveDynamics();
    state.prova = count++;

    ROS_INFO("Sto per pubblicare eta1 = [%f %f %f] \n msg numero: %f ", state.eta_1.x, state.eta_1.y, state.eta_1.z,
              state.prova);

    model_pub.publish(state);

    //ros::spinOnce();

    loop_rate.sleep();
  }



return 0;
}


