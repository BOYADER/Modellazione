#include <iostream>
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "modellazione/state_real.h"
#include "modellazione/tau.h"
#include <sstream>
#include "constant.h"
#include "math_utility.h"
/*  Model node must subscribe /tau topic in order to obtain control actions.
    After solving dynamics differential equations it has to publish on 
    /state_real topic, from which every sensor will subscribe and take the 
    actual informations needed.
*/      
 
using namespace Eigen;

geometry_msgs::Vector3 dyn_force;
geometry_msgs::Vector3 dyn_torque;
modellazione::state_real state;

//calcolo elementi matrici masse aggiunte

float e = 1 - (R_B/R_A) * (R_B/R_A);
float alpha_0 = 2 * (1 - e * e) * ( 0.5 * log((1 + e) / (1 - e)) - e) / (e * e * e);
float beta_0 = 1/(e*e) - (1 - e*e)* log((1+e)/(1-e)) / (2*e*e*e);

float X_u_dot = - alpha_0 / (2 - alpha_0) * R_M;
float Y_v_dot = - beta_0 / (2 - beta_0) * R_M;
//float Z_w_dot = - beta_0 / (2 - beta_0) * R_M;
float Z_w_dot = Y_v_dot;

float K_p_dot = 0;
float N_r_dot = (-1/5*R_M)* ( R_B*R_B - R_A*R_A)*( R_B*R_B - R_A*R_A)*(alpha_0 - beta_0)/(2*(R_B*R_B - R_A*R_A) + (R_B*R_B + R_A*R_A)*(beta_0 - alpha_0));
float M_q_dot = (-1/5*R_M)* ( R_B*R_B - R_A*R_A)*( R_B*R_B - R_A*R_A)*(alpha_0 - beta_0)/(2*(R_B*R_B - R_A*R_A) + (R_B*R_B + R_A*R_A)*(beta_0 - alpha_0));

float compute_damping(u_int lato, float ni_i)
{
  float area;
  switch(lato){
    case 1:
      area = 2 * R_A * 2 * R_B;
      return (-1/2) * RHO_W * area *C_D_X* abs(ni_i);
    case 2:
      area = 2 * R_B * 2 * R_C; 
      return (-1/2) * RHO_W * area *C_D_YZ * abs(ni_i);
    default:
      break;
  }
}

// matrici della dinamica 

MatrixXf M(6,6);
MatrixXf D(6,6);
MatrixXf C(6,6);
VectorXf G(6);

void initializeM(){
  M(0,0) = R_M - X_u_dot;
  M(1,1) = R_M - Y_v_dot;
  M(2,2) = R_M - Z_w_dot;
  M(3,3) = I_X;
  M(4,4) = I_Y;
  M(5,5) = I_Z;
}

void updateD(){
  D(0,0) = - compute_damping(1, state.ni_1.x);
  D(1,1) = - compute_damping(2, state.ni_1.y);
  D(2,2) = - compute_damping(2, state.ni_1.z);
  D(3,3) = - compute_damping(1, state.ni_2.x);
  D(4,4) = - compute_damping(2, state.ni_2.y);
  D(5,5) = - compute_damping(2, state.ni_2.z);
}

void updateC(){
  C(0,4) = - Z_w_dot * state.ni_1.z;
  C(0,5) =   Y_v_dot * state.ni_1.y;
  C(1,3) =   Z_w_dot * state.ni_1.z;
  C(1,5) = - X_u_dot * state.ni_1.x;
  C(2,3) = - Y_v_dot * state.ni_1.y;
  C(2,4) =   X_u_dot * state.ni_1.x;
  C(3,1) = - Z_w_dot * state.ni_1.z;
  C(3,2) =   Y_v_dot * state.ni_1.y;
  C(4,0) =   Z_w_dot * state.ni_1.z;
  C(4,2) = - X_u_dot * state.ni_1.x;
  C(5,0) = - Y_v_dot * state.ni_1.y;
  C(5,1) =   X_u_dot * state.ni_1.x;
  C(3,4) = - N_r_dot * state.ni_2.z;
  C(3,5) =   M_q_dot * state.ni_2.y;
  C(4,3) =   N_r_dot * state.ni_2.z;
  C(4,5) = - K_p_dot * state.ni_2.x;
  C(5,3) = - M_q_dot * state.ni_2.y;
  C(5,4) =   K_p_dot * state.ni_2.x;
 
}

void updateG(){
  Matrix3f J_inv = compute_jacobian1(state.eta_2).transpose();
  Vector3f f_g = J_inv*weight;
  Vector3f f_b = J_inv*buoyancy;
  G.head(3) = f_g + f_b;
  G.tail(3) = r_b.cross(f_b);
}

void resolveDynamics(){
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

  ros::Rate loop_rate(1);
  
  float count = 1;

  initializeM();
  std::cout << "eccentricità = " << e << std::endl;
  std::cout << "alpha0 = " << alpha_0 << std::endl;
  std::cout << "beta0 = " << beta_0 << std::endl;
  std::cout << M << std::endl;

  /* TEST COMPUTE_JACOBIAN */
  modellazione::state_real test_state;
  test_state.eta_2.x = M_PI_2/2;
  test_state.eta_2.y = M_PI_2/2;
  test_state.eta_2.z = M_PI_2/2;
  MatrixXf J_tot = compute_jacobian_tot(test_state.eta_2);
  std::cout <<J_tot << std::endl;
  
    
  while (ros::ok()){
    
    ros::spinOnce();

    resolveDynamics();
    state.prova = count++;

    ROS_INFO("Sto per pubblicare eta1 = [%f %f %f] \n msg numero: %f ", state.eta_1.x, state.eta_1.y, state.eta_1.z,
              state.prova);

    model_pub.publish(state);

    //ros::spinOnce();

    loop_rate.sleep();
  }

return 0;
}


