#include "ros/ros.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include<cmath>

using namespace Eigen;

geometry_msgs::Vector3 eigen2ros(Vector3f v)
{
	geometry_msgs::Vector3 v_msg;
	v_msg.x = v(0);
	v_msg.y = v(1);
	v_msg.z = v(2);
	return v_msg;
}

Vector3f ros2eigen(geometry_msgs::Vector3 v_msg)
{
	Vector3f v(v_msg.x, v_msg.y, v_msg.z);
	return v;
}

float robot_volume= (4/3) * M_PI * R_A * R_B * R_C;
float robot_weigth= RHO_V * robot_volume * G;
float robot_buoyancy = RHO_W * robot_volume * G;




//Returns 6x6 Jacobian BODY=>NED

Matrix3f compute_jacobian1(geometry_msgs::Vector3 eta2){
	Vector3f eta_2 = ros2eigen(eta2);
	Matrix3f J_1(3, 3); 
	J_1(0,0) = cos(eta_2(1))*cos(eta_2(2));
	J_1(0,1) = cos(eta_2(2))*sin(eta_2(0))*sin(eta_2(1)) - cos(eta_2(0))*sin(eta_2(2)); 
	J_1(0,2) = sin(eta_2(0))*sin(eta_2(2)) + cos(eta_2(0))*cos(eta_2(2))*sin(eta_2(1));
	J_1(1,0) = cos(eta_2(1))*sin(eta_2(2));
	J_1(1,1) = cos(eta_2(0))*cos(eta_2(2)) + sin(eta_2(0))*sin(eta_2(1))*sin(eta_2(2));
	J_1(1,2) = cos(eta_2(0))*sin(eta_2(1))*sin(eta_2(2)) - cos(eta_2(2))*sin(eta_2(0));
	J_1(2,0) = -sin(eta_2(1));
	J_1(2,1) = cos(eta_2(1))*sin(eta_2(0));
	J_1(2,2) = cos(eta_2(0))*cos(eta_2(1));
	
	return J_1;

}


Matrix3f compute_jacobian2(geometry_msgs::Vector3 eta2){
	Vector3f eta_2 = ros2eigen(eta2);
	Matrix3f J_2(3, 3); 
	J_2(0,0) =  1;
	J_2(0,1) = sin(eta_2(0))*tan(eta_2(1)); 
	J_2(0,2) = cos(eta_2(0))*tan(eta_2(1));
	J_2(1,0) = 0;
	J_2(1,1) = cos(eta_2(0));
	J_2(1,2) = -sin(eta_2(0));
	J_2(2,0) = 0;
	J_2(2,1) = sin(eta_2(0)) / cos(eta_2(1));
	J_2(2,2) = cos(eta_2(0)) /cos(eta_2(1));
	
	return J_2;

}

MatrixXf compute_jacobian_tot(geometry_msgs::Vector3 eta2){
	Matrix3f J_1= compute_jacobian1(eta2);
	Matrix3f J_2= compute_jacobian2(eta2);
	Matrix3f zeros = MatrixXf::Zero(3,3);
	MatrixXf J_tot (6,6);
	J_tot.block<3,3>(0,0)= J_1;
	J_tot.block<3,3>(3,3)= J_2;
	J_tot.block<3,3>(0,3)= zeros;
	J_tot.block<3,3>(3,0)= zeros;

	return J_tot;
}