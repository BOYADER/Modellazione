#include "ros/ros.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include <geometry_msgs/Vector3.h>
#include <cmath>

#define A 6378137.0
#define F 1/298.257223563

using namespace Eigen;

// Questa funzione permette di passare da un Vector3 di ROS a un Vector3f di Eigen
geometry_msgs::Vector3 eigen2ros(Vector3f v)
{
	geometry_msgs::Vector3 v_msg;
	v_msg.x = v(0);
	v_msg.y = v(1);
	v_msg.z = v(2);
	return v_msg;
}

// Questa funzione permette di passare da un Vector3f di Eigen a un Vector3 di ROS
Vector3f ros2eigen(geometry_msgs::Vector3 v_msg)
{
	Vector3f v(v_msg.x, v_msg.y, v_msg.z);
	return v;
}

float robot_volume = (4.0/3.0) * M_PI * R_A * R_B * R_C; // volume del robot [m^3]
float robot_weight = RHO_V * robot_volume * G_ACC;	 // forza peso del robot [N]
float robot_buoyancy = RHO_W * robot_volume * G_ACC;	 // forza di galleggiamento [N]
Vector3f r_b(0, 0, -R_GB);			// vettore posizione del centro geometrico in terna body
Vector3f weight(0, 0, robot_weight);		// vettore forza peso in terna body
Vector3f buoyancy(0, 0, -robot_buoyancy);	// vettore forza di galleggiamento in terna body


// Questa funzione restituisce la parte lneare del Jacobiano geometrico (3x3)
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
// Questa funzione restituisce la parte angolare del Jacobiano geometrico (3x3)
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

// Questa funzione restituisce il Jacobiano geometrico completo (6x6)
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

// Questa funzione restituisce la hat form di un vettore 
// per fare il prodotto vettoriale in forma di prodotto matrice per vettore 
Matrix3f skew_symmetric(geometry_msgs::Vector3 ni2){
	Vector3f ni_2 = ros2eigen(ni2);
	Matrix3f S(3,3);

	S(0,0) = S(1,1) = S(2,2) = 0;
	S(0,1) = -ni_2(2);
	S(0,2) = ni_2(1);
	S(1,0) = ni_2(2);
	S(1,2) = -ni_2(0);
	S(2,0) = -ni_2(1);
	S(2,1) = ni_2(0);

	return S;
}

// Funzione per la conversione da gradi a radianti
float deg2rad(float degree){
  float rad = degree * M_PI /180;
  return rad;
}

// Funzione per la conversione da radianti a gradi
float rad2deg(float rad){
	float degree = rad * 180 / M_PI;
	return degree;
}

// Funzione che rimappa un angolo in radianti nel range [-pi, +pi]
float constrain_angle(double x){
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

// Funzione per la conversione da NED a LL
Vector2f NEDtoLL_conversion(geometry_msgs::Vector3 eta1, geometry_msgs::Vector3 lla0){
  Vector3f eta_1 = ros2eigen(eta1);
  Vector2f lla0rad;
  lla0rad(0) = deg2rad(lla0.x);
  lla0rad(1) = deg2rad(lla0.y);
  float Rn = A / sqrt (1 - (2 * F - F * F) * sin(lla0rad(0)) * sin(lla0rad(0)));
  float Rm = Rn * (1 - (2 * F - F * F))/(1 - (2 * F - F * F) * sin(lla0rad(0)) * sin(lla0rad(0)));
  Vector2f LL;
  LL(0) = (lla0rad(0) + atan2(1, Rm) * eta_1(0)) * 180/M_PI;
  LL(1) = (lla0rad(1) + atan2(1, Rn * cos(lla0rad(0))) * eta_1(1)) * 180/M_PI;

  return LL;
}

// Funzione per generare un float casuale nel range [min, max]
float frand(float min, float max){
	float range = rand()/(float )RAND_MAX;
	return min + range*(max-min);
}
