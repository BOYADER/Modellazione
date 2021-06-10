#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "constant.h"
#include "math_utility.h"
#include "geometry_msgs/Vector3.h"
#include "modellazione/state_real.h"
#include "modellazione/ahrs.h"
#include <random>

using namespace Eigen;
using namespace std;

modellazione::ahrs ahrs_measure; // contiene le misure che verranno di volta in volta pubblicate 

// Questa funzione legge dalla topic /state_real i valori relativi allo stato attuale del robot
void ahrs_state_read(const modellazione::state_real& state){
  ahrs_measure.rpy = state.eta_2;
  ahrs_measure.acc = state.eta_1_dot_dot;
  ahrs_measure.gyro = state.ni_2;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "ahrs_sensor");

  ros::NodeHandle ahrs_sensor;

  ros::Subscriber ahrs_sub = ahrs_sensor.subscribe("state_real", 1, ahrs_state_read);
  ros::Publisher ahrs_pub = ahrs_sensor.advertise<modellazione::ahrs>("sensor/ahrs", MAX_QUEUE_LENGTH);

  // Variabili per la generazione dei rumori bianchi con cui sporcare le misure
  default_random_engine generator;
  normal_distribution<double> rp_distribution(0, deg2rad(0.03));    //[rad]
  normal_distribution<double> y_distribution(0, deg2rad(1));        //[rad]
  normal_distribution<double> gyro_distribution(0, 0.01);      		  //[rad/s] 

  // Variabile che può essere usata per aggiungere un bias alle misure del giroscopio
  // anche se noi la settiamo a 0 per come concordato col blocco navigazione
  float gyro_bias = 0;
  
  // Settiamo la frequenza di pubblicazione sulla topic
  ros::Rate loop_rate(SENSOR_FREQUENCY);
  
  // Facciamo una spinOnce() prima dell'inizio del loop vero e proprio perché altrimenti 
  // la prima volta che si chiama questa funzione nel loop non viene letto niente dalla topic
  ros::spinOnce();
  loop_rate.sleep();

  // Loop principale del sensore
  while(ros::ok()){
    
    // Lettura delle info di interesse dallo stato attuale
  	ros::spinOnce();
    
    // Si sporcano le misure 
    ahrs_measure.rpy.x += rp_distribution(generator);
    ahrs_measure.rpy.y += rp_distribution(generator);
    ahrs_measure.rpy.z += y_distribution(generator);
    ahrs_measure.gyro.x += gyro_distribution(generator) + gyro_bias;
    ahrs_measure.gyro.y += gyro_distribution(generator) + gyro_bias;
    ahrs_measure.gyro.z += gyro_distribution(generator) + gyro_bias;

    // Informazioni di debug
    /*ROS_INFO("[AHRS]Sto per pubblicare \ngyro = [%f %f %f] \nahrs= [%f %f %f]\n",
            ahrs_measure.gyro.x, ahrs_measure.gyro.y, ahrs_measure.gyro.z,
            ahrs_measure.rpy.x, ahrs_measure.rpy.y, ahrs_measure.rpy.z);*/
    
    // Pubblicazione sulla topic delle misurazioni
    ahrs_pub.publish(ahrs_measure);

    loop_rate.sleep();

  }

  return 0;
}
