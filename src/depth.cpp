#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/depth.h"
#include "modellazione/state_real.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <cmath>
#include <random>

using namespace Eigen;
using namespace std;

modellazione::depth depth_measure; // contiene le misure che verranno di volta in volta pubblicate

// Questa funzione legge dalla topic /state_real i valori relativi allo stato attuale del robot
void depth_state_read(const modellazione::state_real &state)
{
  // Calcoliamo la profondità del sensore rispetto al CdM in terna NED
  Matrix3f J_inv = compute_jacobian1(state.eta_2).transpose();
  float p_depth_z = (J_inv * p_depth)(2);
  // Calcoliamo la profondità totale del profondimetro, sempre in terna NED
  depth_measure.z = state.eta_1.z + p_depth_z; 
  depth_measure.counter++;         
}



int main(int argc, char **argv){

  ros::init(argc, argv, "depth_sensor");

  ros::NodeHandle depth_sensor;

  ros::Subscriber depth_sub = depth_sensor.subscribe("state_real", 1, depth_state_read);
  ros::Publisher depth_pub = depth_sensor.advertise<modellazione::depth>("sensor/depth", MAX_QUEUE_LENGTH);

  // Variabili per la generazione del rumore bianco con cui sporcare le misure
  default_random_engine generator;
  normal_distribution<double> depth_distribution(0, 2e-3);

  // Settiamo la frequenza di pubblicazione sulla topic
  ros::Rate loop_rate(SENSOR_FREQUENCY);
  
  // Facciamo una spinOnce() prima dell'inizio del loop vero e proprio perché altrimenti 
  // la prima volta che si chiama questa funzione nel loop non viene letto niente dalla topic
  ros::spinOnce();
  loop_rate.sleep();

  // Loop principale del sensore
  while(ros::ok()){
    
    // Lettura delle info di interesse
  	ros::spinOnce();
    // Si sporcano le misure
    depth_measure.z += depth_distribution(generator);
    // Informazioni di debug
    ROS_INFO("sto per pubblicare: \n depth = [%f] \n counter = [%f]", depth_measure.z, depth_measure.counter);
    // Pubblicazione sulla topic del sensore
    depth_pub.publish(depth_measure);
  
    loop_rate.sleep();

  }

  return 0;
}
