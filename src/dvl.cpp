#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/dvl.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <random>


using namespace Eigen;
using namespace std;

modellazione::dvl dvl_measure; // Contiene le misure che verranno di volta in volta pubblicate

// Variabili in cui si salvano i valori di deviazione std calcolati di volta in volta in funzione della velocità
double std_dev_x, std_dev_y, std_dev_z; 

// Questa funzione legge dalla topic /state_real i valori relativi allo stato attuale del robot
void dvl_state_read(const modellazione::state_real &state)
{
  // Lettura delle info dalla topic
  Vector3f ni1 = ros2eigen(state.ni_1);
  Matrix3f S = skew_symmetric(state.ni_2);

  // Aggiornamento della variabile globale che contiene le misure
  dvl_measure.lin_vel = eigen2ros( R_dvl_body *(ni1 + S*p_dvl) ); 
  dvl_measure.counter++; 
  
  // Calcolo deviazioni standard che verranno usate nel main per sporcare le misure
  std_dev_x = fabs(0.01 * ni1(0)) + 0.01;
  std_dev_y = fabs(0.01 * ni1(1)) + 0.01;
  std_dev_z = fabs(0.01 * ni1(2)) + 0.01;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "dvl_sensor");

  ros::NodeHandle dvl_sensor;

  ros::Subscriber dvl_sub = dvl_sensor.subscribe("state_real", 1, dvl_state_read);
  ros::Publisher dvl_pub = dvl_sensor.advertise<modellazione::dvl>("sensor/dvl", MAX_QUEUE_LENGTH);
  
  //Inizializzazione della matrice di rotazione tra terna BODY e terna DVL
  initialise_R_dvl_body();
  
  // Variabile per la generazione del rumore bianco con cui sporcare le misure
  // Nota: le varie distribuzioni verranno create nel loop perché variano da ciclo a ciclo 
  default_random_engine generator;
  
  // Settiamo la frequenza di pubblicazione sulla topic
  ros::Rate loop_rate(SENSOR_FREQUENCY);

  while(ros::ok()){ 
  
    //Lettura delle info di interesse dallo stato attuale
    ros::spinOnce();
    
    // Si creano le distribuzioni di rumore
    normal_distribution<double> dvl_distribution_x(0, std_dev_x);
    normal_distribution<double> dvl_distribution_y(0, std_dev_y);
    normal_distribution<double> dvl_distribution_z(0, std_dev_z);
    
    // Si sporcano le misure
    dvl_measure.lin_vel.x += dvl_distribution_x(generator);
    dvl_measure.lin_vel.y += dvl_distribution_y(generator);
    dvl_measure.lin_vel.z += dvl_distribution_z(generator);

    // Informazioni di debug
    /*ROS_INFO("[DVL] Sto per pubblicare misura dvl: \n x = [%f] \n y = [%f] \n z = [%f] ", 
              dvl_measure.lin_vel.x, dvl_measure.lin_vel.y, dvl_measure.lin_vel.z);*/
    
    // Pubblicazione sulla topic delle misurazioni
    dvl_pub.publish(dvl_measure);

    loop_rate.sleep();

  }


  return 0;
}
