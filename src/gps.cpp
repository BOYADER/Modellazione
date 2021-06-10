#include "ros/ros.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/gps.h"
#include "modellazione/state_real.h"
#include "math_utility.h"
#include <random>

using namespace std;
using namespace Eigen;

modellazione::gps gps_measure; // Contiene i messaggi che verranno di volta in volta pubblicate 
geometry_msgs::Vector3 eta1; // Variabile di appoggio per la lettura della posizione dalla topic /state_real

// Questa funzione legge dalla topic /state_real i valori relativi allo stato attuale del robot
void gps_state_read(const modellazione::state_real &state)
{
  eta1 = state.eta_1;
  if(eta1.z >= 0.32)
    gps_measure.under_water = true;
  else 
    gps_measure.under_water = false;
  gps_measure.counter++;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "gps_sensor");

  ros::NodeHandle gps_sensor;

  ros::Subscriber gps_sub = gps_sensor.subscribe("state_real", 1, gps_state_read);
  ros::Publisher gps_pub = gps_sensor.advertise<modellazione::gps>("sensor/gps", MAX_QUEUE_LENGTH);

  // Variabili per la generazione dei rumori bianchi con cui sporcare le misure
  default_random_engine generator;
  normal_distribution<double> gps_distribution(0, 1); //[m]
  
  // Variabile in cui salviamo le info di lat e long dopo la conversione NED => LL
  Vector2f LL;

  // Dati necessari per la conversione NED => LL 
  float lat0, lon0, a0;
  gps_sensor.getParam("/initial_pose/position/latitude",   lat0);
  gps_sensor.getParam("/initial_pose/position/longitude",  lon0);
  gps_sensor.getParam("/initial_pose/position/depth",      a0);
  geometry_msgs::Vector3 pos_0;
  pos_0.x = lat0;
  pos_0.y = lon0;
 
  // Settiamo la frequenza di pubblicazione sulla topic
  ros::Rate loop_rate(1);

  while(ros::ok()){

    // Lettura delle info di interesse dallo stato attuale
  	ros::spinOnce();
    
    // Si sporcano le misure (in terna NED)
   	eta1.x += gps_distribution(generator);
   	eta1.y += gps_distribution(generator);
    
    // Conversione NED => LL
    LL = NEDtoLL_conversion(eta1, pos_0);
    
    // Si salvano le misure nel messaggio che verrà pubblicato
    gps_measure.lla.x = LL(0);
    gps_measure.lla.y = LL(1);

    //Informazioni di debug
    /*ROS_INFO("[GPS] Sto per pubblicare: lat = %f long = %f",gps_measure.lla.x, gps_measure.lla.y);*/
    
    // Pubblicazione sulla topic
    gps_pub.publish(gps_measure);

    loop_rate.sleep();

  }

  return 0;
}
