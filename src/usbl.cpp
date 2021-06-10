#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/usbl.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <cmath>
#include <random>



using namespace Eigen;
using namespace std;

modellazione::usbl usbl_measure; // Contiene i messaggi  che verranno di volta in volta pubblicati
geometry_msgs::Vector3 eta1, eta2; // Variabili di appoggio per la lettura di posizione e orientazione dalla topic /state_real
float dist; // Contiene la distanza tra USBL e transponder usata per calcolare RTT

// Questa funzione legge dalla topic /state_real i valori relativi allo stato attuale del robot
void usbl_state_read(const modellazione::state_real &state)
{
  eta1 = state.eta_1;
  eta2 = state.eta_2;
}

// Questa funzione sfrutta le info lette sullo stato attuale per calcolare i valori da pubblicare sulla topic
void compute_measure(){
    Vector3f p_t(0, 0, 0); 
    Matrix3f J_inv = compute_jacobian1(eta2).transpose(); //ned to usbl(coincide con body)

    p_t = J_inv * ( p_t_ned - ( ros2eigen(eta1) + J_inv * (p_usbl) ) ); //J_inv * p_usbl = 0
    dist = p_t.norm();

    usbl_measure.pos.x = sqrt(p_t(0)*p_t(0) + p_t(1)*p_t(1) + p_t(2)*p_t(2));    			//range [m]
    usbl_measure.pos.y = atan2(p_t(1), p_t(0));                                  			//bearing [rad]
    usbl_measure.pos.z = atan2(p_t(2), sqrt(p_t(0)*p_t(0) + p_t(1)*p_t(1)));   				//elevation [rad]

    usbl_measure.counter++;

}

// Funzione che calcola di volta in volta la frequenza di pubblicazione
// in base alla distanza tra robot e trasponder
float compute_frequency(){
  return V_C/(2*dist);
}

//Questa funzione normalizza il valore degli angoli di
//bearing ed elevation per renderli compresi in [-pi,+pi]
void normalize_angles(){
  usbl_measure.pos.y = constrain_angle(usbl_measure.pos.y);
  usbl_measure.pos.z = constrain_angle(usbl_measure.pos.z);
	return;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "usbl_sensor");

  ros::NodeHandle usbl_sensor;

  ros::Subscriber usbl_sub = usbl_sensor.subscribe("state_real", 1, usbl_state_read);
  ros::Publisher usbl_pub = usbl_sensor.advertise<modellazione::usbl>("sensor/usbl", MAX_QUEUE_LENGTH);


  // Generazione rumore
  default_random_engine generator;
  normal_distribution<double> range_distribution(0, 1e-2);  		 		       //[m]
  normal_distribution<double> bearing_distribution(0, deg2rad(0.1));   		 //[rad]
  normal_distribution<double> elevation_distribution(0, deg2rad(0.1)); 		 //[rad]
  
  // Facciamo una spinOnce() prima dell'inizio del loop vero e proprio perché altrimenti 
  // la prima volta che si chiama questa funzione nel loop non viene letto niente dalla topic
  float RTT = 1; // Valore iniziale di default (ai passi successivi al primo verrà calcolato in modo preciso)
  ros::Rate loop_rate(RTT);  
  ros::spinOnce();  
  loop_rate.sleep();                        

  while(ros::ok()){
	
	// Lettura delle info di interesse dallo stato attuale
  	ros::spinOnce();
	// Calcolo della freuenza di pubblicazione per il ciclo attuale
  	RTT = compute_frequency(); 
    
    // Calcolo dei valori da pubblicare
    compute_measure();
	  
    // Si sporcano i valori 
    usbl_measure.pos.x += range_distribution(generator);
    usbl_measure.pos.y += bearing_distribution(generator);
    usbl_measure.pos.z += elevation_distribution(generator);
	  
    // SI normalizzano gli angolo di bearing ed elevation
    normalize_angles();

    // Si aspetta prima di pubblicare per simulare il tempo di andata e ritorno
    ros::Rate loop_rate(RTT);
    loop_rate.sleep();

	  // Informazioni di debug
    /*ROS_INFO("[USBL] Sto per pubblicare: rbe = \n [%f \n %f \n %f] \n",usbl_measure.pos.x, usbl_measure.pos.y, usbl_measure.pos.z);
    std::cout << "RTT = " << RTT<<endl<<endl;*/
    
	  // Pubblicazione sulla topic delle misurazioni
    usbl_pub.publish(usbl_measure);

  }


  return 0;
}

 
