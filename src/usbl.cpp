#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "math.h"
#include "modellazione/state_real.h"
#include "modellazione/usbl.h"
#include <random>

//TODO: variable frequency & conversion between NED => RBE			

using namespace Eigen;
using namespace std;

geometry_msgs::Vector3 eta1, eta2;
modellazione::usbl usbl_measure; //messaggio da pubblicare

int compute_frequency(){
  //TODO: CALCOLARE LA FREQUENZA
  return 1;

}


void compute_measure(){
    //TODO: OFFSET + ROTAZIONI

    //conversione NED => SPHERICAL
    usbl_measure.pos.x = sqrt(eta1.x*eta1.x + eta1.y*eta1.y + eta1.z*eta1.z);    //range
    usbl_measure.pos.y = atan2(eta1.y, eta1.x);                                  //bearing
    usbl_measure.pos.z = atan2(eta1.z, sqrt(eta1.x*eta1.x + eta1.y*eta1.y));     //elevation

    usbl_measure.counter++;

}


void usbl_state_read(const modellazione::state_real &state)
{

  //Salvo posizione e orientazione in terna NED
  eta1 = state.eta_1;
  eta2 = state.eta_2;

  ROS_INFO("I heard: eta1 = [%f %f %f] \n eta2 =  [%f %f %f] \nmsg numero: %f", eta1.x, eta1.y, eta1.z, eta2.x, eta2.y, eta2.z, state.prova);
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "usbl_sensor");

  ros::NodeHandle usbl_sensor;

  ros::Subscriber usbl_sub = usbl_sensor.subscribe("state_real", 1, usbl_state_read);
  ros::Publisher usbl_pub = usbl_sensor.advertise<modellazione::usbl>("sensor/usbl", MAX_QUEUE_LENGTH);

  int RTT = 1;

  //Generazione rumore
  default_random_engine generator;
  normal_distribution<double> range_distribution(0, 1e-2);  //[m]
  normal_distribution<double> bearing_distribution(0, 1);   //[deg]
  normal_distribution<double> elevation_distribution(0, 1); //[deg]
  
  ros::Rate loop_rate(RTT);  
  ros::spinOnce();  
  loop_rate.sleep();                        

  while(ros::ok()){
  	ros::spinOnce();

  	RTT = compute_frequency(); 
    compute_measure();
    usbl_measure.pos.x += range_distribution(generator);
    usbl_measure.pos.y += bearing_distribution(generator);
    usbl_measure.pos.z += elevation_distribution(generator);


  	ros::Rate loop_rate(RTT);
    loop_rate.sleep();

    ROS_INFO("sto per pubblicare: rbe = \n [%f \n %f \n %f] \n",usbl_measure.pos.x, usbl_measure.pos.y, usbl_measure.pos.z);
    usbl_pub.publish(usbl_measure);

  }


  return 0;
}

