#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Vector3.h>
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "math.h"
#include "modellazione/state_real.h"
#include "modellazione/usbl.h"

//TODO: variable frequency & conversion between NED => RBE			

using namespace Eigen;

geometry_msgs::Vector3 eta1, eta2;


int computeFrequency(){
  //TODO: CALCOLARE LA FREQUENZA
  return 1;

}


modellazione::usbl computeMeasure(){
    //TODO: OFFSET + ROTAZIONI

    modellazione::usbl measure;
    measure.pos.x = sqrt(eta1.x*eta1.x + eta1.y*eta1.y + eta1.z*eta1.z);    //range
    measure.pos.y = atan2(eta1.y, eta1.x);                                  //bearing
    measure.pos.z = atan2(eta1.z, sqrt(eta1.x*eta1.x + eta1.y*eta1.y));     //elevation

    return measure;
}


void usbl_state_read(const modellazione::state_real &state)
{

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
  modellazione::usbl measure;

  //Compute RoundTripTime
  //ros::Rate loop_rate(RTT);
  //loop_rate.sleep();

  ros::Rate loop_rate(RTT); //NECESSARIO PER FAR SI CHE DENTRO IL WHILE 
  ros::spinOnce();  
  loop_rate.sleep();                        //VENGA EFFETTIVAMENTE LETTO IL PRIMO MSG

  while(ros::ok()){

  	ros::spinOnce(); 
  	RTT = computeFrequency(); 
    measure = computeMeasure();

  	ros::Rate loop_rate(RTT);
    loop_rate.sleep();
    ROS_INFO("sto per pubblicare: rbe = \n [%f \n %f \n %f] \n",measure.pos.x, measure.pos.y, measure.pos.z) ;
    usbl_pub.publish(measure);

  }


  return 0;
}

