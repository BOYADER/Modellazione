#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/dvl.h"
#include "math_utility.h"
#include "sensor_utility.h"
#include <random>


using namespace Eigen;
using namespace std;

//TODO: variable variance
modellazione::dvl dvl_measure;


void dvl_state_read(const modellazione::state_real &state)
{


  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "dvl_sensor");

  ros::NodeHandle dvl_sensor;

  ros::Subscriber dvl_sub = dvl_sensor.subscribe("state_real", 1, dvl_state_read);
  ros::Publisher dvl_pub = dvl_sensor.advertise<modellazione::dvl>("sensor/dvl", MAX_QUEUE_LENGTH);

  initialise_R_dvl_body();
  //default_random_engine generator;
  //NOTA: ASSUMIAMO PER ORA CHE LA STD_DEV (1%) SIA CALCOLATA SU UNA 
  //VELOCITA' DI RIFERIMENTO DI 2 m/s
  //normal_distribution<double> dvl_distribution(0, 0.02);


  ros::Rate loop_rate(10);

  while(ros::ok()){ 

    ros::spinOnce();

    dvl_pub.publish(dvl_measure);

    loop_rate.sleep();

  }


  return 0;
}
