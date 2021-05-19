#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/gps.h"
#include "modellazione/state_real.h"
#include <random>

using namespace std;
using namespace Eigen;

modellazione::gps gps_measure;
geometry_msgs::Vector3 eta1;


void gps_state_read(const modellazione::state_real &state)
{
  eta1 = state.eta_1;
  if(eta1.z >= 0.1)
    gps_measure.under_water = true;
  else 
    gps_measure.under_water = false;
}

void compute_measure(){

  //TODO: Conversion between NED to LL


}

int main(int argc, char **argv){

  ros::init(argc, argv, "gps_sensor");

  ros::NodeHandle gps_sensor;

  ros::Subscriber gps_sub = gps_sensor.subscribe("state_real", 1, gps_state_read);
  ros::Publisher gps_pub = gps_sensor.advertise<modellazione::gps>("sensor/gps", MAX_QUEUE_LENGTH);

  //Generazione Rumore
  default_random_engine generator;
  normal_distribution<double> gps_distribution(0, 2); //[m]

  ros::Rate loop_rate(10);

  while(ros::ok()){

  	ros::spinOnce();
    compute_measure();
    gps_measure.lla.x += gps_distribution(generator);
    gps_measure.lla.y += gps_distribution(generator);

    gps_pub.publish(gps_measure);

    loop_rate.sleep();

  }

  return 0;
}