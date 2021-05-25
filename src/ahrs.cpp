#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "constant.h"
#include "geometry_msgs/Vector3.h"
#include "modellazione/state_real.h"
#include "modellazione/ahrs.h"
#include <random>

using namespace Eigen;
using namespace std;

modellazione::ahrs ahrs_measure;



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

  default_random_engine generator;
  normal_distribution<double> rp_distribution(0, 0.3);    //[deg]
  normal_distribution<double> y_distribution(0, 1);       //[deg]
  normal_distribution<double> gyro_distribution(0, 0);    //incognita
  normal_distribution<double> acc_distribution(0, 0);     //incognita
  float acc_bias = 0;
  float gyro_bias = 0;

  ros::Rate loop_rate(SENSOR_FREQUENCY);
  ros::spinOnce();
  loop_rate.sleep();

  while(ros::ok()){

  	ros::spinOnce();

    ahrs_measure.rpy.x += rp_distribution(generator);
    ahrs_measure.rpy.y += rp_distribution(generator);
    ahrs_measure.rpy.z += y_distribution(generator);
    ahrs_measure.gyro.x += gyro_distribution(generator) + gyro_bias;
    ahrs_measure.gyro.y += gyro_distribution(generator) + gyro_bias;
    ahrs_measure.gyro.z += gyro_distribution(generator) + gyro_bias;
    ahrs_measure.acc.x += acc_distribution(generator) + acc_bias;
    ahrs_measure.acc.y += acc_distribution(generator) + acc_bias;
    ahrs_measure.acc.z += acc_distribution(generator) + acc_bias;


    ahrs_pub.publish(ahrs_measure);

    loop_rate.sleep();

  }

  return 0;
}