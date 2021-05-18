#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "geometry_msgs/Vector3.h"
#include "modellazione/state_real.h"
#include "modellazione/ahrs.h"

modellazione::ahrs ahrs_measure;
geometry_msgs::Vector3 old_pos;
geometry_msgs::Vector3 old_vel;
geometry_msgs::Vector3 new_vel;
geometry_msgs::Vector3 new_pos;

void ahrs_state_read(const modellazione::state_real& state){
  ahrs_measure.rpy = state.eta_2;
  ahrs_measure.gyro = state.ni_2;
  new_pos = state.eta_1;
  
}

void compute_measure(){
  pos_dot = (new_pos-old_pos)*SENSOR_FREQUENCY;
  old_pos = new_pos;
  ahrs_measure.acc = (new_vel - old_vel)*SENSOR_FREQUENCY;
  old_vel = new_vel;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "ahrs_sensor");

  ros::NodeHandle ahrs_sensor;

  ros::Subscriber ahrs_sub = ahrs_sensor.subscribe("state_real", 1, ahrs_state_read);
  ros::Publisher ahrs_pub = ahrs_sensor.advertise<modellazione::ahrs>("sensor/ahrs", MAX_QUEUE_LENGTH);

  default_random_engine generator;
  normal_distribution<double> rp_distribution(0, 0.3);    //[deg]
  normal_distribution<double> y_distribution(0, 1);       //[deg]
  normal_distribution<double> gyro_distribution(0, NULL); //incognita
  normal_distribution<double> acc_distribution(0, NULL);  //incognita

  ros::Rate loop_rate(SENSOR_FREQUENCY);
  ros::spinOnce();
  loop_rate.sleep();

  while(ros::ok()){

  	ros::spinOnce();
    compute_measure();
    ahrs_measure.rpy.x += rp_distribution(generator);
    ahrs_measure.rpy.y += rp_distribution(generator);
    ahrs_measure.rpy.z += y_distribution(generator);
    ahrs_measure.gyro.x += gyro_distribution(generator);
    ahrs_measure.gyro.y += gyro_distribution(generator);
    ahrs_measure.gyro.z += gyro_distribution(generator);


    ahrs_pub.publish(ahrs_measure);

    loop_rate.sleep();

  }

  return 0;
}