#include "ros/ros.h"
#include "std_msgs/String.h"
#include "constant.h"
#include "/usr/include/eigen3/Eigen/Dense"
#include "modellazione/state_real.h"
#include "modellazione/dvl.h"

//TODO: variable variance

modellazione::dvl dvl_measure;


void dvl_state_read(const modellazione::state_real & state)
{
  /* ROS_INFO("I heard: [%s]", state->data.c_str());
  msg.data = eta->state.c_str();*/
  
}



int main(int argc, char **argv){

  ros::init(argc, argv, "dvl_sensor");

  ros::NodeHandle dvl_sensor;

  ros::Subscriber dvl_sub = dvl_sensor.subscribe("state_real", 1, dvl_state_read);
  ros::Publisher dvl_pub = dvl_sensor.advertise<modellazione::dvl>("sensor/dvl", MAX_QUEUE_LENGTH);


  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
 
    //ROS_INFO("sto per pubblicare: %s", msg.data.c_str());

    dvl_pub.publish(dvl_measure);

    loop_rate.sleep();

  }


  return 0;
}