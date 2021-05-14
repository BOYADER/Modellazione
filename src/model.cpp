#include <iostream>
<<<<<<< Updated upstream
#include "/usr/include/eigen3/Eigen/Dense"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
=======
//#include "/usr/local/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Dense"
 
/*  Model node must subscribe /tau topic in order to obtain control actions.
    After solving dynamics differential equations it has to publish on 
    /state_real topic, from which every sensor will subscribe and take the 
    actual informations needed.
*/      
>>>>>>> Stashed changes

using namespace Eigen;
using namespace std;
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "model");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("state_real", 1000);

  ros::Rate loop_rate(10);

  
  int count = 0;
  while (ros::ok())
  {
    
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }




  MatrixXd m_rb(6,6);
  m_rb(0,0)=50;
  m_rb(1,1)=50;
  m_rb(2,2)=50;
  m_rb(1,4)=-3.5;
  m_rb(2,3)=3.5;
  m_rb(3,2)=3.5;
  m_rb(4,1)=-3.5;
  m_rb(3,3)=0.695;
  m_rb(4,4)=3.1928;
  m_rb(5,5)=2.9478;

std::cout << "Ricky Costy chiappati questa matrice di ineriza va:\n" << m_rb << std::endl;
return 0;
}



