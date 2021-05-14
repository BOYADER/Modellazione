#include <iostream>
#include "/usr/local/include/eigen3/Eigen/Dense"
 
using namespace Eigen;
using namespace std;
 
int main()
{
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
}

