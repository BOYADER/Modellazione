#include "/usr/include/eigen3/Eigen/Dense"
#include <cmath>

using namespace Eigen;

//PROFONDIMETRO
#define R_PROFONDIMETRO 0.0 // distanza tra centro di massa e profondimetro [m]
Vector3f p_depth(0, 0, R_PROFONDIMETRO); // vettore posizione del profondimetro in terna body

//DVL
#define R_DVL_X 0.37 //coordinata x del dvl rispetto alla terna body
#define R_DVL_Z 0.03 //coordinata z del dvl rispetto alla terna body
#define THETA_DVL 0 //rotazione relativa tra terna body e dvl 
Vector3f p_dvl(R_DVL_X, 0, R_DVL_Z); // vettore posizione del DVL in terna body
MatrixXf R_dvl_body(3,3); // matrice di rotazione da terna body a terna pDVL

// Funzione che inizializza la matrice di rotazione R_dvl_body
void initialise_R_dvl_body(){
	R_dvl_body(0,0) = cos(THETA_DVL); 
	R_dvl_body(0,1) = 0;
	R_dvl_body(0,2) = -sin(THETA_DVL);
	R_dvl_body(1,0) = 0;
	R_dvl_body(1,1) = 1;
	R_dvl_body(1,2) = 0;
	R_dvl_body(2,0) = sin(THETA_DVL); 
	R_dvl_body(2,1) = 0;
	R_dvl_body(2,2) = cos(THETA_DVL);
}


//USBL

//Nota: supponiamo che terna usbl coincida con terna body
#define R_USBL_Z 0.0   // supponiamo che l'usbl si trovi nel centro di massa
#define R_T 50.0         // profondita' del transponder [m]
Vector3f p_usbl(0, 0, R_USBL_Z); // vettore posizione dell'usbl in terna body
Vector3f p_t_ned(0, 0, R_T); // vettore posizione del trasponder in terna NED
