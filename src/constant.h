
/* PARAMETRI DEL ROBOT */
#define R_A 0.5218		//semiasse maggiore [m]   								
#define R_B 0.15		//semiasse minore_1 [m]								
#define R_C 0.15		//semiasse minore_2 [m]								
#define R_M 50			//massa				[kg]												[kg]
#define R_D_GB 0.07 	//distanza tra centro geometrico
						// e centro di massa	[m]


/*PARAMETRI MATRICI DI CORIOLIS E DI DAMPING */

// parametri tensore di inerzia
#define I_X 0.695
#define I_Y 3.1928
#define I_Z 2.9478

#define C_D_X	0.1
#define C_D_YZ	1



/* COSTANTI GENERICHE */
#define V_C 1500 //velocità del suono in acqua: [m/s]

#define MAX_QUEUE_LENGTH 1000
#define SENSOR_FREQUENCY 10

#define RHO_W 1027