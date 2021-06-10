
/* PARAMETRI GEOMETRICI DEL ROBOT */
#define R_A 0.5218	//semiasse maggiore [m]   								
#define R_B 0.15	//semiasse minore_1 [m]								
#define R_C 0.15	//semiasse minore_2 [m]								
#define R_M 50.0	//massa		    [kg]												
#define R_GB 0.07 	//distanza tra centro geometrico e centro di massa	[m]
#define RHO_V 1016.7    //densità veicolo [kg/m^3]

/* PARAMETRI PER LA MATRICE DI DAMPING */
#define C_D_X	0.1
#define C_D_YZ	1.0

/* PARAMETRI PER IL TENSORE DI INERZIA */
#define I_X 0.695
#define I_Y 3.1928
#define I_Z 2.9478

/* COSTANTI GENERICHE */
#define V_C 1500.0 //sound speed in water [m/s]
#define MAX_QUEUE_LENGTH 1000.0
#define RHO_W 1027.0  // water density [kg/m^3]
#define G_ACC 9.80665   // gravity acceleration [m/s^2]

/* PARAMETRI DI FREQUENZA */
#define MODEL_FREQUENCY		100.0 // [Hz]
#define SENSOR_FREQUENCY	10.0  // [Hz]
