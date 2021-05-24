/*****************************************************************************/
/* File: formation_graph_controller.c
/*****************************************************************************/

/**************************************************/
/* LIBRARY */
/**************************************************/
/* standard library */
#include <stdio.h>
#include <math.h>
#include <string.h>

/* webots library */
#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

/* gsl library */
#include <gsl/gsl_vector.h> // vector
#include <gsl/gsl_matrix.h> // matrix
#include <gsl/gsl_blas.h> // matrix operation

/* custom library */
// need to compile as library files and link to executables
// #include "../localization_library_test/localization.h"
// #include "../localization_library_test/trajectories.h"
// #include "../localization_library_test/odometry.h"
// #include "../localization_library_test/kalman.h"

/**************************************************/
/* PARAMETERS */
/**************************************************/
#define NB_SENSORS           8
/* Formation parameters */
// #define D                    0.1      // Distance between robots
// graph-based control
#define FLOCK_SIZE	  2	  // Size of flock
#define EDGE_SIZE	  1	  // Size of flock
#define AXLE_LENGTH          0.052     // Distance between wheels of robot
#define SPEED_UNIT_RADS      0.0628    // Conversion factor between speed unit to radian per second
#define WHEEL_RADIUS         0.0205    // Wheel radius in meters
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define INIT_TYPE_LOCAL true // if true, init with custom setup; else (false), init with supervisor
// init and goal position
// from -1.9 -> 2.1 = 4
#define GOAL_X  4.0  // X-coordinate of goal position
#define GOAL_Y  0.0  // Y-coordinate of goal position
float error[2] = {GOAL_X, GOAL_Y}; // Error dynamics
// initial position
float init_x[FLOCK_SIZE] = {-2.9, -2.9};
float init_y[FLOCK_SIZE] = {0.0, 0.1};

/* TUNING PARAMETERS */
float DELTA_T=0.064;	// Timestep (seconds)
float THRESHOLD=0.05;	// Convergence threshold
int Interconn[16] = {-5,-15,-20,6,4,6,3,5,4,4,6,-18,-15,-5,5,3};

// bias vector
float bias_x[FLOCK_SIZE] = {0, 0};
float bias_y[FLOCK_SIZE] = {0.0, 0.1};

/**************************************************/
/* WEBOTS INITIALIZATION */
/**************************************************/
/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/
WbDeviceTag ds[NB_SENSORS];           // Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

/**************************************************/
/* ROBOT VARIABLES INITIALIZATION */
/**************************************************/
int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with graph-based formation
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with graph-based formation
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
char* robot_name;

/**************************************************/
/* UTILITY FUNCTIONS */
/**************************************************/
/* Compute laplacian matrix, I * W * I^T */
void compute_laplacian(gsl_matrix * incidence, gsl_matrix * weight, gsl_matrix * laplacian){
    // temp matrix (FLOCK_SIZE) xE
    gsl_matrix * temp  = gsl_matrix_alloc (FLOCK_SIZE, EDGE_SIZE);
    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                    1.0, incidence, weight,
                    0.0, temp);
    gsl_blas_dgemm (CblasNoTrans, CblasTrans,
                    1.0, temp, incidence,
                    0.0, laplacian);
    gsl_matrix_free (temp);
}

void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter, out,strlen(out)+1);
}

/* Keep given int number within interval {-limit, limit} */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/**************************************************/
/* ROBOT-RELATED FUNCTIONS */
/**************************************************/
/* Initialization sensor, motor, receiver, etc. */
void reset(){
	wb_robot_init();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);

	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name();

	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],64);
	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}

	printf("Reset: robot %d\n",robot_id_u);
                 // TODO: add error term
	// error[0] = my_position[0] + MIGRATORY_DEST_X;
	// error[1] = my_position[1] + MIGRATORY_DEST_Y;
}

/* Initialize robot's position */
void initial_pos_local(void){
	while (initialized[robot_id] == 0) {
		// Initialize self position with pre-defined array
		my_position[0] = init_x[robot_id];  // x-position
		my_position[1] = init_y[robot_id]; // z-position
		my_position[2] = 0; // theta
		prev_my_position[0] = init_x[robot_id];
		prev_my_position[1] = init_y[robot_id];
		prev_my_position[2] = 0;
		initialized[robot_id] = 1;  // initialized = true
	}
}

// void initial_pos_super(void){
	// char *inbuffer;
	// int rob_nb;
	// float rob_x, rob_z, rob_theta; // Robot position and orientation

	// while (initialized[robot_id] == 0) {

		// // wait for message
		// while (wb_receiver_get_queue_length(receiver) == 0)	wb_robot_step(TIME_STEP);

		// inbuffer = (char*) wb_receiver_get_data(receiver);
		// sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_z,&rob_theta, &migr[0], &migr[1]);
		// // Only info about self will be taken into account at first.

        // robot_nb %= FLOCK_SIZE;
		// if (rob_nb == robot_id) {
			// // Initialize self position
			// loc[rob_nb][0] = rob_x; 		// x-position
			// loc[rob_nb][1] = rob_z; 		// z-position
			// loc[rob_nb][2] = rob_theta; 		// theta
			// prev_loc[rob_nb][0] = loc[rob_nb][0];
			// prev_loc[rob_nb][1] = loc[rob_nb][1];
			// initialized[rob_nb] = 1; 		// initialized = true
		// }
		// wb_receiver_next_packet(receiver);
	// }
// }

/* Compute relative ranger and bearing */
void compute_relative_pos(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[1];

		theta =	-atan2(y,x);
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));

		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

		wb_receiver_next_packet(receiver);
	}

	// TODO: compare performace with global true position
	//get true pos from supervisor
	// if (inbuffer[0] != 'e'){
	// // printf("Robot %d \n",inbuffer[0]- '0');

	// int i = inbuffer[0]- '0';
	// sscanf(inbuffer,"%1d#%f#%f#%f",&i,&true_position[i][0],&true_position[i][1],&true_position[i][2]);
	// wb_receiver_next_packet(receiver);
	// true_position[i][2] += M_PI/2;
	// if (true_position[i][2] > 2*M_PI) true_position[i][2] -= 2.0*M_PI;
	// if (true_position[i][2] < 0) true_position[i][2] += 2.0*M_PI;
	// printf("Robot %d is in %f %f %f\n",i,true_position[i][0],true_position[i][1],true_position[i][2]);
	// continue;
	// }
}

/* Compute laplacian functions */
void update_laplacian(void){
  printf("TODO");
}

/* Advanced method to updates robot position with wheel speeds */
// void update_self_motion()

/* Naive method to updates robot position with wheel speeds */
void update_self_motion_naive(int msl, int msr) {
	float theta = my_position[2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dy = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dy;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

/* Compute wheel speeds arcording to graph */
void compute_wheel_speeds(int nsl, int nsr, int *msl, int *msr, gsl_matrix * laplacian){
	// x'= -L(x-b), Same for y
	//laplacian[robot_id:] * (x-b)
	// init with speed every time with zero
	speed[robot_id][0] = 0; speed[robot_id][1] = 0;
	for(int j = 0; j < FLOCK_SIZE; j++){
		speed[robot_id][0] -= gsl_matrix_get (laplacian, robot_id, j) * (my_position[j] - bias_x[j]);
		speed[robot_id][1] -= gsl_matrix_get (laplacian, robot_id, j) * (my_position[j] - bias_y[j]);
		// printf ("After computing (%d, %d) -> current speed of agent %d (x, y): (%f, %f) \n",i, j, i, speed[i][0], speed[i][1]);
	}

	// Compute speed in global coordinate
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float y = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = -atan2(x, y);	  // Orientation of the wanted position

	// relative localization system to compute forward and rotational speed
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;

	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}

/* Obstacle avoidance */

/*************************/
/* MAIN */
/*************************/
int main(){

                // Incidence Matrix V (FLOCK_SIZE) xE
                gsl_matrix * incidence  = gsl_matrix_calloc (FLOCK_SIZE, EDGE_SIZE);
                // Weight Matrix E x E
                gsl_matrix * weight  = gsl_matrix_calloc (EDGE_SIZE, EDGE_SIZE);
                // Laplacian Matrix VxV I * W * I^T
                gsl_matrix * laplacian  = gsl_matrix_calloc (FLOCK_SIZE, FLOCK_SIZE);
	// init laplacian for the robot
	// In Figure 7, we have linked two vehicles using the above
	// decentralized law with the incidence matrix I = [1, −1]T
	// and the weight matrix W = [1].
	// be careful when setting incidence
	gsl_matrix_set(incidence, 0, 0, 1.0);
	gsl_matrix_set(incidence, 1, 0, -1.0);
	gsl_matrix_set_identity(weight);
	compute_laplacian(incidence, weight, laplacian);

	int msl=0,msr=0;                      // motor speed left and right
	/*Webots 2018b*/
	float msl_w, msr_w;
	/*Webots 2018b*/
	float new_leader_range, new_leader_bearing, new_leader_orientation; // received leader range and bearing
	int distances[NB_SENSORS];        // array keeping the distance sensor readings
	float *rbbuffer;                  // buffer for the range and bearing
	int i,initialized;

	reset();                          // Initialization
	if (INIT_TYPE_LOCAL){
		initial_pos_local(); // Initializing the robot's position from local settings
	}
	else{
		initial_pos_super(); // Initializing the robot's position from supervisor
	}

	for(;;){
		/* I. Obstacle avoidance */
		int sensor_nb;
		int bmsl = 0;
		int bmsr = 0;
		for(sensor_nb=0;sensor_nb<NB_SENSORS;sensor_nb++){  // read sensor values and calculate motor speeds
		  distances[sensor_nb]=wb_distance_sensor_get_value(ds[sensor_nb]);
		  /* Weighted sum of distance sensor values for Braitenburg vehicle */
		  bmsr += distances[sensor_nb] * Interconn[sensor_nb];
		  bmsl += distances[sensor_nb] * Interconn[sensor_nb + NB_SENSORS];
		}

		// TODO: need test!!!
		bmsl /= 400; bmsr /= 400;        // Normalizing speeds
		// Adapt Braitenberg values (empirical tests)
		bmsl/=MIN_SENS; bmsr/=MIN_SENS;
		bmsl+=66; bmsr+=72;

		/* II. Udpate position from receiver */
		send_ping();  // sending a ping to other robot

		// Compute self position & speed
		// 1. update self's position from kalman filter/ encoder info
		prev_my_position[0] = my_position[0];
		prev_my_position[1] = my_position[1];
		update_self_motion_naive(msl,msr);
		speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
		speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

		// 2. update others position from receiver
		compute_relative_pos();

		// III. Update with graph-based control
		// update_laplacian(); // update with range and bearing position
		compute_wheel_speeds(0, 0, &msl, &msr, laplacian);
		msl += bmsl;
		msr += bmsr;


		/*Webots 2018b*/
		// Set speed
		msl_w = msl*MAX_SPEED_WEB/1000;
		msr_w = msr*MAX_SPEED_WEB/1000;
		wb_motor_set_velocity(left_motor, msl_w);
		wb_motor_set_velocity(right_motor, msr_w);
		//wb_differential_wheels_set_speed(msl,msr);
		/*Webots 2018b*/
		wb_robot_step(64);               // Executing the simulation for 64ms
	}
}